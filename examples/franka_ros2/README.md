# franka_ros2 hardware tests on franka-sim

[`franka_ros2`](https://github.com/frankarobotics/franka_ros2) ships two test
files that its own CI refuses to run:

```
franka_bringup/test/test_hardware_example_controllers.py
franka_bringup/test/test_hardware_generic_controller.py
```

They open an FCI connection and move the robot, so upstream's
`.github/workflows/ci.yml` cuts them out by name:

```
colcon test ... --ctest-args --exclude-regex test_hardware
```

[franka-sim](https://github.com/BarisYazici/libfranka-sim) speaks that wire
protocol — the v10 Connect handshake on TCP 1337, a 1 kHz UDP `RobotState`
stream, the gripper protocol on TCP 1338 — so the two excluded ctest entries
become ordinary CI. [`../../.github/workflows/franka_ros2.yml`](../../.github/workflows/franka_ros2.yml)
runs them on a stock `ubuntu-latest` runner, unmodified, against a container.

## What the tests actually do

### `test_hardware_example_controllers.py` — 42 launches

A `launch_testing.parametrize` over 21 `(controller_name, config_file_name)`
pairs, each preceded by a `move_to_start_example_controller` run
(`put_parameter_in_between_parameters` interleaves the initialize step), for 42
parametrizations in one ctest entry. Every one of them:

1. includes `franka_bringup/launch/example.launch.py` with
   `robot_ips:=$ROBOT_IP`, `robot_config_file:=<config>`,
   `controller_names:=<controller>`,
2. waits 3 s (`TimerAction(period=3.0, ...)` in front of `ReadyToTest()`),
3. asserts that no line of the collected launch output contains `[ERROR]`.

So the assertion is coarse but broad: the hardware component has to claim the
FCI, the controller manager has to load and activate a real-time controller,
and 3 s of 1 kHz control has to pass without anything logging an error.

The three configs under `franka_bringup/test/config/`:

| config | robot_type | namespace | notes |
| --- | --- | --- | --- |
| `test_0.config.yaml` | `fr3v2` | none | the default path |
| `test_1.config.yaml` | `fr3` | `test_namespace` | namespaced topics/services |
| `test_use_fake_hardware.config.yaml` | `fr3v2` | none | **misnamed** |

`test_use_fake_hardware.config.yaml` sets `use_fake_hardware: "false"`. Despite
the filename it is a real-FCI config; what it actually varies is
`use_rviz: "true"`. All three set `load_gripper: "true"`.

`gripper_example_controller` drives the hand through `franka_gripper` action
servers, which talk to the gripper command port — so the sim's gripper server
has to be running (it is, by default; `--no-gripper` would break this case).

### `test_hardware_generic_controller.py` — 3 launches

Three controllers, all with `needs_move_to_start: True`:

- `joint_position_example_controller`
- `cartesian_velocity_example_controller`
- `cartesian_orientation_example_controller`

Each includes `franka.launch.py` with `robot_type:=fr3`, `load_gripper:=true`
and spawns the controller `--inactive`. The test body then runs
`move_to_start_example_controller`, switches to the target controller, and lets
it run for 10 s while checking that joint states keep being published. This one
is a much stronger check than the `[ERROR]` grep: it requires a controller
switch on a live FCI connection and sustained streaming afterwards.

### Registration

`franka_bringup/CMakeLists.txt`:

```cmake
if(NOT ROBOT_IP)
  set(ROBOT_IP "172.16.0.1")
endif()
...
add_ros_isolated_launch_test(test/test_hardware_example_controllers.py "robot_ip:=${ROBOT_IP}")
add_ros_isolated_launch_test(test/test_hardware_generic_controller.py "robot_ip:=${ROBOT_IP}")
```

`add_ros_isolated_launch_test` wraps `add_launch_test` with
`ament_cmake_ros`'s `run_test_isolated.py` (its own ROS domain id, so runs do
not cross-talk) and `TIMEOUT 500`.

## The knobs that make it work

- **`--cmake-args -DROBOT_IP=127.0.0.1`.** `ROBOT_IP` is a CMake cache variable
  substituted into `CTestTestfile.cmake` at *configure* time. It is not an
  environment variable at test time; changing it needs a reconfigure (or a
  clean `build/`).
- **`--ctest-args -R test_hardware`.** The inverse of upstream's
  `--exclude-regex test_hardware`: run only the two entries upstream skips.
- **`--packages-ignore` has to be repeated on `colcon test`.** The build only
  needs `--packages-up-to franka_bringup`, but `colcon test` resolves
  `franka_bringup`'s full `package.xml` closure and fails on packages that were
  never built. The five ignored packages
  (`franka_mobile`, `franka_gazebo_bringup`, `franka_mobile_sensors`,
  `franka_vision_and_manipulation_kit`, `franka_mobile_fr3_duo_moveit_config`)
  are the mobile/Gazebo branch, which nothing in these two tests touches; with
  them cut, 17 of the workspace's 45 packages get built instead of 35.
- **`--network host` on both containers.** The FCI is a 1 kHz UDP loop and the
  UDP state stream goes back to the client's own port; published ports put
  Docker's NAT in the middle of it. This is why the workflow is Linux-runner
  only — `--network host` is a no-op on macOS and Windows runners.
- **`--entrypoint /bin/bash`.** Upstream's `franka_entrypoint.sh` re-runs
  `vcs import` into `/ros2_ws`, which is not the mounted workspace.
- **The `manage_overruns` patch.** `franka_arm.ros2_control.xacro` sets an
  attribute that only exists in `ros2_control` with
  `src/franka_ros2/patches/manage_overruns.patch` applied; without it the
  hardware component fails to parse. Upstream's entrypoint applies it, and
  since we bypass the entrypoint the workflow applies it itself (guarded by
  `git apply --check`, so reruns on a warm workspace are idempotent).

## Caveats

- **The GitHub Actions cache only covers the apt layers.** Upstream's
  `Dockerfile` does `COPY . /ros2_ws/src` — including `.git`, whose contents
  differ on every checkout — so every layer from that `COPY` on is a miss. The
  two big `apt-get install` layers above it do hit, which is most of the value.
- **ROS teardown noise is normal.** `Failed shutting down the controllers`
  during launch shutdown, and `rviz2` dying on a headless runner in the
  `use_rviz: "true"` config, are not `[ERROR]` lines the assertions look at.
- **`colcon test-result` reports 47 tests**, which is 42 + 3 launch-test cases
  plus the two ctest entries that wrap them. ctest itself reports `2 tests`;
  the xunit files under `build/franka_bringup/test_results/` carry the 42 and
  the 3.
- **Expected wall time.** On a 12-core box with the apt layers cached and a
  cold workspace: 5m41s image build, 5m32s `colcon build` (17 packages), 5m35s
  tests — 287s for `test_hardware_example_controllers.py`, 48s for
  `test_hardware_generic_controller.py` — about 17 min end to end. A fully cold
  image build (no apt cache) is closer to 8 min, and a 2-core hosted runner
  roughly doubles both builds; the tests are mostly fixed sleeps and barely
  move. The workflow's `timeout-minutes: 75` covers the pessimistic case.

## Running it locally

```bash
examples/franka_ros2/run-local.sh --start-sim
```

The script reads the pinned upstream sha out of the workflow (one source of
truth), clones it into `.validate/franka_ros2/src/franka_ros2`, builds
upstream's image, and runs the *same* container command the workflow runs. It
prints a wall time per step and exits non-zero if the tests fail. Useful
environment:

```bash
WS=/somewhere/else                       # workspace directory
FRANKA_SIM_IMAGE=franka-sim:local        # a locally built sim instead of :edge
FRANKA_ROS2_IMAGE=franka_ros2:ci         # tag for the toolchain image
FRANKA_ROS2_GIT_PROTOCOL0=1              # see below
```

Without `--start-sim` the script assumes a sim is already serving
`127.0.0.1:1337`; with it, it starts one, waits for `franka-sim-check`, and on
exit writes `docker logs` to `$WS/franka-sim.log` before removing the
container.

`FRANKA_ROS2_GIT_PROTOCOL0=1` is a workaround for hosts whose containers cannot
negotiate git protocol v2 against github.com — `vcs import` fails there with
`could not read Username ... expected flush after ref listing`. It generates an
overlay Dockerfile that forces `git config --system protocol.version 0` before
upstream's `vcs import`, and pre-imports the source dependencies on the host,
where git works, so the in-container `--skip-existing` import is a no-op.
GitHub-hosted runners do not need it.

## Upgrading

One line, in the workflow:

```bash
git ls-remote https://github.com/frankarobotics/franka_ros2.git jazzy
```

and put the sha in `FRANKA_ROS2_REF`. `run-local.sh` picks it up from there.
