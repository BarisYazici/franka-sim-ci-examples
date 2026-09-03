# franka-sim examples

Copy-paste CI recipes that run popular libfranka-based robot stacks against
[franka-sim](https://github.com/BarisYazici/libfranka-sim), a drop-in
simulator for the Franka Control Interface. franka-sim speaks the real FCI wire
protocol — the v10 Connect handshake on TCP 1337, a 1 kHz UDP `RobotState`
stream, the gripper protocol on TCP 1338 — so a stack that would otherwise need
an FR3 on the bench runs unmodified on a GitHub-hosted runner. Each example
here is a working workflow in this repository: the tests it runs are the
upstream project's own tests, not ports or reimplementations, and the workflow
badge is what they do against a simulated robot on every push.

## Examples

| Example | Upstream | What runs | Status |
| --- | --- | --- | --- |
| [franka_ros2](examples/franka_ros2/) | [`frankarobotics/franka_ros2`](https://github.com/frankarobotics/franka_ros2) `jazzy` @ [`6cedf7f`](https://github.com/frankarobotics/franka_ros2/commit/6cedf7f1a2ca280c433f643eae697be23eb2a15e) | `franka_bringup` hardware smoke tests, 24 controller launches, 47 tests | ![franka_ros2](https://github.com/BarisYazici/franka-sim-examples/actions/workflows/franka_ros2.yml/badge.svg) |
| [franky](examples/franky/) | [`TimSchneider42/franky`](https://github.com/TimSchneider42/franky) [`v2.0.0`](https://github.com/TimSchneider42/franky/tree/v2.0.0) / [`franky-control==2.0.0`](https://pypi.org/project/franky-control/) | `tests/pytest/test_integration_franky_sim.py`, franky's own integration suite, unmodified: 19 tests over every motion type, torque control, motion callbacks and the gripper | ![franky](https://github.com/BarisYazici/franka-sim-examples/actions/workflows/franky.yml/badge.svg) |

## Copy this into your own repo

Every example is the same three steps: start the simulated robot, build your
code, run your tests.

```yaml
jobs:
  robot-tests:
    runs-on: ubuntu-latest   # Linux only: the sim needs --network host
    steps:
      - uses: actions/checkout@v4

      # 1. The robot. Blocks until a real Connect handshake succeeds, so the
      #    next step can assume a live FCI.
      - uses: BarisYazici/libfranka-sim@v1

      # 2. Your code.
      - run: ./build.sh

      # 3. Your tests, pointed at 127.0.0.1 instead of 172.16.0.2.
      - run: ./run_robot_tests.sh 127.0.0.1
```

The franky example is the whole of that, minus step 2: a `pip install` stack
needs no Docker build and no workspace, so its job is the action, a `curl` of
upstream's test file, and `pytest` once per test.

Three things to get right:

- **`--network host`.** The FCI is a 1 kHz UDP loop whose state stream goes
  back to the client's own port. Publishing ports puts Docker's NAT in the
  middle of it. The action already uses host networking; any container of your
  own that talks to the sim needs `--network host` too, and that restricts the
  job to Linux runners.
- **Point the robot address at `127.0.0.1`.** `-DROBOT_IP=127.0.0.1` for a
  CMake/ctest suite, `robot_ip:=127.0.0.1` for a ROS 2 launch file — wherever
  your stack normally takes the robot's IP.
- **Decide how adversarial the run should be.** By default the sim just works.
  Passing `args: '--enforce-motion-limits'` makes it abort motions the way a
  real FR3 does — commanded discontinuities and limit violations raise the
  matching reflex error — which is what turns a green build into evidence that
  your recovery paths work. The franka_ros2 example runs both as a matrix; the
  franky example runs only the default, because two of upstream's torque tests
  legitimately exceed a joint velocity limit (see its README).
  `--enforce-comm-constraints` exists too, but on a hosted runner without an
  RT kernel it mostly measures the runner's jitter (20-cycle command gaps at
  controller activation), so keep it for machines you control.

See [franka-sim's CI docs](https://github.com/BarisYazici/libfranka-sim/blob/main/docs/ci.md)
for the action inputs, the Docker image, and the pytest fixture.
