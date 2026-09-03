# franky's integration tests on franka-sim

[`franky`](https://github.com/TimSchneider42/franky) is a Python/C++ wrapper
around libfranka: a [Ruckig](https://github.com/pantor/ruckig) online planner in
front of the real 1 kHz control loop. It ships an integration test suite,

```
tests/pytest/test_integration_franky_sim.py
```

that connects a `franky.Robot` and a `franky.Gripper` to a simulated robot and
drives every motion type franky has through it. Upstream, the simulated robot
is franky's own: each test starts a MuJoCo simulator and an FCI server
in-process (the `franky_sim` package) and connects to that.
[franka-sim](https://github.com/BarisYazici/libfranka-sim) serves the same FCI
wire protocol, so the same file runs unmodified against it.
[`../../.github/workflows/franky.yml`](../../.github/workflows/franky.yml) does
that on a stock `ubuntu-latest` runner: `pip install`, `curl` the test file
from the franky tag matching the installed wheel, start the sim action, run
`pytest` once per test.

## What runs

The 19 tests in the file, in this order, each with its own
`@pytest.mark.timeout`:

| test | what it drives | asserts |
| --- | --- | --- |
| `test_joint_position_control` | `JointWaypointMotion`, four 7-joint configurations | every joint within 0.03 rad |
| `test_joint_velocity_control` | `JointVelocityWaypointMotion`, four sign patterns, 400 ms each | displacement sign matches the commanded velocity on all 7 joints |
| `test_cartesian_position_control` | `CartesianMotion`, absolute targets ±x/±y/±z | end-effector within 2.5 cm |
| `test_cartesian_velocity_control` | `CartesianVelocityWaypointMotion`, ±x/±y/±z, 500 ms each | displacement sign per axis |
| `test_joint_impedance_motion` | `JointImpedanceMotion`, asynchronous, `stop()` after 1 s | joints within 0.03 rad of the target |
| `test_cartesian_impedance_motion` | `CartesianImpedanceMotion`, asynchronous, `stop()` after 1 s | end-effector within 2.5 cm |
| `test_cartesian_impedance_nullspace_posture` | `CartesianImpedanceMotion` holding the pose plus a `PostureTask`, per-joint and scalar stiffness | joints 1/3 counter-rotate along the self-motion manifold, end-effector undisturbed |
| `test_joint_impedance_tracker` | `JointImpedanceTracker`, 100 ticks at 10 ms | joints within 0.03 rad |
| `test_cartesian_impedance_tracker` | `CartesianImpedanceTracker`, 100 ticks at 10 ms | end-effector within 2.5 cm |
| `test_simple_torque_motion` | `SimpleTorqueMotion`, 2 Nm on joint 1 fed at 100 Hz, `TorqueStopMotion` | joint 1 moved > 0.05 rad in the torque's direction |
| `test_simple_torque_motion_initial_torque` | `SimpleTorqueMotion(initial_torque=…, signal_timeout=None)`, no `set_torque` | joint 1 moved > 0.05 rad |
| `test_simple_torque_motion_signal_timeout` | `SimpleTorqueMotion` never fed | `TorqueSignalTimeoutException` within 2 s |
| `test_motion_callback_fires_every_time_step` | a 5 s velocity motion with `register_callback` | 5000–6000 invocations, in order, `rel_time` deltas equal to the reported `time_step` |
| `test_gripper_homing` | `Gripper.homing()` | `max_width` 0.08 m, width unchanged |
| `test_gripper_move` | `Gripper.move()` to 0.08/0.04/0.01/0.06/0.0 m | width within 5 mm each time |
| `test_gripper_grasp_success` | `grasp(0.04, …, epsilon 0.02)` from fully open | returns `True`, `is_grasped` |
| `test_gripper_grasp_failure` | `grasp(0.09, …, epsilon 0.005)` — beyond the 0.08 m maximum | `CommandException`, not `is_grasped` |
| `test_gripper_stop` | `stop()` on an idle gripper | returns `True` |
| `test_motion_reuse_raises` | `move()` the same motion object twice, sync and as an async preemption | `MotionReuseException` both times, a fresh motion still works |

Every assertion reads the state back off the wire, so a pass means the sim
integrated the motion — not that the call returned. The callback test is the
sharpest: it requires 1 kHz `RobotState` delivery with no dropped cycle for
five seconds.

## The `franky_sim` stand-in

The test file begins

```python
from franky_sim import SimulationServer
from franky_sim.mujoco_simulator import MujocoSimulator
```

and every test wraps itself in

```python
with MujocoSimulator() as sim:
    robot = sim.add_robot()
    with SimulationServer(sim) as server:
        server.run_async()
        franky.Robot(robot.hostname, ...)
```

[`franky_sim/`](franky_sim/) here is a small package with the same two class
names that starts nothing and hands the tests a hostname — `127.0.0.1`, or
`$FRANKA_SIM_HOST`. It goes on `PYTHONPATH`; the test file is not touched. That
is the entire adapter between franky's suite and franka-sim, with one addition:

Upstream, every test gets a fresh simulator with the arm at
`[0, 0, 0, -1.57, 0, 1.57, 0.785]`. Here all 19 run against one long-lived
sim, and franka-sim holds the last pose across client sessions, so without help
each test would start where the previous one stopped. Most tests would not
care — their targets are relative, or absolute and reachable — but the
impedance tests get one second to converge on an absolute target and fail from
the pose `test_cartesian_velocity_control` leaves behind (the sim logs a 12 Nm
torque spike at the start of the motion). So `MujocoSimulator.__enter__` does
what a fresh upstream simulator would: it connects, moves the arm to that
configuration (which is also franka-sim's own initial pose), and closes its
session before the test opens its own. A no-op when the arm is already there.

## One pytest process per test

[`run-tests.sh`](run-tests.sh) collects the selected tests once and then runs
`pytest` once per test, each under its own `timeout`. The workflow and
`run-local.sh` both call it. The reason is what happens after a failure: the
failing test's `franky.Robot` stays referenced from the traceback pytest keeps
for the report, so its FCI session never closes; libfranka allows one client,
so the next test blocks in `connect()` — inside C++, where `pytest-timeout`'s
signal cannot reach it — and so does every test after that, until the job
timeout. In a single process "1 failed" turns into "1 failed, 14 hung, no
output for ten minutes". A process exit closes the socket unconditionally, so
a failure costs one test.

The cost is ~1 s of interpreter start-up per test and the reset move above.
The suite takes about a minute either way.

## franka-sim version

franka-sim ≥ 1.1.3 — `ghcr.io/barisyazici/franka-sim:1.1.3`, which is what
the workflow's `tag: latest` resolves to; all 19 pass. On 1.1.2 five fail:

- `test_joint_impedance_motion`, `test_cartesian_impedance_motion`,
  `test_cartesian_impedance_nullspace_posture`: the `StopMove` that ends an
  asynchronous impedance motion must make the interrupted Move reply
  `kPreempted`, which libfranka turns into the `ControlException` a stopped
  motion raises. 1.1.2 replied `kSuccess`, which falls through to
  `ProtocolException: Unexpected reply to a Move command`.
- `test_gripper_grasp_success`, `test_gripper_grasp_failure`: libfranka calls a
  grasp successful when the final width lands within the epsilon band around
  the *requested* width. 1.1.2 used a stall heuristic instead, so with nothing
  between the fingers `grasp(0.04, …, epsilon 0.02)` reported failure and
  `grasp(0.09, …, epsilon 0.005)` reported success, each the opposite of a
  real hand.

Both are fixed in 1.1.3. Pin `tag:` in the workflow if you need to hold a
version; anything below 1.1.3 is red on those five and green on the other 14.

## Motion-limit enforcement

The workflow runs one lane with the sim's default arguments: limit violations
are logged, nothing aborts. With `--enforce-motion-limits` 17 of 19 pass and two
fail deterministically: `test_simple_torque_motion` and
`test_simple_torque_motion_initial_torque` apply ±2 Nm to joint 1 for 1 s,
which drives it past the FR3's joint velocity limit, and the sim aborts with
`joint_velocity_violation` the way Control does. A real FR3 would reflex there
too — the test is written for a simulator that does not enforce limits. To add
an enforcement lane, deselect those two (the third `simple_torque_motion` test,
the watchdog one, passes):

```bash
examples/franky/run-local.sh --start-sim --args '--enforce-motion-limits' \
  -k 'not (simple_torque_motion and not signal_timeout)'
```

17 selected, 17 passed, 42 s, and the sim log has no abort in it — the reset
moves the stand-in makes are within limits too.

`--enforce-comm-constraints` is not worth a lane on a hosted runner: without an
RT kernel it measures the runner's jitter, not franky.

## State carried between clients

franky's own `sim_server_context()` gives every test a fresh simulator, and
`run-tests.sh` gives every test a fresh container, so no test sees the pose
the previous one left the arm in. A real robot keeps its pose between clients,
and so does a long-lived franka-sim: run the suite against one shared server
and the first torque test after the motion tests trips
`tau_J_range_violation` on joint 7 (12.35 Nm against the 12 Nm limit) —
`test_simple_torque_motion_initial_torque` applies its open-loop torque from
wherever the arm happens to be. That is the sim being faithful, not a bug;
it is also why a shared sim is the harder and more realistic setup. If you
run one server for a whole suite, start each test from a known pose
(`robot.move(JointMotion(home))`) the way you would on hardware.

## Protocol and versions

franky ships as a binary wheel with libfranka statically linked, so **the wheel
picks the protocol**, not your system:

| franky | bundled libfranka | FCI protocol | works against franka-sim |
| --- | --- | --- | --- |
| `franky-control==2.0.0` (PyPI) | 0.21.2 | v10 | yes |
| GitHub release wheels for libfranka ≥ 0.20 | 0.20.x–0.21.x | v10 | yes |
| GitHub release wheels for libfranka ≤ 0.15 | 0.9–0.15 | v9 and earlier | no |

franky publishes wheels built against several libfranka versions on its
[releases page](https://github.com/TimSchneider42/franky/releases), because the
version has to match the robot's firmware. franka-sim serves v10, so pick a
wheel built against libfranka ≥ 0.20 — which is what plain
`pip install franky-control` gives you.

The test file is fetched from the franky tag `v$FRANKY_VERSION`
(`v2.0.0` → commit `e7e63c7`), so the tests always match the wheel under test.
Nothing from franky is vendored here.

## The RT-kernel gotcha

The tests construct

```python
franky.Robot(hostname, realtime_config=franky.RealtimeConfig.Ignore, relative_dynamics_factor=0.2)
```

franky defaults to `RealtimeConfig.Enforce` and refuses to construct a `Robot`
if it cannot get `SCHED_FIFO`. GitHub-hosted runners have no `PREEMPT_RT`
kernel and no realtime priority, and neither does a typical dev box. The sim
does not need one — it is not driving motors — so `Ignore` is correct here,
and upstream already writes it that way.

## Running it locally

```bash
examples/franky/run-local.sh --start-sim
examples/franky/run-local.sh --start-sim -k gripper
examples/franky/run-local.sh -- --timeout=60      # anything after -- goes to pytest
```

The script creates `.validate/franky-venv` (reusing it if the pinned
`franky-control`, `pytest` and `pytest-timeout` are already installed), fetches
the test file into `.validate/franky-tests/` and prints the tag's commit and the
file's sha256, starts the sim, waits for `franka-sim-check` the way the action
does, runs `run-tests.sh` exactly as the workflow does, and on exit writes
`docker logs` to `.validate/franky-sim.log` before removing the container. The
pinned version comes from `FRANKY_VERSION` in the workflow, so there is one
source of truth. Without `--start-sim` it assumes a sim is already serving
`127.0.0.1:1337`.

```bash
VENV=/somewhere/else                            # virtualenv directory
FRANKA_SIM_IMAGE=franka-sim:dev                 # a locally built sim instead of :latest
FRANKY_VERSION=2.0.0                            # override the workflow's pin
```

Expected output:

```
=== fetch franky's integration tests ===
franky tag: v2.0.0
e7e63c7c7118f427ad33211500b33b81b4011bb4	refs/tags/v2.0.0
ad5853f4...  .validate/franky-tests/test_integration_franky_sim.py
19 tests
...
=== franky integration tests ===
19 tests selected
.validate/franky-tests/test_integration_franky_sim.py::test_joint_position_control PASSED [100%]
============================== 1 passed in 5.26s ===============================
...
.validate/franky-tests/test_integration_franky_sim.py::test_motion_reuse_raises PASSED [100%]
============================== 1 passed in 1.99s ===============================

19 tests: 19 passed, 0 failed, in 58s
=== franky integration tests: 0m58s ===
total: 1m01s (exit 0)
```

About a minute on a 12-core box with a warm venv and image — the longest
single tests are the null-space posture test (7 s) and the two 5–6 s torque
tests; on a hosted runner add ~20 s for the wheel and ~10 s for the sim to come
up.

## Upgrading

One line, in the workflow:

```bash
pip index versions franky-control
```

and put it in `FRANKY_VERSION`. `run-local.sh` reads it from there, and both
fetch the tests from the matching `v…` tag. Check that the new wheel still
bundles libfranka ≥ 0.20 and that the tag exists — franky tags releases
`vX.Y.Z`.
