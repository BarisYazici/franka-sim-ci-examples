# franky quick-start on franka-sim

[`franky`](https://github.com/TimSchneider42/franky) is a Python/C++ wrapper
around libfranka: a [Ruckig](https://github.com/pantor/ruckig) online planner in
front of the real 1 kHz control loop, with a Python API thin enough that its
README is six lines of motion code. Every one of those lines starts the same
way:

```python
robot = Robot("172.16.0.2")
```

which is why none of franky's examples appear in its CI — they need an FR3.
[franka-sim](https://github.com/BarisYazici/libfranka-sim) serves the same FCI
wire protocol, so the entire change is the address:

```python
robot = Robot("127.0.0.1", realtime_config=RealtimeConfig.Ignore)
```

[`../../.github/workflows/franky.yml`](../../.github/workflows/franky.yml) runs
that on a stock `ubuntu-latest` runner. There is no Docker build and no
workspace: `pip install franky-control`, start the sim action, run two scripts.

## What runs

### [`smoke.py`](smoke.py) — the quick-start, asserted

Connect, read `RobotState`, then the three things a franky user does first:

| step | assertion |
| --- | --- |
| `JointMotion` — +0.2 rad on joint 1 | reached within 0.02 rad |
| `CartesianMotion(..., ReferenceType.Relative)` — 5 cm | the resulting `O_T_EE` moved 5 cm ±5 mm |
| `Gripper.open()` / `Gripper.move(0.02, 0.05)` | `gripper.width` within 5 mm of the target |

Each assertion reads the state back off the wire, so a pass means the sim
actually integrated the motion — not that the call returned.

### [`reflex.py`](reflex.py) — the adversarial lane

Only run against a sim started with `--enforce-motion-limits`. It commands
`JointMotion` with joint 1 at 3.0 rad, outside the FR3's ±2.7437 rad range, and
asserts that libfranka raises the reflex:

```
libfranka: Move command aborted: motion aborted by reflex!
["joint_velocity_violation", "joint_motion_generator_velocity_limits_violation"]
```

Then `robot.recover_from_errors()`, and a normal move back to the start —
proving the recovery path works, not just the failure.

The *velocity* reflex is what fires, and that is the interesting part. The FR3's
per-joint velocity limit is a function of position: it collapses to zero as the
joint approaches its travel limit. franky's planner uses the constant limit, so
its cruise speed becomes illegal at about 2.706 rad — the robot is refused ~0.04
rad before it could ever reach the position limit. That is the real robot's
behaviour, and no amount of reading franky's source would have predicted it.

Sometimes only `joint_velocity_violation` is reported and sometimes the
commanded-rate reflex `joint_motion_generator_velocity_limits_violation` too,
depending on which crosses first in that millisecond; the script asserts on the
former, which is in every abort.

## Two lanes

| lane | sim args | what it proves |
|---|---|---|
| `nominal` | none | The quick-start drives the simulated robot end to end. The sim log has zero warnings. |
| `motion-limits` | `--enforce-motion-limits` | The same, plus a real reflex abort and a real recovery. The sim log has exactly one abort — the one `reflex.py` provokes. |

`reflex.py` is skipped in the `nominal` lane rather than made conditional:
nothing on the wire tells a client whether the sim is enforcing limits, so the
lane is the switch.

`--enforce-comm-constraints` is deliberately not in the matrix. Without an RT
kernel a hosted runner drops command cycles on its own, which measures the
runner rather than franky.

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
`pip install franky-control` gives you. franky exposes no version accessor of
its own, so `smoke.py` prints the distribution version instead.

## The RT-kernel gotcha

The one line that is not the upstream quick-start:

```python
Robot(host, realtime_config=RealtimeConfig.Ignore)
```

franky defaults to `RealtimeConfig.Enforce` and refuses to construct a `Robot`
if it cannot get `SCHED_FIFO`. GitHub-hosted runners have no `PREEMPT_RT`
kernel and no realtime priority, and neither does a typical dev box. The sim
does not need one — it is not driving motors — so `Ignore` is correct here. On
a real FR3, leave the default.

## Running it locally

```bash
examples/franky/run-local.sh --start-sim
examples/franky/run-local.sh --start-sim --args '--enforce-motion-limits' --reflex
```

The script creates `.validate/franky-venv` (reusing it if the pinned
`franky-control` is already installed), starts the sim, waits for
`franka-sim-check` the way the action does, runs the scripts, and on exit writes
`docker logs` to `.validate/franky-sim.log` before removing the container. The
pinned version comes from `FRANKY_VERSION` in the workflow, so there is one
source of truth.

```bash
VENV=/somewhere/else                            # virtualenv directory
FRANKA_SIM_IMAGE=ghcr.io/barisyazici/franka-sim:1.1.2
FRANKY_VERSION=2.0.0                            # override the workflow's pin
```

Expected output, `motion-limits` lane:

```
=== smoke.py ===
franky-control 2.0.0
connected: mode = RobotMode.Idle q = [-0.0, -0.0, 0.0, -1.57, -0.0, 1.57, 0.785]
joint move: joint1 = 0.2006 (target 0.2000, err 0.0006 rad)
cartesian move: |d| = 0.0502 m (target 0.0500), dz = +0.0502 m
gripper: width = 0.0200 m after move(0.02)
SMOKE OK
=== smoke.py: 0m02s ===

=== reflex.py ===
normal move: ok, joint1 = -0.0013
reflex: libfranka: Move command aborted: motion aborted by reflex! ["joint_velocity_violation", "joint_motion_generator_velocity_limits_violation"]
aborted at joint1 = 2.7060 rad (limit 2.7437)
after recovery: back home, joint1 = -0.0012 (err 0.0006 rad)
REFLEX OK
=== reflex.py: 0m14s ===
```

Wall time on a 12-core box with a warm venv and image: 3 s for the `nominal`
lane, 17 s for `motion-limits` — 14 s of which is the provoked motion driving
joint 1 across its whole range at `relative_dynamics_factor = 0.2`. On a hosted
runner add ~20 s for the wheel and ~10 s for the sim to come up.

## Upgrading

One line, in the workflow:

```bash
pip index versions franky-control
```

and put it in `FRANKY_VERSION`. `run-local.sh` reads it from there. Check that
the new wheel still bundles libfranka ≥ 0.20.
