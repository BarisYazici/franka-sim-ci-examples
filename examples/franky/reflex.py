#!/usr/bin/env python3
"""franky against a franka-sim started with --enforce-motion-limits.

Commands a motion a real FR3 would refuse and asserts that libfranka raises
the matching reflex, then recovers and proves the robot still moves. A
permissive sim only logs it, so the workflow runs this in one lane only.
"""
import argparse

from franky import ControlException, JointMotion, RealtimeConfig, Robot

# FR3 joint 1 travels +/- 2.7437 rad. Commanding 3.0 is out of range, and the
# joint's velocity envelope shrinks to zero as it approaches the limit, so the
# planner's cruise speed becomes illegal before the position ever could be.
# The abort names joint_velocity_violation, sometimes with the commanded-rate
# reflex joint_motion_generator_velocity_limits_violation alongside it.
JOINT1_TARGET = 3.0
EXPECTED = "joint_velocity_violation"

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--host", default="127.0.0.1", help="FCI address")
args = parser.parse_args()

robot = Robot(args.host, realtime_config=RealtimeConfig.Ignore)
robot.relative_dynamics_factor = 0.2

# --- 1. a normal move first, so the reflex is not the connection's fault ----
q_home = [float(x) for x in robot.state.q]
q_nudge = list(q_home)
q_nudge[0] += 0.2
robot.move(JointMotion(q_nudge))
robot.move(JointMotion(q_home))
print("normal move: ok, joint1 = %.4f" % robot.state.q[0])

# --- 2. the provocation -----------------------------------------------------
q_bad = list(q_home)
q_bad[0] = JOINT1_TARGET
try:
    robot.move(JointMotion(q_bad))
except ControlException as exc:
    message = str(exc)
else:
    raise AssertionError("motion completed; sim without --enforce-motion-limits?")

print("reflex: %s" % message.splitlines()[0])
assert "motion aborted by reflex" in message, message
assert EXPECTED in message, "expected %s in: %s" % (EXPECTED, message)
print("aborted at joint1 = %.4f rad (limit 2.7437)" % robot.state.q[0])

# --- 3. recover, and prove the robot is usable again ------------------------
assert robot.has_errors, "robot should be in an error state after the reflex"
assert robot.recover_from_errors(), "recover_from_errors() failed"
assert not robot.has_errors, "errors survived recover_from_errors()"
robot.move(JointMotion(q_home))
err = abs(float(robot.state.q[0]) - q_home[0])
print("after recovery: back home, joint1 = %.4f (err %.4f rad)"
      % (robot.state.q[0], err))
assert err < 0.02, err

print("REFLEX OK")
