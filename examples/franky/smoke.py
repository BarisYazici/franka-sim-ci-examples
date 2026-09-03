#!/usr/bin/env python3
"""franky quick-start against franka-sim: connect, read, move, grip.

The same calls a real-robot quick-start makes -- pointed at 127.0.0.1 instead
of 172.16.0.2. Nothing here is sim-aware.
"""
import argparse
from importlib.metadata import version

import numpy as np
from franky import (Affine, CartesianMotion, Gripper, JointMotion,
                    RealtimeConfig, ReferenceType, Robot)

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--host", default="127.0.0.1", help="FCI address")
args = parser.parse_args()

# franky exposes no libfranka/protocol version of its own; the wheel's bundled
# libfranka is what has to speak FCI v10 (see README).
print("franky-control", version("franky-control"))

# Neither a hosted runner nor a typical dev box has a PREEMPT_RT kernel, and
# the sim does not need one. On a real FR3 you would leave this at Enforce.
robot = Robot(args.host, realtime_config=RealtimeConfig.Ignore)
robot.relative_dynamics_factor = 0.2

state = robot.state
print("connected: mode =", state.robot_mode,
      "q =", [round(float(x), 3) for x in state.q])

# --- joint space: +0.2 rad on joint 1, then back ----------------------------
q_home = [float(x) for x in state.q]
q_target = list(q_home)
q_target[0] += 0.2
robot.move(JointMotion(q_target))
q_reached = float(robot.state.q[0])
print("joint move: joint1 = %.4f (target %.4f, err %.4f rad)"
      % (q_reached, q_target[0], abs(q_reached - q_target[0])))
assert abs(q_reached - q_target[0]) < 0.02, q_reached
robot.move(JointMotion(q_home))

# --- cartesian: 5 cm along the approach axis, checked against O_T_EE --------
# ReferenceType.Relative offsets in the end-effector frame, whose -Z is the
# approach direction; the check is on the base-frame O_T_EE it produces.
p_before = np.asarray(robot.current_pose.end_effector_pose.translation)
robot.move(CartesianMotion(Affine([0.0, 0.0, -0.05]), ReferenceType.Relative))
delta = np.asarray(robot.state.O_T_EE.translation) - p_before
print("cartesian move: |d| = %.4f m (target 0.0500), dz = %+.4f m"
      % (np.linalg.norm(delta), delta[2]))
assert abs(np.linalg.norm(delta) - 0.05) < 0.005, delta
robot.move(CartesianMotion(Affine([0.0, 0.0, 0.05]), ReferenceType.Relative))

# --- gripper (its own TCP server, port 1338) --------------------------------
gripper = Gripper(args.host)
gripper.open(0.05)
gripper.move(0.02, 0.05)
print("gripper: width = %.4f m after move(0.02)" % gripper.width)
assert abs(gripper.width - 0.02) < 0.005, gripper.width

print("SMOKE OK")
