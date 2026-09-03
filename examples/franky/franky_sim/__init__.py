"""Stand-in for franky's ``franky_sim`` package, so that franky's own
``tests/pytest/test_integration_franky_sim.py`` runs unmodified.

Upstream, every test does::

    with MujocoSimulator() as sim:
        robot = sim.add_robot()
        with SimulationServer(sim) as server:
            server.run_async()
            franky.Robot(robot.hostname)

i.e. it starts franky's own MuJoCo simulator and FCI server in-process, with
the arm at its initial configuration, and connects to that. Here a franka-sim
container is already serving the FCI on 127.0.0.1:1337 (and the gripper on
1338), so these two classes start nothing and hand the tests the hostname to
connect to. ``FRANKA_SIM_HOST`` overrides it.

The one thing a long-lived sim does not give each test is a fresh arm: franka-sim
holds the last pose across client sessions, and several tests assume the
upstream initial configuration (the impedance tests only get one second to
converge on an absolute target). ``MujocoSimulator.__enter__`` therefore moves
the arm back to that configuration -- the same one franka-sim starts at -- and
closes its session before the test opens its own.
"""

import os

HOSTNAME = os.environ.get("FRANKA_SIM_HOST", "127.0.0.1")

# Upstream franky_sim's and franka-sim's initial arm configuration.
INITIAL_Q = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.785]


def reset_arm():
    import franky

    robot = franky.Robot(
        HOSTNAME,
        realtime_config=franky.RealtimeConfig.Ignore,
        relative_dynamics_factor=0.2,
    )
    robot.move(franky.JointWaypointMotion([franky.JointWaypoint(INITIAL_Q)]))
    # libfranka allows one client; dropping the Robot closes the FCI session.
    del robot


class _Robot:
    hostname = HOSTNAME


class SimulationServer:
    def __init__(self, simulator):
        self.simulator = simulator
        self.hostname = HOSTNAME

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def run_async(self):
        pass
