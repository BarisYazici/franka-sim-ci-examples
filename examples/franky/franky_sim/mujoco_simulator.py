"""See ``franky_sim/__init__.py``: no simulator is started here."""

from . import _Robot, reset_arm


class MujocoSimulator:
    def __enter__(self):
        # What a fresh upstream simulator would give the test: the arm at its
        # initial configuration.
        reset_arm()
        return self

    def __exit__(self, *exc):
        return False

    def add_robot(self, *args, **kwargs):
        return _Robot()
