from pathlib import Path

import rpyc

conn = rpyc.classic.connect("ev3dev.local")
ev3 = conn.modules['ev3dev.ev3']  # import ev3dev.ev3 remotely

# These objects are on the brick!
objects = conn.modules['ev3control.objects']


class Robot:
    """This robot rocks!!"""

    def __init__(self, camera_capture, object_detector=None, tracker=None):
        self.left_track = ev3.LargeMotor("outA")
        self.right_track = ev3.LargeMotor("outB")
        self.grip = objects.Grip("outC", closed_position=-650)
        self.elevator = objects.Elevator("outD",lowered_position=650)

        self.cap = camera_capture
        self.map = []
        self.position = [0, 0, 0]
        self.object_detector = object_detector
        self.tracker = tracker

    def move(self, vel_left, vel_right, time=4000):
        self.left_track.run_timed(speed_sp=vel_left, time_sp=time)
        self.right_track.run_timed(speed_sp=vel_right, time_sp=time)

    def move_straight(self, vel, time=4000):
        self.move(vel, vel, time)

    def wait_until_not_moving(self, timeout=300):
        self.left_track.wait_until_not_moving(timeout=timeout)
        self.right_track.wait_until_not_moving(timeout=timeout)

    def rotate(self, vel, time=300):
        self.move(vel, -vel, time)

    def rotate_right(self, vel, time=300):
        self.rotate(vel, time)

    def rotate_left(self, vel, time=300):
        self.rotate(-vel, time)

    def pick_up(self):
        self.grip.close()
        self.grip.wait_until_not_moving()
        self.elevator.up()

    def reset(self):
        self.grip.open()
        self.elevator.up()
        self.left_track.stop(stop_action="brake")
        self.right_track.stop(stop_action="brake")

    def __enter__(self):
        """Needed for python <= 3.5 compatibility."""
        return self

    def __exit__(self, type, value, traceback):
        self.reset()
