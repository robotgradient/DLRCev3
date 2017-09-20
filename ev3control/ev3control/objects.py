"""Define some custom objects on slave side."""
from ev3dev.core import LargeMotor


class GearBox:
    """Controls two motors"""

    def __init__(self, left_motor_port, right_motor_port):
        self.left_motor = LargeMotor(left_motor_port)
        self.right_motor = LargeMotor(right_motor_port)

    def drive(self, left_vel, right_vel, time):
        self.left_motor.run_timed(speed_sp=left_vel, time_sp=time)
        self.right_motor.run_timed(speed_sp=right_vel, time_sp=time)

    def drive_straight(self, vel, time):
        self.drive(vel, vel, time)

    def rotate(self, vel, time):
        self.drive(vel, -vel, time)
