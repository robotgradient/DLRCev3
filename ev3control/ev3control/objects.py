"""Define some custom objects on slave side."""
from ev3dev.core import LargeMotor, MediumMotor


class GearBox:
    """Controls two motors responsible for driving."""

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

    def rotate_right(self, vel, time=1000):
        self.rotate(vel, time)

    def rotate_left(self, vel, time=1000):
        self.rotate(-vel, time)

    def stop(self):
        self.left_motor.stop(stop_action="brake")
        self.right_motor.stop(stop_action="brake")


class Grip(MediumMotor):
    """Controls the motor responsible for driving."""

    def __init__(self, port, closed_position=-550):
        super(Grip, self).__init__(port)
        self.reset()
        self.open_position = 0
        self.closed_position = closed_position
        self.speed_sp = 200

    @property
    def is_open(self):
        return self.position >= self.open_position

    def close(self):
        if self.is_open:
            self.run_to_abs_pos(position_sp=self.closed_position)

    def open(self):
        if not self.is_open:
            self.run_to_abs_pos(position_sp=self.open_position)


class Elevator(LargeMotor):
    """Raises and lowers the grip"""

    def __init__(self, port, lowered_position=400):
        super(Elevator, self).__init__(port)
        self.reset()
        self.raised_position = 0
        self.lowered_position = lowered_position
        self.speed_sp = 150

    @property
    def is_raised(self):
        return self.position <= self.raised_position

    def up(self):
        if not self.is_raised:
            self.run_to_abs_pos(position_sp=self.raised_position - 1)
            # we need to hold the elevation explicitly
            self.wait_until_not_moving()
            self.stop(stop_action="hold")

    def down(self):
        if self.is_raised:
            self.run_to_abs_pos(position_sp=self.lowered_position)


class RaisableGrabber:
    """Controls the elevator and the grip.

    Makes the following assumptions about the construction:
    - two motors, large one for elevator, medium for gripper
    - the elevator has two states, Raised and Lowered
    - the grip has two states, Open and Closed
    - the system starts in with elevator raised and grip open
    """

    def __init__(self, grip_port, elevator_port):
        self.grip = Grip(grip_port)
        self.elevator = Elevator(elevator_port)

    def pick_up(self):
        self.elevator.down()
        self.elevator.wait_until_not_moving()
        self.grip.close()
        self.grip.wait_until_not_moving()
        self.elevator.up()

    def put_down(self):
        self.elevator.down()
        self.elevator.wait_until_not_moving()
        self.grip.open()
        self.grip.wait_until_not_moving()
        self.elevator.up()

    def drop(self):
        self.grip.open()

    def reset(self):
        self.grip.open()
        self.elevator.up()


if __name__ == '__main__':
    rg = RaisableGrabber("outC", "outD")
    for _ in range(3):
        rg.pick_up()
        rg.drop()
