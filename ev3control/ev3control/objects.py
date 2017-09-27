"""Define some custom objects on slave side."""
from ev3dev.core import LargeMotor, MediumMotor


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
        return self.position >= self.open_position - 40

    def close(self):
        if self.is_open:
            self.run_to_abs_pos(position_sp=self.closed_position)

    def open(self):
        if not self.is_open:
            self.run_to_abs_pos(position_sp=self.open_position)


class Elevator(LargeMotor):
    """Raises and lowers the grip"""

    def __init__(self, port, lowered_position=390):
        super(Elevator, self).__init__(port)
        self.reset()
        self.raised_position = 0
        self.lowered_position = lowered_position
        self.speed_sp = 100

    @property
    def is_raised(self):
        print("position", self.position)
        return self.position <= (self.raised_position + 40)

    def up(self):
        if not self.is_raised:
            self.run_to_abs_pos(position_sp=self.raised_position - 1)
            # we need to hold the elevation explicitly
            self.wait_until_not_moving()
            self.stop(stop_action="hold")

        print("test raised", self.is_raised, self.position)
        if self.is_raised:
            self.run_to_abs_pos(position_sp=self.lowered_position)
