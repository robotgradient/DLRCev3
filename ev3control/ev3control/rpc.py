import rpyc
conn = rpyc.classic.connect('10.42.0.114')  # host name or IP address of the EV3
ev3 = conn.modules['ev3dev.ev3']      # import ev3dev.ev3 remotely

# These objects are on the brick!
objects = conn.modules['ev3control.objects']


class Robot(object):
    """This robot rocks!!"""

    def __init__(self):
        self.left_track = ev3.LargeMotor("outA")
        self.right_track = ev3.LargeMotor("outB")
        self.grip = objects.Grip("outC")
        self.elevator = objects.Elevator("outD")

    def move(self, vel_left, vel_right, time=4000):
        self.left_track.run_timed(speed_sp=vel_left, time_sp=time)
        self.right_track.run_timed(speed_sp=vel_right, time_sp=time)

    def move_straight(self, vel, time=4000):
        self.move(vel, vel, time)

    def rotate(self, vel, time=300):
        self.move(vel, -vel, time)

    def rotate_right(self, vel, time=300):
        self.rotate(vel, time)

    def rotate_left(self, vel, time=300):
        self.rotate(-vel, time)

    def pick_up(self):
        self.grip.close()
        self.elevator.up()

