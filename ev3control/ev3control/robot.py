from collections import deque

from .messages import *
from .master import *

"""
    Interaface to EV3 for higher level functions.
"""

class Robot(object):


    naming_convention = [

        "leftMotor",
        "rightMotor",
        "gripper",
        "lightSensor"
    ]


    def __init__(self, device_constructors):
        """
        Constructor for the class, adds the devices automatically
        :param ports: List of ports for the devices listed in the naming conventions
        """
        self.m = start_master()
        self.devices = []

        for name in self.naming_convention:
            setattr(self, name, None)

        for device in device_constructors:
            if not device in self.naming_convention:
                raise Exception("Device " + device + " not known, please follow the naming conventions")
            setattr(self, device, device)
            print("Adding ", device , " with constructor ", device_constructors[device])
            self.publish(AddObjectMessage(device, device_constructors[device]))

        # This stores the messages published via MQTT in an attribute of this class (a deque)
        self.m.on_message = self.update_sensor_state
        self._print_messages = deque()
        # This is non-blocking! It starts listening on any topics the client is subscribed to
        self.m.start_loop()

    def update_sensor_state(self, client, userdata, msg):
        """Bad name, this just adds a message to a deque/queue."""
        # TODO: add message type checking
        message = eval(msg.payload.decode())
        self._print_messages.append(message)

    def get_light_sensor(self):
        self.publish(ShowAttrMessage(self.lightSensor, "reflected_light_intensity"))
        while True:
            if self._print_messages:
                return self._print_messages.pop()

    def publish(self, msg):
        publish_cmd(self.m, msg)



    def rotate(self, deg, vel=500, time=None):
        """
        Rotates the robot around its axis
        :param deg: Degrees
        :param vel: Velocity
        :param time:
        :return:
        """
        pass

    def rotate_forever(self, vel):
        """
        Rotates the robot forever with given velocity
        :param vel: Velocity
        :return:
        """

        print("Running ", self.leftMotor)
        self.publish(RunMethodMessage(self.leftMotor, "run_forever", {"speed_sp": vel}))
        self.publish(RunMethodMessage(self.rightMotor, "run_forever", {"speed_sp": -vel}))


    def stop_motors(self, action="brake"):

        self.publish(RunMethodMessage(self.leftMotor, "stop", {"stop_action": action}))
        self.publish(RunMethodMessage(self.rightMotor, "stop", {"stop_action": action}))

    def stop_motor(self, name, action="brake"):

        self.publish(RunMethodMessage(name, "stop", {"stop_action": action}))


    def stop_all_motors(self):

        self.stop_motors()
        self.stop_motor("gripper")

    def close_grip(self, time=4000, vel=1000):

        self.publish(RunMethodMessage(self.gripper, "run_timed", {"time_sp" : time, "speed_sp" : vel}))


    def open_grip(self, time=4000, vel=1000):

        self.publish(RunMethodMessage(self.gripper, "run_timed", {"time_sp" : time, "speed_sp" : -vel}))

    def move_forward(self, vel=600):
        """
        Move forward with given speed, top speed is default
        :param vel: Velocity
        :return:
        """
        self.publish(RunMethodMessage(self.leftMotor, "run_forever", {"speed_sp": vel}))
        self.publish(RunMethodMessage(self.rightMotor, "run_forever", {"speed_sp": vel}))



    def move_to_target(self,  vec):
        """
        Method for moving to target over a trajectory
        :param vec: Direction vector
        :return:
        """
        pass



