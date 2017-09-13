
from .messages import *
from .master import *

"""
    Interaface to EV3 for higher level functions. 
"""

class Robot(object):


    naming_convention = [

        "leftMotor",
        "rightMotor",
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
            self.publish(AddDeviceMessage(device, device_constructors[device]))


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



