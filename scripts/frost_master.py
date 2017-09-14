"""Example of a master script with robot interface.

"""

import time
from ev3control import Robot

print("Creating robot...")
robot = Robot({

    "leftMotor": "LargeMotor('outA')",
    "rightMotor": "LargeMotor('outB')",
    "gripper": "MediumMotor('outC')",

})


def run_robot():
    """
    PUT YOUR ROBOT COMMANDS IN THIS FUNCTION
    :return:
    """
    robot.move_forward(300)
    time.sleep(2.)
    robot.dance()


try:
    run_robot()
except Exception as e:
    print(e)
    robot.stop_all_motors()
    print("Code is wrong, turned off motors")