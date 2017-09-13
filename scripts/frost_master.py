"""Example of a master script with robot interface.

"""

import time
from ev3control import Robot

print("Creating robot...")
robot = Robot({

    "leftMotor": "LargeMotor('outA')",
    "rightMotor": "LargeMotor('outB')",

})

robot.rotate_forever(600)
time.sleep(1)
robot.rotate_forever(-600)
time.sleep(1)
robot.move_forward(600)
time.sleep(1)
robot.stop_motors()