"""Example of a master script with robot interface.

"""

import time
from ev3control import Robot
from ev3control.messages import RunMethodMessage

print("Creating robot...")
robot = Robot({

    "leftMotor": "LargeMotor('outA')",
    "rightMotor": "LargeMotor('outB')",
    "gripper": "MediumMotor('outC')",

})

robot.publish(RunMethodMessage("gripper", "run_timed", {"speed_sp" : -1000, "time_sp" : 2000}))
