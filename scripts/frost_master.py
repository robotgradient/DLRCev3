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


#robot.publish(RunMethodMessage("gripper", "run_timed", {"speed_sp" : -400, "time_sp" : 1000}))
robot.move_forward()
time.sleep(0.1)
robot.stop_motors()

robot.close_grip(vel=300, time=1500)
time.sleep(3)

robot.rotate_forever(1000)
time.sleep(1)
robot.stop_motors()

robot.open_grip(vel=300, time=1500)

'''robot.close_grip(vel=300, time=1500)
time.sleep(3)
#robot.move_forward()
#time.sleep(1)
robot.rotate_forever(1000)
time.sleep(1)
robot.stop_motors()'''
