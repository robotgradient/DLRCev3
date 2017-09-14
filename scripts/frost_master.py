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


<<<<<<< HEAD
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
=======
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
>>>>>>> acb86190241e3deaa6629aabc9836ca7d3e1e0a5