import time
from ev3control import Robot
import sys

if len(sys.argv)>2:
	time2=int(sys.argv[2])
	vel=int(sys.argv[1])
else:
	vel=-200
	time2=2000

print("Creating robot...")
robot = Robot({

    "leftMotor": "LargeMotor('outD')",
    "rightMotor": "LargeMotor('outB')",
    "gripper": "LargeMotor('outA')",

}, None)



#robot.publish(RunMethodMessage("gripper", "run_timed", {"speed_sp" : -400, "time_sp" : 1000}))
#robot.move_forward()
#time.sleep(0.1)
#robot.stop_motors()

#robot.close_grip(vel=900, time=4500)
#time.sleep(5)


robot.open_grip(vel=vel, time=time2)
time.sleep(3)

#robot.close_grip(vel=300, time=1500)
#time.sleep(3)
#robot.move_forward()
#time.sleep(1)
#robot.rotate_forever(1000)
#time.sleep(1)