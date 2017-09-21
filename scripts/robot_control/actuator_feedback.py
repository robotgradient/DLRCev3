"""Helps eye-ball rpc-based motor control"""

import time

from ev3control.rpc import Robot

# for convenience set the time to one second
duration = 1000
speed = 50

with Robot(None) as robot:
    for _ in range(1):
        t0 = time.time()
        print('starting position ', robot.left_track.position)
        print(time.time() - t0)
        robot.move_straight(speed, duration)
        robot.wait_until_not_moving()
        t0 = time.time()
        print("ending position ", robot.left_track.position)
        print(time.time() - t0)
