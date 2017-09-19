"""Example of a master script with robot interface.

"""

import time
import cv2
import ev3control
from ev3control import Robot
from ev3control.controllers import move_to_brick_simple, \
    move_to_brick_blind_and_grip, rotation_search, rotation_search_box, move_to_box_simple
from ev3control import State
from ev3control import main_loop

print("Creating robot...")
robot = Robot({

    "leftMotor": "LargeMotor('outA')",
    "rightMotor": "LargeMotor('outB')",
    "gripper": "MediumMotor('outC')",
    "elevator": "LargeMotor('outD')"

}, cap=cv2.VideoCapture(0))

def stop_after_dec(controller):

    def func(robot,frame, **kwargs):
        controller(robot,frame)
        robot.stop_all_motors()
        return "FINAL_STATE",frame, {}
    return func


# Define the state graph, we can do this better, currently each method returns the next state name
states = [
    State(name="MOVE_TO_BRICK_BLIND_AND_GRIP", act=stop_after_dec(move_to_brick_blind_and_grip)),
    State(name="FINAL_STATE", act=lambda x,y:None)
]
state_dict = {}
for state in states:
    state_dict[state.name] = state

start_state = states[0]
main_loop(robot, start_state, state_dict)

