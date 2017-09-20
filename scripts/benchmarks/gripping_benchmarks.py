"""Example of a master script with robot interface.

"""

import time
import cv2
import ev3control
from ev3control import Robot
from brain.controllers import move_to_brick_simple, \
    move_to_brick_blind_and_grip, rotation_search_brick, rotation_search_box, move_to_box_simple
from brain.core import State
from brain.core import main_loop

print("Creating robot...")
robot = Robot({
    "gearBox" : "GearBox('outA', 'outB')",
    "gripper": "RaisableGrabber('outC', 'outD')",

}, cv2.VideoCapture(1))

# Define the state graph, we can do this better, currently each method returns the next state name
states = [
    State(name="MOVE_TO_BRICK_BLIND_AND_GRIP", act=move_to_brick_blind_and_grip),
    State(name="FINAL_STATE", act=lambda x,y:None)
]
state_dict = {}
for state in states:
    state_dict[state.name] = state

start_state = states[0]
main_loop(robot, start_state, state_dict)

