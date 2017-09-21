import time
import cv2
import ev3control
from ev3control import Robot
from brain.controllers import move_to_brick_simple, move_to_brick_blind_and_grip, \
    rotation_search_brick,euclidian_path_planning_control
from brain.core import State
from brain.core import main_loop

# Define the state graph, we can do this better, currently each method returns the next state name
states = [
    State(name="MOVE_BY_MAP", act=euclidian_path_planning_control),
]
state_dict = {}
for state in states:
    state_dict[state.name] = state

start_state = states[0]

# Robot now created inside the main loop
main_loop(cv2.VideoCapture(1), start_state, state_dict, delay=0.02)
