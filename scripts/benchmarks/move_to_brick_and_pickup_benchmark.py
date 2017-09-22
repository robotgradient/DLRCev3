import time

import cv2
from ev3control.rpc import Robot
from brain.controllers import move_to_brick_blind_and_grip, wait_for_brick
from brain.core import State
from brain.core import main_loop

print("Creating robot...")

with Robot(cv2.VideoCapture(1)) as robot:
    states = [
        State(
            name="WAIT_FOR_BRICK",
            act=wait_for_brick,
            default_args={}),
        # State(
        #     name="MOVE_TO_BRICK",
        #     act=move_to_brick_simple,
        #     default_args={"atol": 30,
        #                   "atol_move_blind" : 30
        #                   }
        # ),
        State(
            name="MOVE_TO_BRICK_BLIND_AND_GRIP",
            act=move_to_brick_blind_and_grip,
            default_args={}
        ),
        State(
            name="FINAL_STATE",
            act=lambda robot, frame, **args: time.sleep(.5)
        )
    ]
    print(states[0])
    state_dict = {}
    for state in states:
        state_dict[state.name] = state

    start_state = states[0]

    main_loop(robot, start_state, state_dict, delay=0.03)
