import time
import cv2
from ev3control.rpc import Robot
from rick.controllers import euclidian_move_to_brick, rotation_search_brick,move_to_brick_simple, move_to_brick_blind_and_grip
from rick.core import State
from rick.core import main_loop

print("Creating robot...")

with Robot(cv2.VideoCapture(1)) as robot:
    robot.map = [(0, 110)]
    robot.sampling_rate = 0.1
    print("These are the robot motor positions before planning:", robot.left_track.position, robot.right_track.position)
    # Define the state graph, we can do this better, currently each method
    # returns the next state name
    states = [
        State(
            name="MOVE_BY_MAP",
            act=euclidian_move_to_brick,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time()
            }
            ),
        State(
             name="MOVE_TO_BRICK",
             act=move_to_brick_simple,
             default_args={"atol": 30,
                           "atol_move_blind" : 30,
                           }
         ),
        State(
             name="SEARCH",
             act=rotation_search_brick
         ),
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

    main_loop(robot, start_state, state_dict, delay=0.1)
