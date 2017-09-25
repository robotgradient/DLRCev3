import time
import cv2
from ev3control.rpc import Robot
from nn_object_detection.object_detectors import NNObjectDetector
from rick.controllers import *
from rick.core import State
from rick.core import main_loop

print("Creating robot...")

print("Creating robot...")
PATH_TO_CKPT = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/models/ssd_mobilenet_v1_lego/train/frozen_inference_graph.pb"
PATH_TO_LABELS = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/label_map.pbtxt"
NUM_CLASSES = 2

detector = NNObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

with Robot(cap, detector) as robot:
    robot.map = [(60,-90)]
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
            },
        State(
             name="MOVE_TO_BRICK",
             act=move_to_brick_nn_v1,
             default_args={"atol": 2,
                           "atol_move_blind" : 30,
                           "vel_rot" : 30,
                           "vel_forward" : 200
                           }
         ),
        State(
             name="SEARCH",
             act=rotation_search_brick
         ),
        State(
            name="MOVE_TO_BRICK_BLIND_AND_GRIP",
            act=move_to_brick_blind_and_grip,
            default_args={"vel" : 400,
                          "t" : 1300,
                            }
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
