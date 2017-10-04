
import time
from ev3control.rpc import Robot
from rick.controllers import *
from rick.core import State
from rick.core import main_loop

from rick.utils import *
from rick.async import *
from nn_object_detection.object_detectors import NNObjectDetector
import cv2 


cap = AsyncCamera(1)
print("Creating robot...")
PATH_TO_CKPT = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/models/ssd_mobilenet_v1_lego/train/frozen_inference_graph.pb"
PATH_TO_LABELS = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/label_map.pbtxt"
NUM_CLASSES = 2

detector = NNObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS)
tracker = TrackerWrapper(cv2.TrackerKCF_create)

with Robot(cap, detector, tracker) as robot:
    robot.map = [(60,-90)]
    robot.sampling_rate = 0.1
    states = [
        State(
             name="MOVE_TO_BRICK",
             act=move_to_brick_nn_v2,
             default_args={"atol": 2,
                           "atol_move_blind" : 30,
                           "vel_rot" : 400,
                           "vel_forward" : 200
                           }
         ),
        State(
             name="SEARCH",
             act=rotation_search_brick_nn
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
