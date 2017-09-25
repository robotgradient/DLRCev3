from ev3control.rpc import Robot
from nn_object_detection.object_detectors import NNObjectDetector
import cv2
from brain.controllers import move_to_brick_blind_and_grip, wait_for_brick
from brain.core import State
from brain.core import main_loop
import time

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/models/ssd_mobilenet_v1_lego/train/frozen_inference_graph.pb"
PATH_TO_LABELS = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/label_map.pbtxt"
NUM_CLASSES = 2

detector = NNObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS)



with Robot(cv2.VideoCapture(0), object_detector=detector) as robot:
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
