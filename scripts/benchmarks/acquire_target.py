from detection.opencv import get_lego_boxes
from rick.core import State
from rick.core import main_loop
from rick.async import AsyncCamera
import sys

from dlrc_one_shot_learning.similarity_detectors import EuclidianNNFeaturesBrickFinder

from collections import namedtuple


similarity_detector = EuclidianNNFeaturesBrickFinder()

class DummyRobot:
    """silly robot for testing"""
    def __init__(self, camera):
        self.cap = camera
        self.target = None


def acquire_target(robot, frame, **kwargs):
    BB_legos = get_lego_boxes(frame)

    # We wait until there's only one lego in view
    if len(BB_legos) == 1:
        print("found a brick")
        bboxes = [frame[bbox[0]:bbox[2], bbox[1]:bbox[3]] for bbox in BB_legos]
        robot.target = bounding_box_features = similarity_detector.extract_features(bboxes)[0]
        return "END", frame, {}
    else:
        print(len(BB_legos))
        return "ACQUIRE_TARGET", frame, {}

def final_state(robot, frame, **kwargs):
    print(robot.target)
    sys.exit(0)


if __name__ == '__main__':
    states = [
        State(
            name="ACQUIRE_TARGET",
            act=acquire_target,
            ),
        State(
            name="END",
            act=final_state,
         ),
    ]
    state_dict = {}
    for state in states:
        state_dict[state.name] = state

    start_state = states[0]

    main_loop(DummyRobot(AsyncCamera(0)), start_state, state_dict, delay=0)
