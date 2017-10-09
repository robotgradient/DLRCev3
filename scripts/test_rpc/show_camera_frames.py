from rick.core import State
from rick.core import main_loop
from rick.async import AsyncCamera

import cv2

import time


class DummyRobot:
    """silly robot for testing"""

    def __init__(self, camera):
        self.cap = camera
        self.target = None


def final_state(robot, frame, **kwargs):
    sys.exit(0)


def stream_video(robot, frame, iteration=0):
    if iteration < 500:
        iteration += 1
        return ("STREAM_VIDEO", frame, {"iteration": iteration})
    return ("END", frame, {})


if __name__ == '__main__':
    # time.sleep(30)
    # print("Done sleeping, starting streaming.")
    states = [
        State(
            name="STREAM_VIDEO",
            act=stream_video,
            default_args={"iteration": 0}
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

    main_loop(DummyRobot(cv2.VideoCapture(0)), start_state, state_dict, delay=0,
        remote_display="192.168.0.101"
        )
