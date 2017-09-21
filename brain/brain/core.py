from collections import namedtuple
import cv2
from object_detection.opencv import draw_lines
import time

from ev3control.rpc import Robot

State = namedtuple("State", "name act default_args")
State.__new__.__defaults__ = tuple([None] * 3)

def main_loop(camera, start_state, state_dict, delay=0.02):

    print("Checking states...")
    for state in state_dict.values():
        if not isinstance(state, State):
            raise Exception("The state " + str(state) + "is not of type State.")
    state = start_state
    kwargs = {}
    print("Creating Robot")
    with Robot(camera) as robot:

        while True:
            time.sleep(delay)
            _, frame = robot.cap.read()
            draw_lines(frame)
            print("Current state: ", state.name, " state args: ", str(kwargs))
            next_state_name, processed_frame, kwargs = state.act(robot,frame, **kwargs)
            state = state_dict[next_state_name]

            if state.default_args:
                kwargs.update(state.default_args)

            cv2.imshow("frame", processed_frame)
            if cv2.waitKey(1) & 0xFF == 27:
                print("Stopping motors...")
                robot.stop_all_motors()
                break

        robot.cap.release()
        cv2.destroyAllWindows()
