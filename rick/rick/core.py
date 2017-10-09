from collections import namedtuple
import rpyc
import cv2
from detection.opencv import draw_lines
import time
import numpy as np

State = namedtuple("State", "name act default_args")
State.__new__.__defaults__ = tuple([None] * 2) + ({},)


class RemoteDisplay:
    """Creates remotestuff"""

    def imshow(self, name, thing):
        print("here we go")
        cv2.imshow(name, np.array(thing))

    def waitKey(self, n):
        cv2.waitKey(n)

    def destroyAllWindows(self):
        cv2.destroyAllWindows()



def main_loop(robot, start_state, state_dict, delay=0.02, remote_display=None):

    print("Checking states...")
    for state in state_dict.values():
        if not isinstance(state, State):
            raise Exception("The state " + str(state) + "is not of type State.")
    state = start_state
    kwargs = state.default_args

    if remote_display is None:
        cv2display = cv2
    else:
        print("using remote")
        conn = rpyc.classic.connect(remote_display)
        _mod = conn.modules["rick.core"]
        cv2display = _mod.RemoteDisplay()

    tstart = time.time()

    while True:
        print("CURRENT_STATE",state.name)
        tend=tstart
        time.sleep(max(0, delay-( time.time()-tstart)))
        tstart = time.time()
        print("elapsed time :",tend-tstart)

        #draw_lines(frame)

        _, frame = robot.cap.read()
        next_state_name, processed_frame, kwargs = state.act(robot,frame, **kwargs)
        state = state_dict[next_state_name]
        kwargs = {**state.default_args, **kwargs}

        cv2display.imshow("frame", processed_frame)

        if cv2display.waitKey(1) & 0xFF == 27:
            break




    robot.cap.release()
    cv2display.destroyAllWindows()
