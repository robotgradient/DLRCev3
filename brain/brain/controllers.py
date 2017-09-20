import numpy as np
from ev3control import Robot
from object_detection.opencv import get_lego_piece
from object_detection.opencv import get_purple_lego
import time

def move_to_brick_simple(robot, frame, img_res=(640, 480), atol=10,
                         vel_forward = 400, vel_rot = 60, vel_forward_slow=60):
    """
    Moves the robot towards the brick.

    :param robot: The robot instance
    :param img_res: The image resolution
    :param atol: The absolute error tolerance
    :return: Direction string
    """
    _, frame = robot.cap.read()


    coords, closest=get_purple_lego(frame)


    if not coords:
        return "SEARCH", frame, {}

    img_res = np.asarray(img_res)
    coords = np.asarray(coords)

    img_center = img_res / 2
    error = img_center - coords

    # Move forward till light sensor detects brick if brick is near the bottom of image
    # and centered
    if np.isclose(coords[0], img_center[0], atol=atol) and np.isclose(coords[1], img_res[1], atol=10):
        robot.move_straight(vel_forward, 500)
        return "MOVE_TO_BRICK_BLIND_AND_GRIP", frame, {}

    if np.isclose(coords[0], img_center[0], atol=atol):
        robot.move_straight(vel_forward)
        return "MOVE_TO_BRICK", frame, {}
    elif error[0] < 0:
        robot.rotate(vel=-vel_rot)
        return "MOVE_TO_BRICK", frame, {}
    else:
        # Positive velocity for turning left
        robot.rotate(vel=vel_rot)
        return "MOVE_TO_BRICK", frame, {}


def rotation_search_brick(robot, frame, vel=400):

    lego_coords, center = get_purple_lego(frame)
    if lego_coords:
        return "MOVE_TO_BRICK", frame, {}
    else:
        robot.rotate(vel)
        return "SEARCH", frame, {}

def move_to_box_simple(robot, frame, img_res=(640, 480), atol=10,
                         vel_forward = 400, vel_rot = 60, vel_forward_slow=60):
    """
        Moves the robot towards the brick.

        :param robot: The robot instance
        :param img_res: The image resolution
        :param atol: The absolute error tolerance
        :return: Direction string
        """
    _, frame = robot.cap.read()

    coords = get_white_box(frame)

    # Did not detect
    if not coords:
        return "SEARCH_BOX", frame, {}

    img_res = np.asarray(img_res)
    coords = np.asarray(coords)

    img_center = img_res / 2

    error = img_center - coords

    # Move forward till light sensor detects brick if brick is near the bottom of image
    # and centered
    if np.isclose(coords[0], img_center[0], atol=atol) and np.isclose(coords[1], img_res[1], atol=10):
        #leave the piece and go little bit backwards
        robot.elevator_down()
        time.sleep(2.5)
        robot.move_straight(vel_forward)
        time.sleep(0.3)
        robot.open_grip()
        robot.elevator_up()
        time.sleep(3)
        robot.move_straight(vel=-200)
        time.sleep(2)

        return "SEARCH", {}

    if np.isclose(coords[0], img_center[0], atol=atol):
        robot.move_straight(vel_forward)
        return "MOVE_TO_BOX", frame, {}
    elif error[0] < 0:
        robot.rotate(vel=-vel_rot)
        return "MOVE_TO_BOX", frame, {}
    else:
        # Positive velocity for turning left
        robot.rotate(vel=vel_rot)
        return "MOVE_TO_BOX", frame, {}


def rotation_search_box(robot, frame, vel=400):

    box_coords = get_white_box(frame)
    if box_coords:
        return "MOVE_TO_BOX", frame, {}
    else:
        robot.rotate(vel)
        return "SEARCH_BOX", frame, {}




def control_PID(robot, coords, img_res=(640, 480),K1=0 ,K2 = 0):

    # Compute the relative position from the image

    relObjPos = pixel_normalized_pos(coords,img_res)

    # Move to circular params
    relObjCir[0] = np.sqrt(np.pow(relObjPos(1),2) + np.pow(relObjPos(2),2));
    relObjCir[1] = np.atan(relObjPos(1)/relObjPos(2))

    # From circular params to wheels vel

    rotationMat = np.matrix([K1/2,K1/2],[K2,K2])

    velWheels = np.matmul(rotationMat,relObjCir)



def move_to_brick_blind_no_sensor(robot, frame, vel=60):

    robot.elevator_down()
    time.sleep(2.5)
    robot.move_straight(vel)
    time.sleep(0.5)
    robot.close_grip(150)
    time.sleep(3)
    robot.elevator_up()
    time.sleep(2.5)

    return "SEARCH_BOX", frame, {}

def move_to_brick_blind_and_grip(robot, frame, vel=60):
    t = 1529
    robot.move_straight(vel=300, time=t)
    time.sleep(t / 1000 + .5)
    robot.elevator_down()
    time.sleep(4)
    robot.close_grip()
    time.sleep(4)
    robot.elevator_up()
    return "END", frame, {}



