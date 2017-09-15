import numpy as np
from ev3control import Robot
from ev3control.object_detection.opencv_object_detection import get_lego_piece
from collections import namedtuple


def move_to_brick_simple(robot, frame, img_res=(640, 480), atol=10,
                         vel_forward = 400, vel_rot = 60, vel_forward_slow=60):
    """
    Moves the robot towards the brick.

    :param robot: The robot instance
    :param img_res: The image resolution
    :param atol: The absolute error tolerance
    :return: Direction string
    """


    coords = get_lego_piece(frame)
    img_res = np.asarray(img_res)
    coords = np.asarray(coords)

    img_center = img_res / 2
    error = img_center - coords

    # Move forward till light sensor detects brick if brick is near the bottom of image
    # and centered
    if np.isclose(coords[0], img_center[0], atol=atol) and np.isclose(coords[1], img_res[1], atol=10):
        robot.move_forward
        return "MOVE_TO_BRICK_BLIND_AND_GRIP", {"frame": frame}

    if np.isclose(coords[0], img_center[0], atol=atol):
        robot.move_forward(vel_forward)
        return "MOVE_TO_BRICK", {"frame": frame}
        # Positive velocity for turning left
        robot.rotate_forever(vel=vel_rot)
        return  "MOVE_TO_BRICK", {"frame": frame}
    elif error[0] < 0:
        robot.rotate_forever(vel=-vel_rot)
        return "MOVE_TO_BRICK", {"frame": frame}



def rotation_search(robot, frame, vel=400):

    lego_coords = get_lego_piece(frame)
    if lego_coords:
        return "MOVE_TO_BRICK", {}
    else:
        robot.rotate_forever(vel)
        return "SEARCH", {}




def move_to_brick_blind_and_grip(robot, frame, vel=60):
    if True:
        robot.stop_motors()
        robot.close_grip()
        return "SEARCH", {}
    else:
        robot.move_forward()
        return "MOVE_TO_BRICK_BLIND_AND_GRIP", {}





def control_PID(robot, coords,K1=0 ,K2 = 0, img_res=(640, 480)):

    # Compute the relative position from the image

    relObjPos = pixel_normalized_pos(coords,img_res)

    # Move to circular params(not working!!)
    relObjCir = np.zeros(2)
    #relObjCir[0] = np.sqrt(np.power(relObjPos[0],2) + np.power(relObjPos[1],2));
    #relObjCir[1] = np.arctan(relObjPos[0]/relObjPos[1])

    relObjCir[0] = relObjPos[1]
    relObjCir[1] = relObjPos[0]

    # From circular params to wheels vel

    rotationMat = np.matrix([[K1/2, K1/2], [-K2, K2]])


    velWheels = np.matmul(rotationMat.T,relObjCir).T

    print('wheels velocities',velWheels)
    robot.move(vel_left=velWheels[0,0], vel_right=velWheels[1,0])

    return "move"

def pixel_normalized_pos(coords, img_res=(640, 480)):

    img_res = np.asarray(img_res)
    coords = np.asarray(coords)

    er = np.zeros(2)
    er[0] = (coords[0] - img_res[0]/2)/img_res[0];
    er[1] = -(coords[1]-img_res[1])/img_res[1];
    print("relative normalized pos:", er)

    return er



def main_loop():

    robot = Robot({

        "leftMotor": "LargeMotor('outA')",
        "rightMotor": "LargeMotor('outB')",
        "gripper": "MediumMotor('outC')",

    })

    while True:

        # Read from camera
        # Get coordinates of brick
        coords = (0,0)
        # Issue commands to robot
        act(robot, coords)





