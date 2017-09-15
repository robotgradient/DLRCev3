import numpy as np
from ev3control import Robot

def act(robot, coords, img_res=(640, 480), atol=10):
    """
    Moves the robot towards the brick.

    :param robot: The robot instance
    :param coords: The brick coordinates
    :param img_res: The image resolution
    :param atol: The absolute error tolerance
    :return: Direction string
    """

    img_res = np.asarray(img_res)
    coords = np.asarray(coords)

    img_center = img_res / 2
    error = img_center - coords

    vel_forward = 400
    vel_rot = 60

    if np.isclose(coords[0], img_center[0], atol=atol):
        robot.move_forward(vel_forward)
        return "forward"
    elif error[0] > 0:
        # Positive velocity for turning left
        robot.rotate_forever(vel=vel_rot)
        return "left"
    elif error[0] < 0:
        robot.rotate_forever(vel=-vel_rot)
        return "right"

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





