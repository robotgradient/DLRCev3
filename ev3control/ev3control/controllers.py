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

def control_PID(robot, relObjPos, K1,K2 = 0):

    # Move to circular params

    relObjCir[0] = np.sqrt(np.pow(relObjPos(1),2) + np.pow(relObjPos(2),2));
    relObjCir[1] = np.atan(relObjPos(1)/relObjPos(2))

    # From circular params to wheels vel

    rotationMat = np.matrix([K1/2,K1/2],[K2,K2])

    velWheels = np.matmul(rotationMat,relObjCir)


    robot.move

    return velWheels

def pixel_normalized_pos(coords, img_res=(640, 480), atol=10):

    img_res = np.asarray(img_res)
    coords = np.asarray(coords)









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





