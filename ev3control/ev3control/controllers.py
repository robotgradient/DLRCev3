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

    vel = 500

    if np.isclose(coords[0], img_center[0], atol=atol):
        robot.move_forward(vel)
        return "forward"
    elif error[0] > 0:
        robot.rotate_forever(vel=-vel)
        return "left"
    elif error[0] < 0:
        robot.rotate_forever(vel=vel)
        return "right"


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





