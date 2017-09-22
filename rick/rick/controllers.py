import numpy as np
from ev3control import Robot
from object_detection.opencv import get_lego_piece
from object_detection.opencv import get_purple_lego
import time
from .motion_control import euclidian_path_planning_control


def wait_for_brick(robot, frame, vel=400):
    lego_coords, center = get_purple_lego(frame)
    if lego_coords:
        return "MOVE_TO_BRICK_BLIND_AND_GRIP", frame, {}
    else:
        # robot.rotate_right(vel)
        return "WAIT_FOR_BRICK", frame, {}


def wait_for_brick_nn(robot,frame):
    img = frame
    kernel = np.ones((5,5),np.float32)/25
    img = cv2.filter2D(img,-1,kernel)
    res = robot.object_detector.detect_with_threshold(img,threshold=0.95, return_closest=True)
    if res:
        img_res = np.asarray([640,480])
        box, score = res
        cv2.rectangle(frame, (int(box[1]*img_res[0]), int(box[0]*img_res[1])), 
                (int(box[3]*img_res[0]), int(box[2]*img_res[1])), (0, 255, 0), thickness=3)    
        cv2.putText(frame, str(score) + "%", (int(box[1]*img_res[0]),
         int(box[0]*img_res[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=(255,0,0))

        return "MOVE_TO_BRICK_BLIND_AND_GRIP", frame, {}
    else:
        # robot.rotate_right(vel)
        return "WAIT_FOR_BRICK", frame, {}


def move_to_brick_simple(robot, frame, img_res=(640, 480), atol=10,
                         vel_forward = 100, vel_rot = 60, atol_move_blind=90):
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
    if np.isclose(coords[0], img_center[0], atol=atol_move_blind) and np.isclose(coords[1], img_res[1], atol=atol_move_blind):
        robot.move_straight(vel_forward, 500)
        return "MOVE_TO_BRICK_BLIND_AND_GRIP", frame, {}

    if np.isclose(coords[0], img_center[0], atol=atol):
        robot.move_straight(vel_forward)
        return "MOVE_TO_BRICK", frame, {}
    elif error[0] < 0:
        robot.rotate_right(vel=vel_rot)
        return "MOVE_TO_BRICK", frame, {}
    else:
        # Positive velocity for turning left
        robot.rotate_left(vel=vel_rot)
        return "MOVE_TO_BRICK", frame, {}

def euclidian_move_to_brick(robot, frame,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0):

    img_res = np.asarray((640,480))

    lego_coords, center = get_purple_lego(frame)
    if lego_coords and np.isclose(lego_coords[1], img_res[1], atol=200):
        return "MOVE_TO_BRICK", frame, {}

    brick_position = robot.map[0]
    t0 = time.time()
    print('t0 ', t0, 'TIME', TIME)
    time_diff = t0 - TIME
    print('time',time_diff)
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    estim_rob_pos, vel_wheels, new_path = euclidian_path_planning_control(robot.position,
                                                                          brick_position, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path)


    robot.position = estim_rob_pos
    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    iteration += 1

    # print("Path: ", pat, iteration)
    # print("Robot positij "ffefrobot.position)
    # print("Velocities rl: ", vel_wheels)
    # print("##" *20)


    return "MOVE_BY_MAP", frame, {"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0}


def rotation_search_brick(robot, frame, vel=60):

    lego_coords, center = get_purple_lego(frame)
    if lego_coords:
        return "MOVE_TO_BRICK", frame, {}
  #  elif len(robot.map) > 0 :
        #return "MOVE_BY_MAP", frame, {"iteration": 0, "path": []}
    else:
        robot.rotate_right(vel)
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
        robot.elevator.down()
        time.sleep(2.5)
        robot.move_straight(vel_forward)
        time.sleep(0.3)
        robot.grip.open()
        robot.elevator_up()
        time.sleep(3)
        robot.move_straight(vel=-200)
        time.sleep(2)

        return "SEARCH", {}

    if np.isclose(coords[0], img_center[0], atol=atol):
        robot.move_straight(vel_forward)
        return "MOVE_TO_BOX", frame, {}
    elif error[0] < 0:
        robot.rotate_right(vel=vel_rot)
        return "MOVE_TO_BOX", frame, {}
    else:
        # Positive velocity for turning left
        robot.rotate_left(vel=vel_rot)
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



def move_to_brick_blind_no_sensor(robot, frame, vel=60, time=500):

    robot.elevator.down()
    time.sleep(2.5)
    robot.move_straight(vel, time)
    time.sleep(0.5)
    robot.pick_up()
    time.sleep(2.5)

    return "SEARCH_BOX", frame, {}

def move_to_brick_blind_and_grip(robot, frame, vel=60):
    t = 1500
    # Make sure the grip is open
    robot.grip.open()
    # Make sure the elevator is down
    print(robot.elevator.is_raised)
    print(robot.elevator.position)
    robot.elevator.down()
    robot.elevator.wait_until_not_moving()
    robot.move_straight(vel=300, time=t)
    robot.wait_until_not_moving()
    robot.pick_up()
    return "FINAL_STATE", frame, {}



