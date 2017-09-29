
from rick.motion_control import euclidian_path_planning_control
from detection.opencv import get_lego_piece
from detection.opencv import get_purple_lego, get_lego_boxes, detect_purple
import time
from .motion_control import euclidian_path_planning_control, kalman_filter
from .utils import *
import cv2
import numpy as np

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


def move_to_brick_simple(robot, frame, img_res=(640, 480), atol=5,
                         vel_forward = 100, vel_rot = 50, atol_move_blind=90, fail_counter=0):
    """
    Moves the robot towards the brick.

    :param robot: The robot instance
    :param img_res: The image resolution
    :param atol: The absolute error tolerance
    :return: Direction string
    """
    _, frame = robot.cap.read()

    coords, center = get_lego_piece(frame)

    if not coords and fail_counter==5:
        return "SEARCH", frame, {}
    elif not coords:
        return "MOVE_TO_BRICK", frame, {"fail_counter" : fail_counter+1}


    img_res = np.asarray(img_res)
    img_center = img_res / 2
    error = img_center - coords
    print("Coords: ", coords)
    # Move forward till light sensor detects brick if brick is near the bottom of image
    # and centered
    if np.isclose(coords[0], img_center[0], atol=atol) and np.isclose(coords[1], img_res[1], atol=atol_move_blind):
        robot.move_straight(vel_forward, 500)
        return "MOVE_TO_BRICK_BLIND_AND_GRIP", frame, {}

    if np.isclose(coords[0], img_center[0], atol=atol):
        print("Move straight")
        robot.move_straight(vel_forward)
        return "MOVE_TO_BRICK", frame, {}
    elif error[0] < 0:
        print("Move left")
        robot.rotate_left(vel=vel_rot)
        return "MOVE_TO_BRICK", frame, {}
    else:
        # Positive velocity for turning left
        print("Move left")
        robot.rotate_right(vel=vel_rot)
        return "MOVE_TO_BRICK", frame, {}

import cv2
import time

def search_v3(robot, frame, vel_rot):
    res = robot.object_detector.detect_with_threshold(frame,threshold=0.9, return_closest=True)

    if len(res) == 0:
        return "SEARCH", frame, {}

    else:
        boxes = map(lambda x: x[0], res)
        #find the purple box

        box = None
        if box:
            robot.reset()
            robot.tracker.init(frame, box)
            tracker = tracker
            return "MOVE_TO_BRICK", frame, {"tracker" : tracker}
        else:
            robot.rotate_right(vel=vel_rot)
            return "SEARCH", frame, {}

    

def move_to_brick_v3_pid(robot, frame, img_res=np.asarray((640, 480)), atol=5,
                         vel_forward = 299, vel_rot = 50, atol_move_blind=90, 
                         fail_counter=0, center_position_error = 10, tracker=None, m=None, trajectory=None):

    ok, bbox = tracker.update(frame)

    vel = np.asarray(vel_forward)
    K1, K2 = 1., 1./280

    if not ok:
        BB_legos=get_lego_boxes(frame)
        # res = robot.object_detector.detect_with_threshold(frame,threshold=0.9, return_closest=False)
        # BB_legos = map(lambda x: x[0], res)
        BB_target = detect_purple(frame,BB_legos)
        if len(BB_target) == 0:
            return "SEARCH_TARGET", frame, {}
        tracker.init(frame, BB_target[0])        
        bbox = BB_target[0]

    coords = bbox_center(*bbox)
    img_center = img_res / 2 - center_position_error 
    #img_center[0] = 285
    error = img_center - coords
    atol = 10 + coords[1]/480 * 40


    vel = vel/np.abs(coords[1]/400) + np.asarray([vel_rot, -vel_rot]) * error


    print("Errror:", error, "Coords ", coords, " ok ", ok)
    frame = plot_bbox(frame,bbox, 0, (255,0,0))
    img_center = img_res/2.

    if np.isclose(coords[0], img_center[0], atol=atol) and np.isclose(coords[1], img_res[1], atol=atol_move_blind):
        robot.reset()
        return "MOVE_TO_BRICK_BLIND_AND_GRIP", frame, {}

    else:
        robot.left_track.run_forever(speed_sp = vel[0])
        robot.right_track.run_forever(speed_sp = vel[1])
        return "GO_TO_TARGET", frame, {"tracker" : tracker}


def move_to_brick_v3(robot, frame, img_res=np.asarray((640, 480)), atol=5,
                         vel_forward = 299, vel_rot = 50, atol_move_blind=90, 
                         fail_counter=0, center_position_error = 10, tracker=None, m=None, trajectory=None):

    ok, bbox = tracker.update(frame)

    if not ok:
        BB_legos=get_lego_boxes(frame)
        # res = robot.object_detector.detect_with_threshold(frame,threshold=0.9, return_closest=False)
        # BB_legos = map(lambda x: x[0], res)
        BB_target = detect_purple(frame,BB_legos)
        if len(BB_target) == 0:
            return "SEARCH_TARGET", frame, {}
        tracker.init(frame, BB_target[0])        
        bbox = BB_target[0]

    coords = bbox_center(*bbox)
    img_center = img_res / 2 - center_position_error 
    #img_center[0] = 285
    error = img_center - coords
    atol = 10 + coords[1]/480 * 40

    print("Errror:", error, "Coords ", coords, " ok ", ok)
    frame = plot_bbox(frame,bbox, 0, (255,0,0))
    img_center = img_res/2.

    if np.isclose(coords[0], img_center[0], atol=atol) and np.isclose(coords[1], img_res[1], atol=atol_move_blind):
        robot.move_straight(vel_forward, 500)
        return "MOVE_TO_BRICK_BLIND_AND_GRIP", frame, {}

    if np.isclose(coords[0], img_center[0], atol=atol):
        print("Move straight")
        robot.move_straight(vel_forward)
        return "GO_TO_TARGET", frame, {"tracker" : tracker}
    elif error[0] < 0:
        robot.rotate_left(vel=vel_rot)
        return "GO_TO_TARGET", frame, {"tracker" : tracker}
    else:
        # Positive velocity for turning left
        robot.rotate_right(vel=vel_rot)
        return "GO_TO_TARGET", frame, {"tracker" : tracker}


def move_to_brick_nn_v2(robot, frame, img_res=(640, 480), atol=5,
                         vel_forward = 299, vel_rot = 50, atol_move_blind=90, 
                         fail_counter=0, center_position_error = 10, tracking=False):
    
    img_res = np.array(img_res)
    ts = time.time()
    res = robot.object_detector.detect_with_threshold(frame,threshold=0.9, return_closest=True)
    tdet = time.time() - ts
    detected = len(res) > 0
    score = 0
    if not tracking and not detected:
        return "SEARCH", frame, {}
    elif detected:
        color = (0,0,255)
        box, score = res[0]
        dbox = np.asarray((int(box[1]*img_res[0]), int(box[0]*img_res[1]), int(box[3]*img_res[0]), int(box[2]*img_res[1])))
        if tracking:
            ok, tbox = robot.tracker.update(frame)
            if bboxes_are_overlapping(tbox,dbox) or tbox[0]==0:
                robot.tracker.init(frame, dbox)
                bbox = dbox
            else:
                color = (255,0,0)
                bbox = tbox
        else:
            robot.tracker.init(frame, dbox)
            bbox = dbox
    elif tracking:
        color = (255,0,0)
        ok, bbox = robot.tracker.update(frame)
        if not ok:
            return "SEARCH", frame, {}
    
    tif = time.time() - tdet


    print("Time deteection: ", tdet, "Time if statement: ", tif)


    coords = bbox_center(*bbox)
    img_center = img_res / 2 - center_position_error 
    error = img_center - coords

    plot_bbox(frame,bbox, score, color)

    img_center = img_res/2.

    if np.isclose(coords[0], img_center[0], atol=atol) and np.isclose(coords[1], img_res[1], atol=atol_move_blind):
        robot.move_straight(vel_forward, 500)
        return "MOVE_TO_BRICK_BLIND_AND_GRIP", frame, {}

    if np.isclose(coords[0], img_center[0], atol=atol):
        print("Move straight")
        robot.move_straight(vel_forward)
        return "MOVE_TO_BRICK", frame, {"tracking" : True}
    elif error[0] < 0:
        robot.rotate_right(vel=vel_rot)
        return "MOVE_TO_BRICK", frame, {"tracking" : True}
    else:
        # Positive velocity for turning left
        robot.rotate_left(vel=vel_rot)
        return "MOVE_TO_BRICK", frame, {"tracking" : True}


    


def move_to_brick_nn_v1(robot, frame, img_res=(640, 480), atol=5,
                         vel_forward = 299, vel_rot = 50, atol_move_blind=90, 
                         fail_counter=0, center_position_error = 10):
    """
    Moves the robot towards the brick.

    :param robot: The robot instance
    :param img_res: The image resolution
    :param atol: The absolute error tolerance
    :return: Direction string
    """
    _, frame = robot.cap.read()

    img = frame
    kernel = np.ones((5,5),np.float32)/25
    img = cv2.filter2D(img,-1,kernel)
    res = robot.object_detector.detect_with_threshold(img,threshold=0.7, return_closest=True)
    print("Res: ", res)
    if len(res)==0 and fail_counter==5:
        return "SEARCH", frame, {}
    elif len(res)==0:
        print("Fail: ", fail_counter, res)
        return "MOVE_TO_BRICK", frame, {"fail_counter" : fail_counter+1}
    print("Shape: ", res[0])
    box, score = res[0] 
    img_res = np.asarray(img_res)
    coords = bbox_center(box[1], box[0], box[3], box[2]) * img_res
    coords = coords.astype(np.int16)
    atol = 10 + coords[1]/480 * 50
    print("Coords: ", coords)
    cv2.rectangle(frame, (int(box[1]*img_res[0]), int(box[0]*img_res[1])), 
            (int(box[3]*img_res[0]), int(box[2]*img_res[1])), (0, 255, 0), thickness=3)    
    cv2.putText(frame, str(score) + "%", (int(box[1]*img_res[0]),
     int(box[0]*img_res[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=(255,0,0))
    cv2.circle(frame,tuple(coords),2,(255,255,255),thickness=2)

    img_center = img_res / 2 - center_position_error 
    error = img_center - coords

    # Move forward till light sensor detects brick if brick is near the bottom of image
    # and centered
    if np.isclose(coords[0], img_center[0], atol=atol) and np.isclose(coords[1], img_res[1], atol=atol_move_blind):
        robot.move_straight(vel_forward, 500)
        return "MOVE_TO_BRICK_BLIND_AND_GRIP", frame, {}

    if np.isclose(coords[0], img_center[0], atol=atol):
        print("Move straight")
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

    res = robot.object_detector.detect_with_threshold(img,threshold=0.7, return_closest=True)
    if len(res) > 0:
        box, score = res[0]
        box = np.asarray((int(box[1]*img_res[0]), int(box[0]*img_res[1]), int(box[3]*img_res[0]), int(box[2]*img_res[1])))
        lego_coords = bbox_center(box)
    else:
        lego_coords = None

    if lego_coords:
        return "MOVE_TO_BRICK", frame, {}
  #  elif len(robot.map) > 0 :
        #return "MOVE_BY_MAP", frame, {"iteration": 0, "path": []}
    else:
        robot.rotate_right(vel)
        return "SEARCH", frame, {}

def rotation_search_brick_nn(robot, frame, vel=60):

    res = robot.object_detector.detect_with_threshold(frame,threshold=0.9, return_closest=True)
    detected = len(res) > 0
    if detected:
        lego_coords, score = res[0]
    else:
        lego_coords = None
    if lego_coords is not None:
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

def move_to_brick_blind_and_grip(robot, frame, vel=400, t=1700):
    # Make sure the grip is open
    robot.grip.open()
    print("Velocity: ", vel)
    # Make sure the elevator is down
    print(robot.elevator.is_raised)
    print(robot.elevator.position)
    robot.elevator.down()
    robot.elevator.wait_until_not_moving()
    robot.move_straight(vel=vel, time=t)
    robot.wait_until_not_moving()
    robot.pick_up()
    return "FINAL_STATE", frame, {}



