"""Go big and go home :)"""
import time
from math import pi
import sys

import cv2
import numpy as np
import cv2.aruco as aruco
import matplotlib.pyplot as plt

from ev3control.rpc import Robot
from rick.A_star_planning import *
from rick.async import AsyncCamera
from rick.controllers import *
from rick.core import State
from rick.core import main_loop
from rick.live_plotting import MapRenderer
from rick.motion_control import (euclidian_kalman, kalman_filter, kalman_filter2, robot_control,
                                 odom_estimation)
from rick.mc_please_github_donot_fuck_with_this_ones import (A_star_path_planning_control,
                                                             compute_A_star_path, A_star_control)
from rick.utils import TrackerWrapper, debug_print
from nn_object_detection.object_detectors import NNObjectDetector
from detection.marker_localization import get_marker_pose, load_camera_params
from detection.marker_localization import get_specific_marker_pose, load_camera_params
from detection.opencv import get_lego_boxes
from detection.opencv import detect_purple

sys.path.append("../slam/")
import mapping

# yapf: disable
MARKER_MAP = np.array([
    [200, 100, 0],
    [50, 0, 0],
    [100, 0, 0],
    [0, 100, 0],
    [100, 100, 0],
    [200, 0, 0]
    ])
# yapf: enable
KALMAN_SAMPLE_TIME = 0.3

H = np.load('Homographygood.npz')["arr_0"]

map_renderer = MapRenderer()


def plot_mapa(mapa, robot_traj):

    mapa1 = np.array(mapa)
    rob = np.array(robot_traj)
    if mapa1.size:
        plt.scatter(mapa1[:, 0], mapa1[:, 1])
    if rob.size > 100:
        plt.plot(rob[:, 0], rob[:, 1])
        plt.axis([-100, 150, -100, 150])
        plt.legend(["Lego", "path"])
        plt.show(block=False)


def search_control(state_search, mapa, pos_rob, t_old):

    t1 = 0
    if state_search == 1:
        target = [0.1, 0.1]  # THE POINT REPRESENTS THE MIDDLE OF THE WORKSPACE
        vel_wheels = robot_control(pos_rob, target, K_x=1, K_y=1, K_an=1)
        distance = np.sqrt(
            np.power(pos_rob[0] - target[0], 2) + np.power(pos_rob[1] - target[1], 2))
        if distance < 10:
            state_search = 2
            t1 = time.time()

    elif state_search == 2:

        vel_wheels = [-100, 100]

    return vel_wheels, state_search, t1


def index23(BB_legos, BB_target):
    index = 1000
    i = 0
    for box in BB_legos:
        if box[0] == BB_target[0][0] and box[1] == BB_target[0][1]:

            index = i
        i += 1
    return index


def PID_control(robot, marker_map, box_coords, hist):
    vel_st = 100
    vel_rot = 60
    lat_tol = 4
    yshift = 6
    er_x = marker_map[0, 0] - robot[0]
    er_y = marker_map[0, 1] - robot[1]
    er_angle = np.arctan2(er_y, er_x) - robot[2] * pi / 180

    if er_angle > pi:
        er_angle = er_angle - 2 * pi
    if er_angle < -pi:
        er_angle = er_angle + 2 * pi

    distance = np.sqrt(np.power(er_x, 2) + np.power(er_y, 2))

    if box_coords:
        if abs(box_coords[1] + yshift) > lat_tol:
            vel_wheels = np.asarray([-vel_rot, vel_rot]) * np.sign(-box_coords[1])
        elif box_coords[0] > 35:
            vel_wheels = np.asarray([vel_st, vel_st])
        else:
            vel_wheels = np.asarray([0, 0])
            hist = 0

    else:
        if hist == 0:
            vel_wheels = np.asarray([0, 0])
        elif er_angle > 0.7:
            vel_wheels = np.asarray([vel_rot, -vel_rot])
            hist = 1
        elif er_angle < -0.7:
            vel_wheels = np.asarray([-vel_rot, vel_rot])
            hist = -1
        elif hist == 1:
            vel_wheels = np.asarray([vel_rot, -vel_rot])
        else:
            vel_wheels = np.asarray([-vel_rot, vel_rot])
    return vel_wheels, hist


# ==========
# Callbacks
# ==========


def explore_workspace(robot,
                      frame,
                      ltrack_pos=0,
                      rtrack_pos=0,
                      P=np.identity(3),
                      marker_list=None,
                      delete_countdown=0,
                      mapa=None,
                      robot_trajectory=None,
                      R=None,
                      state_search=2,
                      t1=0):
    marker_list = [] if marker_list is None else marker_list
    mapa = [] if mapa is None else mapa
    robot_trajectory = [] if robot_trajectory is None else robot_trajectory
    R = [] if R is None else R

    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos


    BB_legos = get_lego_boxes(frame)

    lego_landmarks = mapping.cam2rob(BB_legos, H)

    estim_rob_pos_odom = odom_estimation(odom_r, odom_l, robot.position)
    # 2. UPDATE THE MAP WITH ODOMETRY INFO
    #mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,lego_landmarks,estim_rob_pos_odom,P,delete_countdown, robot_trajectory, index)

    estim_rob_pos, P = kalman_filter(odom_r, odom_l, robot.position, marker_list, MARKER_MAP,
                                     KALMAN_SAMPLE_TIME, P)
    robot.position = estim_rob_pos
    # 4. UPDATE MAP POINTS RELATED TO KALMAN
    mapa = mapping.after_kalman_improvement(mapa, robot.position, estim_rob_pos_odom)

    # GET GRIPPER POS

    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 * np.cos(estim_rob_pos[2] * pi / 180)
    d[1] = estim_rob_pos[1] + 28 * np.sin(estim_rob_pos[2] * pi / 180)
    d[2] = estim_rob_pos[2]
    R.append(d)
    # SHOW THE MAP
    map_renderer.plot_bricks_and_trajectory(mapa, R)

    # DEFINE MOTION CONTROL FOR SEARCHING

    # THE CONTROL IS : 1. GO TO THE CENTER OF THE WORKSPACE, 2. ROUND FOR 2
    # secs ,  SELECT A POINT CLOSE TO THE CENTER as new target

    vel_wheels, state_search, t1 = search_control(state_search, mapa, robot.position, t1)

    if len(BB_target) > 0:
        robot.tracker.init(frame, BB_target[0])
        return ("GO_TO_TARGET", frame, {
            "tracker": robot.tracker,
            "ltrack_pos": robot.left_track.position,
            "rtrack_pos": robot.right_track.position,
            "robot_trajectory": robot_trajectory,
            "pos_rob": robot.position,
            "R": R,
            "mapa": mapa
        })
    else:
        robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
        return ("SEARCH_TARGET", frame, {
            "ltrack_pos": new_ltrack_pos,
            "rtrack_pos": new_rtrack_pos,
            "P": P,
            "marker_list": [],
            "delete_countdown": delete_countdown,
            "mapa": mapa,
            "robot_trajectory": robot_trajectory,
            "R": R,
            "state_search": 2,
            "t1": t1
        })


def search_target_with_Kalman_and_mapping(robot,
                                          frame,
                                          ltrack_pos=0,
                                          rtrack_pos=0,
                                          P=np.identity(3),
                                          marker_list=None,
                                          delete_countdown=0,
                                          mapa=None,
                                          robot_trajectory=None,
                                          R=None,
                                          state_search=2,
                                          t1=0,
                                          iteration=0,
                                          feature_map=None):
    marker_list = [] if marker_list is None else marker_list
    mapa = [] if mapa is None else mapa
    robot_trajectory = [] if robot_trajectory is None else robot_trajectory
    R = [] if R is None else R
    feature_map = [] if feature_map is None else feature_map

    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    # Markers information coming from the compuetr vision stuff

    #frame,marker_list  = camera_related(frame = frame)
    marker_map = MARKER_MAP

    # Information related with lego blocks mapping

    BB_legos = get_lego_boxes(frame)
    BB_legos2 = []

    for bbox in BB_legos:
        if np.abs(bbox[0] - bbox[2]) > 2 and np.abs(bbox[1] - bbox[3]) > 2:
            BB_legos2.append(bbox)

    lego_landmarks = mapping.cam2rob(BB_legos2, H)

    # WHAT SLAM IS!

    # 1. ESTIMATE POSITION BY ODOMETRY

    estim_rob_pos_odom = odom_estimation(odom_r, odom_l, robot.position)

    # 2. UPDATE THE MAP WITH ODOMETRY INFO
    #mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,lego_landmarks,estim_rob_pos_odom,P,delete_countdown, robot_trajectory, index)

    index = 1000
    mapa, delete_countdown, robot_trajectory, links = mapping.update_mapa2(
        mapa, lego_landmarks, estim_rob_pos_odom, P, delete_countdown, robot_trajectory, index)

    # 3. KALMAN FILTER

    estim_rob_pos, P = kalman_filter(odom_r, odom_l, robot.position, marker_list, marker_map,
                                     KALMAN_SAMPLE_TIME, P)

    robot.position = estim_rob_pos

    # 4. UPDATE MAP POINTS RELATED TO KALMAN

    mapa = mapping.after_kalman_improvement(mapa, robot.position, estim_rob_pos_odom)

    # GET GRIPPER POS

    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 * np.cos(estim_rob_pos[2] * pi / 180)
    d[1] = estim_rob_pos[1] + 28 * np.sin(estim_rob_pos[2] * pi / 180)
    d[2] = estim_rob_pos[2]

    R.append(d)

    map_renderer.plot_bricks_and_trajectory(mapa, R)

    # Feature extraction from bounding boxes
    bboxes = []

    for i in range(0, len(links)):
        bbox = BB_legos2[links[i][0]]
        bboxes.append(frame[bbox[1]:bbox[3], bbox[0]:bbox[2], :])
    bounding_box_features = similarity_detector.extract_features(bboxes)

    for i in range(0, len(links)):

        feature_map[links[i][1]] = bounding_box_features[i]

    # DEFINE MOTION CONTROL FOR SEARCHING

    # THE CONTROL IS : 1. GO TO THE CENTER OF THE WORKSPACE, 2. ROUND FOR 2
    # secs ,  SELECT A POINT CLOSE TO THE CENTER as new target

    vel_wheels, state_search, t1 = search_control(state_search, mapa, robot.position, t1)

    iteration += 1
    if iteration == 100:

        closest_feature_index = None
        closest_score = -1000000
        target = robot.target_features
        for i, feature in enumerate(feature_map):
            score = -np.sum(np.square(target - feature))
            if score > closest_score:
                closest_score = score
                closest_feature_index = i
        target_point[0] = mapa[i][0]
        target_point[1] = mapa[i][1]
        mapa[i][4] = 5

        map_renderer.plot_bricks_and_trajectory(mapa, R)

        return ("GO_TO_TARGET", frame, {
            "ltrack_pos": robot.left_track.position,
            "rtrack_pos": robot.right_track.position,
            "R": R,
            "obj": target_point,
            "mapa": mapa
        })
    else:
        robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
        return ("SEARCH_TARGET", frame, {
            "ltrack_pos": new_ltrack_pos,
            "rtrack_pos": new_rtrack_pos,
            "P": P,
            "marker_list": [],
            "delete_countdown": delete_countdown,
            "mapa": mapa,
            "robot_trajectory": robot_trajectory,
            "R": R,
            "state_search": 2,
            "t1": t1,
            "iteration": iteration,
            "feature_map": feature_map
        })


def move_to_brick_v3(robot,
                     frame,
                     img_res=np.asarray((640, 480)),
                     atol=10,
                     vel_forward=299,
                     vel_rot=50,
                     atol_move_blind=70,
                     fail_counter=0,
                     center_position_error=75,
                     tracker=None,
                     ltrack_pos=0,
                     rtrack_pos=0,
                     pos_rob=None,
                     marker_list=None,
                     P=np.identity(3),
                     R=None,
                     mapa=None,
                     robot_trajectory=None):
    """Go to target."""
    pos_rob = [] if pos_rob is None else pos_rob
    marker_list = [] if marker_list is None else marker_list
    mapa = [] if mapa is None else mapa
    robot_trajectory = [] if robot_trajectory is None else robot_trajectory
    R = [] if R is None else R

    # LOCALIZATION AND MAPPING
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    # Markers information coming from the compuetr vision stuff
    # 3 RELATED WITH LEGOS
    BB_legos = get_lego_boxes(frame)
    # DETECT IF ANY OF THE BOXES IS PURPLE
    BB_target = detect_purple(frame, BB_legos)
    index = 1000
    if len(BB_target) != 0:
        index = index23(BB_legos, BB_target)
    # GET LEGOS LANDMARKS
    lego_landmarks = mapping.cam2rob(BB_legos, H)
    delete_countdown = 0

    # WHAT SLAM IS!

    # 1. ESTIMATE POSITION BY ODOMETRY

    estim_rob_pos_odom = odom_estimation(odom_r, odom_l, robot.position)
    # 2. UPDATE THE MAP WITH ODOMETRY INFO
    #mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,lego_landmarks,estim_rob_pos_odom,P,delete_countdown, robot_trajectory, index)
    # 3. KALMAN FILTER
    marker_map = MARKER_MAP

    estim_rob_pos, P = kalman_filter(odom_r, odom_l, robot.position, marker_list, marker_map,
                                     KALMAN_SAMPLE_TIME, P)
    robot.position = estim_rob_pos
    # 4. UPDATE MAP POINTS RELATED TO KALMAN
    #mapa = mapping.after_kalman_improvement(mapa, robot.position, estim_rob_pos_odom)

    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 * np.cos(estim_rob_pos[2] * pi / 180)
    d[1] = estim_rob_pos[1] + 28 * np.sin(estim_rob_pos[2] * pi / 180)
    d[2] = estim_rob_pos[2]

    R.append(d)

    map_renderer.plot_bricks_and_trajectory(mapa, R)

    # Information related with lego blocks mapping

    ok, bbox = tracker.update(frame)

    if not ok:
        BB_legos = get_lego_boxes(frame)
        # res = robot.object_detector.detect_with_threshold(frame,threshold=0.9, return_closest=False)
        # BB_legos = map(lambda x: x[0], res)
        BB_target = detect_purple(frame, BB_legos)
        if len(BB_target) == 0:
            return ("SEARCH_TARGET", frame, {
                "ltrack_pos": new_ltrack_pos,
                "rtrack_pos": new_rtrack_pos,
                "P": P,
                "marker_list": [],
                "delete_countdown": delete_countdown,
                "mapa": mapa,
                "robot_trajectory": robot_trajectory,
                "R": R,
                "state_search": 2
            })
        tracker.init(frame, BB_target[0])
        bbox = BB_target[0]

    # MOVE TO THE TARGET

    coords = bbox_center(*bbox)
    img_center = img_res / 2 - center_position_error
    error = img_center - coords
    atol = 5 + coords[1] / 480 * 40
    cv2.line(frame, (int(img_center[0]), 0), (int(img_center[0]), 480), (0, 255, 0))
    frame = plot_bbox(frame, bbox, 0, (255, 0, 0))
    #img_center = img_res/2.

    if np.isclose(
            coords[0], img_center[0], atol=atol) and np.isclose(
                coords[1], img_res[1], atol=atol_move_blind):
        robot.move_straight(vel_forward, 500)
        return ("MOVE_TO_BRICK_BLIND_AND_GRIP", frame, {
            "R": R,
            "ltrack_pos": new_ltrack_pos,
            "rtrack_pos": new_rtrack_pos,
            "mapa": mapa
        })

    if np.isclose(coords[0], img_center[0], atol=atol):
        robot.move_straight(vel_forward)
        return ("GO_TO_TARGET", frame, {
            "tracker": tracker,
            "ltrack_pos": new_ltrack_pos,
            "rtrack_pos": new_rtrack_pos,
            "pos_rob": robot.position,
            "R": R,
            "mapa": mapa
        })
    elif error[0] < 0:
        robot.rotate_left(vel=vel_rot)
        return ("GO_TO_TARGET", frame, {
            "tracker": tracker,
            "ltrack_pos": new_ltrack_pos,
            "rtrack_pos": new_rtrack_pos,
            "pos_rob": robot.position,
            "R": R,
            "mapa": mapa
        })
    else:
        # Positive velocity for turning left
        robot.rotate_right(vel=vel_rot)
        return ("GO_TO_TARGET", frame, {
            "tracker": tracker,
            "ltrack_pos": new_ltrack_pos,
            "rtrack_pos": new_rtrack_pos,
            "pos_rob": robot.position,
            "R": R,
            "mapa": mapa
        })


# MOVE TO BRICK BLIND
def move_to_brick_blind_and_grip(robot,
                                 frame,
                                 R=None,
                                 ltrack_pos=0,
                                 rtrack_pos=0,
                                 marker_list=None,
                                 mapa=None,
                                 vel=400,
                                 t=1700):
    marker_list = [] if marker_list is None else marker_list
    mapa = [] if mapa is None else mapa
    R = [] if R is None else R
    # Make sure the grip is open
    robot.grip.open()
    # Make sure the elevator is down
    robot.elevator.down()
    robot.elevator.wait_until_not_moving()
    robot.move_straight(vel=vel, time=t)
    robot.wait_until_not_moving()
    robot.pick_up()

    # odometry update
    marker_map = MARKER_MAP

    P = np.identity(3)
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    estim_rob_pos, P = kalman_filter(odom_r, odom_l, robot.position, marker_list, marker_map,
                                     KALMAN_SAMPLE_TIME, P)
    robot.position = estim_rob_pos

    estim_rob_pos_odom = odom_estimation(odom_r, odom_l, robot.position)
    return ("SEARCH_BOX", frame, {
        "ltrack_pos": new_ltrack_pos,
        "rtrack_pos": new_rtrack_pos,
        "mapa": mapa,
        "R": R
    })


# SEARCH BOX
def search_box(robot,
               frame,
               ltrack_pos=0,
               rtrack_pos=0,
               P=np.identity(3),
               marker_list=None,
               delete_countdown=0,
               mapa=None,
               robot_trajectory=None,
               R=None,
               state_search=2,
               t1=0):
    marker_list = [] if marker_list is None else marker_list
    mapa = [] if mapa is None else mapa
    robot_trajectory = [] if robot_trajectory is None else robot_trajectory
    R = [] if R is None else R

    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    estim_rob_pos_odom = odom_estimation(odom_r, odom_l, robot.position)
    marker_map = np.array([[150, 0, 0]])
    marker_map_obj = np.array([[110, 0, 0]])

    estim_rob_pos, P = kalman_filter(odom_r, odom_l, robot.position, marker_list, marker_map_obj,
                                     KALMAN_SAMPLE_TIME, P)

    robot.position = estim_rob_pos

    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 * np.cos(estim_rob_pos[2] * pi / 180)
    d[1] = estim_rob_pos[1] + 28 * np.sin(estim_rob_pos[2] * pi / 180)
    d[2] = estim_rob_pos[2]

    R.append(d)

    map_renderer.plot_bricks_and_trajectory(mapa, R)

    mtx, dist = load_camera_params()
    frame, box_coords = get_specific_marker_pose(
        frame=frame, mtx=mtx, dist=dist, marker_id=0, markerLength=8.6)

    vel_wheels, state_search, t1 = search_control(state_search, mapa, robot.position, t1)

    if box_coords:

        return ("COMPUTE_PATH", frame, {
            "box_coords": box_coords,
            "ltrack_pos": new_ltrack_pos,
            "rtrack_pos": new_rtrack_pos,
            "mapa": mapa,
            "R": R
        })
    else:
        robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
        return "SEARCH_BOX", frame, {
            "ltrack_pos": new_ltrack_pos,
            "rtrack_pos": new_rtrack_pos,
            "P": P,
            "marker_list": [],
            "delete_countdown": delete_countdown,
            "mapa": mapa,
            "robot_trajectory": robot_trajectory,
            "R": R,
            "state_search": 2,
            "t1": t1
        }


def compute_path(robot, frame, box_coords, ltrack_pos=0, rtrack_pos=0, mapa=None, R=None):
    mapa = [] if mapa is None else mapa
    R = [] if R is None else R

    x = box_coords[0]
    y = box_coords[1]
    yaw = box_coords[2]
    if (y > 0 and yaw > -80) or (y < 0 and yaw < -100):
    thm = 40
    thobj = 40

    x2 = x + thm * np.sin(yaw * np.pi / 180.)
    y2 = y - thm * np.cos(yaw * np.pi / 180.)
    yaw2 = 0
    xobj = x + thobj * np.sin(yaw * np.pi / 180.)
    yobj = y - thobj * np.cos(yaw * np.pi / 180.)

    obj = [x, y]
    obslist = [[50, 0]]
    Map = create_map(obslist)
    path = A_star([0, 0], obj, Map)
    robot.grip.close()
    R = R

    return ("MOVE_TO_BOX", frame, {
        "Map": Map,
        "obj": obj,
        "ltrack_pos": ltrack_pos,
        "rtrack_pos": rtrack_pos,
        "TIME": time.time(),
        "R": R
    })


def A_star_move_to_box_blind(robot,
                             frame,
                             Map,
                             obj,
                             replan=1,
                             path=None,
                             iteration=0,
                             ltrack_pos=0,
                             rtrack_pos=0,
                             TIME=0,
                             P=np.identity(3),
                             R=None):
    path = [] if path is None else path
    R = [] if R is None else R

    mtx, dist = load_camera_params()
    frame, box_coords = get_specific_marker_pose(
        frame=frame, mtx=mtx, dist=dist, marker_id=0, markerLength=8.6)
    old_path = path
    # REPLANNING

    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    marker_list = []
    if box_coords:
        x = box_coords[0]
        y = box_coords[1]
        yaw = box_coords[2]
        thobj = 40
        xobj = x + thobj * np.sin(yaw * np.pi / 180.)
        yobj = y - thobj * np.cos(yaw * np.pi / 180.)
        obj = [xobj + robot.position[0], yobj + robot.position[1]]

        angle = np.arctan2(yobj, xobj)
        distance = np.sqrt(np.power(xobj, 2) + np.power(yobj, 2))
        marker_list.append([angle, distance, (yaw + 90) * pi / 180])

    marker_map = np.array([[150, 0, 0]])
    marker_map_obj = np.array([[110, 0, 0]])

    estim_rob_pos, P = kalman_filter2(odom_r, odom_l, robot.position, marker_list, marker_map_obj,
                                      KALMAN_SAMPLE_TIME, P)

    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 * np.cos(estim_rob_pos[2] * pi / 180)
    d[1] = estim_rob_pos[1] + 28 * np.sin(estim_rob_pos[2] * pi / 180)
    d[2] = estim_rob_pos[2]

    R.append(d)
    mapa = []
    Map = Map
    map_renderer.plot_bricks_and_trajectory(mapa, R)
    robot.position = estim_rob_pos

    replan = 1
    goal_pos = marker_map_obj[0, :]
    t0 = time.time()

    vel_wheels, new_path = A_star_control(
        robot.position,
        goal_pos,
        Map,
        robot.sampling_rate,
        odom_r=odom_r,
        odom_l=odom_l,
        iteration=iteration,
        path=path)

    distance_to_target = np.sqrt(
        np.power(estim_rob_pos[0] - marker_map_obj[0, 0], 2) + np.power(
            estim_rob_pos[1] - marker_map_obj[0, 1], 2))

    if abs(estim_rob_pos[0] - marker_map_obj[0, 0]) < 15 and abs(estim_rob_pos[1] -
                                                                 marker_map_obj[0, 1]) < 20:
        return ("MOVE_TO_BOX_BY_VISION", frame, {
            "replan": replan,
            "iteration": iteration,
            "path": new_path,
            "ltrack_pos": new_ltrack_pos,
            "rtrack_pos": new_rtrack_pos,
            "TIME": t0
        })

    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    iteration += 1

    return ("MOVE_TO_BOX", frame, {
        "replan": replan,
        "Map": Map,
        "obj": goal_pos,
        "iteration": iteration,
        "path": new_path,
        "ltrack_pos": new_ltrack_pos,
        "rtrack_pos": new_rtrack_pos,
        "TIME": t0,
        "R": R
    })


def move_to_box_by_vision(robot,
                          frame,
                          replan=1,
                          path=None,
                          iteration=0,
                          ltrack_pos=0,
                          rtrack_pos=0,
                          TIME=0,
                          P=np.identity(3),
                          histeresis=1):
    path = [] if path is None else path

    mtx, dist = load_camera_params()
    frame, box_coords = get_specific_marker_pose(
        frame=frame, mtx=mtx, dist=dist, marker_id=0, markerLength=8.6)

    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    marker_list = []
    if box_coords:
        x = box_coords[0]
        y = box_coords[1]
        yaw = box_coords[2]
        thobj = 40
        xobj = x + thobj * np.sin(yaw * np.pi / 180.)
        yobj = y - thobj * np.cos(yaw * np.pi / 180.)
        obj = [xobj + robot.position[0], yobj + robot.position[1]]
        angle = np.arctan2(yobj, xobj)
        distance = np.sqrt(np.power(xobj, 2) + np.power(yobj, 2))
        marker_list.append([angle, distance, (yaw + 90) * pi / 180])

    marker_map = np.array([[150, 0, 0]])
    marker_map_obj = np.array([[110, 0, 0]])

    estim_rob_pos, P = kalman_filter2(odom_r, odom_l, robot.position, marker_list, marker_map_obj,
                                      KALMAN_SAMPLE_TIME, P)

    robot.position = estim_rob_pos

    vel_wheels, hist = PID_control(estim_rob_pos, marker_map, box_coords, histeresis)
    if hist == 0:
        return "PLACE_OBJECT_IN_THE_BOX", frame, {}

    robot.move(vel_wheels[0], vel_wheels[1])
    return ("MOVE_TO_BOX_BY_VISION", frame, {
        "ltrack_pos": new_ltrack_pos,
        "rtrack_pos": new_rtrack_pos,
        "histeresis": hist
    })


def place_object_in_the_box(robot, frame):
    robot.move(vel_left=100, vel_right=100, time=2000)
    robot.left_track.wait_until_not_moving(timeout=3000)
    robot.reset()
    robot.grip.wait_until_not_moving(timeout=3000)
    return ("FINAL_STATE", frame, {})


with Robot(
        AsyncCamera(1), tracker=TrackerWrapper(cv2.TrackerKCF_create),
        object_detector=None) as robot:
    robot.map = [(200, 0)]
    robot.sampling_rate = 0.1
    states = [
        State(
            name="EXPLORE",
            act=explore_workspace,),
        State(
            name="SEARCH_TARGET",
            act=search_target_with_Kalman_and_mapping,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "P": np.identity(3),
                "delete_countdown": 0,
                "mapa": [],
                "robot_trajectory": []
            }),
        State(
            name="GO_TO_TARGET",
            act=move_to_brick_v3,
            default_args={"vel_forward": 200,
                          "vel_rot": 60,
                          "atol_move_blind": 100}),
        State(
            name="MOVE_TO_BRICK_BLIND_AND_GRIP",
            act=move_to_brick_blind_and_grip,
            default_args={
                "vel": 250,
                "t": 1200,
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
            }),
        State(
            name="SEARCH_BOX",
            act=search_box,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
            }),
        State(
            name="COMPUTE_PATH",
            act=compute_path,
            default_args={
                "box_coords": [200, 0, 0],
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
            }),
        State(
            name="MOVE_TO_BOX",
            act=A_star_move_to_box_blind,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time()
            }),
        State(
            name="MOVE_TO_BOX_BY_VISION",
            act=move_to_box_by_vision,),
        State(
            name="PLACE_OBJECT_IN_THE_BOX",
            act=place_object_in_the_box,),
        State(name="FINAL_STATE", act=lambda robot, frame, **args: time.sleep(.5))
    ]
    state_dict = {}
    for state in states:
        state_dict[state.name] = state

    start_state = states[0]

    main_loop(robot, start_state, state_dict, delay=0)
