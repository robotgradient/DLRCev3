#Euclidean path planning control with Kalman filter for localization

import time
import cv2
from ev3control.rpc import Robot
from rick.controllers import *
from rick.core import State
from rick.core import main_loop
from rick.async import AsyncCamera
from rick.utils import TrackerWrapper
from nn_object_detection.object_detectors import NNObjectDetector

from detection.marker_localization import get_marker_pose, load_camera_params
import cv2.aruco as aruco


import numpy as np


from detection.opencv import get_lego_boxes



from rick.motion_control import euclidian_kalman , kalman_filter , robot_control

import sys

sys.path.append("../slam/")

import mapping

import matplotlib.pyplot as plt

from detection.opencv import detect_purple

PATH_TO_CKPT = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/models/faster_rcnn_resnet_lego_v1/train/frozen_inference_graph.pb"
PATH_TO_LABELS = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/label_map.pbtxt"


print("Creating robot...")


data = np.load('Homography.npz')
H=data["arr_0"]

def plot_mapa(mapa,robot_traj=[]):


    mapa1 = np.array(mapa)
    rob = np.array(robot_traj)
    print("Before stop")
    if mapa1.size:
        print("In")
        plt.scatter(mapa1[:,0],mapa1[:,1])
        print("Out")
    if rob.size > 100:
        plt.plot(rob[:,0],rob[:,1])
        plt.axis([-100, 150, -100, 150])
        plt.legend(["Lego", "path"])
        plt.show()
    print("After stop")


def search_control(state_search,mapa, pos_rob, t_old):

    t1 = 0
    if state_search ==1:

        target = [0.1,0.1]# THE POINT REPRESENTS THE MIDDLE OF THE WORKSPACE
        vel_wheels = robot_control(pos_rob,target, K_x=1,K_y=1,K_an=1)

        distance = np.sqrt(np.power(pos_rob[0]-target[0],2) + np.power(pos_rob[1]-target[1],2))

        if distance < 10:
            state_search = 2
            t1 = time.time()
    
    elif state_search ==2:

        vel_wheels = [50,160]

    return vel_wheels,state_search,t1





def search_target_with_Kalman_and_mapping(robot, frame
                            , ltrack_pos=0, rtrack_pos=0, P=np.identity(3), marker_list = [], delete_countdown = 0 , mapa = [], robot_trajectory = [],R=[],state_search = 2 , t1=0):
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    ######################  Markers information coming from the compuetr vision stuff
    
    frame,marker_list  = camera_related(frame = frame)
    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])


    ################### ESTIMATE ROBOT'S POSE
    #estim_rob_pos, P  = kalman_filter(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)

    #robot.position = estim_rob_pos

    ###################### Information related with lego blocks mapping

    res = robot.object_detector.detect_with_threshold(frame,threshold=0.9, return_closest=False)
    BB_legos = map(lambda x: x[0], res)

    #BB_legos = get_lego_boxes(frame)

    #######################     DETECT IF ANY OF THE BOXES IS PURPLE

    BB_target = detect_purple(frame,BB_legos)

    print(BB_legos)
    print("BB target ", BB_target)
    lego_landmarks = mapping.cam2rob(BB_legos,H)

    print("lego_landmarks", lego_landmarks)

    #lego_landmarks[index,4] = 5

    ############   UPDATE MAP

    mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,lego_landmarks,robot.position,P,delete_countdown, robot_trajectory)
    plot_mapa(mapa)


    #DEFINE MOTION CONTROL FOR SEARCHING

    # THE CONTROL IS : 1. GO TO THE CENTER OF THE WORKSPACE, 2. ROUND FOR 2 secs ,  SELECT A POINT CLOSE TO THE CENTER as new target

    vel_wheels,state_search,t1 = search_control(state_search, mapa, robot.position, t1)

    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    purple_yes = False
    if len(BB_target) > 0:

        # return "GO_TO_TARGET", frame, {"iteration" : 0, "path" : [], "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": time.time() , "P": P , "marker_list": marker_list,
        #                             "delete_countdown" : delete_countdown , "mapa": mapa, "robot_trajectory": robot_trajectory, "R" : R}
        robot.tracker.init(frame, BB_target[0])
        return "GO_TO_TARGET", frame, {"tracker" : robot.tracker}
    else:

        return "SEARCH_TARGET", frame, {"ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "P": P , "marker_list": marker_list,
                                        "delete_countdown" : delete_countdown , "mapa": mapa, "robot_trajectory": robot_trajectory, "R" : R,
                                        "state_search" : 2, "t1" : t1 }
    
def euclidian_move_with_kalman_and_map(robot, frame,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0, P=np.identity(3), marker_list = [], delete_countdown =0 , mapa = [], robot_trajectory = [],R=[] ):
    
    #DEFINE TARGET POSITION
    brick_position = robot.map[0]


    t0 = time.time()
    print('t0 ', t0, 'TIME', TIME)
    time_diff = t0 - TIME
    print('time',time_diff)
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    print("odometry: ", odom_r,odom_l)

 
    # Markers information coming from the compuetr vision stuff
    
    t2 = time.time()
    frame,marker_list  = camera_related(frame = frame)
    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])

    t3 =time.time()

    print("time for optical markers: ",t3-t2)

    #print("marker_list1: ",marker_list)
    t2 = time.time()
    estim_rob_pos, vel_wheels, new_path , P ,  = euclidian_kalman(robot.position,
                                                                          brick_position, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path,
                                                                          P=P, marker_list = marker_list ,marker_map=marker_map)
    t3 = time.time()
    print("time for Kalman",t3-t2)
    print("estimated position: ",estim_rob_pos)

   

    robot.position = estim_rob_pos

    # Information related with lego blocks mapping
    t2 = time.time()
    res = robot.object_detector.detect_with_threshold(frame,threshold=0.9, return_closest=False)
    BB_legos = map(lambda x: x[0], res)


    #######################     DETECT IF ANY OF THE BOXES IS PURPLE

    BB_target = detect_purple(frame,BB_legos)


    lego_landmarks = mapping.cam2rob(BB_target,H)

    print("lego_landmarks", lego_landmarks)

    t3 = time.time()
    print("time to detect legos", t3-t2)
    #UPDATE MAP

    t2 = time.time()
    mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,lego_landmarks,robot.position,P,delete_countdown, robot_trajectory)
    t3 = time.time()

    t2 = time.time()
    vel_wheels = [0,0]
    print("time updating map", t3-t2)
    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    iteration += 1
    t3 = time.time()
    print("send command", t3-t2)
    
    t2 = time.time()
    R.append(estim_rob_pos)
    plot_mapa(mapa,R)
    t3 = time.time()
    print("send command", t3-t2)


    print("{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{")

    #COMPUTE DISTANCE TO THE TARGET
    distance = np.sqrt(np.power(estim_rob_pos[0]-brick_position[0],2) + np.power(estim_rob_pos[1]-brick_position[1],2))

    print("time in Main code : ", time.time()-t0)
    if (distance < -5) :
        return "PICK_TARGET", frame, {"ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "P": P , "marker_list": marker_list,
                                        "delete_countdown" : delete_countdown , "mapa": mapa, "robot_trajectory": robot_trajectory, "R" : R}
    else:
        # return "GO_TO_TARGET", frame, {"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0 , "P": P , "marker_list": marker_list,
        #                             "delete_countdown" : delete_countdown , "mapa": mapa, "robot_trajectory": robot_trajectory, "R" : R}
        robot.tracker.init(frame, BB_target[0])
        return "GO_TO_TARGET", frame, {"tracker" : robot.tracker}



def search_and_pick(robot, frame
                            , ltrack_pos=0, rtrack_pos=0, P=np.identity(3), marker_list = [], delete_countdown =0 , mapa = [], robot_trajectory = [],R=[] ):

    pass


def camera_related(frame):

	arucoParams = aruco.DetectorParameters_create()
	mtx,dist = load_camera_params()
	image,marker_pos  = get_marker_pose(frame, mtx, dist,arucoParams=arucoParams, marker_list=[0,1,2,3,4,5], markerLength = 3.3)


	#print("Output marco function:",marker_pos)

	return image,marker_pos



with Robot(AsyncCamera(0), tracker=TrackerWrapper(cv2.TrackerKCF_create), object_detector=NNObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS)) as robot:
    robot.map = [(200, 0)]
    robot.sampling_rate = 0.1
    print("These are the robot motor positions before planning:", robot.left_track.position, robot.right_track.position)
    # Define the state graph, we can do this better, currently each method
    # returns the next state name
    states = [
        State(
            name="SEARCH_TARGET",
            act=search_target_with_Kalman_and_mapping,
             default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "P" : np.identity(3),
                "delete_countdown" : 0,
                "mapa": [], 
                "robot_trajectory": []
            }
         ),
        # State(
        #     name="GO_TO_TARGET",
        #     act=euclidian_move_with_kalman_and_map,
        #     default_args={
        #         "ltrack_pos": robot.left_track.position,
        #         "rtrack_pos": robot.right_track.position,
        #         "TIME": time.time(),
        #         "P" : np.identity(3),
        #         "delete_countdown" : 0 ,
        #         "mapa": [], 
        #         "robot_trajectory": []
        #     }
        #     ),
         State(
            name="GO_TO_TARGET",
            act=move_to_brick_v3,
            default_args={
            "vel_forward" : 200,
            "vel_rot" : 60,
            "atol_move_blind" : 100
            }
            ),

        State(
             name="MOVE_TO_BRICK_BLIND_AND_GRIP",
             act=move_to_brick_blind_and_grip,
             default_args={"vel": 400,
                           "t" : 1200
                           }
         )
    ]
    print(states[0])
    state_dict = {}
    for state in states:
        state_dict[state.name] = state

    start_state = states[0]

    main_loop(robot, start_state, state_dict, delay=0)




