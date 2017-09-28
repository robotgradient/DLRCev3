#Euclidean path planning control with Kalman filter for localization

import time
import cv2
from ev3control.rpc import Robot
from rick.controllers import euclidian_move_to_brick, rotation_search_brick,move_to_brick_simple, move_to_brick_blind_and_grip
from rick.core import State
from rick.core import main_loop
from rick.async import AsyncCamera


from detection.marker_localization import get_marker_pose, load_camera_params
import cv2.aruco as aruco


import numpy as np


from detection.opencv import get_lego_boxes



from rick.motion_control import euclidian_kalman

import sys

sys.path.append("../slam/")

import mapping

import matplotlib.pyplot as plt

print("Creating robot...")


data = np.load('Homography.npz')
H=data["arr_0"]

def plot_mapa(mapa,robot_traj):


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


def euclidian_move_with_kalman_and_map(robot, frame,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0, P=np.identity(3), marker_list = [], delete_countdown =0 , mapa = [], robot_trajectory = [],R=[] ):

    img_res = np.asarray((640,480))


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
    
    frame,marker_list  = camera_related(frame = frame)
    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])

    #print("marker_list1: ",marker_list)
    estim_rob_pos, vel_wheels, new_path , P ,  = euclidian_kalman(robot.position,
                                                                          brick_position, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path,
                                                                          P=P, marker_list = marker_list ,marker_map=marker_map)

    print("estimated position: ",estim_rob_pos)

    print("{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{")




    robot.position = estim_rob_pos


    # Information related with lego blocks mapping

    BB_legos=get_lego_boxes(frame)

    lego_landmarks = mapping.cam2rob(BB_legos,H)
    #       UPDATE MAP

    mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,lego_landmarks,robot.position,P,delete_countdown, robot_trajectory)

    print(mapa)



    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    iteration += 1

    R.append(estim_rob_pos)
    plot_mapa(mapa,R)

    

    # print("Path: ", pat, iteration)
    # print("Robot positij "ffefrobot.position)
    # print("Velocities rl: ", vel_wheels)
    # print("##" *20)


    return "GO_TO_TARGET", frame, {"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0 , "P": P , "marker_list": marker_list,
                                    "delete_countdown" : delete_countdown , "mapa": mapa, "robot_trajectory": robot_trajectory, "R" : R}

def camera_related(frame):

	arucoParams = aruco.DetectorParameters_create()
	mtx,dist = load_camera_params()
	image,marker_pos  = get_marker_pose(frame, mtx, dist,arucoParams=arucoParams, marker_list=[0,1,2,3,4,5], markerLength = 3.3)


	#print("Output marco function:",marker_pos)

	return image,marker_pos


def search_target_with_Kalman_and_mapping(robot, frame,
                                        path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0, P=np.identity(3), marker_list = [], delete_countdown =0 , mapa = [], robot_trajectory = [],R=[]):


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
    
    frame,marker_list  = camera_related(frame = frame)
    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])

    #print("marker_list1: ",marker_list)
    estim_rob_pos, vel_wheels, new_path , P ,  = euclidian_kalman(robot.position,
                                                                          brick_position, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path,
                                                                          P=P, marker_list = marker_list ,marker_map=marker_map)

    

    print("estimated position: ",estim_rob_pos)

    print("{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{")




    robot.position = estim_rob_pos


    # Information related with lego blocks mapping

    BB_legos=get_lego_boxes(frame)

    lego_landmarks = mapping.cam2rob(BB_legos,H)
    #       UPDATE MAP

    mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,lego_landmarks,robot.position,P,delete_countdown, robot_trajectory)

    print(mapa)



    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    iteration += 1

    R.append(estim_rob_pos)
    plot_mapa(mapa,R)



with Robot(AsyncCamera(1)) as robot:
    robot.map = [(200, 100)]
    robot.sampling_rate = 0.1
    print("These are the robot motor positions before planning:", robot.left_track.position, robot.right_track.position)
    # Define the state graph, we can do this better, currently each method
    # returns the next state name
    states = [
        State(
            name="GO_TO_TARGET",
            act=euclidian_move_with_kalman_and_map,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time(),
                "P" : np.identity(3),
                "delete_countdown" : 0 ,
                "mapa": [], 
                "robot_trajectory": []
            }
            ),
        State(
             name="MOVE_TO_BRICK",
             act=move_to_brick_simple,
             default_args={"atol": 30,
                           "atol_move_blind" : 30,
                           }
         ),
        State(
             name="SEARCH_TARGET",
             act=search_target_with_Kalman_and_mapping
         ),
        State(
            name="MOVE_TO_BRICK_BLIND_AND_GRIP",
            act=move_to_brick_blind_and_grip,
            default_args={}
        ),
        State(
            name="FINAL_STATE",
            act=lambda robot, frame, **args: time.sleep(.5)
        )
    ]
    print(states[0])
    state_dict = {}
    for state in states:
        state_dict[state.name] = state

    start_state = states[0]

    main_loop(robot, start_state, state_dict, delay=0.05)




