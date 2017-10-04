## PLACING WITH KALMAN




import time
import cv2
from ev3control.rpc import Robot
from rick.controllers import *
from rick.core import State
from rick.core import main_loop
from rick.async import AsyncCamera
from rick.utils import TrackerWrapper
from nn_object_detection.object_detectors import NNObjectDetector
from rick.live_plotting import MapRenderer

from detection.marker_localization import get_marker_pose, load_camera_params
import cv2.aruco as aruco


import numpy as np

from math import pi


from detection.opencv import get_lego_boxes



from rick.motion_control import euclidian_kalman , kalman_filter , robot_control, odom_estimation

import sys

sys.path.append("../slam/")

import mapping

import matplotlib.pyplot as plt

from detection.opencv import detect_purple

PATH_TO_CKPT = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/models/faster_rcnn_resnet_lego_v1/train/frozen_inference_graph.pb"
PATH_TO_LABELS = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/label_map.pbtxt"


print("Creating robot...")


data = np.load('Homographygood.npz')
H=data["arr_0"]

map_renderer = MapRenderer()


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

def index23(BB_legos,BB_target):
    index=1000
    i=0
    for box in BB_legos:
        if box[0]==BB_target[0][0] and box[1] == BB_target[0][1]:

            index = i
        i+=1
    return index



def search_target_with_Kalman_and_mapping(robot, frame
                            , ltrack_pos=0, rtrack_pos=0, P=np.identity(3), marker_list = [], delete_countdown = 0 , mapa = [], robot_trajectory = [],R=[],state_search = 2 , t1=0):

    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    ######################  Markers information coming from the compuetr vision stuff
    
    #frame,marker_list  = camera_related(frame = frame)
    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])


    ###################### Information related with lego blocks mapping

    BB_legos=get_lego_boxes(frame)

    #######################     DETECT IF ANY OF THE BOXES IS PURPLE

    BB_target = detect_purple(frame,BB_legos)

    index = 1000
    if len(BB_target) !=0:
        index = index23(BB_legos,  BB_target)


    lego_landmarks = mapping.cam2rob(BB_legos,H)


    
    
    print("####################################################################################")

    #################### WHAT SLAM IS!

    ######## 1. ESTIMATE POSITION BY ODOMETRY

    estim_rob_pos_odom = odom_estimation(odom_r,odom_l,robot.position)


    ####### 2. UPDATE THE MAP WITH ODOMETRY INFO
    mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,lego_landmarks,estim_rob_pos_odom,P,delete_countdown, robot_trajectory, index)


    ####### 3. KALMAN FILTER

    Ts = 0.3
    estim_rob_pos, P  = kalman_filter(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)

    robot.position = estim_rob_pos

    #print("rob_pos odom:", estim_rob_pos_odom, " rob_pos -Kalman", estim_rob_pos)


    ####### 4. UPDATE MAP POINTS RELATED TO KALMAN

    mapa = mapping.after_kalman_improvement(mapa, robot.position, estim_rob_pos_odom)

    print("EL MAPA : ", mapa)
    #### GET GRIPPER POS

    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 *np.cos(estim_rob_pos[2] * pi/180)
    d[1] = estim_rob_pos[1] + 28* np.sin(estim_rob_pos[2]*pi/180)
    d[2] = estim_rob_pos[2]

    R.append(d)


    map_renderer.plot_bricks_and_trajectory(mapa, R)


    ## DETECT THE BOX

    mtx,dist=load_camera_params()
    frame,box_coords = get_specific_marker_pose(frame=frame,mtx=mtx,dist=dist,marker_id=0)




    #DEFINE MOTION CONTROL FOR SEARCHING

    # THE CONTROL IS : 1. GO TO THE CENTER OF THE WORKSPACE, 2. ROUND FOR 2 secs ,  SELECT A POINT CLOSE TO THE CENTER as new target

    vel_wheels,state_search,t1 = search_control(state_search, mapa, robot.position, t1)

    
    
    if box_coords:

        return ("COMPUTE_PATH", frame, {"box_coords": box_coords, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "mapa": mapa, "R" : R })
    else:
        robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
        return "SEARCH_TARGET", frame, {"ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "P": P , "marker_list": [],
                                        "delete_countdown" : delete_countdown , "mapa": mapa, "robot_trajectory": robot_trajectory, "R" : R,
                                        "state_search" : 2, "t1" : t1 }

def compute_path(robot,frame,box_coords, ltrack_pos = 0, rtrack_pos = 0, mapa = [], R = []):
    x=box_coords[0]
    y=box_coords[1]
    yaw=box_coords[2]
    if (y>0 and yaw>-80) or (y<0 and yaw< -100):
        print("NICE PATH")
    thm=40
    thobj=10


    x2=x+thm*np.sin(yaw*np.pi/180.)
    y2=y-thm*np.cos(yaw*np.pi/180.)
    yaw2=0
    xobj=x+thobj*np.sin(yaw*np.pi/180.)
    yobj=y-thobj*np.cos(yaw*np.pi/180.)

    obj=[x,y]
    obslist=[[40,0],[40,10],[40,-10],[40,20],[40,-20],[40,30],[40,40],[40,50],[40,60]]
    Map=create_map(obslist)
    path=A_star([0,0],obj, Map)
    plt.plot(path[:,0],path[:,1])
    plt.axis([-100, 150, -100, 150])
    plt.show()
    return ("MOVE_TO_BOX",frame, {"Map": Map, "obj":obj,  "ltrack_pos": ltrack_pos,
                "rtrack_pos": rtrack_pos,
                "TIME": time.time()})


def A_star_move_to_box_blind(robot, frame, Map,obj, replan=1,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0, P = np.identity(3)):
    mtx,dist=load_camera_params()
    frame,box_coords = get_specific_marker_pose(frame=frame,mtx=mtx,dist=dist,marker_id=0)
    old_path=path
    #REPLANNING

    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    
    marker_list = []
    if box_coords:
        print("REPLANNNIG")
        x=box_coords[0]
        y=box_coords[1]
        yaw=box_coords[2]
        thobj=10
        xobj=x+thobj*np.sin(yaw*np.pi/180.)
        yobj=y-thobj*np.cos(yaw*np.pi/180.)
        obj=[xobj+robot.position[0],yobj+robot.position[1]]
        marker_list.append(obj)


    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])

    Ts = 0.3
    estim_rob_pos, P  = kalman_filter(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)

    robot.position= estim_rob_pos


    #update map
    path=A_star(robot.position[0:2], marker_map[0], Map)
    
    replan=1
    goal_pos=marker_map[0]

    t0 = time.time()

    estim_rob_pos, vel_wheels, new_path = A_star_path_planning_control(robot.position,goal_pos,
                                                                          Map, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path)
    
    

    #print("DIFFERENTCE WITH THE GOAL:",abs(estim_rob_pos[0]-goal_pos[0]),abs(estim_rob_pos[1]-goal_pos[1]))
    
    #CONDITION FOR EXITTING
    if abs(estim_rob_pos[0]-goal_pos[0])<40 and abs(estim_rob_pos[1]-goal_pos[1])<20:
        return ("MOVE_TO_BOX_BY_VISION", frame, {})

    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    iteration += 1
   
    return "MOVE_TO_BOX", frame, {"replan":replan,"Map":Map,"obj":goal_pos,"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0}

