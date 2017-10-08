import time
import cv2
from ev3control.rpc import Robot
from rick.controllers import *
from rick.A_star_planning import *
from rick.core import State
from rick.core import main_loop
from rick.async import AsyncCamera
from rick.utils import TrackerWrapper,plot_bbox
from nn_object_detection.object_detectors import NNObjectDetector
from rick.live_plotting import MapRenderer

from detection.marker_localization import get_marker_pose, load_camera_params
import cv2.aruco as aruco



from detection.marker_localization import get_specific_marker_pose, load_camera_params
import numpy as np
from rick.mc_please_github_donot_fuck_with_this_ones import A_star_path_planning_control,compute_A_star_path, A_star_control
from math import pi


from detection.opencv import get_lego_boxes,BB_ligth_green
from rick.motion_control import euclidian_kalman , kalman_filter , kalman_filter2 , robot_control, odom_estimation

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
        plt.show(block=False)
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

        vel_wheels = [-100,100]

    return vel_wheels,state_search,t1

def index23(BB_legos,BB_target):
    index=1000
    i=0
    for box in BB_legos:
        if box[0]==BB_target[0][0] and box[1] == BB_target[0][1]:

            index = i
        i+=1
    return index

# LOOKING FOR THE TARGET
def search_target_with_Kalman_and_mapping(robot, frame
                            , ltrack_pos=0, rtrack_pos=0, P=np.identity(3), marker_list = [], delete_countdown = 0 , mapa = [], robot_trajectory = [],R=[],state_search = 2 , t1=0):
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    ######################  Markers information coming from the compuetr vision stuff

    ###################### Information related with lego blocks mapping
    BB_target2=BB_ligth_green(frame)
    if BB_target2:
        BB_target=BB_target2[0]
        plot_bbox(frame, BB_target)
    else:
        BB_target=[]
 

    delete_countdown=0
    #GET LIST OF LEGO LANDMARKS
    #lego_landmarks = mapping.cam2rob(BB_legos,H)
    print("####################################################################################")
    #################### WHAT SLAM IS!

    ######## 1. ESTIMATE POSITION BY ODOMETRY
    estim_rob_pos_odom = odom_estimation(odom_r,odom_l,robot.position)
    ####### 2. UPDATE THE MAP WITH ODOMETRY INFO
    #mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,lego_landmarks,estim_rob_pos_odom,P,delete_countdown, robot_trajectory, index)
    mapa=[]
    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])

    ####### 3. KALMAN FILTER
    Ts = 0.3
    estim_rob_pos, P  = kalman_filter(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)
    robot.position = estim_rob_pos
    #print("rob_pos odom:", estim_rob_pos_odom, " rob_pos -Kalman", estim_rob_pos)
    ####### 4. UPDATE MAP POINTS RELATED TO KALMAN
    mapa = mapping.after_kalman_improvement(mapa, robot.position, estim_rob_pos_odom)

    #### GET GRIPPER POS

    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 *np.cos(estim_rob_pos[2] * pi/180)
    d[1] = estim_rob_pos[1] + 28* np.sin(estim_rob_pos[2]*pi/180)
    d[2] = estim_rob_pos[2]
    R.append(d)
    #SHOW THE MAP
    map_renderer.plot_bricks_and_trajectory(mapa, R)

    #DEFINE MOTION CONTROL FOR SEARCHING

    # THE CONTROL IS : 1. GO TO THE CENTER OF THE WORKSPACE, 2. ROUND FOR 2 secs ,  SELECT A POINT CLOSE TO THE CENTER as new target

    vel_wheels,state_search,t1 = search_control(state_search, mapa, robot.position, t1)

    if len(BB_target) > 0:
        robot.tracker.init(frame, BB_target)
        return ("GO_TO_TARGET", frame, {"tracker" : robot.tracker, "ltrack_pos" : robot.left_track.position ,"rtrack_pos" : robot.right_track.position, "robot_trajectory": robot_trajectory,"pos_rob" : robot.position,"R" :  R, "mapa" : mapa})
    else:
        robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
        return "SEARCH_TARGET", frame, {"ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "P": P , "marker_list": [],
                                        "delete_countdown" : delete_countdown , "mapa": mapa, "robot_trajectory": robot_trajectory, "R" : R,
                                        "state_search" : 2, "t1" : t1 }


#GO TO TARGET
def move_to_brick_v3(robot, frame, img_res=np.asarray((640, 480)), atol=10,
                         vel_forward = 200, vel_rot = 50, atol_move_blind=30, 
                         fail_counter=0, center_position_error = 25, tracker=None,ltrack_pos=0 ,rtrack_pos=0, pos_rob=[],marker_list=[],P = np.identity(3),R = [], mapa = [], robot_trajectory = []):
    
    #LOCALIZATION AND MAPPING
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    ######################  Markers information coming from the compuetr vision stuff
    #################3 RELATED WITH LEGOS
    BB_target2=BB_ligth_green(frame)
    if BB_target2:
        BB_target=BB_target2[0]
        plot_bbox(frame, BB_target)
    else:
        BB_target=[]

    print("####################################################################################")

    #################### WHAT SLAM IS!

    ######## 1. ESTIMATE POSITION BY ODOMETRY
    delete_countdown=0
    estim_rob_pos_odom = odom_estimation(odom_r,odom_l,robot.position)
    ####### 2. UPDATE THE MAP WITH ODOMETRY INFO
    #mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,lego_landmarks,estim_rob_pos_odom,P,delete_countdown, robot_trajectory, index)
    ####### 3. KALMAN FILTER
    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])
    Ts = 0.3
    estim_rob_pos, P  = kalman_filter(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)
    robot.position = estim_rob_pos
    print("rob_pos odom:", estim_rob_pos_odom, " rob_pos -Kalman", estim_rob_pos)
    ####### 4. UPDATE MAP POINTS RELATED TO KALMAN
    #mapa = mapping.after_kalman_improvement(mapa, robot.position, estim_rob_pos_odom)
    mapa=[]
    #### GET GRIPPER POS

    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 *np.cos(estim_rob_pos[2] * pi/180)
    d[1] = estim_rob_pos[1] + 28* np.sin(estim_rob_pos[2]*pi/180)
    d[2] = estim_rob_pos[2]

    R.append(d)


    map_renderer.plot_bricks_and_trajectory(mapa, R)

    ###################### Information related with lego blocks mapping

    ok, bbox = tracker.update(frame)

    if not ok:
        BB_legos=get_lego_boxes(frame)
        # res = robot.object_detector.detect_with_threshold(frame,threshold=0.9, return_closest=False)
        # BB_legos = map(lambda x: x[0], res)
        BB_target = detect_purple(frame,BB_legos)
        if len(BB_target) == 0:
            return "SEARCH_TARGET", frame, {"ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "P": P , "marker_list": [],
                                        "delete_countdown" : delete_countdown , "mapa": mapa, "robot_trajectory": robot_trajectory, "R" : R,
                                        "state_search" : 2}
        tracker.init(frame, BB_target[0])        
        bbox = BB_target[0]

    #MOVE TO THE TARGET


    coords = bbox_center(*bbox)

    img_center = img_res / 2 - center_position_error 
    error = img_center - coords
    atol = 5 + coords[1]/480 * 25
    cv2.line(frame, (5+int(img_center[0]),0), (25+int(img_center[0]),480), (255,0,0))
    cv2.line(frame, (-5+int(img_center[0]),0), (-25+int(img_center[0]),480), (255,0,0))
    cv2.line(frame, (int(img_center[0]),0), (int(img_center[0]),480), (0,255,0))
    cv2.line(frame, (0,480-atol_move_blind),(640,480-atol_move_blind), (0,0,255))
    frame = plot_bbox(frame,bbox, 0, (255,0,0))
    #img_center = img_res/2.

    if np.isclose(coords[0], img_center[0], atol=atol) and np.isclose(coords[1], img_res[1], atol=atol_move_blind):
        robot.move_straight(vel_forward, 500)
        return "MOVE_TO_BRICK_BLIND_AND_GRIP", frame, {"R":R,"ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos,"mapa":mapa}

    if np.isclose(coords[0], img_center[0], atol=atol):
        print("Move straight")
        robot.move_straight(vel_forward)
        return "GO_TO_TARGET", frame, {"tracker" : tracker, "ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos, "pos_rob" : robot.position,"R" :  R, "mapa" : mapa}
    elif error[0] < 0:
        robot.rotate_left(vel=vel_rot)
        return "GO_TO_TARGET", frame, {"tracker" : tracker, "ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos, "pos_rob" : robot.position,"R" :  R, "mapa" : mapa}
    else:
        # Positive velocity for turning left
        robot.rotate_right(vel=vel_rot)
        return "GO_TO_TARGET", frame, {"tracker" : tracker, "ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos, "pos_rob" : robot.position,"R" :  R, "mapa" : mapa}
    
       

#MOVE TO BRICK BLIND
def move_to_brick_blind_and_grip(robot, frame, R=[],ltrack_pos=0 ,rtrack_pos=0,marker_list=[],mapa=[], vel=500, t=1500):
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

    #odometry update
    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])
    P = np.identity(3)
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    Ts = 0.3
    estim_rob_pos, P  = kalman_filter(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)
    robot.position = estim_rob_pos

    estim_rob_pos_odom = odom_estimation(odom_r,odom_l,robot.position)
    return "SEARCH_BOX", frame, {"ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos
                                         , "mapa": mapa,  "R" : R}
#SEARCH BOX
def search_box(robot, frame, ltrack_pos=0, rtrack_pos=0, P=np.identity(3), marker_list = [], delete_countdown = 0 , mapa = [], robot_trajectory = [],R=[],state_search = 2 , t1=0):

    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos



    estim_rob_pos_odom = odom_estimation(odom_r,odom_l,robot.position)
    marker_map = np.array([[150,0,0]])
    marker_map_obj = np.array([[110,0,0]])
    
    Ts = 0.3
    estim_rob_pos, P  = kalman_filter(odom_r,odom_l,robot.position,marker_list, marker_map_obj,Ts,P)

    robot.position = estim_rob_pos
   

    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 *np.cos(estim_rob_pos[2] * pi/180)
    d[1] = estim_rob_pos[1] + 28* np.sin(estim_rob_pos[2]*pi/180)
    d[2] = estim_rob_pos[2]

    R.append(d)


    map_renderer.plot_bricks_and_trajectory(mapa, R)


   

    mtx,dist=load_camera_params()
    frame,box_coords = get_specific_marker_pose(frame=frame,mtx=mtx,dist=dist,marker_id=0,markerLength=8.6)

    
    vel_wheels,state_search,t1 = search_control(state_search, mapa, robot.position, t1)

    if box_coords:

        return ("COMPUTE_PATH", frame, {"box_coords": box_coords, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "mapa": mapa, "R" : R })
    else:
        robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
        return "SEARCH_BOX", frame, {"ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "P": P , "marker_list": [],
                                        "delete_countdown" : delete_countdown , "mapa": mapa, "robot_trajectory": robot_trajectory, "R" : R,
                                        "state_search" : 2, "t1" : t1 }

def compute_path(robot,frame,box_coords, ltrack_pos = 0, rtrack_pos = 0, mapa = [], R = []):
    x=box_coords[0]
    y=box_coords[1]
    yaw=box_coords[2]
    if (y>0 and yaw>-80) or (y<0 and yaw< -100):
        print("NICE PATH")
    thm=40
    thobj=40


    x2=x+thm*np.sin(yaw*np.pi/180.)
    y2=y-thm*np.cos(yaw*np.pi/180.)
    yaw2=0
    xobj=x+thobj*np.sin(yaw*np.pi/180.)
    yobj=y-thobj*np.cos(yaw*np.pi/180.)

    obj=[x,y]
    obslist=[]
    Map=create_map(obslist)
    path=A_star([0,0],obj, Map)
    robot.grip.close()
    R=R
    
    return ("MOVE_TO_BOX",frame, {"Map": Map, "obj":obj,  "ltrack_pos": ltrack_pos,
                "rtrack_pos": rtrack_pos,
                "TIME": time.time(), "R":R})


def A_star_move_to_box_blind(robot, frame, Map,obj=[100,0,0], replan=1,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0, P = np.identity(3),R=[]):
    mtx,dist=load_camera_params()
    frame,box_coords = get_specific_marker_pose(frame=frame,mtx=mtx,dist=dist,marker_id=0,markerLength=8.6)
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
        thobj=40
        xobj=x+thobj*np.sin(yaw*np.pi/180.)
        yobj=y-thobj*np.cos(yaw*np.pi/180.)
        obj=[xobj+robot.position[0],yobj+robot.position[1]]

        angle = np.arctan2(yobj,xobj)
        distance = np.sqrt(np.power(xobj,2) + np.power(yobj,2))
        marker_list.append([angle,distance,(yaw+90)*pi/180])
        print("MARKER POSITION X AND Y: ", x , y)

    
    marker_map = np.array([[100,0,0]])
    marker_map_obj = np.array([[60,0,0]])

    Ts = 0.3
    estim_rob_pos, P  = kalman_filter2(odom_r,odom_l,robot.position,marker_list, marker_map_obj,Ts,P)


    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 *np.cos(estim_rob_pos[2] * pi/180)
    d[1] = estim_rob_pos[1] + 28* np.sin(estim_rob_pos[2]*pi/180)
    d[2] = estim_rob_pos[2]

    R.append(d)
    mapa=[]
    map_renderer.plot_bricks_and_trajectory(mapa, R)
    robot.position= estim_rob_pos
    print("robot_estim_pos_Astar: ", robot.position)

    #update map
    #plt.close()
    #fig = plt.figure()
    #ax = fig.gca()
    #ax.set_xticks(np.arange(-10, 150, 1))
    #ax.set_yticks(np.arange(-40, 40, 1))
    #path=A_star(robot.position[0:2], marker_map_obj[0,0:2], Map)
    #plt.plot(path[:,0],path[:,1])
    #plt.show(block=False)

    #print("PATH FROM THE REPLANNING",path.shape)

    replan=1
    goal_pos=marker_map_obj[0,:]
    t0 = time.time()

    vel_wheels, new_path = A_star_control(robot.position,goal_pos,
                                        Map, robot.sampling_rate,
                                             odom_r= odom_r,odom_l=odom_l,
                                        iteration=iteration, path=path)
    
    

    #print("DIFFERENTCE WITH THE GOAL:",abs(estim_rob_pos[0]-goal_pos[0]),abs(estim_rob_pos[1]-goal_pos[1]))
    
    #CONDITION FOR EXITTING

    distance_to_target = np.sqrt(np.power(estim_rob_pos[0]-marker_map_obj[0,0],2)+ np.power(estim_rob_pos[1]-marker_map_obj[0,1],2))

    print("###########################################################################################################")
    print("disatnce to target: ", distance_to_target)
    print("estimated vs goal", estim_rob_pos[0:2],goal_pos)
    print("###########################################################################################################")
    
    if abs(estim_rob_pos[0]-marker_map_obj[0,0]) < 20 and abs(estim_rob_pos[1]-marker_map_obj[0,1]) < 10:
        return ("MOVE_TO_BOX_BY_VISION", frame, {"replan":replan,"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0})

    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    iteration += 1
   
    return ("MOVE_TO_BOX", frame, {"replan":replan,"Map":Map,"obj":goal_pos,"iteration" : iteration, "path" : new_path,
             "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0,"R":R})

def PID_control(robot, marker_map, box_coords,hist):
    vel_st=100
    vel_rot=40
    lat_tol=4
    vel_st2=100
    yshift=2
    er_x = marker_map[0,0] - robot[0]
    er_y = marker_map[0,1] - robot[1]
    er_angle = np.arctan2(er_y, er_x) - robot[2]*pi/180
    print("ANGLES WITH MARKER,WITH WORLD AND ERROR",np.arctan2(er_y, er_x)*180/pi,robot[2],er_angle*180/pi)

    if er_angle > pi:
        er_angle = er_angle - 2*pi
    if er_angle < -pi:
        er_angle = er_angle + 2*pi

    distance = np.sqrt(np.power(er_x,2)+np.power(er_y,2))

    if box_coords:
        print("Y_DISTANCE_TO_MARKER",box_coords[1])
        if abs(box_coords[1]+yshift)>lat_tol and box_coords[0]>35:
            vel_wheels=np.asarray([-vel_rot,vel_rot])*np.sign(-box_coords[1]-yshift)+np.asarray([vel_st2,vel_st2])
            print("Case 1")
        if abs(box_coords[1]+yshift)>lat_tol and box_coords[0]<35:
            vel_wheels=np.asarray([-vel_rot,vel_rot])*np.sign(-box_coords[1]-yshift)
            print("Case 2")
        elif box_coords[0]>35:
            vel_wheels=np.asarray([vel_st,vel_st])
            print("Case3")
        else:
            vel_wheels=np.asarray([0,0])
            hist = 0
            print("STOP")


    else:
        if hist == 0:
            vel_wheels=np.asarray([0,0])

        elif hist==2:
            vel_wheels=np.asarray([vel_rot,-vel_rot])*np.sign(er_angle)
            hist=np.sign(er_angle)
            print("first hist",hist)
        elif er_angle > 0.7:
            vel_wheels=np.asarray([vel_rot,-vel_rot])
            hist = 1
        elif er_angle <-0.7:
            vel_wheels=np.asarray([-vel_rot,vel_rot])
            hist = -1
        elif hist ==1 : 
            vel_wheels=np.asarray([vel_rot,-vel_rot])
        elif hist==-1 : 
            vel_wheels=np.asarray([-vel_rot,vel_rot])

        print("CORRECTING ANGLE and HISTERESIS",er_angle,hist)
    return vel_wheels, hist






def move_to_box_by_vision(robot, frame, replan=1,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0, P = np.identity(3),
                            histeresis = 2):
    mtx,dist=load_camera_params()
    frame,box_coords = get_specific_marker_pose(frame=frame,mtx=mtx,dist=dist,marker_id=0,markerLength=8.6)

    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    


    marker_list = []
    if box_coords:
        x=box_coords[0]
        y=box_coords[1]
        yaw=box_coords[2]
        thobj=40
        xobj=x+thobj*np.sin(yaw*np.pi/180.)
        yobj=y-thobj*np.cos(yaw*np.pi/180.)
        obj=[xobj+robot.position[0],yobj+robot.position[1]]
        angle = np.arctan2(yobj,xobj)
        distance = np.sqrt(np.power(xobj,2) + np.power(yobj,2))
        marker_list.append([angle,distance,(yaw+90)*pi/180])
        print("MARKER POSITION X AND Y: ", x , y)



    marker_map = np.array([[100,0,0]])
    marker_map_obj = np.array([[60,0,0]])

    Ts = 0.3
    estim_rob_pos, P  = kalman_filter2(odom_r,odom_l,robot.position,marker_list, marker_map_obj,Ts,P)

    robot.position= estim_rob_pos
    print("robot_estim_pos: ", robot.position)

    
    vel_wheels, hist = PID_control(estim_rob_pos, marker_map,box_coords, histeresis)
    if hist==0:
        return "PLACE_OBJECT_IN_THE_BOX",frame,{}
    
    robot.move(vel_wheels[0],vel_wheels[1])
    return ("MOVE_TO_BOX_BY_VISION", frame, {"ltrack_pos": new_ltrack_pos, "rtrack_pos" : new_rtrack_pos, "histeresis" : hist})

def place_object_in_the_box(robot,frame):
    robot.move(vel_left=100,vel_right=100,time=2000)
    print("MOVING")
    robot.left_track.wait_until_not_moving(timeout=3000)
    robot.reset()
    robot.grip.wait_until_not_moving(timeout=3000)
    print("finish")
    return "FINAL_STATE"

with Robot(AsyncCamera(1), tracker=TrackerWrapper(cv2.TrackerKCF_create), object_detector=None ) as robot:
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
             default_args={"vel": 250,
                           "t" : 1200,
                        "ltrack_pos": robot.left_track.position,
                        "rtrack_pos": robot.right_track.position,
                           }
        ),
        State(
            name="SEARCH_BOX",
            act=search_box,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
            }
        ),
        State(
             name="COMPUTE_PATH",
             act=compute_path,
             default_args={"box_coords": [100,0,0],
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                }
         ),
         State(
             name="MOVE_TO_BOX",
             act=A_star_move_to_box_blind,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time()
            }
         ),
        State(
             name="MOVE_TO_BOX_BY_VISION",
             act=move_to_box_by_vision,
             default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time()
            }
         ),
        State(
             name="PLACE_OBJECT_IN_THE_BOX",
             act=place_object_in_the_box,
         ),
        State(
            name="FINAL_STATE",
            act=lambda robot, frame, **args: time.sleep(.5)
        )
    ]
    state_dict = {}
    for state in states:
        state_dict[state.name] = state

    start_state = states[0]

    main_loop(robot, start_state, state_dict, delay=0)



