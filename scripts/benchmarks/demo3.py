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
from rick.live_plotting import MapRenderer

from detection.marker_localization import get_marker_pose, load_camera_params
import cv2.aruco as aruco
from dlrc_one_shot_learning.similarity_detectors import EuclidianNNFeaturesBrickFinder


import numpy as np

from math import pi


from detection.opencv import get_lego_boxes



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


similarity_detector = EuclidianNNFeaturesBrickFinder()


def acquire_target(robot, frame, **kwargs):
    """Callback for acquiring a lego target."""
    BB_legos = get_lego_boxes(frame)

    # We wait until there's only one lego in view
    if len(BB_legos) == 1:
        print("found a brick")
        bboxes = [frame[bbox[0]:bbox[2], bbox[1]:bbox[3]] for bbox in BB_legos]
        robot.target = bounding_box_features = similarity_detector.extract_features(bboxes)[0]
        return "SEARCH_TARGET", frame, {}
    else:
        print(len(BB_legos))
        return "ACQUIRE_TARGET", frame, {}


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

        vel_wheels = [100,100]

    return vel_wheels,state_search,t1

def naive_obstacle_avoidance_control(mapa, pos_rob): 


    max_dist = 40
    min_angle = -pi/2
    max_angle = pi/2
    mapa_ar = np.array(mapa)
    

    vel_wheels = [160,150]


    for i in range(0, len(mapa)):

        er_x = mapa[i][0] - pos_rob[0]
        er_y = mapa[i][1] - pos_rob[1]

        distance = np.sqrt(np.power(er_x,2) + np.power(er_y,2))

        er_ angle = np.arctan2(er_y, er_x) - pos_rob[2]*pi/180

        if er_angle >pi:
            er_angle = er_angle -2*pi
        if er_angle < -pi:
            er_angle = er_angle +2*pi

        next_x = pos_rob[0] + 10*np.cos(pos_rob[3] * pi/180)
        next_y = pos_rob[1] + 10*np.sin(pos_rob[3] * pi/180)

        if (distance< max_dist and angle > min_angle and angle< max_angle): # AVOID OBSTACLES
            vel_wheels = [-100,100]
            break
        elif next_x < 0 or next_x > 300 or next_y < 0 or next_y> 300:
            vel_wheels = [-100,100]

    return vel_wheels

    
    




def index23(BB_legos,BB_target):
    index=1000
    i=0
    for box in BB_legos:
        if box[0]==BB_target[0][0] and box[1] == BB_target[0][1]:

            index = i
        i+=1
    return index


def wait_for_target(robot, frame):

    pass




def search_target_with_Kalman_and_mapping(robot, frame
                            , ltrack_pos=0, rtrack_pos=0, P=np.identity(3), marker_list = [], delete_countdown = 0 , mapa = [], robot_trajectory = [],R=[],state_search = 2 , t1=0, 
                            iteration = 0,feature_map = []):
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    ######################  Markers information coming from the compuetr vision stuff
    
    #frame,marker_list  = camera_related(frame = frame)
    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])


    ###################### Information related with lego blocks mapping

    BB_legos=get_lego_boxes(frame)
    BB_legos2 = []

    for bbox in BB_legos:

        if np.abs(bbox[0] - bbox[2]) >2 and np.abs(bbox[1] - bbox[3]) >2 and iteration>5:
            BB_legos2.append(bbox)



    lego_landmarks = mapping.cam2rob(BB_legos2,H)


    
    
    print("####################################################################################")

    #################### WHAT SLAM IS!

    ######## 1. ESTIMATE POSITION BY ODOMETRY

    estim_rob_pos_odom = odom_estimation(odom_r,odom_l,robot.position)


    ####### 2. UPDATE THE MAP WITH ODOMETRY INFO
    #mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,lego_landmarks,estim_rob_pos_odom,P,delete_countdown, robot_trajectory, index)

    index = 1000
    mapa, delete_countdown,robot_trajectory, links = mapping.update_mapa2(mapa,lego_landmarks,estim_rob_pos_odom,P,delete_countdown, robot_trajectory, index)
    



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


    map_renderer.plot_bricks_and_trajectory(mapa, R)



    #Feature extraction from bounding boxes
    bboxes = []

    for i in range(0,len(links)):
        bbox = BB_legos2[links[i][0]]
        #print(bbox)
        bboxes.append(frame[bbox[1]:bbox[3], bbox[0]:bbox[2],:])
    bounding_box_features = similarity_detector.extract_features(bboxes)

    #print(links)

    #print(len(bounding_box_features))
    print(len(links))
    #print(len(BB_legos2))
    for i in range(0,len(links)):

        feature_map[links[i][1]] = bounding_box_features[i]


    print("EL MAPA : ", mapa)
    print("FEATURE MAP : ", feature_map)




    #DEFINE MOTION CONTROL FOR SEARCHING

    # THE CONTROL IS : 1. GO TO THE CENTER OF THE WORKSPACE, 2. ROUND FOR 2 secs ,  SELECT A POINT CLOSE TO THE CENTER as new target

    vel_wheels,state_search,t1 = search_control(state_search, mapa, robot.position, t1)

    
    iteration += 1 
    if iteration == 100 :

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

        
        return "GO_TO_TARGET", frame, { "ltrack_pos" : robot.left_track.position ,"rtrack_pos" : robot.right_track.position,
                                         "R" :  R, "obj" : target_point, "mapa" : mapa}
    else:
        robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
        return "SEARCH_TARGET", frame, {"ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "P": P , "marker_list": [],
                                        "delete_countdown" : delete_countdown , "mapa": mapa, "robot_trajectory": robot_trajectory, "R" : R,
                                        "state_search" : 2, "t1" : t1, "iteration" : iteration, "feature_map":feature_map }

def A_star_move_to_box_blind(robot, frame, Map,obj, replan=1,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0, P = np.identity(3),R=[], mapa = []):


    ################## RELATED WITH DETECTING THE BOX 
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

    
    marker_map = np.array([[150,0,0]])
    marker_map_obj = np.array([[110,0,0]])

    print("####################################################################################")

    ################## DETECT THE LEGO BOXES ##############################################

    BB_legos=get_lego_boxes(frame)

    lego_landmarks = mapping.cam2rob(BB_legos,H)

    #################### WHAT SLAM IS!

    ######## 1. ESTIMATE POSITION BY ODOMETRY

    estim_rob_pos_odom = odom_estimation(odom_r,odom_l,robot.position)


    ####### 2. UPDATE THE MAP WITH ODOMETRY INFO
    #mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,lego_landmarks,estim_rob_pos_odom,P,delete_countdown, robot_trajectory, index)

    mapa, delete_countdown,robot_trajectory, links = mapping.update_mapa2(mapa,lego_landmarks,estim_rob_pos_odom,P,delete_countdown, robot_trajectory, index)
    

    ####### 3. KALMAN FILTER

    Ts = 0.3
    estim_rob_pos, P  = kalman_filter2(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)

    robot.position = estim_rob_pos

    #print("rob_pos odom:", estim_rob_pos_odom, " rob_pos -Kalman", estim_rob_pos)


    ####### 4. UPDATE MAP POINTS RELATED TO KALMAN

    mapa = mapping.after_kalman_improvement(mapa, robot.position, estim_rob_pos_odom)

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
    path=A_star(robot.position[0:2], obj, Map)
    
    replan=1
    goal_pos=obj

    t0 = time.time()

    vel_wheels, new_path = A_star_control(robot.position,goal_pos,
                                        Map, robot.sampling_rate,
                                             odom_r= odom_r,odom_l=odom_l,
                                        iteration=iteration, path=path)
    
    

    #print("DIFFERENTCE WITH THE GOAL:",abs(estim_rob_pos[0]-goal_pos[0]),abs(estim_rob_pos[1]-goal_pos[1]))
    
    #CONDITION FOR EXITTING

    distance_to_target = np.sqrt(np.power(estim_rob_pos[0]-marker_map_obj[0,0],2)+ np.power(estim_rob_pos[1]-marker_map_obj[0,1],2))

    print("###########################################################################################################")
    print("distance to target: ", distance_to_target)
    print("estimated vs goal", estim_rob_pos[0:2],goal_pos)
    print("###########################################################################################################")
    
    if distance_to_target < 20:

        






        
        return ("MOVE_TO_BOX_BY_VISION", frame, {"tracker" : tracker, "ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos, "pos_rob" : robot.position,"R" :  R, "mapa" : mapa})

    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    iteration += 1
   
    return ("GO_TO_TARGET", frame, {"replan":replan,"Map":Map,"obj":goal_pos,"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, 
                "rtrack_pos": new_rtrack_pos, "TIME": t0,"R":R, "mapa" : mapa})


def move_to_brick_v3(robot, frame, img_res=np.asarray((640, 480)), atol=5,
                         vel_forward = 299, vel_rot = 50, atol_move_blind=90, 
                         fail_counter=0, center_position_error = 10, tracker=None,ltrack_pos=0 ,rtrack_pos=0, pos_rob=[],marker_list=[],P = np.identity(3),R = [], mapa = [], robot_trajectory = []):
    

    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    ######################  Markers information coming from the compuetr vision stuff
    
    #frame,marker_list  = camera_related(frame = frame)
    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])


    #################3 RELATED WITH LEGOS
    BB_legos=get_lego_boxes(frame)

    #######################     DETECT IF ANY OF THE BOXES IS PURPLE

    BB_target = detect_purple(frame,BB_legos)

    index = 1000
    if len(BB_target) !=0:
        index = index23(BB_legos,  BB_target)

    

    lego_landmarks = mapping.cam2rob(BB_legos,H)

    delete_countdown = 0
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

    print("rob_pos odom:", estim_rob_pos_odom, " rob_pos -Kalman", estim_rob_pos)

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
        return "GO_TO_TARGET", frame, {"tracker" : tracker, "ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos, "pos_rob" : robot.position,"R" :  R, "mapa" : mapa}
    elif error[0] < 0:
        robot.rotate_left(vel=vel_rot)
        return "GO_TO_TARGET", frame, {"tracker" : tracker, "ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos, "pos_rob" : robot.position,"R" :  R, "mapa" : mapa}
    else:
        # Positive velocity for turning left
        robot.rotate_right(vel=vel_rot)
        return "GO_TO_TARGET", frame, {"tracker" : tracker, "ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos, "pos_rob" : robot.position,"R" :  R, "mapa" : mapa}
    
def search_and_pick(robot, frame
                            , ltrack_pos=0, rtrack_pos=0, P=np.identity(3), marker_list = [], delete_countdown =0 , mapa = [], robot_trajectory = [],R=[] ):

    pass


def camera_related(frame):

    arucoParams = aruco.DetectorParameters_create()
    mtx,dist = load_camera_params()
    image,marker_pos  = get_marker_pose(frame, mtx, dist,arucoParams=arucoParams, marker_list=[0,1,2,3,4,5], markerLength = 3.3)


    #print("Output marco function:",marker_pos)

    return image,marker_pos



with Robot(AsyncCamera(1), tracker=TrackerWrapper(cv2.TrackerKCF_create), object_detector=NNObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS) ) as robot:
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
                "robot_trajectory": [],
                "feature_map" : [0] * 300
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
            act= A_star_move_to_box_blind,
            default_args={}
            ),

        State(
             name="MOVE_TO_BOX_BY_VISION",
             act= move_to_brick_v3,
             default_args={}
         )
    ]
    print(states[0])
    state_dict = {}
    for state in states:
        state_dict[state.name] = state

    start_state = states[0]

    main_loop(robot, start_state, state_dict, delay=0)




