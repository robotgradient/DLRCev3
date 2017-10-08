#Euclidean path planning control with Kalman filter for localization

import time
import cv2
from ev3control.rpc import Robot
from rick.controllers import *
from rick.A_star_planning import *
from rick.core import State
from rick.core import main_loop
from rick.async import AsyncCamera
from rick.utils import TrackerWrapper, bbox_center
from nn_object_detection.object_detectors import NNObjectDetector
from rick.live_plotting import MapRenderer

from detection.marker_localization import get_marker_pose, load_camera_params
import cv2.aruco as aruco
from dlrc_one_shot_learning.similarity_detectors import EuclidianNNFeaturesBrickFinder

from rick.mc_please_github_donot_fuck_with_this_ones import A_star_path_planning_control,compute_A_star_path, A_star_control

import numpy as np

from math import pi


from detection.opencv import eliminate_grip
from detection.opencv import get_lego_boxes as gl
from clustering import BBoxKMeansClustering


from rick.motion_control import euclidian_kalman , kalman_filter , kalman_filter2 , robot_control, odom_estimation

import sys

sys.path.append("../slam/")

import mapping

import matplotlib.pyplot as plt

from detection.opencv import detect_purple

PATH_TO_CKPT = "/home/julen/dlrc_models/frozen_inference_graph.pb"
PATH_TO_LABELS = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/label_map.pbtxt"


print("Creating robot...")


data = np.load('Homographygood.npz')
H=data["arr_0"]

map_renderer = MapRenderer()
object_detector = NNObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS)
similarity_detector = EuclidianNNFeaturesBrickFinder()
clustering_alg = BBoxKMeansClustering()


NUM_CLUSTERS = 2




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


    max_dist = 30
    min_angle = -pi/5
    max_angle = pi/5
    mapa_ar = np.array(mapa)
    

    vel_wheels = [160,150]


    for i in range(0, len(mapa)):

        er_x = mapa[i][0] - pos_rob[0]
        er_y = mapa[i][1] - pos_rob[1]

        distance = np.sqrt(np.power(er_x,2) + np.power(er_y,2))

        er_angle = np.arctan2(er_y, er_x) - pos_rob[2]*pi/180

        if er_angle >pi:
            er_angle = er_angle -2*pi
        if er_angle < -pi:
            er_angle = er_angle +2*pi

        next_x = pos_rob[0] + 10*np.cos(pos_rob[2] * pi/180)
        next_y = pos_rob[1] + 10*np.sin(pos_rob[2] * pi/180)

        if (distance< max_dist and er_angle > min_angle and er_angle< max_angle): # AVOID OBSTACLES
            vel_wheels = [-100,100]
            break
        elif next_x < 0 or next_x > 300 or next_y < 0 or next_y> 300:
            vel_wheels = [-100,100]

    return vel_wheels
    
def get_lego_boxes(frame, threshold=0.9, return_closest=False):
    #res = object_detector.detect_with_threshold(frame,threshold=threshold, return_closest=return_closest)
    #BB_legos = map(lambda x: x[0], res)
    #return list(BB_legos)
    BB_legos=gl(frame)
    return BB_legos



def index23(BB_legos,BB_target):
    index=1000
    i=0
    for box in BB_legos:
        if box[0]==BB_target[0][0] and box[1] == BB_target[0][1]:

            index = i
        i+=1
    return index



def search_target_with_Kalman_and_mapping(robot, frame
                            , ltrack_pos=0, rtrack_pos=0, P=np.identity(3), marker_list = [], delete_countdown = 0 , mapa = [], robot_trajectory = [],R=[],state_search = 2 , t1=0, 
                            t = None,feature_map = []):
    
    if not t:
        t = time.time()


    ################ THIS IS ALLL
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    
    marker_map = np.array([[150,0,0],[91,110,pi/2],[0,41,pi],[0,0,0]])
    frame = eliminate_grip(frame)
    BB_legos2=get_lego_boxes(frame)

    BB_legos=[]

    for bbox in BB_legos2:
        if bbox[3]<460 and (bbox[2]<380 or bbox[2]>420):
            BB_legos.append(bbox)

    

    for bbox in BB_legos:
        frame = plot_bbox(frame, bbox)
    lego_landmarks = mapping.cam2rob(BB_legos,H)
    mtx,dist=load_camera_params()
    frame,marker_list=get_marker_pose(frame,mtx,dist,marker_list=[0,1,2,3],markerLength=8.6)
    
    print("####################################################################################")
    estim_rob_pos_odom = odom_estimation(odom_r,odom_l,robot.position)
    index = 1000
    mapa, delete_countdown,robot_trajectory, links = mapping.update_mapa2(mapa,lego_landmarks,estim_rob_pos_odom,P,delete_countdown, robot_trajectory, index)
    Ts = 0.3
    estim_rob_pos, P  = kalman_filter2(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)
    robot.position = estim_rob_pos
    mapa = mapping.after_kalman_improvement(mapa, robot.position, estim_rob_pos_odom)
    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 *np.cos(estim_rob_pos[2] * pi/180)
    d[1] = estim_rob_pos[1] + 28* np.sin(estim_rob_pos[2]*pi/180)
    d[2] = estim_rob_pos[2]
    R.append(d)

    box_print = [x + [0] for x in marker_map.tolist()]

    map_renderer.plot_bricks_and_trajectory_and_robot_and_boxes(mapa, R, d, box_print)
  
    ############################################
    print("odom :", estim_rob_pos_odom, "kalmancito" , estim_rob_pos )



    #Feature extraction from bounding boxes
    bboxes = []

    for i in range(0,len(links)):
        bbox = BB_legos[links[i][0]]
        bboxes.append(frame[bbox[1]:bbox[3], bbox[0]:bbox[2],:])
    bounding_box_features = similarity_detector.extract_features(bboxes)

    for i in range(0,len(links)):

        feature_map[links[i][1]] = bounding_box_features[i]

    print("SHAPE: ", len(feature_map))
    #print("FEAT" , feature_map)
    #DEFINE MOTION CONTROL FOR SEARCHING

    # THE CONTROL IS : 1. GO TO THE CENTER OF THE WORKSPACE, 2. ROUND FOR 2 secs ,  SELECT A POINT CLOSE TO THE CENTER as new target

    vel_wheels = naive_obstacle_avoidance_control(mapa, robot.position)
    
    if time.time()-t  > 30:

        clust_feats=[]
        for item in feature_map:
            if not np.all(item==0):
                clust_feats.append(item)

        clustering_alg.fit(clust_feats, n_clusters=NUM_CLUSTERS)
        map_renderer.plot_bricks_and_trajectory_and_robot(mapa, R, d)

        
        return "SELECT_AND_GO", frame, {"ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos,"R" :  R, "mapa" : mapa}
    else:
        robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
        return "SEARCH_TARGET", frame, {"ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "P": P , "marker_list": [],
                                        "delete_countdown" : delete_countdown , "mapa": mapa, "robot_trajectory": robot_trajectory, "R" : R,
                                        "state_search" : 2, "t1" : t1, "t" : t, "feature_map":feature_map }



def select_and_go(robot,frame, cluster = 0,ltrack_pos=0, rtrack_pos=0,P = np.identity(3),R=[], mapa = [],tracker=None
                    ,img_res=np.asarray((640, 480)), atol=10,
                         vel_forward = 299, vel_rot = 100, atol_move_blind=140, 
                         fail_counter=0, center_position_error = 55, robot_trajectory = [], prev_BB_target = []):
    
    
    ################ THIS IS ALLL
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    marker_map = np.array([[150,0,0],[91,110,pi/2],[0,41,pi],[0,0,0]])
    frame = eliminate_grip(frame)

    BB_legos2=get_lego_boxes(frame, return_closest=True)

    BB_legos=[]

    for bbox in BB_legos2:
        if bbox[3]<460:
            BB_legos.append(bbox)

    lego_landmarks = mapping.cam2rob(BB_legos,H)
    mtx,dist=load_camera_params()
    frame,marker_list=get_marker_pose(frame,mtx,dist,marker_list=[0,1,2,3],markerLength=8.6)
    print("####################################################################################")
    estim_rob_pos_odom = odom_estimation(odom_r,odom_l,robot.position)
    index = 1000
    mapa, delete_countdown,robot_trajectory, links = mapping.update_mapa2(mapa,lego_landmarks,estim_rob_pos_odom,P,0, robot_trajectory, index)
    Ts = 0.3
    estim_rob_pos, P  = kalman_filter2(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)
    robot.position = estim_rob_pos
    mapa = mapping.after_kalman_improvement(mapa, robot.position, estim_rob_pos_odom)
    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 *np.cos(estim_rob_pos[2] * pi/180)
    d[1] = estim_rob_pos[1] + 28* np.sin(estim_rob_pos[2]*pi/180)
    d[2] = estim_rob_pos[2]
    R.append(d)

    box_print = [x + [0] for x in marker_map.tolist()]

    map_renderer.plot_bricks_and_trajectory_and_robot_and_boxes(mapa, R, d, box_print)
    ############################################

    print("robot pos in blind grip: ", robot.position)

    if not tracker:
        tracker = TrackerWrapper(cv2.TrackerKCF_create)


    ################################## control################################
    #print("BB target ", BB_target)

    ok, BB_target = tracker.update(frame)
    print("BB LEGOS", len(BB_legos))
    if not ok:
        if len(BB_legos) >0:
            if len(prev_BB_target)>0:
                center_old = bbox_center(*prev_BB_target)
                sh_dist = 999999999999
                for box in BB_legos:
                    center_new = bbox_center(*box)
                    distance = np.sqrt(np.power(center_new[0]-center_old[0],2)+ np.power(center_new[1]- center_old[1],2))
                    if distance < sh_dist:

                        sh_dist = distance
                        BB_target = box
            else:
                BB_target = BB_legos[0]
            tracker.init(frame, BB_target)        
        else:
            robot.rotate_left(vel=100)
            return "SELECT_AND_GO", frame, {"cluster" : cluster, 
            "ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos, "R" :  R, "mapa" : mapa}


    ############ CLUSTERING RELATED #######################        
    bboxes = []
    print("EL TARGET",BB_target)
    bbox = BB_target
    bboxes.append(frame[bbox[1]:bbox[3], bbox[0]:bbox[2],:])
    bounding_box_features = similarity_detector.extract_features(bboxes)
    #cluster = clustering_alg.predict(bounding_box_features)
    cluster = [1]
    #########################################################


    coords = bbox_center(*bbox)
    img_center = img_res / 2 - center_position_error 
    #img_center[0] = 285
    error = img_center - coords
    atol = 10 + coords[1]/480 * 40

    print("Errror:", error, "Coords ", coords, " ok ", ok)
    frame = plot_bbox(frame,bbox, 0, (255,0,0))

    if np.isclose(coords[0], img_center[0], atol=atol) and np.isclose(coords[1], img_res[1], atol=atol_move_blind):
        robot.move_straight(vel_forward, 500)
        return "MOVE_TO_BRICK_BLIND_AND_GRIP", frame, {"cluster" : cluster, "ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos, "R" :  R, "mapa" : mapa}

    if np.isclose(coords[0], img_center[0], atol=atol):
        print("Move straight")
        robot.move_straight(vel_forward)
        return "SELECT_AND_GO", frame, {"prev_BB_target" : BB_target, "cluster" : cluster,"tracker" : tracker, "ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos,"R" :  R, "mapa" : mapa}
    elif error[0] < 0:
        robot.rotate_left(vel=vel_rot)
        return "SELECT_AND_GO", frame, {"prev_BB_target" : BB_target, "cluster" : cluster,"tracker" : tracker, "ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos, "R" :  R, "mapa" : mapa}
    else:
        # Positive velocity for turning left
        robot.rotate_right(vel=vel_rot)
        return "SELECT_AND_GO", frame, {"prev_BB_target" : BB_target, "cluster" : cluster,"tracker" : tracker, "ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos, "R" :  R, "mapa" : mapa}
    


#MOVE TO BRICK BLIND
def move_to_brick_blind_and_grip(robot, frame, R=[],ltrack_pos=0 ,
    rtrack_pos=0,marker_list=[],mapa=[], vel=400, t=1700, cluster=None):
    # Make sure the grip is open
    robot.grip.open()
    robot.elevator.down()
    robot.elevator.wait_until_not_moving()
    robot.move_straight(vel=vel, time=t)
    robot.wait_until_not_moving()
    robot.pick_up()

    #odometry update
    marker_map = np.array([[150,0,0],[91,110,pi/2],[0,41,pi],[0,0,0]])
    P = np.identity(3)
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    Ts = 0.3
    estim_rob_pos, P  = kalman_filter2(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)
    robot.position = estim_rob_pos
    print("robot pos in blind grip: ", robot.position)

    obj_list = []
    Map = create_map(obj_list)
    return "GO_TO_BOX", frame, {"ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos
                                         , "mapa": mapa,  "R" : R, "cluster" : cluster, "Map" : Map}



def A_star_move_to_box_blind(robot, frame, Map=[],cluster = 0, replan=1,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0, P = np.identity(3),R=[], mapa = []):


   ################ THIS IS ALLL
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    marker_map = np.array([[150,0,0],[91,110,pi/2],[0,41,pi],[0,0,0]]) # WHERE VTHE OPTICAL MARKERS ARE IN THE ENVIROMENT
    frame = eliminate_grip(frame)
    BB_legos=get_lego_boxes(frame)

    lego_landmarks = mapping.cam2rob(BB_legos,H)
    mtx,dist=load_camera_params()
    frame,marker_list=get_marker_pose(frame,mtx,dist,marker_list=[0,1,2,3],markerLength=8.6)
    print("####################################################################################")
    estim_rob_pos_odom = odom_estimation(odom_r,odom_l,robot.position)
    index = 1000
    mapa, delete_countdown,robot_trajectory, links = mapping.update_mapa2(mapa,lego_landmarks,estim_rob_pos_odom,P,0, [], index)
    Ts = 0.3
    estim_rob_pos, P  = kalman_filter2(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)
    robot.position = estim_rob_pos
    mapa = mapping.after_kalman_improvement(mapa, robot.position, estim_rob_pos_odom)
    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 *np.cos(estim_rob_pos[2] * pi/180)
    d[1] = estim_rob_pos[1] + 28* np.sin(estim_rob_pos[2]*pi/180)
    d[2] = estim_rob_pos[2]
    R.append(d)

    box_print = [x + [0] for x in marker_map.tolist()]

    box_print[cluster][3] = 1

    map_renderer.plot_bricks_and_trajectory_and_robot_and_boxes(mapa, R, d, box_print)
  
    ############################################
    print("robot_estim_pos_Astar: ", robot.position)

    marker_map_obj = [[110,0,0],[91,70,pi/2],[41,40,pi],[0,0,0]]
    marker_map_obj = np.int_(np.array(marker_map_obj))
    obj = marker_map_obj[cluster[0],:2]
    print("THE BOX TO GO", obj, cluster)
    
    

    #print("DIFFERENTCE WITH THE GOAL:",abs(estim_rob_pos[0]-goal_pos[0]),abs(estim_rob_pos[1]-goal_pos[1]))
    
    #CONDITION FOR EXITTING

    distance_to_target = np.sqrt(np.power(estim_rob_pos[0]-marker_map_obj[cluster[0],0],2)+ np.power(estim_rob_pos[1]-marker_map_obj[cluster[0],1],2))

    
    if distance_to_target < 20: 
        return ("MOVE_TO_BOX_BY_VISION", frame, { "cluster": cluster,"ltrack_pos" : new_ltrack_pos ,"rtrack_pos" : new_rtrack_pos, "mapa":mapa})

    #update map
    path=A_star(robot.position[0:2], obj, Map)
    
    replan=1
    goal_pos=obj

    t0 = time.time()

    vel_wheels, new_path = A_star_control(robot.position,goal_pos,
                                        Map, robot.sampling_rate,
                                             odom_r= odom_r,odom_l=odom_l,
                                        iteration=iteration, path=path)
    
    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    iteration += 1

    print("###########################################################################################################")
    print("distance to target: ", distance_to_target)
    print("estimated vs goal", estim_rob_pos[0:2],goal_pos)
    print("###########################################################################################################")
    
    return ("GO_TO_BOX", frame, {"cluster":cluster, "replan":replan,"Map":Map,"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, 
                "rtrack_pos": new_rtrack_pos, "TIME": t0,"R":R, "mapa" : mapa})

def PID_control(robot, marker_map, box_coords,hist):
    vel_st=100
    vel_rot=60
    lat_tol=4
    yshift=2
    er_x = marker_map[0,0] - robot[0]
    er_y = marker_map[0,1] - robot[1]
    er_angle = np.arctan2(er_y, er_x) - robot[2]*pi/180
    print("ANGLES WITH MARKER AND ERROR",np.arctan2(er_y, er_x)*180/pi,robot[2])

    if er_angle > pi:
        er_angle = er_angle - 2*pi
    if er_angle < -pi:
        er_angle = er_angle + 2*pi

    distance = np.sqrt(np.power(er_x,2)+np.power(er_y,2))

    if box_coords:
        print("Y_DISTANCE_TO_MARKER",box_coords[1])
        if abs(box_coords[1]+yshift)>lat_tol:
            vel_wheels=np.asarray([-vel_rot,vel_rot])*np.sign(-box_coords[1])
            print("GUIDDE BY VISION")
        elif box_coords[0]>35:
            vel_wheels=np.asarray([vel_st,vel_st])
            print("GUIDDE BY VISION")
        else:
            vel_wheels=np.asarray([0,0])
            hist = 0
            print("STOP")


    else:
        if hist == 0:
            vel_wheels=np.asarray([0,0])
        elif er_angle > 0.7:
            vel_wheels=np.asarray([vel_rot,-vel_rot])
            hist = 1
        elif er_angle <-0.7:
            vel_wheels=np.asarray([-vel_rot,vel_rot])
            hist = -1
        elif hist ==1 : 
            vel_wheels=np.asarray([vel_rot,-vel_rot])
        else : 
            vel_wheels=np.asarray([-vel_rot,vel_rot])
        print("CORRECTING ANGLE",er_angle)
    return vel_wheels, hist
def move_to_box_by_vision(robot, frame, cluster =0, replan=1,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0, P = np.identity(3),
                            histeresis = 1,mapa=[],robot_trajectory=[]):
    ################ THIS IS ALLL
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    marker_map = marker_map = np.array([[150,0,0],[91,110,pi/2],[0,41,pi],[0,0,0]])
    frame = eliminate_grip(frame)
    BB_legos=get_lego_boxes(frame)

    lego_landmarks = mapping.cam2rob(BB_legos,H)
    mtx,dist=load_camera_params()
    frame,marker_list=get_marker_pose(frame,mtx,dist,marker_list=[0,1,2,3],markerLength=8.6)
    print("####################################################################################")
    estim_rob_pos_odom = odom_estimation(odom_r,odom_l,robot.position)
    index = 1000
    mapa, delete_countdown,robot_trajectory, links = mapping.update_mapa2(mapa,lego_landmarks,estim_rob_pos_odom,P,0, robot_trajectory, index)
    Ts = 0.3
    estim_rob_pos, P  = kalman_filter2(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)
    robot.position = estim_rob_pos
    mapa = mapping.after_kalman_improvement(mapa, robot.position, estim_rob_pos_odom)
    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 *np.cos(estim_rob_pos[2] * pi/180)
    d[1] = estim_rob_pos[1] + 28* np.sin(estim_rob_pos[2]*pi/180)
    d[2] = estim_rob_pos[2]
    R.append(d)

    box_print = [x + [0] for x in marker_map.tolist()]

    box_print[cluster][3] = 1

    map_renderer.plot_bricks_and_trajectory_and_robot_and_boxes(mapa, R, d, box_print)
  
    #map_renderer.plot_bricks_and_trajectory(mapa, R)
    ############################################
    print("######################################")
    print("robot_estim_pos_vision: ", robot.position)
    print("######################################")

    
    marker_map_obj = [[110,0,0],[91,70,pi/2],[40,41,pi],[0,0,0]]
    obj = marker_map_obj[cluster[0]]
    print("robot_estim_pos_PID: ", robot.position)

    
    box_coords = [marker_list[cluster[0],1]*np.cos(marker_list[cluster[0],0]),marker_list[cluster[0],1]*np.sin(marker_list[cluster[0],0])]
    vel_wheels, hist = PID_control(estim_rob_pos, marker_map,box_coords, histeresis)
    if hist==0:
        return "PLACE_OBJECT_IN_THE_BOX",frame,{"ltrack_pos": new_ltrack_pos, "rtrack_pos" : new_rtrack_pos, "mapa" : mapa}
    
    robot.move(vel_wheels[0],vel_wheels[1])
    return ("MOVE_TO_BOX_BY_VISION", frame, {"ltrack_pos": new_ltrack_pos, "rtrack_pos" : new_rtrack_pos, "histeresis" : hist, "cluster": cluster})


def place_object_in_the_box(robot,frame, ltrack_pos=0, rtrack_pos=0, P = np.identity(3), mapa = []):

    ################ THIS IS ALLL
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    marker_map = np.array([[150,0,0],[91,110,pi/2],[0,41,pi],[0,0,0]]) # WHERE VTHE OPTICAL MARKERS ARE IN THE ENVIROMENT
    BB_legos=get_lego_boxes(frame)

    lego_landmarks = mapping.cam2rob(BB_legos,H)
    mtx,dist=load_camera_params()
    frame,marker_list=get_marker_pose(frame,mtx,dist,marker_list=[0,1,2,3],markerLength=8.6)
    print("####################################################################################")
    estim_rob_pos_odom = odom_estimation(odom_r,odom_l,robot.position)
    index = 1000
    mapa, delete_countdown,robot_trajectory, links = mapping.update_mapa2(mapa,lego_landmarks,estim_rob_pos_odom,P,0, [], index)
    Ts = 0.3
    estim_rob_pos, P  = kalman_filter2(odom_r,odom_l,robot.position,marker_list, marker_map,Ts,P)
    robot.position = estim_rob_pos
    mapa = mapping.after_kalman_improvement(mapa, robot.position, estim_rob_pos_odom)
    d = np.ones(3)
    d[0] = estim_rob_pos[0] + 28 *np.cos(estim_rob_pos[2] * pi/180)
    d[1] = estim_rob_pos[1] + 28* np.sin(estim_rob_pos[2]*pi/180)
    d[2] = estim_rob_pos[2]
    #R.append(d)
    #map_renderer.plot_bricks_and_trajectory(mapa, R)
    ############################################
    robot.move(vel_left=100,vel_right=100,time=2000)
    print("MOVING")
    robot.left_track.wait_until_not_moving(timeout=3000)
    robot.reset()
    robot.grip.wait_until_not_moving(timeout=3000)
    robot.move_straight(vel=-100,time=2000)
    robot.left_track.wait_until_not_moving(timeout=3000)
    robot.rotate_left(100,time=6000)
    robot.left_track.wait_until_not_moving(timeout=10000)
    print("finish")
    return "SELECT_AND_GO", frame, {"ltrack_pos" :  new_ltrack_pos ,"rtrack_pos" :  new_rtrack_pos,"R" :  [], "mapa" : mapa}


def camera_related(frame):

    arucoParams = aruco.DetectorParameters_create()
    mtx,dist = load_camera_params()
    image,marker_pos  = get_marker_pose(frame, mtx, dist,arucoParams=arucoParams, marker_list=[0,1,2,3,4,5], markerLength = 3.3)


    #print("Output marco function:",marker_pos)

    return image,marker_pos



with Robot(AsyncCamera(1)) as robot:
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
        State(
            name="SELECT_AND_GO",
            act=select_and_go,
             default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "P" : np.identity(3),
                "mapa": []            }
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
            name="GO_TO_BOX",
            act= A_star_move_to_box_blind,
            default_args={}
            ),

        State(
             name="MOVE_TO_BOX_BY_VISION",
             act= move_to_box_by_vision,
             default_args={}
         ),
        State(
             name="PLACE_OBJECT_IN_THE_BOX",
             act= place_object_in_the_box,
             default_args={}
         )

    ]
    print(states[0])
    state_dict = {}
    for state in states:
        state_dict[state.name] = state

    start_state = states[0]

    main_loop(robot, start_state, state_dict, delay=0)




