import time
import cv2
from ev3control.rpc import Robot
from rick.core import State
from rick.core import main_loop
from detection.marker_localization import get_specific_marker_pose, load_camera_params,get_marker_pose
import numpy as np
from rick.mc_please_github_donot_fuck_with_this_ones import A_star_path_planning_control,compute_A_star_path,A_star_kalman
from rick.A_star_planning import *
import matplotlib.pyplot as plt
import cv2.aruco as aruco

print("Creating robot...")

def camera_related(frame):

    arucoParams = aruco.DetectorParameters_create()
    mtx,dist = load_camera_params()
    image,marker_pos  = get_marker_pose(frame, mtx, dist,arucoParams=arucoParams, marker_list=[0,1,2,3,4,5], markerLength = 3.3)


    #print("Output marco function:",marker_pos)

    return image,marker_pos

def search_box(robot, frame, vel=60):
    mtx,dist=load_camera_params()
    _,box_coords = get_specific_marker_pose(frame=frame,mtx=mtx,dist=dist,marker_id=0)
    if box_coords:
        #robot.left_track.stop(stop_action="brake")
        #robot.right_track.stop(stop_action="brake")
        print("BOX_COORDINATES:",box_coords[0],box_coords[1])
        return ("COMPUTE_PATH", frame, {
                        "box_coords": box_coords
                    })
    else:
        robot.rotate(vel)
        return "SEARCH_BOX", frame, {}

def compute_path(robot,frame,box_coords):
    x=box_coords[0]
    y=box_coords[1]
    yaw=box_coords[2]
    if (y>0 and yaw>-80) or (y<0 and yaw< -100):
        print("NICE PATH")
    thm=40
    thobj=20


    x2=x+thm*np.sin(yaw*np.pi/180.)
    y2=y-thm*np.cos(yaw*np.pi/180.)
    yaw2=0
    xobj=x+thobj*np.sin(yaw*np.pi/180.)
    yobj=y-thobj*np.cos(yaw*np.pi/180.)

    obj=[xobj,yobj]
    obslist=[]
    Map=create_map(obslist)
    path=A_star([0,0],obj, Map)
    plt.plot(path[:,0],path[:,1])
    plt.axis([-100, 150, -100, 150])
    plt.show()
    P=np.identity(3)
    frame,marker_list  = camera_related(frame = frame)
    return ("MOVE_TO_BOX_FIRST",frame, {"Map":Map, "obj":obj,  "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time(),"P": P , "marker_list": marker_list})


def A_star_to_box_kalman_first(robot, frame, Map,obj,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0, P=np.identity(3), marker_list = []):


    
    goal_pos=obj
    t0 = time.time()
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    frame,marker_list  = camera_related(frame = frame)
    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])

    estim_rob_pos, vel_wheels, new_path,P = A_star_kalman(robot.position,goal_pos,
                                                                          Map, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path,
                                                                          P=P, marker_list = marker_list ,marker_map=marker_map)
    robot.position = estim_rob_pos
    #print("ROBOT POSITION: ", estim_rob_pos)

    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    iteration += 1
    goal_pos=goal_pos



    return "MOVE_TO_BOX", frame, {"Map":Map, "obj":obj,  "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time(),"P": P , "marker_list": marker_list}

def A_star_to_box_kalman_replanning(robot, frame, Map,obj,
                            replan=0,path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0,P=np.identity(3), marker_list = []):
    mtx,dist=load_camera_params()
    _,box_coords = get_specific_marker_pose(frame=frame,mtx=mtx,dist=dist,marker_id=0)
        
    #REPLANNING WITH VISUAL FEEDBACK
    replan+=1
    if replan>20:
        if box_coords:
            x=box_coords[0]
            y=box_coords[1]
            yaw=box_coords[2]
            thobj=20
            xobj=x+thobj*np.sin(yaw*np.pi/180.)
            yobj=y-thobj*np.cos(yaw*np.pi/180.)

            obj=[xobj+robot.position[0],yobj+robot.position[1]]
            #update map
            path=A_star(robot.position[0:2], obj, Map)
            # if the box is detected renew the objective
            replan=0

    #CALL THE MOTION CONTROL FUNCTION KALMAN
    goal_pos=obj
    t0 = time.time()
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    frame,marker_list  = camera_related(frame = frame)
    marker_map = np.array([[200,100,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0],[200,0,0]])
    estim_rob_pos, vel_wheels, new_path,P = A_star_kalman(robot.position,goal_pos,
                                                                          Map, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path,
                                                                          P=P, marker_list = marker_list ,marker_map=marker_map)
    robot.position = estim_rob_pos
    print("ROBOT ESTIMATED:",robot.position)
    print("GOAL_POSITION",goal_pos)
    #print("ROBOT POSITION: ", estim_rob_pos)
    

    print("DIFFERENTCE WITH THE GOAL:",abs(estim_rob_pos[0]-goal_pos[0]),abs(estim_rob_pos[1]-goal_pos[1]))
    
    #CONDITION FOR EXITTING
    if abs(estim_rob_pos[0]-goal_pos[0])<25 and abs(estim_rob_pos[1]-goal_pos[1])<80:
        robot.move(vel_left=0,vel_right=0)
        print("ENTERING BLIND SPOT")
        return "PLACE_OBJECT", frame, {}

    #MOVEMENT OF THE ROBOT
    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    
    #UPDATE THE ITERATION
    iteration += 1
   



    return "MOVE_TO_BOX", frame, {"Map":Map,"replan":replan,"obj":goal_pos,"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0}



def blind_placing(robot,frame,vel=200):

    robot.move_straight(vel=vel,time=3000)
    robot.left_track.wait_until_not_moving(timeout=4000)
    robot.grip.open()
    robot.grip.wait_until_not_moving(timeout=3000)
    robot.move_straight(-500,time=1000)
    robot.left_track.wait_until_not_moving(timeout=1100)
    robot.rotate_right(vel=600,time=1500)
    robot.left_track.wait_until_not_moving(timeout=1100)
    print("finish")
    return "FINAL_STATE"



with Robot(cv2.VideoCapture(1)) as robot:
    robot.grip.close()
    robot.map = [(0, 110)]
    robot.sampling_rate = 0.1
    print("These are the robot motor positions before planning:", robot.left_track.position, robot.right_track.position)
    # Define the state graph, we can do this better, currently each method
    # returns the next state name
    states = [
        State(
            name="SEARCH_BOX",
            act=search_box,
        ),
        State(
             name="PLACE_OBJECT",
             act=blind_placing,
         ),
        State(
             name="COMPUTE_PATH",
             act=compute_path,
         ),
                 State(
             name="MOVE_TO_BOX_FIRST",
             act=A_star_to_box_kalman_first,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time(),
                "P" : np.identity(3)
            }
        ),

         State(
             name="MOVE_TO_BOX",
             act=A_star_to_box_kalman_replanning,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time(),
                "P" : np.identity(3)
            }
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

    main_loop(robot, start_state, state_dict, delay=0.1)