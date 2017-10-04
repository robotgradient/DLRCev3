import time
import cv2
from ev3control.rpc import Robot
from rick.controllers import euclidian_move_to_brick, rotation_search_brick,move_to_brick_simple, move_to_brick_blind_and_grip
from rick.core import State
from rick.core import main_loop
from detection.marker_localization import get_specific_marker_pose, load_camera_params
import numpy as np
from rick.motion_control import euclidian_path_planning_control,piecewise_path_planning_control
print("Creating robot...")

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

    path_points=np.array([[0,0,0],[x2,y2,yaw2],[xobj,yobj,yaw]])
    print("The path is going to be create by",path_points[0],path_points[1],path_points[2])

    return ("MOVE_TO_BOX_FIRST",frame, {"path_points":path_points,  "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time()})


def piecewise_move_to_box_first(robot, frame, path_points,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0):


    
    brick_position=path_points[0]
    middle_pos=path_points[1]
    goal_pos=path_points[2]
    t0 = time.time()
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    print("odom: ", odom_r,odom_l)
    print("(((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((")

    estim_rob_pos, vel_wheels, new_path = piecewise_path_planning_control(robot.position,middle_pos,
                                                                          goal_pos, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path)
    robot.position = estim_rob_pos
    #print("ROBOT POSITION: ", estim_rob_pos)

    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    iteration += 1
    goal_pos=goal_pos
    path_points=path_points



    return "MOVE_TO_BOX_BLIND", frame, {"path_points":path_points,"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0}

def piecewise_move_to_box_blind(robot, frame, path_points,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0):


    
    brick_position=path_points[0]
    middle_pos=path_points[1]
    goal_pos=path_points[2]
    t0 = time.time()
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos


    estim_rob_pos, vel_wheels, new_path = piecewise_path_planning_control(robot.position,middle_pos,
                                                                          goal_pos, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path)
    robot.position = estim_rob_pos
    print("robot pose estimated:",robot.position)
    print("goal pose",goal_pos)
    #print("ROBOT POSITION: ", estim_rob_pos)
    print("Difference with goal:",abs(estim_rob_pos[0]-goal_pos[0]),abs(estim_rob_pos[1]-goal_pos[1]))
    if abs(estim_rob_pos[0]-goal_pos[0])<10 and abs(estim_rob_pos[1]-goal_pos[1])<50:
        return "PLACE_OBJECT", frame, {}

    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    iteration += 1
    goal_pos=goal_pos
    path_points=path_points



    return "MOVE_TO_BOX_BLIND", frame, {"path_points":path_points,"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0}



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
             act=piecewise_move_to_box_first,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time()
            }
         ),
         State(
             name="MOVE_TO_BOX_BLIND",
             act=piecewise_move_to_box_blind,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time()
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
