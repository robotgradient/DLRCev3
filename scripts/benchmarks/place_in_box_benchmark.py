import time
import cv2
from ev3control.rpc import Robot
from rick.controllers import euclidian_move_to_brick, rotation_search_brick,move_to_brick_simple, move_to_brick_blind_and_grip
from rick.core import State
from rick.core import main_loop
from detection.marker_localization import get_specific_marker_pose, load_camera_params
import numpy as np
from rick.motion_control import euclidian_path_planning_control
print("Creating robot...")

def search_box(robot, frame, vel=60):
    mtx,dist=load_camera_params()
    _,box_coords = get_specific_marker_pose(frame=frame,mtx=mtx,dist=dist,marker_id=0)
    if box_coords:
        print("BOX_COORDINATES:",box_coords[0],box_coords[1])
        return ("MOVE_FIRST_MAP", frame, {
                        "ltrack_pos": robot.left_track.position,
                        "rtrack_pos": robot.right_track.position,
                        "TIME": time.time()
                    })
    else:
        robot.rotate(vel)
        return "SEARCH_BOX", frame, {}

def move_towards_box(robot,frame,vel=200,vel_rot=60,atoly=5,atolx=50):
    mtx,dist=load_camera_params()
    _,box_coords = get_specific_marker_pose(frame,mtx,dist,2)
    if not box_coords:
        return "SEARCH_BOX", frame, {}
    box_coords=np.array(box_coords)
    if box_coords[1]>atoly:
        robot.rotate_left(vel=vel_rot)
    elif box_coords[1]<-atoly:
        robot.rotate_right(vel=vel_rot)
    else:
        robot.move_straight(vel=vel,time=1000)

    if box_coords[0]<atolx:
        return "PLACE_OBJECT", frame, {}
    else:
        return "MOVE_TO_BOX", frame, {}


def euclidian_move_to_brick2(robot, frame,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0):

    img_res = np.asarray((640,480))
    mtx,dist=load_camera_params()
    _,box_coords = get_specific_marker_pose(frame,mtx,dist,0)

    #if not box_coords:
    #    print("No second box detectedf")
    #    return "SEARCH_BOX", frame, {}
    if not box_coords:
        return "SEARCH_BOX", frame, {}
    if box_coords[0]<40:
        return "PLACE_OBJECT", frame, {}
    brick_position = [box_coords[0],-box_coords[1]]
    print("GOAL_POSITION",brick_position)
    t0 = time.time()
    print('t0 ', t0, 'TIME', TIME)
    time_diff = t0 - TIME
    print('time',time_diff)
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    print("odometry: ", odom_l, odom_r)
    estim_rob_pos, vel_wheels, new_path = euclidian_path_planning_control(robot.position,
                                                                          brick_position, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path)
    print("rob pos: ", estim_rob_pos)
    print("WHEELS VELOCITIES:",vel_wheels[1],vel_wheels[0])
    robot.position = estim_rob_pos
    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    #robot.left_track.wait_until_not_moving(timeout=3000)
    iteration += 1
    goal_pos=brick_position
    # print("Path: ", pat, iteration)
    # print("Robot positij "ffefrobot.position)
    # print("Velocities rl: ", vel_wheels)
    # print("##" *20)


    return "MOVE_SECOND_MAP", frame, {"goal_pos":goal_pos,"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0}

def euclidian_move_to_brick_blind(robot, frame, goal_pos,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0):


    #if not box_coords:
    #    print("No second box detectedf")
    #    return "SEARCH_BOX", frame, {}
    '''if iteration > 30:
        mtx,dist=load_camera_params()
        _,box_coords = get_specific_marker_pose(frame,mtx,dist,2)
        if not box_coords:
            return "SEARCH_BOX", frame, {}
        elif box_coords[0]<40:
            return "PLACE_OBJECT", frame, {}
        else:
            return ("MOVE_FIRST_MAP", frame, {
                        "ltrack_pos": robot.left_track.position,
                        "rtrack_pos": robot.right_track.position,
                        "TIME": time.time()
                        })   '''
    
    brick_position=[0,0]
    t0 = time.time()
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    estim_rob_pos, vel_wheels, new_path = euclidian_path_planning_control(robot.position,
                                                                          brick_position, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path)
    robot.position = estim_rob_pos
    #print("ROBOT POSITION: ", estim_rob_pos)
    print("Difference with goal:",abs(estim_rob_pos[0]-goal_pos[0]),abs(estim_rob_pos[1]-goal_pos[1]))
    if abs(estim_rob_pos[0]-goal_pos[0])<50 and abs(estim_rob_pos[1]-goal_pos[1])<10:
        return "PLACE_OBJECT", frame, {}
    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])
    #robot.left_track.wait_until_not_moving(timeout=3000)
    iteration += 1
    goal_pos=goal_pos
    # print("Path: ", pat, iteration)
    # print("Robot positij "ffefrobot.position)
    # print("Velocities rl: ", vel_wheels)
    # print("##" *20)


    return "MOVE_SECOND_MAP", frame, {"goal_pos":goal_pos,"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0}



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
             name="MOVE_TO_BOX",
             act=move_towards_box,
         ),
        State(
             name="PLACE_OBJECT",
             act=blind_placing,
         ),
        State(
             name="MOVE_FIRST_MAP",
             act=euclidian_move_to_brick2,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time()
            }
         ),
         State(
             name="MOVE_SECOND_MAP",
             act=euclidian_move_to_brick_blind,
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
