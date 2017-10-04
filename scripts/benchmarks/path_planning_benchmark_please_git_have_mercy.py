import time
import cv2
from ev3control.rpc import Robot
#from rick.controllers import euclidian_move_to_brick, rotation_search_brick,move_to_brick_simple, move_to_brick_blind_and_grip
from rick.core import State
from rick.core import main_loop
from detection.marker_localization import get_specific_marker_pose, load_camera_params
import numpy as np
from rick.mc_please_github_donot_fuck_with_this_ones import A_star_path_planning_control,compute_A_star_path

from rick.A_star_planning import *
print("Creating robot...")
import matplotlib.pyplot as plt

def initialize(robot, frame, vel=60):

    return ("MOVE_FIRST_MAP", frame, {
                        "ltrack_pos": robot.left_track.position,
                        "rtrack_pos": robot.right_track.position,
                        "TIME": time.time()
                    })


def plot_mapa(robot_traj):
    rob = np.array(robot_traj)
    print("Before stop")
    if rob.size > 100:
        plt.plot(rob[:,0],rob[:,1])
        plt.axis([-100, 150, -100, 150])
        plt.legend(["Lego", "path"])
        plt.show()
    print("After stop")
def A_start_firstmove(robot, frame,
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0):

    rob = [0,0,0]
    obj = [100,0]
    obslist=[[50,0],[60,0],[40,0],[30,0][70,0]]
    Map=create_map(obslist)

    #obj=[obj[0]+offsetx,obj[1]+offsety]



    brick_position = obj
    print("GOAL_POSITION",brick_position)
    t0 = time.time()
    print('t0 ', t0, 'TIME', TIME)
    time_diff = t0 - TIME
    print('time',time_diff)
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos
    print("Position in the first move",robot.position)
    print("odometry: ", odom_l, odom_r)
    estim_rob_pos, vel_wheels, new_path = A_star_path_planning_control(robot.position,
                                                                          brick_position,Map, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path)
    print("new path shjape",new_path.shape)
    plt.plot(new_path[:,0],new_path[:,1])
    plt.show()
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


    return ("MOVE_SECOND_MAP", frame, {"goal_pos":goal_pos,"Map":Map,"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0})

def A_start_secondmove(robot, frame, goal_pos,Map=[],
                            path=[], iteration=0, ltrack_pos=0, rtrack_pos=0, TIME=0, R = []):


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
    t0=time.time()
    brick_position=goal_pos
    t0 = time.time()
    new_ltrack_pos = robot.left_track.position
    new_rtrack_pos = robot.right_track.position
    odom_l, odom_r = new_ltrack_pos - ltrack_pos, new_rtrack_pos - rtrack_pos

    estim_rob_pos, vel_wheels, new_path = A_star_path_planning_control(robot.position,
                                                                          brick_position,Map, robot.sampling_rate,
                                                                          odom_r= odom_r,odom_l=odom_l,
                                                                          iteration=iteration, path=path)
    robot.position = estim_rob_pos
    #print("ROBOT POSITION: ", estim_rob_pos)
    #plt.plot(estim_rob_pos[0],estim_rob_pos[1])
    #plt.plot()
    print("Robot position:",estim_rob_pos)
    print("INMEDIATE VALUE", new_path[0,0],new_path[0,1])
    print("Difference with goal:",abs(estim_rob_pos[0]-goal_pos[0]),abs(estim_rob_pos[1]-goal_pos[1]))
    if abs(estim_rob_pos[0]-goal_pos[0])<10 and abs(estim_rob_pos[1]-goal_pos[1])<10:
        plot_mapa(R)
        return "FINAL_STATE"
    robot.move(vel_left=vel_wheels[1], vel_right=vel_wheels[0])

    R.append(estim_rob_pos)
    #lot_mapa(R)
    #robot.left_track.wait_until_not_moving(timeout=3000)
    iteration += 1
    goal_pos=goal_pos
    # print("Path: ", pat, iteration)
    # print("Robot positij "ffefrobot.position)
    # print("Velocities rl: ", vel_wheels)
    # print("##" *20)
    print("retard is:",time.time()-t0)

    return ("MOVE_SECOND_MAP", frame, {"goal_pos":goal_pos,"Map":Map,"iteration" : iteration, "path" : new_path, "ltrack_pos": new_ltrack_pos, "rtrack_pos": new_rtrack_pos, "TIME": t0, "R" : R})







with Robot(cv2.VideoCapture(1)) as robot:
    robot.grip.close()
    robot.map = [(0, 110)]
    robot.sampling_rate = 0.1
    print("These are the robot motor positions before planning:", robot.left_track.position, robot.right_track.position)
    # Define the state graph, we can do this better, currently each method
    # returns the next state name
    states = [
        State(
             name="INITIAL_STATE",
             act=initialize,
         ),
        State(
             name="MOVE_FIRST_MAP",
             act=A_start_firstmove,
            default_args={
                "ltrack_pos": robot.left_track.position,
                "rtrack_pos": robot.right_track.position,
                "TIME": time.time()
            }
         ),
         State(
             name="MOVE_SECOND_MAP",
             act=A_start_secondmove,
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
