from tdmclient import ClientAsync, aw
import matplotlib.pyplot as plt
import numpy as np

#import the classes from the other modules
#from local_navigation import LocalNavigation
#from filtering import Filtering
from classes import Thymio
from filtering import KalmanFilter
import global_navigation
import motion_control
import local_navigation

from vision import *
from global_visibility import *

client = ClientAsync()
node = aw(client.wait_for_node())
aw(node.lock())
aw(node.wait_for_variables())

""" test_occupancy_grid = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
                                    [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
                                    [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
start = (0,0)
goal = (10,15)"""

"""  
 if local_navigation.test_saw_osb(Thymio, node,500):
        local_navigation.obstacle_avoidance(Thymio, node, client)
    else :

 """
while(1) :


    robot = Thymio()

# Code for Vision + Visibility global nav 
    vision = Vision()
    vision.capture_image()
    print('image captured')
    vision.find_goal_pos()
    vision.find_start_pos()
    vision.find_angle(robot)
    print(robot.goal_X,robot.goal_Y,robot.pos_X,robot.pos_Y)
    vision.find_corners()
    vision.trace_contours()
    vision.compute_dist_mx(robot)
    print('end vision')
    
    global_nav = Global_Nav()
    global_nav.dijkstra(robot)
    global_nav.extract_path(robot)
    print('end global')
    print(robot.path)

# Code for vision + Astar (to be completed by Kyke)
    """ vision = Vision()
    vision.capture_image()
    vision.find_goal_pos()
    vision.find_start_pos()
    vision.find_angle(robot)
    vision.return_occupancy_matrix(robot)
    + global Astar part 
 """


    if not robot.goal_reached_t:
        motion_control.turn(robot.theta,robot,node)
    if robot.goal_reached_t and not robot.goal_reached_f:
        motion_control.go_to_next_point(0,robot.path[0],0,robot,node)

    """ global_nav = globalNavigation()
    path, visitedNodes = global_nav.A_star(start, goal, test_occupancy_grid) """


      






#class Main :

   # def __init__() :