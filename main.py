from tdmclient import ClientAsync, aw
import matplotlib.pyplot as plt
import numpy as np
import time

#import the classes from the other modules
from classes import Thymio
from filtering import KalmanFilter
from global_navigation import globalNavigation
import motion_control
import local_navigation

from vision import *
from global_visibility import *

client = ClientAsync()
node = aw(client.wait_for_node())
aw(node.lock())
aw(node.wait_for_variables())

#Classes initialization
robot=Thymio() # Set Thym as class Thymio as initialization before the while
KF = KalmanFilter()
vision = Vision()
global_nav = Global_Nav()

#Video capturing 
cap = cv2.VideoCapture(1)

a = 0
while(1) :
    a = a + 1
    #VISION
    vision.capture_image(cap)
    vision.find_goal_pos()
    vision.find_start_pos(robot,a)
    vision.find_angle(robot)
    if (a == 1) : 
        vision.find_corners()
        vision.trace_contours()
    if ((robot.vision == 1) & (a == 1))or((a != 1) & (robot.kidnap == True) & (robot.vision == 1)):
        vision.compute_dist_mx(robot)
        global_nav.dijkstra(robot)
        global_nav.extract_path(robot)
        if robot.kidnap == True:
            robot.kidnap = False
    robot.vision = True
    #END VISION
    #FILTERING
    for i in range(1,10) :
        KF.odometry_update(robot)
        KF.filter_kalman(robot)
        #print(np.degrees(KF.X_est[4][0]),(np.degrees(robot.goal_angle)))
        print(KF.X_est)
    #MOTION CONTROL
        if not robot.goal_reached_t:
            motion_control.turn(robot.theta,robot,node)
        if robot.goal_reached_t and not robot.goal_reached_f:
            motion_control.go_to_next_point(robot.theta,[robot.pos_X,robot.pos_Y],0,robot,node)
    

# Code for Vision + Visibility global nav 
"""     
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
 """
# Code for vision + Astar (to be completed by Kyke)
"""
      vision = Vision()
    vision.capture_image()
    vision.find_goal_pos()
    vision.find_start_pos()
    vision.find_angle(robot)
    vision.return_occupancy_matrix(robot)
    + global Astar part 
 """


"""     if not robot.goal_reached_t:
        motion_control.turn(robot.theta,robot,node)
    if robot.goal_reached_t and not robot.goal_reached_f:
        motion_control.go_to_next_point(0,robot.path[0],0,robot,node) """

""" 
    global_nav = globalNavigation()
    path, visitedNodes = global_nav.A_star(start, goal, test_occupancy_grid) """


      





