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
goal = (10,15) """

robot=Thymio() # Set Thym as class Thymio as initialization before the while
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

a = 0
while(1) :
    a = a + 1
    """ if local_navigation.test_saw_osb(Thym, node,500):   # Check if an obstacle has been detected
        local_navigation.obstacle_avoidance(Thym, node, client)     # Execute obstacle avoidance

    # Set test parameters to simulate a path (for testing motion without vision):
    
    globalNavigation().A_Star(Thym,start,goal,test_occupancy_grid)  # Compute the Astar based on a fake occupancy grid and set the Thym.path variable
    current_angle=np.linspace(0,2*np.pi,720)    # Set a fake current angle
    Thym.goal_angle=0   # Set a fake goal angle
    test_functions=1

    # Move the thymio with obstacle avoidance implemented:

    for i in range(6):
        for angle in current_angle:
            if not Thym.goal_reached_t:
                motion_control.turn(angle,Thym,node)
            if Thym.goal_reached_t and not Thym.goal_reached_f:
                # The following line sends the local_navigation.test_saw_obs to be able to
                # exit the motion_control.go_to_next_point and move on in the while loop
                
                motion_control.go_to_next_point(0,Thym.path[i],local_navigation.test_saw_osb(Thym, node, 500),Thym,node) """

    vision = Vision()
    vision.capture_image(cap)
    vision.find_goal_pos()
    vision.find_start_pos()
    vision.find_angle(robot)

    print(robot.path)
    time.sleep(0.2)
    if ((robot.vision == 1) & (a == 1))or((robot.vision == 1) & (robot.kidnap == True)):
        print(robot.goal_X,robot.goal_Y,robot.pos_X,robot.pos_Y)
        vision.find_corners()
        vision.trace_contours()
        vision.compute_dist_mx(robot)
        global_nav = Global_Nav()
        global_nav.dijkstra(robot)
        global_nav.extract_path(robot)
        print('end global')
        print(robot.path)
        if robot.kidnap == True:
            robot.kidnap == False

    print('end vision')

    KF = KalmanFilter()
    
    #KF.odometry_update(robot)
    KF.filter_kalman(robot)
    #print(np.degrees(KF.X_est[4][0]),(np.degrees(robot.goal_angle)))
    print(KF.X_est)

    if not robot.goal_reached_t:
        motion_control.turn(KF.X_est[4][0],robot,node)
    if robot.goal_reached_t and not robot.goal_reached_f:
        motion_control.go_to_next_point(KF.X_est[4][0],[KF.X_est[0][0],KF.X_est[2][0]],0,robot,node)


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


      





