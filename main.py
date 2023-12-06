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

# Constant variables
VISION_VERBOSE = True
FILTERING_VERBOSE = False

#Classes initialization
robot=Thymio() # Set Thym as class Thymio as initialization before the while
KF = KalmanFilter()
vision = Vision()
global_nav = Global_Nav()
GN = globalNavigation()

#Video capturing 
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) # CAP_DSHOW is needed for other computers (Diana, Kike and Emilie) 

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
        #print('path recomputed')
        vision.compute_dist_mx(robot)
        global_nav.dijkstra(robot)
        global_nav.extract_path(robot)

        # Plotting
        plt.imshow(cv2.cvtColor(vision.frame, cv2.COLOR_BGR2RGB))
        corn_x, corn_y = zip(*vision.cor)
        path_x, path_y = zip(*robot.path)
        plt.scatter(corn_x, corn_y, color='red', marker='o', s=15)
        plt.plot(path_x, path_y, linestyle='-', color='blue', linewidth=2)
        plt.show()

        if robot.kidnap == True: # this can be eliminated because you only enter here if kidnap==True
            #print('kidnap reset to zero')
            robot.kidnap = False
    robot.vision = True
    #END VISION

    if VISION_VERBOSE == True:
        print("--------------------------")
        print("Vision output")
        print("--------------------------")
        print("goal=", robot.goal_X,",", robot.goal_Y)
        print("pos=", robot.pos_X,",", robot.pos_Y)
        print(f"angle={robot.theta:.2f}")
        print(f"goal_angle={robot.goal_angle:.2f}")
        print("path=", robot.path)
        print("corners=", vision.cor)

    #FILTERING
    KF.odometry_update(robot)
    KF.filter_kalman(robot)
    
    if FILTERING_VERBOSE == True:
        #print(f"f_angle= {KF.X_est[4][0]:.2f}")
        #print(f"goal_angle= {robot.goal_angle:.2f}")
        #print("pos", robot.pos_X, robot.pos_Y)
        #print("f_pos",KF.X_est[0][0],KF.X_est[2][0])
        print("X_est_pre", KF.X_est_pre)
        print("X_est", KF.X_est)
    

    #MOTION CONTROL
    
    if not robot.goal_reached_t:
        motion_control.turn(KF.X_est[4][0],robot,node)
    if robot.goal_reached_t and not robot.goal_reached_f:
        motion_control.go_to_next_point(KF.X_est[4][0],[KF.X_est[0][0],KF.X_est[2][0]],0,robot,node)
    
    if len(robot.path) <= 1:
        break

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


      




