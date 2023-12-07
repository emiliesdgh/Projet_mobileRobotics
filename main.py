from tdmclient import ClientAsync, aw
import matplotlib.pyplot as plt
import numpy as np
import time
import copy


#import the classes from the other modules
from classes import Thymio
from filtering import KalmanFilter
import motion_control
import local_navigation
from vision import *
from global_visibility import *

client = ClientAsync()
node = aw(client.wait_for_node())
aw(node.lock())
aw(node.wait_for_variables())

#Classes initialization
robot=Thymio() 
KF = KalmanFilter()
vision = Vision()
global_nav = Global_Nav()

cv2.namedWindow('Window',cv2.WINDOW_AUTOSIZE)

#Video capturing 
cap = cv2.VideoCapture(1)

a = 0

def prepare_img(vision,real_path):
        vision.img_show = vision.img_out
        for i in range(1,len(real_path)):
            vision.img_show = cv2.line(vision.img_show, real_path[i], real_path[i-1], (255, 0, 0) , 3) 
        for m in  vision.m_cor:
            cv2.circle(vision.img_show, [m[0],m[1]], 3, (0, 255, 0), cv2.FILLED)
        for c in vision.cornerss:  
            cv2.circle(vision.img_show, [c[0],c[1]], 3, (255, 0, 0), cv2.FILLED)
        for p in vision.positions: 
            cv2.circle(vision.img_show, [p[0],p[1]], 3, (0, 0, 255), cv2.FILLED)

real_path = []

while(1) :
    a = a + 1
    #VISION
    vision.capture_image(cap)
    if (a == 1) : 
        vision.find_borders()
    vision.tilt_image()
    vision.find_goal_pos()
    vision.find_start_pos(robot,a)
    vision.find_angle(robot)
    if (a == 1) : 
        vision.find_corners()
        vision.trace_contours()
    if ((robot.vision == 1) and (a == 1))or((a != 1) and (robot.kidnap == True) and (robot.vision == 1)):
        vision.compute_dist_mx(robot)
        global_nav.dijkstra(robot)
        global_nav.extract_path(robot)
        if a == 1 : 
            real_path = copy.copy(robot.path)
        if robot.kidnap == True: 
            robot.setSpeedLeft(0, node)
            robot.setSpeedRight(0, node)
            robot.kidnap = False
    #END VISION

    #LIVE VISUALIZATION
    prepare_img(vision,real_path)
    cv2.startWindowThread()
    cv2.imshow('Window',vision.img_show)
    cv2.waitKey(1)

    #FILTERING
    KF.odometry_update(robot, node)
    KF.filter_kalman(robot)

    # LOCAL NAVIGATION AND MOTION CONTROL
    if local_navigation.test_saw_osb(robot,node,500):
        local_navigation.obstacle_avoidance(robot,node,client,obs_threshold=500)
    else:
        if not robot.goal_reached_t:
            motion_control.turn(KF.X_est[4][0],robot,node)
        if robot.goal_reached_t and not robot.goal_reached_f:
            motion_control.go_to_next_point(KF.X_est[4][0],[KF.X_est[0][0],KF.X_est[2][0]],robot,node)

    #STOP CONDITION WHEN GOAL IS REACHED
    if len(robot.path) <= 1:
        print("Goal reached")
        break


      




