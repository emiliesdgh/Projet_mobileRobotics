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
VISION_VERBOSE = False
FILTERING_VERBOSE = True
OBS_AVOIDANCE = False

#Classes initialization
robot=Thymio() # Set Thym as class Thymio as initialization before the while
KF = KalmanFilter()
vision = Vision()
global_nav = Global_Nav()
GN = globalNavigation()

cv2.namedWindow('Window',cv2.WINDOW_AUTOSIZE)

#Video capturing 
cap = cv2.VideoCapture(1) # CAP_DSHOW is needed for other computers (Diana, Kike and Emilie) 

a = 0

""" def update_image():
    plt.imshow(cv2.cvtColor(vision.img_out, cv2.COLOR_BGR2RGB))
    # Plotting
    plt.ion()
    corn_x, corn_y = zip(*vision.cor)
    path_x, path_y = zip(*robot.path)
    plt.scatter(corn_x, corn_y, color='red', marker='o', s=15)
    plt.plot(path_x, path_y, linestyle='-', color='blue', linewidth=2)
    plt.show()
    plt.pause(0.1)
    time.sleep(1) """

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
        #print('path recomputed')
        vision.compute_dist_mx(robot)
        print('RECOMPUTE PATHHHHHHH')
        global_nav.dijkstra(robot)
        global_nav.extract_path(robot)
        if a == 1 : 
            real_path = robot.path
        if robot.kidnap == True: # this can be eliminated because you only enter here if kidnap==True
            #print('kidnap reset to zero')
            print("---------KIDNAPPED------------")
            robot.setSpeedLeft(0, node)
            robot.setSpeedRight(0, node)
            robot.kidnap = False
    #END VISION
    prepare_img(vision,real_path)
    cv2.startWindowThread()
    cv2.imshow('Window',vision.img_show)
    cv2.waitKey(1)

    """ t1 = Thread(target = update_image(), args=(),)
    t1.start()  """

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
    
    print("visiontrue",robot.vision)


    #FILTERING
    KF.odometry_update(robot, node)
    KF.filter_kalman(robot)
    
    if FILTERING_VERBOSE == True:
        #print("visiontrue",robot.vision)
        print(f"f_angle= {np.degrees(KF.X_est[4][0]):.2f}")
        #print(f"goal_angle= {np.degrees(robot.goal_angle):.2f}")
        print("pos", robot.pos_X, robot.pos_Y)
        print("f_pos",KF.X_est[0][0],KF.X_est[2][0])
        print("path=", robot.path)
        """ print("X_est_pre", KF.X_est_pre)
        print("X_est", KF.X_est) """
        print("------------------------------")
    

    # LOCAL NAVIGATION

    if OBS_AVOIDANCE == True:
        if local_navigation.test_saw_osb(robot,node,500):
            print("--------------CRASH------------------")
            local_navigation.obstacle_avoidance(robot,node,client,obs_threshold=500)
        else:
            if not robot.goal_reached_t:
                motion_control.turn(KF.X_est[4][0],robot,node)
            if robot.goal_reached_t and not robot.goal_reached_f:
                motion_control.go_to_next_point(KF.X_est[4][0],[KF.X_est[0][0],KF.X_est[2][0]],robot,node)


    print(f"goal_angle= {np.degrees(robot.goal_angle):.2f}")
    
    #MOTION CONTROL
    
    if not robot.goal_reached_t:
        motion_control.turn(KF.X_est[4][0],robot,node)
    if robot.goal_reached_t and not robot.goal_reached_f:
        motion_control.go_to_next_point(KF.X_est[4][0],[KF.X_est[0][0],KF.X_est[2][0]],0,robot,node)    

    if len(robot.path) <= 1:
        print("Goal reached")
        break


      




