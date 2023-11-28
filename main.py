import matplotlib.pyplot as plt
import numpy as np
import time

#import the classes from the other modules
from local_navigation import LocalNavigation
#from filtering import Filtering
from classes import Thymio
from filtering import KalmanFilter

from vision import *
from global_visibility import *

while(1) :

    a = 0
    motor_left_speed = 0
    motor_right_speed = 0
    pre_time = 0
    X_init = 0
    Y_init = 0
    theta_init = 0

    #print(Thymio)

    robot = Thymio()

    vision = Vision()
    vision.capture_image()
    vision.find_goal_pos()
    vision.find_start_pos()
    vision.find_angle(robot)
    vision.find_corners()
    vision.trace_contours()
    vision.compute_dist_mx(robot)
    
    global_nav = Global_Nav()
    global_nav.dijkstra(robot)
    global_nav.extract_path(robot)








#class Main :

   # def __init__() :