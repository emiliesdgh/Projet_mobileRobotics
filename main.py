import matplotlib.pyplot as plt
import numpy as np
import time

#import the classes from the other modules
from local_navigation import LocalNavigation
#from filtering import Filtering
from classes import Thymio
import odometry
from filtering import KalmanFilter



while(1) :

    a = 0
    motor_left_speed = 0
    motor_right_speed = 0
    pre_time = 0
    X_init = 0
    Y_init = 0
    theta_init = 0

    odometry(motor_left_speed, motor_right_speed, pre_time, X_init, Y_init, theta_init, Thymio) 

    print(Thymio)





#class Main :

   # def __init__() :