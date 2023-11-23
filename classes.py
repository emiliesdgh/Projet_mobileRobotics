import matplotlib.pyplot as plt
import numpy as np
import time

#import the classes from the other modules
from local_navigation import LocalNavigation
from filtering import KalmanFilter

class Classes :

    def __init__(self) :

        self.variable = 0 


class Thymio :

    def __init__ (self) :

        self.pos_X = 0
        self.pos_Y = 0

        self.theta = 0
        self.vision = 0     #if CV is done or not

        self.goal_reached=False #has the robot reached the next point
        self.prev_error=0   #for derivative part of PID control
        self.int_error=0    #for integral part of PID control

        self.motor_target_left=0
        self.motor_target_right=0

    def setPositions (self) :
        
        self.pos_X = 0
        self.pos_Y = 0

        self.theta = 0

    def setGoalReached(self,boolean):
        self.goal_reached=boolean

    def getGoalReached(self):
        return self.goal_reached

    def setPrevError(self, error):
        self.prev_error=error

    def getPrevError(self):
        return self.prev_error
    
    def setIntError(self, error):
        self.int_error=error

    def getIntError(self):
        return 

    def setSpeed(self,speed):
        self.motor_target_left=speed
        self.motor_target_right=speed