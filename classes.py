import matplotlib.pyplot as plt
import numpy as np
import time

""" #import the classes from the other modules
from local_navigation import LocalNavigation
from filtering import KalmanFilter """

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

        self.path=[]
        self.goal_angle=np.mod(np.arctan2(-(self.path[0][1]-self.path[1][1]),self.path[0][0]-self.path[1][0],2*np.pi))

    def setPositions (self,x,y,theta) :
        
        self.pos_X = x
        self.pos_Y = y

        self.theta = theta

    def getPositions(self):
        return (self.pos_X,self.pos_Y)
    
    def getAngle(self):
        return self.theta

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
        return self.int_error

    def setSpeedLeft(self,speed):
        self.motor_target_left=speed
    
    def setSpeedRight(self,speed):
        self.motor_target_right=speed

    def setPath(self,path):
        self.path=path

    def getPath(self):
        return self.path

    def popPath(self):
        self.path=self.path.pop(0)

    def getPath(self):
        return self.path
    
    def setGoalAngle(self,angle):
        self.goal_angle=angle
    
    def getGoalAngle(self):
        return self.goal_angle
