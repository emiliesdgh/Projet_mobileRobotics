import matplotlib.pyplot as plt
import numpy as np
import time
from tdmclient import ClientAsync, aw

class Thymio :

    def __init__ (self) :

        self.obs_avoided = False

        # Thymio position (coordinates and angle)
        self.pos_X = 0
        self.pos_Y = 0
        self.theta = 0

        self.vision = 0     #if CV is done or not
        self.kidnap = False

        self.goal_reached_t=False   #has the robot reached right angle
        self.goal_reached_f=False   #has the robot reached the next point
        self.prev_error=0           #for derivative part of PID control
        self.int_error=0            #for integral part of PID control
        self.prev_time=0

        self.motor_target_left=0
        self.motor_target_right=0

        self.motor_left_speed=0
        self.motor_right_speed=0

        self.path=[]
        self.goal_angle=0 

        self.goal_X=0
        self.goal_Y=0

        self.dist_mx = []
        self.cor=[]
        self.S=0

        self.occupancy_matrix = []

    def setPositions (self,x,y,x_g,y_g,theta) :
        
        self.pos_X = x
        self.pos_Y = y

        self.goal_X = x_g
        self.goal_Y = y_g

        self.theta = theta

    def getPositions(self):
        return (self.pos_X,self.pos_Y)
    
    def getAngle(self):
        return self.theta

    def setSpeedLeft(self,speed,node):
        self.motor_target_left=speed
        aw(node.set_variables({"motor.left.target": [speed]}))
    
    def setSpeedRight(self,speed,node):
        self.motor_target_right=speed
        aw(node.set_variables({"motor.right.target": [speed]}))

    def getSpeeds(self,node):
        aw(node.wait_for_variables({"motor.left.speed", "motor.right.speed"}))
        self.motor_left_speed = node["motor.left.speed"]
        self.motor_right_speed = node["motor.right.speed"]


    def setAngle(self, pos_X, pos_Y):
        self.goal_angle=np.mod(np.arctan2(-(self.path[1][1] - pos_Y), self.path[1][0] - pos_X), 2*np.pi)
    
    def setDistMx(self,dist):
        self.dist_mx=dist

    def getDistMx(self):
        return self.dist_mx

    def setCor(self,cor):
        self.cor=cor

    def getCor(self):
        return self.cor

    def setS(self,s):
        self.S=s

    def getS(self):
        return self.S
    
    def setVisionDone(self, vision):
        self.vision = vision