

import matplotlib.pyplot as plt
import numpy as np

class LocalNavigation :

    def __init__(self, prox_horizontal, motor_left_target, motor_right_target) :

        self.proximity_sensors = prox_horizontal #proxymity_sensors, read only variable
        # prox_horizontal [0,1,2,3,4] -> front sensors [5,6] -> back sensors

        # initialization to 0 of the motors
        self.motor_left = 0 #motor_left_target --> target value set by the user (not read only)
        self.motor_right = 0 #motor_left_target
        self.state = 0

    def obstacle(self, prox_horizontal) :

        self.proximity_sensors = prox_horizontal

        if self.proximity_sensors > 0 :

            #smth smth
            self.state = 1
        
        else :
            self.state = 0

        return self.state


    def avoidance(self, prox_horizontal, motor_left_target, motor_right_target) :

        self.proximity_sensors = prox_horizontal #proxymity_sensors
        self.motor_left = motor_left_target
        self.motor_right = motor_right_target








speed0 = 100       # nominal speed
speedGain = 2      # gain used with ground gradient
obstThrL = 10      # low obstacle threshold to switch state 1->0
obstThrH = 20      # high obstacle threshold to switch state 0->1
obstSpeedGain = 5  # /100 (actual gain: 5/100=0.05)

state = 1          # 0=gradient, 1=obstacle avoidance
obst = [0,0]       # measurements from left and right prox sensors

#timer_period[0] = 10   # 10ms sampling time


 
def timer0():
    global prox_ground_delta, prox_horizontal, motor_left_target, motor_right_target, state, obst, obstThrH, obstThrL, obstSpeedGain, speed0, speedGain 
    # acquisition from ground sensor for going toward the goal
    diffDelta = prox_ground_delta[1] - prox_ground_delta[0]

    # acquisition from the proximity sensors to detect obstacles
    obst = [prox_horizontal[0], prox_horizontal[4]]
    
    # tdmclient does not support yet multiple and/or in if statements:
    if state == 0: 
        # switch from goal tracking to obst avoidance if obstacle detected
        if (obst[0] > obstThrH):
            state = 1
        elif (obst[1] > obstThrH):
            state = 1
    elif state == 1:
        if obst[0] < obstThrL:
            if obst[1] < obstThrL : 
                # switch from obst avoidance to goal tracking if obstacle got unseen
                state = 0
    if  state == 0 :
        # goal tracking: turn toward the goal
        leds_top = [0,0,0]
        motor_left_target = speed0 - speedGain * diffDelta
        motor_right_target = speed0 + speedGain * diffDelta
    else:
        leds_top = [30,30,30]
        # obstacle avoidance: accelerate wheel near obstacle
        motor_left_target = speed0 + obstSpeedGain * (obst[0] // 100)
        motor_right_target = speed0 + obstSpeedGain * (obst[1] // 100)

def rotate(angle, coords):
    """
    Rotates the coordinates of a matrix by the desired angle
    :param angle: angle in radians by which we want to rotate
    :return: numpy.array() that contains rotated coordinates
    """
    R = np.array(((np.cos(angle), -np.sin(angle)),
                  (np.sin(angle),  np.cos(angle))))
    
    return R.dot(coords.transpose()).transpose()

#other way to do is using the potential field navigation (view exo 4)