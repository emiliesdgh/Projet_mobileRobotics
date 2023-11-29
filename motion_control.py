from classes import Thymio
import numpy as np

#Define constants
ANGLE_ERROR_TRESH = 0.02
DIST_ERROR_TRESH = 0.1      #find this value
DIST_TRESH=2                #find this value
MAX_SPEED=150               #find this value
KP=200                      #find this value
KI=10                       #find this value
KD=5                        #find this value
K=200                       #find this values


def PIDcontrol(error,robot):
        p_speed = KP * error
        robot.int_error += error
        i_speed = KI * robot.int_error
        d_speed = KD * (error - robot.prev_error)
        robot.prev_error = error
        return p_speed+i_speed+d_speed

def turn(current_angle, robot, node):       #might need to adapt speed units   NEED: variable for current angle from odometry
    error=robot.goal_angle-current_angle
    if (abs(error)<=ANGLE_ERROR_TRESH):
        robot.goal_reached = True
        robot.prev_error = 0
        robot.int_error = 0
        robot.setSpeedRight(0,node)
        robot.setSpeedLeft(0,node)
    else:
        speed=min(MAX_SPEED, max(-MAX_SPEED, PIDcontrol(error,robot)))
        robot.setSpeedRight(speed,node)
        robot.setSpeedLeft(-speed,node)   
            

def go_to_next_point(current_angle, current_position, obstacle, robot, node): 
    robot.goal_reached = False    #not sure this is useful anymore
    deltax= current_position[0]-robot.path[1][0]
    deltay= current_position[1]-robot.path[1][1]
    distance=deltax**2+deltay**2
    error=np.sqrt(distance)
    if (obstacle==0 and abs(distance)>DIST_TRESH**2):
        rspeed = PIDcontrol(current_angle-robot.goal_angle,robot)
        leftspeed = min(MAX_SPEED, max(-MAX_SPEED, MAX_SPEED-rspeed))
        rightspeed = min(MAX_SPEED, max(-MAX_SPEED, MAX_SPEED+rspeed))
        robot.setSpeedRight(MAX_SPEED,node)
        robot.setSpeedLeft(MAX_SPEED,node)
    elif (obstacle==0 and abs(distance)<DIST_TRESH**2):
        fspeed = K*MAX_SPEED*error
        rspeed = PIDcontrol(current_angle-robot.goal_angle,robot)
        leftspeed = min(MAX_SPEED, max(-MAX_SPEED, fspeed-rspeed))
        rightspeed = min(MAX_SPEED, max(-MAX_SPEED, fspeed+rspeed))
        robot.setSpeedRight(rightspeed,node)
        robot.setSpeedLeft(leftspeed,node)
    elif (obstacle==0 and abs(distance)<=DIST_ERROR_TRESH**2):
        robot.goal_reached = True
        robot.prev_error = 0
        robot.int_error = 0
        robot.setSpeedRight(0,node)
        robot.setSpeedLeft(0,node)
        robot.setPath(robot.path.pop(0))                                  
    else:
        pass              
