from classes import Thymio
import numpy as np
import time

#Define constants
ANGLE_ERROR_TRESH = 0.1
DIST_ERROR_TRESH = 5              
MAX_SPEED=150               
NOM_SPEED=70
KP=100                 
KI=20                  
KD=10                 
K=0.5                  


def PIDcontrol(error,robot):
    """ #dt=time.time()-robot.prev_time
    if (error > np.pi):
        error=np.pi-error
        p_speed = KP * error
        robot.int_error += error
        i_speed = KI * robot.int_error
        d_speed = KD * (error - robot.prev_error)#/dt
        robot.prev_error = error

    elif (error < -np.pi):
        error=-error-np.pi
        p_speed = KP * error
        robot.int_error += error
        i_speed = KI * robot.int_error
        d_speed = KD * (error - robot.prev_error)#/dt
        robot.prev_error = error
    else: """
    error = (error + np.pi) %(2*np.pi) - np.pi
    p_speed = KP * error
    robot.int_error += error
    i_speed = KI * robot.int_error
    d_speed = KD * (error - robot.prev_error)#/dt
    robot.prev_error = error
    #robot.prev_time=time.time()
    return p_speed#+i_speed#+d_speed

def turn(current_angle, robot, node):

    if len(robot.path) > 1:
        error=(robot.goal_angle-current_angle)
        #print("error_angle", error)
        #print("error_angle_norm", error%(2*np.pi))
        if (abs(error)<=ANGLE_ERROR_TRESH):
            robot.goal_reached_t = True
            robot.goal_reached_f = False
            robot.prev_error = 0
            robot.int_error = 0
            robot.setSpeedRight(0,node)
            robot.setSpeedLeft(0,node)
        else:
            speed=int(min(MAX_SPEED, max(-MAX_SPEED, PIDcontrol(error,robot))))
            #print(speed)
            
            robot.setSpeedRight(speed,node)
            robot.setSpeedLeft(-speed,node)   
    else:
        pass 
                

def go_to_next_point(current_angle, current_position, obstacle, robot, node): 
    if len(robot.path) > 1:  
        robot.setAngle(current_position[0],current_position[1])
        deltax= current_position[0]-robot.path[1][0]
        deltay= current_position[1]-robot.path[1][1]
        distance=deltax**2+deltay**2
        #print(deltax,deltay,'distance =',distance)
        error=np.sqrt(distance)
        if (obstacle==False and ((abs(deltax)<=DIST_ERROR_TRESH) and (abs(deltay)<=DIST_ERROR_TRESH))):
            robot.goal_reached_f = True
            robot.goal_reached_t = False
            robot.prev_error = 0
            robot.int_error = 0
            robot.setSpeedRight(0,node)
            robot.setSpeedLeft(0,node)
            robot.path.pop(1)
            if len(robot.path) >1:
                robot.setAngle(current_position[0],current_position[1])

        else:
            
            #print('first loop')
            fspeed = K*error+NOM_SPEED
            #print('fspeed=', fspeed)
            rspeed = PIDcontrol(robot.goal_angle - current_angle,robot)
            #print("rspeed",rspeed)
            leftspeed = int(min(MAX_SPEED, max(-MAX_SPEED, fspeed-rspeed)))
            rightspeed = int(min(MAX_SPEED, max(-MAX_SPEED, fspeed+rspeed)))
            #print("speed=", leftspeed, rightspeed)
            robot.setSpeedRight(rightspeed,node)
            robot.setSpeedLeft(leftspeed,node)
            
    else:
        pass     

