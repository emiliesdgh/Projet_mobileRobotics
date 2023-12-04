from classes import Thymio
import numpy as np

#Define constants
ANGLE_ERROR_TRESH = 0.1
DIST_ERROR_TRESH = 5               
MAX_SPEED=50               
KP=200                 
KI=10                  
KD=10                 
K=0.1                    


def PIDcontrol(error,robot):
        p_speed = KP * error
        robot.int_error += error
        i_speed = KI * robot.int_error
        d_speed = KD * (error - robot.prev_error)
        robot.prev_error = error
        return p_speed+i_speed+d_speed

def turn(current_angle, robot, node):
    robot.goal_reached_t = False
    error=robot.goal_angle-robot.theta
    if (abs(error)<=ANGLE_ERROR_TRESH):
        robot.goal_reached_t = True
        robot.goal_reached_f = False
        robot.prev_error = 0
        robot.int_error = 0
        robot.setSpeedRight(0,node)
        robot.setSpeedLeft(0,node)
    else:
        speed=int(min(MAX_SPEED, max(-MAX_SPEED, PIDcontrol(error,robot))))
        print(speed)
        robot.setSpeedRight(speed,node)
        robot.setSpeedLeft(-speed,node)   
            

def go_to_next_point(current_angle, current_position, obstacle, robot, node): 
    if len(robot.path) > 1:
        robot.goal_reached_f = False    
        deltax= current_position[0]-robot.path[1][0]
        deltay= current_position[1]-robot.path[1][1]
        distance=deltax**2+deltay**2
        print(deltax,deltay,'distance =',distance)
        error=np.sqrt(distance)
        if (obstacle==False and abs(deltax)>=DIST_ERROR_TRESH and abs(deltay)>=DIST_ERROR_TRESH):
            print('first loop')
            fspeed = K*MAX_SPEED*error
            rspeed = PIDcontrol(current_angle-robot.goal_angle,robot)
            leftspeed = min(MAX_SPEED, max(-MAX_SPEED, fspeed-rspeed))
            rightspeed = min(MAX_SPEED, max(-MAX_SPEED, fspeed+rspeed))
            robot.setSpeedRight(rightspeed,node)
            robot.setSpeedLeft(leftspeed,node)
            robot.setAngle()
        else:
            print('second loop')
            robot.goal_reached_f = True
            robot.goal_reached_t = False
            robot.prev_error = 0
            robot.int_error = 0
            robot.setSpeedRight(0,node)
            robot.setSpeedLeft(0,node)
            robot.path.pop(1)
            robot.setAngle()
    else:
        pass     

