from classes import Thymio
import numpy as np
import time

#Define constants
ANGLE_ERROR_TRESH = 0.1
DIST_ERROR_TRESH = 5              
MAX_SPEED=150               
NOM_SPEED=70
KP=100                                
K=0.5                  


def Pcontrol(error,robot):
    """ 
    This function computes the angular speed necessary to correct the angular error of the robot specified in the input.

    Inputs:

    error: the angular error
    robot: the robot that needs correcting

    Output:

    p_speed: the speed given by the proportional computation
    """

    error = (error + np.pi) %(2*np.pi) - np.pi
    p_speed = KP * error

    return p_speed

def turn(current_angle, robot, node):
    """
    This function makes the robot turn towards the next point to reach

    Inputs:

    current_angle: the angle of the robot
    robot: the robot that needs turning
    node: the node through which information is sent to the Thymio (physical object)
    """

    if len(robot.path) > 1:
        error=(robot.goal_angle-current_angle)

        if (abs(error)<=ANGLE_ERROR_TRESH):
            robot.goal_reached_t = True
            robot.goal_reached_f = False
            robot.prev_error = 0
            robot.int_error = 0
            robot.setSpeedRight(0,node)
            robot.setSpeedLeft(0,node)
        else:
            speed=int(min(MAX_SPEED, max(-MAX_SPEED, Pcontrol(error,robot))))
            robot.setSpeedRight(speed,node)
            robot.setSpeedLeft(-speed,node)   
    else:
        pass 
                

def go_to_next_point(current_angle, current_position, robot, node): 
    """
    This function makes the robot advance to the next point to reach

    Inputs:

    current_angle: the angle of the robot
    current_position: the position of the robot
    obstacle: is in obstacle detected
    robot: the robot that needs advancing
    node: the node through which information is sent to the Thymio (physical object)
    """

    if len(robot.path) > 1:  
        robot.setAngle(current_position[0],current_position[1])
        deltax= current_position[0]-robot.path[1][0]
        deltay= current_position[1]-robot.path[1][1]
        distance=deltax**2+deltay**2
        error=np.sqrt(distance)
        
        if ((abs(deltax)<=DIST_ERROR_TRESH) and (abs(deltay)<=DIST_ERROR_TRESH)):
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
            
            fspeed = K*error+NOM_SPEED
            rspeed = Pcontrol(robot.goal_angle - current_angle,robot)

            leftspeed = int(min(MAX_SPEED, max(-MAX_SPEED, fspeed-rspeed)))
            rightspeed = int(min(MAX_SPEED, max(-MAX_SPEED, fspeed+rspeed)))

            robot.setSpeedRight(rightspeed,node)
            robot.setSpeedLeft(leftspeed,node)
            
    else:
        pass     

