from classes import Thymio
import local_navigation
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
        robot.setIntError(robot.getIntError()+error)
        i_speed = KI * robot.getIntError()
        d_speed = KD * ((error - robot.getPrevError()))
        robot.setPrevError(error)
        return p_speed+i_speed+d_speed

def turn(current_angle, robot):       #might need to adapt speed units   NEED: variable for current angle from odometry
    goal_angle = robot.getGoalAngle()

    error=goal_angle-current_angle
    if (abs(error)<=ANGLE_ERROR_TRESH):
        robot.setGoalReached(True)
        robot.setPrevError(0)
        robot.setIntError(0)
        robot.setSpeed(0)
    else:
        speed=PIDcontrol(error,robot)
        robot.setSpeedRight(speed)
        robot.setSpeedLeft(-speed)
        

def go_to_next_point(current_angle, current_position, obstacle, robot):  #current position given by odometry, obstacle is of class Local Navigation, check if other KP, KI, KD needed
    robot.setGoalReached(False)                                 #NEED: variable for obstacle from local navigation
    deltax= current_position[0]-robot.getPath()[1][0]
    deltay= current_position[1]-robot.getPath()[1][1]
    distance=deltax**2+deltay**2
    error=np.sqrt(distance)
    if (obstacle==0 and abs(distance)>DIST_TRESH**2):
        robot.setSpeed(MAX_SPEED)
    elif (obstacle==0 and abs(distance)<DIST_TRESH**2):
        fspeed = K*MAX_SPEED*error
        rspeed = PIDcontrol(current_angle-robot.getGoalAngle(),robot)
        robot.setSpeedRight(fspeed+rspeed)
        robot.setSpeedLeft(fspeed-rspeed)
    elif (obstacle==0 and abs(distance)<=DIST_ERROR_TRESH**2):
        robot.setGoalReached(True)
        robot.setPrevError(0)
        robot.setIntError(0)
        robot.setSpeed(0)
        robot.popPath()                                             
    else:
        pass        
