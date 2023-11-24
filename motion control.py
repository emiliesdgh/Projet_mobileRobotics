from classes import Thymio
import local_navigation
import numpy as np

#Define constants
ANGLE_ERROR_TRESH = 0.02
DIST_ERROR_TRESH = 0.1      #find this value
DIST_TRESH=2                #find this value
MAX_SPEED=150               #find this value
KP=200
KI=10
KD=5



def turn(current_angle, path, robot):       #might need to adapt speed units
    deltax= path[0][0]-path[1][0]
    deltay= path[0][1]-path[1][1]

    if deltax !=0:
        if deltax < 0:
            goal_angle = np.pi
        else:
            goal_angle = 0
    if deltay != 0:
        if deltay < 0:
            goal_angle = 1/2*np.pi
        else:
            goal_angle = 3/2*np.pi
    else:
        goal_angle = current_angle

    error=goal_angle-current_angle
    if (abs(error)<=ANGLE_ERROR_TRESH):
        robot.setGoalReached(True)
        robot.setPrevError(0)
        robot.setIntError(0)
        robot.setSpeed(0)
    else:
        p_speed = KP * error
        robot.setIntError(robot.getIntError()+error)
        i_speed = KI * robot.getIntError()
        d_speed = KD * ((error - robot.getPrevError()))
        robot.setPrevError(error)
        robot.setSpeed(p_speed+i_speed+d_speed)
        

def go_to_next_point(path, current_position, obstacle, robot): #current position given by odometry, obstacle is of class Local Navigation, check if other KP, KI, KD needed
    robot.setGoalReached(False)
    deltax= current_position[0]-path[1][0]
    deltay= current_position[1]-path[1][1]
    distance=deltax**2+deltay**2
    error=np.sqrt(distance)
    if (obstacle==0 and abs(distance)>DIST_TRESH**2):
        robot.setSpeed(MAX_SPEED)
    elif (obstacle==0 and abs(distance)<DIST_TRESH**2):
        p_speed = KP * error
        robot.setIntError(robot.getIntError()+error)
        i_speed = KI * robot.getIntError()
        d_speed = KD * ((error - robot.getPrevError()))
        robot.setPrevError(error)
        robot.setSpeed(p_speed+i_speed+d_speed)
    elif (obstacle==0 and abs(distance)<=DIST_ERROR_TRESH**2):
        robot.setGoalReached(True)
        robot.setPrevError(0)
        robot.setIntError(0)
        robot.setSpeed(0)
        path.pop(0)                                                 #adapt this with the class
    else:
        pass      