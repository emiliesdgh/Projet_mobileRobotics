import time

#Define constants
N=0
S=1
E=2
W=3
TURN_SPEED=50
HALF_TURN=4
QUARTER_TURN=2
ADVANCE_SPEED=100
ADVANCE_TIME=1


def turn(current_angle, path):
        current_pos=path[0]
        next_pos=path[1]
        delta_x=current_pos[0]-next_pos[0]
        delta_y=current_pos[1]-next_pos[1]
        if delta_x != 0:
            if delta_x > 0:
                motor_target_left=TURN_SPEED
                motor_target_right=-TURN_SPEED
                time.sleep() #don't use sleep
                motor_target_left=0
                motor_target_right=0
            else:
                motor_target_left=0
                motor_target_right=0
                time.sleep()
                motor_target_left=0
                motor_target_right=0
        if delta_y != 0:
            if delta_y > 0:
                motor_target_left=0
                motor_target_right=0
                time.sleep()
                motor_target_left=0
                motor_target_right=0
            else:
                motor_target_left=0
                motor_target_right=0
                time.sleep()
                motor_target_left=0
                motor_target_right=0  
        path.pop(0)  


def go_to_next_point(path):
    motor_target_left=ADVANCE_SPEED
    motor_target_right=ADVANCE_SPEED
    time.sleep()

#ask about: how does the filter affect me here: correct the angle/position
#           how do I get the angles: camera
#           correction of speed only corresponds to mine here: just use my robot

#PID controller for the angle: smoothen out turn function    
# faire étape avancer et étape tourner séparé pour chaque point!!! (petites fonctions)
# 
#        