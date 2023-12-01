import classes
from classes import Thymio

from tdmclient import ClientAsync, aw
import numpy as np

test_functions = True

def test_the_fct1(Thymio, node, client, test_functions) :
    if test_functions:
        #aw(node.set_variables(motors(100, 100))) #test with lower speed value
        Thymio.setSpeedRight(100,node)
        Thymio.setSpeedLeft(100,node)
        aw(client.sleep(2))
        #client.sleep(2)
        #aw(node.set_variables(motors(0, 0)))
        Thymio.setSpeedRight(0,node)
        Thymio.setSpeedLeft(0,node)


def test_saw_osb(Thymio, node, obs_threshold, verbose=False) :

    '''This function verrifies if one of the proximity horiontal sensors
        sees and obstacle within the giving threashold.'''
    
    if any([x>obs_threshold for x in node['prox.horizontal'][:-2]]):
        #if verbose: print("\t\t Saw a obstacle")
        return True
    return False

def clockwise(node, verbose=False) :

    '''This function verrifies if the obstacle to avoid is more on its right or left,
        therefore, the Thymio will contourn it accordingly.'''

    prox = list(node["prox.horizontal"]) + [0]
    print(prox[1])
    print(prox[3])
    if prox[1] > prox[3] :
        return True
    
    return False


def obstacle_avoidance(Thymio, node, client, motor_speed=100, obs_threshold=500, verbose=False): #, clockwise = False):
    """
    Wall following behaviour of the FSM
    param motor_speed: the Thymio's motor speed
    param wall_threshold: threshold starting which it is considered that the sensor saw a wall
    param white_threshold: threshold starting which it is considered that the ground sensor saw white
    param verbose: whether to print status messages or not
    """


    #if verbose: print("Starting wall following behaviour")

    obs_avoided = False     # Booleen to state when the obstacle has been avoided or not
    clockwise_true = False  # Booleen to state if the Thymio has to contourn on the left or right
    
    #if verbose: print("\t Moving forward")

    Thymio.setSpeedLeft(motor_speed, node)
    Thymio.setSpeedRight(motor_speed, node)
           
    prev_state = "turning" # Stated of movement of the Thymio
    
    while not obs_avoided :     # As long as the obstacle isn't avoided, stay in the while loop 
    
        #print("\in while loop")
    ##tester si les conditions des if/elif fonctionnent!!    
        if test_saw_osb(Thymio, node, obs_threshold, verbose=False):
            #print("in first test of obs")
            
            if prev_state == "turning": # little rotation on it's own to then do the contourning
                
                #if verbose: 
                    
                    #print("\tSaw wall, turning")
                    #print("\clockwise OR counterclockwise")

                    #if clockwise(node, prox, verbose=False) :
                if clockwise(node, verbose=False) :
                    
                    #print("in second test of obs")

                    Thymio.setSpeedLeft(motor_speed, node)
                    Thymio.setSpeedRight(-motor_speed, node)

                    print("\tSaw wall, turning clockwise CLOCCCKKK")
                    clockwise_true = True   # Thymio needs to contourn the obstacle clockwise

                else :
                    # Thymio needs to contourn the obstacle counterclockwise
                    
                    Thymio.setSpeedLeft(-motor_speed, node)
                    Thymio.setSpeedRight(motor_speed, node)

                    print("\tSaw wall, turning COUNTEERRR counterclockwise")
                    
                prev_state = "contourning" # Change the state so the Thymio countourns the obstacle fully
        
        else:
            if prev_state == "contourning": 
                #if verbose: 
                    #print("\t Contourning obstacle")

                if clockwise_true :

                    Thymio.setSpeedLeft(motor_speed-40, node)
                    Thymio.setSpeedRight(motor_speed, node)

                    prev_state = "forward"
                    print("\t CLOOOOCCCKKK Clockwise")
                    aw(client.sleep(18))

                    Thymio.setSpeedLeft(motor_speed, node)
                    Thymio.setSpeedRight(-motor_speed, node)

                    aw(client.sleep(2))

                    obs_avoided = True  # obstacle has been avoided, change the state booleen
                    
                else :

                    Thymio.setSpeedLeft(motor_speed,node)
                    Thymio.setSpeedRight(motor_speed-40,node)

                    prev_state="turning"
                    print("\t COUNTEEERRR Counterclockwise")
                    aw(client.sleep(18))

                    Thymio.setSpeedLeft(-motor_speed,node)
                    Thymio.setSpeedRight(motor_speed,node)

                    aw(client.sleep(2))

                    obs_avoided = True  # obstacle has been avoided, change the state booleen

        aw(client.sleep(0.1)) #otherwise, variables would not be updated

    if obs_avoided :
        '''Funtion to stop the Thymio's movement after avoiding the obstacle'''
        #test_the_fct2(Thymio, node, client, test_functions)
        Thymio.setSpeedRight(0,node)
        Thymio.setSpeedLeft(0,node)



