'''from tdmclient import ClientAsync
client = ClientAsync()
node = await client.wait_for_node()
await node.lock()'''

#from classes import Thymio

import classes

import numpy as np

test_functions = True

#aw(node.wait_for_variables())

def motors(l_speed=500, r_speed=500, verbose=False):
    """
    Sets the motor speeds of the Thymio 
    param l_speed: left motor speed
    param r_speed: right motor speed
    param verbose: whether to print status messages or not
    """
    # Printing the speeds if requested
    if verbose:
        print("\t\t Setting speed : ", l_speed, r_speed)
    return {
        "motor.left.target": [l_speed],
        "motor.right.target": [r_speed],
    }

if test_functions:
    aw(node.set_variables(motors(100, 100))) #test with lower speed value
    aw(client.sleep(2))
    aw(node.set_variables(motors(0, 0)))


#test 28.11.23 - 16h30

def test_saw_osb(obs_threshold, verbose=False): ## fonction necessaire !
    """
    Tests whether one of the proximity sensors saw a wall
    param wall_threshold: threshold starting which it is considered that the sensor saw a wall
    param verbose: whether to print status messages or not
    """

    if any([x>obs_threshold for x in node['prox.horizontal'][:-2]]):
        if verbose: print("\t\t Saw a obstacle")
        return True
    
    return False

def clockwise(verbose=False) :
    #print(prox.horizontal[1])
    prox = list(node["prox.horizontal"]) + [0]
    print(prox[1])
    print(prox[3])
    if prox[1] > prox[3] :
        return True
    
    return False

async def obstacle_avoidance(motor_speed=100, obs_threshold=500, verbose=False): #, clockwise = False):
    """
    Wall following behaviour of the FSM
    param motor_speed: the Thymio's motor speed
    param wall_threshold: threshold starting which it is considered that the sensor saw a wall
    param white_threshold: threshold starting which it is considered that the ground sensor saw white
    param verbose: whether to print status messages or not
    """
    
    if verbose: print("Starting wall following behaviour")
    obs_avoided = False
    
    prox = list(node["prox.horizontal"]) + [0]
    clockwise_true = False
    
    if verbose: print("\t Moving forward")
    await node.set_variables(motors(motor_speed, motor_speed))
           
    prev_state="forward"
    
    while not obs_avoided :

    ##tester si les conditions des if/elif fonctionnent!!    
        if test_saw_osb(obs_threshold, verbose=False):
            
            if prev_state=="forward": 
                
                if verbose: 
                    
                    print("\tSaw wall, turning")
                    print("\clockwise OR counterclockwise")

                    if clockwise(verbose=False) :
                        await print_sensor_values('prox.horizontal')

                        await node.set_variables(motors(motor_speed, -motor_speed))
                        print("\tSaw wall, turning clockwise CLOCCCKKK")
                        clockwise_true = True

                    else :

                        await node.set_variables(motors(-motor_speed, motor_speed))
                        print("\tSaw wall, turning COUNTEERRR counterclockwise")
                    
                prev_state="turning"
        
        else:
            if prev_state=="turning": 
                if verbose: 
                    print("\t Contourning obstacle")

                    if clockwise_true :
                        await node.set_variables(motors(motor_speed-40, motor_speed))
                        prev_state="forward"
                        print("\t CLOOOOCCCKKK Clockwise")
                        await client.sleep(18)
                        await node.set_variables(motors(motor_speed, -motor_speed))
                        await client.sleep(2)
                        obs_avoided = True
                        
                    else :
                        await node.set_variables(motors(motor_speed, motor_speed-40))
                        prev_state="forward"
                        print("\t COUNTEEERRR Counterclockwise")
                        await client.sleep(18)
                        await node.set_variables(motors(-motor_speed, motor_speed))
                        await client.sleep(2)
                        obs_avoided = True

        await client.sleep(0.1) #otherwise, variables would not be updated
    return 

if test_functions:
    aw(obstacle_avoidance(verbose=True))
    aw(node.set_variables(motors(0, 0)))



'''
def test_saw_wall(wall_threshold, verbose=False): ## fonction necessaire !
    """
    Tests whether one of the proximity sensors saw a wall
    param wall_threshold: threshold starting which it is considered that the sensor saw a wall
    param verbose: whether to print status messages or not
    """

    if any([x>wall_threshold for x in node['prox.horizontal'][:-2]]):
        if verbose: print("\t\t Saw a wall")
        return True
    
    return False

def clockwise(verbose=False) :
    #print(prox.horizontal[1])
    prox = list(node["prox.horizontal"]) + [0]
    print(prox[1])
    print(prox[3])
    if prox[1] > prox[3] :
        return True
    
    return False

async def wall_following(motor_speed=100, wall_threshold=500, white_threshold=200, verbose=False): #, clockwise = False):
    """
    Wall following behaviour of the FSM
    param motor_speed: the Thymio's motor speed
    param wall_threshold: threshold starting which it is considered that the sensor saw a wall
    param white_threshold: threshold starting which it is considered that the ground sensor saw white
    param verbose: whether to print status messages or not
    """
    
    if verbose: print("Starting wall following behaviour")
    saw_black = False

    clockwise_true = False
    
    if verbose: print("\t Moving forward")
    await node.set_variables(motors(motor_speed, motor_speed))
           
    prev_state="forward"
    
    while not saw_black:

    ##tester si les conditions des if/elif fonctionnent!!    
        if test_saw_wall(wall_threshold, verbose=False):
            
            if prev_state=="forward": 
                
                if verbose: 
                    
                    print("\tSaw wall, turning")
                    print("\clockwise OR counterclockwise")

                    if clockwise(verbose=False) :
                        await print_sensor_values('prox.horizontal')

                        await node.set_variables(motors(motor_speed, -motor_speed))
                        print("\tSaw wall, turning clockwise CLOCCCKKK")
                        clockwise_true = True

                    else :

                        await node.set_variables(motors(-motor_speed, motor_speed))
                        print("\tSaw wall, turning COUNTEERRR counterclockwise")
                    
                prev_state="turning"
        
        else:
            if prev_state=="turning": 
                if verbose: 
                    print("\t Contourning obstacle")

                    if clockwise_true :
                        await node.set_variables(motors(motor_speed-40, motor_speed))
                        prev_state="forward"
                        print("\t CLOOOOCCCKKK Clockwise")
                        await client.sleep(18)
                        await node.set_variables(motors(motor_speed, -motor_speed))
                        await client.sleep(2)
                        saw_black = True
                        
                    else :
                        await node.set_variables(motors(motor_speed, motor_speed-40))
                        prev_state="forward"
                        print("\t COUNTEEERRR Counterclockwise")
                        await client.sleep(18)
                        await node.set_variables(motors(-motor_speed, motor_speed))
                        await client.sleep(2)
                        saw_black = True

        await client.sleep(0.1) #otherwise, variables would not be updated
    return 

if test_functions:
    await wall_following(verbose=True)
    await node.set_variables(motors(0, 0))'''