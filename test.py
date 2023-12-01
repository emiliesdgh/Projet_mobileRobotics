from tdmclient import ClientAsync,aw
from classes import Thymio
import local_navigation_2
import local_navigation
import motion_control
import numpy as np


client = ClientAsync()
node = aw(client.wait_for_node())
aw(node.lock())
aw(node.wait_for_variables())

Thym=Thymio()
Thym.setPath([[0,0],[0,10],[2,50]])
current_pos1_y=np.linspace(0,10,3)
current_pos1_arr=[[0,y] for y in current_pos1_y]
current_pos2_x=np.linspace(0,2,3)
current_pos2_y=np.linspace(10,50,3)
current_pos2_arr=[[current_pos2_x[i],current_pos2_y[i]] for i in range(3)]
current_pos_arr= np.vstack((current_pos1_arr, current_pos2_arr))
current_angle=np.linspace(0,2*np.pi,720)
print(current_pos_arr[1])
test_functions=1

for i in range(6):

    for angle in current_angle:    
        if not Thym.goal_reached_t:
            motion_control.turn(angle,Thym,node)
            print(angle, Thym.goal_angle)
        if Thym.goal_reached_t and not Thym.goal_reached_f:
            motion_control.go_to_next_point(0,current_pos_arr[i],0,Thym,node)