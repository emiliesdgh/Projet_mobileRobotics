import cv2
import os
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors
import math
from skimage import draw

class Global_Nav :
    
    def __init__ (self) :

        #Variables
        self.dist_mx = []
        self.cor = []
        self.s = 0
        self.graph = []
        self.current_node = 0
        self.path = []
        self.path_coord = []

    def dijkstra(self,robot):

        self.dist_mx = robot.getDistMx()
        self.s = robot.getS()
        graph = np.zeros((3,self.s))
        graph[0] = 3000 
        graph[0][0] = 0
        graph[2] = 42
        result = 0
        current_node = 0
        a = 0
        while 1:
            b = 0
            for i in range (0,self.s):
                if graph[1][i] == 0: 
                    b = b + 1
                    if graph[0][i] < graph[0][result]:
                        result = i
                    if b == 1:
                        result = i
            current_node = result 
            graph[1][current_node] = 1
            for i in range (0,self.s):
                if self.dist_mx[i][current_node] != 0:
                    if graph[1][i] == 0: 
                        new_score = graph[0][current_node] + self.dist_mx[i][current_node]
                        if new_score < graph[0][i]:
                            graph[0][i] = new_score 
                            graph[2][i] = current_node 
            if current_node == int(self.s-1):
                break
            for i in range (0,self.s):
                if graph[1][i] == 0: 
                    if graph[0][i] < graph[0][result]:
                        result = i
                        if graph[0][result] == 3000:
                            break

        self.graph = graph 
        self.current_node = current_node 

    def extract_path(self,robot):
        cor = robot.getCor()
        path = [self.current_node]
        node = self.current_node
        while node != 0:
            path.append(int(self.graph[2][node]))
            node = path[-1]
        path = np.flip(path)
        path_coord = [[robot.pos_X,robot.pos_Y]]
        a = 0
        for p in path : 
            a = a + 1 
            if a != 1:
                if a != np.size(path):
                    i = int(p-1)
                    path_coord.append(cor[i])
        path_coord.append([robot.goal_X,robot.goal_Y])
        robot.setPath(path_coord)
        self.path = path 
        self.path_coord = path_coord