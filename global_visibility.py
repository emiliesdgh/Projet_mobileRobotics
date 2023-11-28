import cv2
import os
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors
import math
from skimage import draw

dist_mx = []
graph = []
current_node = 0
path = []
path_coord = []
cor = []
s = 0 

def dijkstra(robot):

    dist_mx = robot.getDistMx()
    s = robot.getS()
    graph = np.zeros((3,s))
    graph[0] = 3000 
    graph[0][0] = 0
    graph[2] = 42
    result = 0
    a = 0
    while 1:
        b = 0
        for i in range (0,s):
            if graph[1][i] == 0: 
                b = b + 1
                if graph[0][i] < graph[0][result]:
                    result = i
                if b == 1:
                    result = i
        current_node = result 
        graph[1][current_node] = 1
        for i in range (0,s):
            if dist_mx[i][current_node] != 0:
                if graph[1][i] == 0: 
                    new_score = graph[0][current_node] + dist_mx[i][current_node]
                    if new_score < graph[0][i]:
                        graph[0][i] = new_score 
                        graph[2][i] = current_node 
        if current_node == int(s-1):
            break
        for i in range (0,s):
            if graph[1][i] == 0: 
                if graph[0][i] < graph[0][result]:
                    result = i
                    if graph[0][result] == 3000:
                        break


def extract_path(robot):
    cor = robot.getCor()
    path = [current_node]
    node = current_node
    while node != 0:
        path.append(int(graph[2][node]))
        node = path[-1]
    path = np.flip(path)
    path_coord = []
    a = 0
    for p in path : 
        a = a + 1 
        if a != np.size(path):
            i = int(p-1)
            path_coord.append(cor[i])
    robot.setPath(path_coord)