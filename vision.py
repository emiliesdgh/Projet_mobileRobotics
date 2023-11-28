import cv2
import os
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors
import math
from skimage import draw
from classes import Thymio

#Define constants
ORIGINAL_DIM = [640,480]
max_y = 479
max_x = 639
min_x_y = 0
palier_min = 50
palier_max_x = 550
palier_max_y = 430
BLACK_THRESHOLD = 40
LOW_RED = (110,50,50)
HIGH_RED = (130,255,255)
LOW_BLUE = (104, 42, 0)
HIGH_BLUE = (105, 250, 250)
LOW_GREEN = (55,42,0)
HIGH_GREEN = (84,255,255)
red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)
line_width = 2
NB_SHAPES = 3
NB_CORNERS = 11
d_wide_cor = 45
mean_value_along_line = 125
max_nb_threshold = 3
v_inf = 3000
no_node = 42

frame = 0 
img = 0 
hsv = 0
thresh1 = 0
x_front = 0
x_back = 0
x_goal = 0
y_front = 0
y_back = 0
y_goal = 0
cor = []
cornerss = []
m_cor = []
path = []
dist_mx = []

def capture_image(): 
    # Put the image captured 'frame' in the class
    cap = cv2.VideoCapture(1) # --> 1 c'est la caméra sur mon ordi
    ret, frame = cap.read()
    img = cv2.medianBlur(frame,5)

def find_goal_pos():
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, LOW_RED, HIGH_RED)
    imask = mask>0
    red = np.zeros_like(img, np.uint8)
    red[imask] = img[imask]

    im = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
    ret,thresh1 = cv2.threshold(im,2,255,cv2.THRESH_BINARY)
    plt.imshow(thresh1,'gray')
    contours, _ = cv2.findContours( 
        thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    i = 0
    for contour in contours: 
        if i == 0: 
            i = 1
            continue
        M = cv2.moments(contour) 
        if M['m00'] != 0.0: 
            x_goal = int(M['m10']/M['m00']) 
            y_goal = int(M['m01']/M['m00']) 

def find_start_pos():
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, LOW_GREEN, HIGH_GREEN)
    green = cv2.bitwise_and(frame,frame, mask = mask)
    gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY) 
    _, thresh1 = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY) 
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(thresh1)
    [x_back,y_back] = maxLoc

def find_angle(robot):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOW_BLUE, HIGH_BLUE)
    blue = cv2.bitwise_and(frame,frame, mask = mask)
    gray = cv2.cvtColor(blue, cv2.COLOR_RGB2GRAY) 
    _, thresh1 = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY) 
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(thresh1)
    [x_front,y_front] = maxLoc
    if y_front < y_back : 
        if x_front > x_back : 
            teta = np.arccos((x_front-x_back)/(np.sqrt(np.power((x_front-x_back),2)+np.power((y_front-y_back),2))))
        if x_front <= x_back : 
            teta = np.pi - np.arccos((x_back-x_front)/(np.sqrt(np.power((x_front-x_back),2)+np.power((y_front-y_back),2))))
    if y_front >= y_back : 
        if x_front > x_back : 
            teta = 2*np.pi - np.arccos((x_front-x_back)/(np.sqrt(np.power((x_front-x_back),2)+np.power((y_front-y_back),2))))
        if x_front <= x_back : 
            teta = np.pi + np.arccos((x_back-x_front)/(np.sqrt(np.power((x_front-x_back),2)+np.power((y_front-y_back),2))))
    robot.setPositions(x_back,y_back,x_goal,y_goal,teta)

def find_corners():
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    ret, thresh2 = cv2.threshold(gray, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY) 
    plt.imshow(thresh2,'gray')
    contours, _ = cv2.findContours( 
        thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    i = 0
    m = []
    for contour in contours:  
        if i == 0: 
            i = 1
            continue
        M = cv2.moments(contour) 
        if M['m00'] != 0.0: 
            x = int(M['m10']/M['m00']) 
            y = int(M['m01']/M['m00']) 
            if len(m)<NB_SHAPES:
                m.append([x,y])
    th3 = cv2.adaptiveThreshold(thresh2,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                                cv2.THRESH_BINARY,11,2)
    corners = cv2.goodFeaturesToTrack(th3, NB_CORNERS, 0.01, 45) #(img, max_nb_corners, quality level, min distance)
    corners = np.int0(corners) 
    d = d_wide_cor #distance d'éloignement des points des corners 
    b = int(np.size(m)/2)
    cor = []
    m_cor = []
    cornerss = [] 
    for i in corners: 
        x, y = i.ravel()
        n_m = 0 
        d_min = np.sqrt(np.power((m[0][0]-x),2)+np.power((m[0][1]-y),2))
        for j in range (1,b):
            d_c = np.sqrt(np.power((m[j][0]-x),2)+np.power((m[j][1]-y),2))
            if d_c<d_min:
                d_min = d_c
                n_m = j
        a = np.abs((m[n_m][1]-y)/(m[n_m][0]-x))
        delta_x = np.abs(d/(np.sqrt(1+np.power(a,2))))
        delta_y = np.abs(a*delta_x)
        if m[n_m][0]<=x:
            if m[n_m][1]<=y:
                x1 = x + delta_x
                y1 = y + delta_y
            if m[n_m][1]>y:
                x1 = x + delta_x
                y1 = y - delta_y
        if m[n_m][0]>x:
            if m[n_m][1]<=y:
                x1 = x - delta_x
                y1 = y + delta_y
            if m[n_m][1]>y:
                x1 = x - delta_x
                y1 = y - delta_y
        x1 = int(x1)
        y1 = int(y1)
        if x1 < 0:
            x1 = 0
        if y1 < 0:
            y1 = 0
        if y1 >= 480:
            y1 = 479
        if x1 >= 640:
            x1 = 639
        if x1 >50:
            if x1<550:
                if y1 >50:
                    if y1<420:
                        cor.append([x1,y1])
        cornerss.append([x1,y1])
        m_cor.append(m[n_m])
        cv2.circle(img, (x1, y1), 3, 255, -1)

    for mo in  m:
        cv2.circle(img, [mo[0],mo[1]], 3, (0, 255, 0), cv2.FILLED)

def trace_contours(cornerss,m_cor,thresh1):
    a = int(np.size(cornerss)/2)
    b = int(np.size(m_cor)/2)
    for i in range (0,a):
        for j in range (0,b):
            if i != j:
                if m_cor[i] == m_cor[j]:
                    line = np.transpose(np.array(draw.line(cornerss[i][0], cornerss[i][1], cornerss[j][0], cornerss[j][1])))
                    data = thresh1[line[:, 1], line[:, 0]]
                    if np.mean(data) > mean_value_along_line:
                        cv2.line(thresh1, cornerss[i], cornerss[j], blue, 4)


def compute_dist_mx(robot):
    s = int(((np.size(cor))/2)+2) 
    dist_mx = np.zeros((s,s))
    for i in range(1,s):
        if i == (s-1):
            line = np.transpose(np.array(draw.line(x_back,y_back, x_g,y_g)))
            data = thresh1[line[:, 1], line[:, 0]]
            if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
                dist_mx[i][0] = np.sqrt(np.power((x_g-x_back),2)+np.power((y_g-y_back),2))
            continue 
        line = np.transpose(np.array(draw.line(x_back,y_back,cor[i-1][0],cor[i-1][1])))
        data = thresh1[line[:, 1], line[:, 0]]
        if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
            dist_mx[i][0] = np.sqrt(np.power((cor[i-1][0]-x_back),2)+np.power((cor[i-1][1]-y_back),2))
    for i in range(0,(s-2)):
        for j in range(0,s):
            if j == (i+1):
                continue 
            if j == 0 : 
                line = np.transpose(np.array(draw.line(x_back,y_back, cor[i][0],cor[i][1])))
                data = thresh1[line[:, 1], line[:, 0]]
                if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
                    dist_mx[j][i+1] = np.sqrt(np.power((cor[i][0]-x_back),2)+np.power((cor[i][1]-y_back),2))
                continue 
            if j == int(s-1):
                line = np.transpose(np.array(draw.line(x_g,y_g, cor[i][0],cor[i][1])))
                data = thresh1[line[:, 1], line[:, 0]]
                if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
                    dist_mx[j][i+1] = np.sqrt(np.power((cor[i][0]-x_g),2)+np.power((cor[i][1]-y_g),2))
                continue
            line = np.transpose(np.array(draw.line(cor[j-1][0],cor[j-1][1],cor[i][0],cor[i][1])))
            data = thresh1[line[:, 1], line[:, 0]]
            if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
                dist_mx[j][i+1] = np.sqrt(np.power((cor[i][0]-cor[j-1][0]),2)+np.power((cor[i][1]-cor[j-1][1]),2))
    for i in range(0,(s-1)):
        if i == 0:
            line = np.transpose(np.array(draw.line(x_back,y_back, x_g,y_g)))
            data = thresh1[line[:, 1], line[:, 0]]
            if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
                dist_mx[0][s-1] = np.sqrt(np.power((x_g-x_back),2)+np.power((y_g-y_back),2))
            continue 
        line = np.transpose(np.array(draw.line(x_g,y_g,cor[i-1][0],cor[i-1][1])))
        data = thresh1[line[:, 1], line[:, 0]]
        if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
            dist_mx[i][s-1] = np.sqrt(np.power((cor[i-1][0]-x_g),2)+np.power((cor[i-1][1]-y_g),2))
    robot.setDistMx(dist_mx)
    robot.setCor(cor)
    robot.setS(s)

