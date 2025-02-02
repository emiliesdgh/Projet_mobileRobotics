import cv2
import os
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors
import math
from skimage import draw
from classes import Thymio
from IPython.display import clear_output
from operator import itemgetter

class Vision :
    
    def __init__ (self) :

        #Constants
        self.ORIGINAL_DIM = [519,398]
        self.max_y = 398
        self.max_x = 519
        self.min_x_y = 0
        self.palier_min = 50
        self.palier_max_x = 500
        self.palier_max_y = 370
        self.BLACK_THRESHOLD = 40
        self.LOW_RED = (115,50,130)
        self.HIGH_RED = (135,200,170)
        self.LOW_BLUE = (100,150,0)
        self.HIGH_BLUE = (140,255,255)
        self.LOW_GREEN = (55,42,0)
        self.HIGH_GREEN = (84,255,255)
        self.LOW_YELLOW = (90, 85, 110)
        self.HIGH_YELLOW = (110, 155, 175)
        self.redpx = (255,0,0)
        self.greenpx = (0,255,0)
        self.bluepx = (0,0,255)
        self.line_width = 2
        self.NB_SHAPES = 3
        self.NB_CORNERS = 11
        self.d_wide_cor = 75 #au lieu de 60
        self.mean_value_along_line = 185
        self.max_nb_threshold = 3
        self.v_inf = 3000
        self.no_node = 42
        self.kid_threshold = 200

        #Variables
        self.frame = 0 
        self.img = 0 
        self.img_out = 0
        self.thresh1 = 0
        self.thresh2 = 0
        self.x_front = 0
        self.x_back = 0
        self.x_goal = 0
        self.y_front = 0
        self.y_back = 0
        self.y_goal = 0
        self.cor = []
        self.corners = []
        self.m_cor = []
        self.path = []
        self.dist_mx = []
        self.border_points = []
        self.M = 0 
        self.img_show = 0 
        self.positions = []

    def capture_image(self,cap): 
        #capture a frame out of the video that will be used through CV part 
        #filter the frame to erase some of the noise of the camera
        #Nb : we captured two frames because sometimes, at first connection of the camera to the PC, the 
        #first frame catpured is "yellowish" and unusable 
        ret, self.frame = cap.read()
        ret, self.frame = cap.read()

        kernel = np.ones((5,5),np.float32)/25
        img = cv2.filter2D(self.frame,-1,kernel)
        img = cv2.blur(img,(5,5))
        self.img = cv2.medianBlur(img,5)

    def find_borders(self):
        border_points = []

        kernel = np.ones((5,5),np.float32)/25
        img = cv2.filter2D(self.frame,-1,kernel)
        img = cv2.blur(img,(5,5))
        img = cv2.medianBlur(img,5)

        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(hsv, self.LOW_YELLOW, self.HIGH_YELLOW)
        imask = mask>0
        yellow = np.zeros_like(img, np.uint8)
        yellow[imask] = img[imask]
        im = cv2.cvtColor(yellow, cv2.COLOR_BGR2GRAY)
        im = cv2.medianBlur(im,5)
        ret,thresh1 = cv2.threshold(im,20,255,cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours( 
            thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        i = 0
        for contour in contours: 
            if i == 0: 
                i = 1
                continue
            M = cv2.moments(contour) 
            if M['m00'] != 0.0: 
                x = int(M['m10']/M['m00']) 
                y = int(M['m01']/M['m00']) 
                border_points.append([x,y])

        self.border_points = border_points

    def tilt_image(self):
        b = sorted(self.border_points, key=itemgetter(0))
        border_left = [b[0],b[1]]
        border_right = [b[2],b[3]]
        border_left = sorted(border_left, key=itemgetter(1))
        border_right = sorted(border_right, key=itemgetter(1))
        b_up_r = border_right[0]
        b_up_l = border_left[0]
        b_down_r = border_right[1]
        b_down_l = border_left[1]
        #Heights
        height1 = np.sqrt(((b_down_l[0] - b_up_l[0]) ** 2) + ((b_down_l[1] - b_up_l[1]) ** 2))
        height2 = np.sqrt(((b_down_r[0] - b_up_r[0]) ** 2) + ((b_down_r[1] - b_up_r[1]) ** 2))
        maxHeight = max(int(height1), int(height2))
        #Widths
        width1 = np.sqrt(((b_down_l[0] - b_down_r[0]) ** 2) + ((b_down_l[1] - b_down_r[1]) ** 2))
        width2 = np.sqrt(((b_up_l[0] - b_up_r[0]) ** 2) + ((b_up_l[1] - b_up_r[1]) ** 2))
        maxWidth = max(int(width1), int(width2))
        inputs = np.float32([b_up_l,b_down_l,b_down_r,b_up_r])
        outputs = np.float32([[0, 0], [0,maxHeight - 1],[maxWidth - 1, maxHeight - 1],[maxWidth - 1, 0]])
        self.M = cv2.getPerspectiveTransform(inputs,outputs)
        self.img_out = cv2.warpPerspective(self.frame,self.M,(maxWidth, maxHeight),flags=cv2.INTER_LINEAR)

    def find_goal_pos(self):
        #Isolate goal red square and find the center position 
        kernel = np.ones((5,5),np.float32)/25
        img = cv2.filter2D(self.img_out,-1,kernel)
        img = cv2.blur(img,(5,5))
        img = cv2.medianBlur(img,5)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.LOW_RED, self.HIGH_RED)
        imask = mask>0
        red = np.zeros_like(img, np.uint8)
        red[imask] = img[imask]

        im = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
        ret,self.thresh1 = cv2.threshold(im,2,255,cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours( 
            self.thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        i = 0
        for contour in contours: 
            if i == 0: 
                i = 1
                continue
            M = cv2.moments(contour) 
            if M['m00'] != 0.0: 
                self.x_goal = int(M['m10']/M['m00']) 
                self.y_goal = int(M['m01']/M['m00']) 

    def find_start_pos(self,robot,a):
        #Isolate middle green point on the thymio through color filtering and find its approximate position 
        #using maxLoc function 
        kernel = np.ones((5,5),np.float32)/25
        img = cv2.filter2D(self.img_out,-1,kernel)
        img = cv2.blur(img,(5,5))
        img = cv2.medianBlur(img,5)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.LOW_GREEN, self.HIGH_GREEN)
        green = cv2.bitwise_and(self.img_out,self.img_out, mask = mask)
        gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY) 
        _, self.thresh1 = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY) 
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(self.thresh1)
        [x_back,y_back] = maxLoc
        if (x_back != 0) or (y_back != 0):
            if ((abs(self.x_back - x_back) > self.kid_threshold) or (abs(self.y_back - y_back) > self.kid_threshold)) & (a !=1):
                robot.kidnap = True
            [self.x_back,self.y_back] = [x_back,y_back]
            self.positions.append([self.x_back,self.y_back])

    def find_angle(self,robot):
        kernel = np.ones((5,5),np.float32)/25
        img = cv2.filter2D(self.img_out,-1,kernel)
        img = cv2.blur(img,(5,5))
        img = cv2.medianBlur(img,5)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.LOW_BLUE, self.HIGH_BLUE)
        blue = cv2.bitwise_and(self.img_out,self.img_out, mask = mask)
        gray = cv2.cvtColor(blue, cv2.COLOR_RGB2GRAY) 
        _, self.thresh1 = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY) 
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(self.thresh1)
        [x_front,y_front] = maxLoc
        if (x_front != 0) or (y_front != 0):
            [self.x_front,self.y_front] = [x_front,y_front]
            if self.y_front < self.y_back : 
                if self.x_front > self.x_back : 
                    self.teta = np.arccos((self.x_front-self.x_back)/(np.sqrt(np.power((self.x_front-self.x_back),2)+np.power((self.y_front-self.y_back),2))))
                if self.x_front <= self.x_back : 
                    self.teta = np.pi - np.arccos((self.x_back-self.x_front)/(np.sqrt(np.power((self.x_front-self.x_back),2)+np.power((self.y_front-self.y_back),2))))
            if self.y_front >= self.y_back : 
                if self.x_front > self.x_back : 
                    self.teta = 2*np.pi - np.arccos((self.x_front-self.x_back)/(np.sqrt(np.power((self.x_front-self.x_back),2)+np.power((self.y_front-self.y_back),2))))
                if self.x_front <= self.x_back : 
                    self.teta = np.pi + np.arccos((self.x_back-self.x_front)/(np.sqrt(np.power((self.x_front-self.x_back),2)+np.power((self.y_front-self.y_back),2))))
            robot.setPositions(self.x_back,self.y_back,self.x_goal,self.y_goal,self.teta)
            print('vision angle', self.teta)
            robot.setVisionDone(True)
        #Hidden thymio, do not update the vision variables 
        else:
            robot.setVisionDone(False)

    def find_corners(self):
        kernel = np.ones((5,5),np.float32)/25
        img = cv2.filter2D(self.img_out,-1,kernel)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        ret, thresh2 = cv2.threshold(gray, self.BLACK_THRESHOLD, 255, cv2.THRESH_BINARY) 
        contours, _ = cv2.findContours( 
            thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        self.thresh2 = thresh2
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
                if len(m)<self.NB_SHAPES:
                    m.append([x,y])
        th3 = cv2.adaptiveThreshold(self.thresh2,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                                    cv2.THRESH_BINARY,11,2)
        corners = cv2.goodFeaturesToTrack(th3, self.NB_CORNERS, 0.01, 45) 
        corners = np.int0(corners) 
        d = self.d_wide_cor 
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
            if y1 >= 397:
                y1 = 396
            if x1 >= 519:
                x1 = 518
            if x1 >50:
                if x1<480:
                    if y1 >50:
                        if y1<350:
                            cor.append([x1,y1])
            cornerss.append([x1,y1])
            m_cor.append(m[n_m])
            cv2.circle(self.img_out, (x1, y1), 3, 255, -1)

        for mo in  m:
            cv2.circle(self.img_out, [mo[0],mo[1]], 3, (0, 255, 0), cv2.FILLED)

        self.cor = cor
        self.cornerss = cornerss
        self.m_cor = m_cor 

    def trace_contours(self):
        a = int(np.size(self.cornerss)/2)
        b = int(np.size(self.m_cor)/2)
        for i in range (0,a):
            for j in range (0,b):
                if i != j:
                    if self.m_cor[i] == self.m_cor[j]:
                        line = np.transpose(np.array(draw.line(self.cornerss[i][0], self.cornerss[i][1], self.cornerss[j][0], self.cornerss[j][1])))
                        data = self.thresh2[line[:, 1], line[:, 0]]
                        if np.size(np.where(abs(np.diff(data))>0)[0]) <= 2:
                            if np.mean(data) > self.mean_value_along_line:
                                cv2.line(self.thresh2, self.cornerss[i], self.cornerss[j], self.bluepx, 4)

        cv2.circle(self.thresh2, [self.x_goal,self.y_goal], 4, (0, 0, 255), cv2.FILLED)
        cv2.circle(self.thresh2, [self.x_back,self.y_back], 4, (0, 0, 255), cv2.FILLED)

    def compute_dist_mx(self,robot):
        s = int(((np.size(self.cor))/2)+2) 
        dist_mx = np.zeros((s,s))
        for i in range(1,s):
            if i == (s-1):
                line = np.transpose(np.array(draw.line(self.x_back,self.y_back, self.x_goal,self.y_goal)))
                data = self.thresh2[line[:, 1], line[:, 0]]
                if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
                    dist_mx[i][0] = np.sqrt(np.power((self.x_goal-self.x_back),2)+np.power((self.y_goal-self.y_back),2))
                continue 
            line = np.transpose(np.array(draw.line(self.x_back,self.y_back,self.cor[i-1][0],self.cor[i-1][1])))
            data = self.thresh2[line[:, 1], line[:, 0]]
            if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
                dist_mx[i][0] = np.sqrt(np.power((self.cor[i-1][0]-self.x_back),2)+np.power((self.cor[i-1][1]-self.y_back),2))
        for i in range(0,(s-2)):
            for j in range(0,s):
                if j == (i+1):
                    continue 
                if j == 0 : 
                    line = np.transpose(np.array(draw.line(self.x_back,self.y_back, self.cor[i][0],self.cor[i][1])))
                    data = self.thresh2[line[:, 1], line[:, 0]]
                    if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
                        dist_mx[j][i+1] = np.sqrt(np.power((self.cor[i][0]-self.x_back),2)+np.power((self.cor[i][1]-self.y_back),2))
                    continue 
                if j == int(s-1):
                    line = np.transpose(np.array(draw.line(self.x_goal,self.y_goal, self.cor[i][0],self.cor[i][1])))
                    data = self.thresh2[line[:, 1], line[:, 0]]
                    if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
                        dist_mx[j][i+1] = np.sqrt(np.power((self.cor[i][0]-self.x_goal),2)+np.power((self.cor[i][1]-self.y_goal),2))
                    continue
                line = np.transpose(np.array(draw.line(self.cor[j-1][0],self.cor[j-1][1],self.cor[i][0],self.cor[i][1])))
                data = self.thresh2[line[:, 1], line[:, 0]]
                if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
                    dist_mx[j][i+1] = np.sqrt(np.power((self.cor[i][0]-self.cor[j-1][0]),2)+np.power((self.cor[i][1]-self.cor[j-1][1]),2))
        for i in range(0,(s-1)):
            if i == 0:
                line = np.transpose(np.array(draw.line(self.x_back,self.y_back, self.x_goal,self.y_goal)))
                data = self.thresh2[line[:, 1], line[:, 0]]
                if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
                    dist_mx[0][s-1] = np.sqrt(np.power((self.x_goal-self.x_back),2)+np.power((self.y_goal-self.y_back),2))
                continue 
            line = np.transpose(np.array(draw.line(self.x_goal,self.y_goal,self.cor[i-1][0],self.cor[i-1][1])))
            data = self.thresh2[line[:, 1], line[:, 0]]
            if np.size(np.where(abs(np.diff(data))>0)[0]) <= 3 : 
                dist_mx[i][s-1] = np.sqrt(np.power((self.cor[i-1][0]-self.x_goal),2)+np.power((self.cor[i-1][1]-self.y_goal),2))
        self.dist_mx = dist_mx
        robot.setDistMx(dist_mx)
        robot.setCor(self.cor)
        robot.setS(s)