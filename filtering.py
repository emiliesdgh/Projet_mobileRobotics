import matplotlib.pyplot as plt
import numpy as np

from classes import Thymio
from vision import Vision

Thymio_speedTo_mm_ratio = 0.4
speed_ratio = Thymio_speedTo_mm_ratio * Vision.ORIGINAL_DIM / 952
wheel_dist = 52 #in pixels

class KalmanFilter :

    def __init__(self) : 
        '''Initialiation of the variables for the Kalman Filter'''

        self.v_X = 0
        self.v_Y = 0
        self.v_Theta = 0 
        self.Ts = 0.1

        self.P_est = np.eye(6)
        self.X_est = np.array([[0],[0],[0],[0],[0],[0]]) # Contains all values : [x, x°, y, y°, theta, theta°]
        
        self.X_est_pre = self.X_est
        self.P_est_pre = self.P_est
        
        self.A = np.array([[1, self.Ts, 0,    0,    0,    0], 
                           [0,    1,    0,    0,    0,    0],
                           [0,    0,    1, self.Ts, 0,    0],
                           [0,    0,    0,    1,    0,    0],
                           [0,    0,    0,    0,    1, self.Ts],
                           [0,    0,    0,    0,    0,    1]])
        
        self.Q = np.array([[0.337, 0, 0, 0, 0,  0], 
                           [0, 6.48,  0, 0, 0,  0],
                           [0, 0, 0.337, 0, 0,  0],
                           [0, 0, 0, 6.48,  0,  0],
                           [0, 0, 0, 0, 0.01,   0],
                           [0, 0, 0, 0, 0,  0.615]])
         
    def filter_kalman(self, Thymio) :
        '''Kalman Filter calculations'''

        X_estimation = np.dot(self.A, self.X_est_pre)
        P_estimation = np.dot(self.A, np.dot(self.P_est_pre, np.transpose(self.A)))
        P_estimation = P_estimation + self.Q if type(self.Q) != type(None) else P_estimation  # still not sure if we need this line of not
        
        if (Thymio.vision) : # If we have the Computer Vision and Odometry

            y = np.array([[Thymio.pos_X],
                          [self.v_X],
                          [Thymio.pos_Y],
                          [self.v_Y],
                          [Thymio.theta],
                          [self.v_Theta]])

            H = np.eye(6) # y = [x, x°, y, y°, theta, theta°]'

            # R : Covariance Matrix of the mesures/sensors
            R = np.array([[0.337, 0, 0, 0, 0, 0], 
                          [0, 6.48,  0, 0, 0, 0],
                          [0, 0, 0.337, 0, 0, 0],
                          [0, 0, 0, 6.48,  0, 0],
                          [0, 0, 0, 0, 0.01,  0],
                          [0, 0, 0, 0, 0, 0.615]])

        else : # if we only have the Odometry

            y = np.array([[self.v_X],
                          [self.v_Y],
                          [self.v_Theta]])

            H = np.array([[0, 1, 0, 0, 0, 0],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 0, 1]]) # y = [0, x°, 0, y°, 0, theta°]' -> only the velocities
            
            R = np.array([[6.48, 0, 0],
                          [0, 6.48, 0],
                          [0, 0, 0.615]])

        i = y - np.dot(H, X_estimation)

        S = np.dot(H, np.dot(P_estimation, np.transpose(H))) + R

        K = np.dot(P_estimation, np.dot(np.transpose(H),  np.linalg.inv(S)))

        self.X_est = X_estimation + np.dot(K, i)

        self.X_est[4][0] = np.mod((self.X_est[4][0] + np.pi), 2*np.pi) - np.pi
        self.X_est[4][0] = np.mod(self.X_est[4][0], 2*np.pi)

        self.P_est = P_estimation - np.dot(K, np.dot(H, P_estimation))

        self.X_est_pre = self.X_est
        self.P_est_pre = self.P_est
        
        Thymio.vision = 0 # Reset the Computer Vision booleen to 0

    def odometry_update(self, Thymio, node) :
        '''Odometry calculation'''

        Thymio.getSpeeds(node)
       
        self.real_speed_L = Thymio.motor_left_speed 
        self.real_speed_R = Thymio.motor_right_speed 
        
        delta_S = (self.real_speed_R + self.real_speed_L) / 2                         # Forward speed
        self.v_Theta = (self.real_speed_R - self.real_speed_L) * speed_ratio / (2*wheel_dist)   # Angular velocity 
        
        self.v_X = delta_S * np.cos(self.X_est[4][0]) * speed_ratio         # SPEED in X
        self.v_Y = delta_S * np.sin(self.X_est[4][0]) * speed_ratio         # SPEED in y