import matplotlib.pyplot as plt
import numpy as np
import time

from classes import Classes

import classes
from classes import Thymio



class KalmanFilter :

    def __init__(self) : #, motor_left_speed, motor_right_speed) :

        '''Initialiation of the variables for the Kalman Filter'''
        self.pre_time = 0
        self.kalman_state = 0
        #self.speed_L = motor_left_speed #motor_left_speed read only variable --> in main : get_motor_speed ?
        #self.speed_R = motor_right_speed #motor_right_speed read only variable
        #self.proximity_sensors = prox_horizontal #proxymity_sensors
        
        self.pos_X_odo = 0
        self.pos_Y_odo = 0
        self.pos_theta_odo = 0
        self.angle = 0

        self.v_X = 0
        self.v_Y = 0
        self.v_Theta = 0 

        self.Ts = 0.1

        self.P_est = np.eye(6)
        self.X_est = [[0],[0],[0],[0],[0],[0]] # Contains all values : [x, x°, y, y°, theta, theta°]

        self.X_est_pre = self.X_est
        self.P_est_pre = self.P_est
        
        self.A = [[1, self.Ts, 0, 0, 0, 0], 
                  [0,  1, 0, 0, 0, 0],
                  [0, 0, 1, self.Ts, 0, 0],
                  [0, 0, 0,  1, 0, 0],
                  [0, 0, 0, 0, 1, self.Ts],
                  [0, 0, 0, 0, 0,  1]]
        
        self.Q = [[7.73, 0, 0, 0, 0, 0], 
                  [0, 1000, 0, 0, 0, 0],
                  [0, 0, 7.73, 0, 0, 0],
                  [0, 0, 0, 1000, 0, 0],
                  [0, 0, 0, 0, 0.0049, 0],
                  [0, 0, 0, 0, 0, 1000]]
         
    def filter_kalman(self, Thymio) :

        '''Kalman Filter calculations'''
        ###MANQUE une valeur initiale pour X_est_pre ? ou est-ce simplement X_est = 0?

        X_estimation = np.dot(self.A, self.X_est_pre)
        P_estimation = np.dot(self.A, np.dot(self.P_est_pre, np.transpose(self.A)))
        P_estimation = P_estimation + self.Q 
        """  if type(self.Q) != type(None) else P_estimation """
        
        if (Thymio.vision) : # If we have the Computer Vision and Odometry

            #self.odometry_update(Thymio)
            y = [[Thymio.pos_X],[self.v_X],[Thymio.pos_Y],[self.v_Y],[Thymio.theta],[self.v_Theta]] 
            # -> x, y, theta : values measured by the CV 
            # -> v_x, v_y, v_theta : velocities from Odometry

            H = np.eye(6) # y = [x, x°, y, y°, theta, theta°]'

            # R : Covariance Matrix of the mesures/sensors
            R = [[7.73, 0, 0, 0, 0, 0], 
                 [0, 6.48, 0, 0, 0, 0],
                 [0, 0, 7.73, 0, 0, 0],
                 [0, 0, 0, 6.48, 0, 0],
                 [0, 0, 0, 0, 0.0049, 0],
                 [0, 0, 0, 0, 0, 0.615]]

            Thymio.vision = 0 # Reset the Computer Vision booleen to 0

        else : # if we only have the Odometry

            #self.odometry_update(Thymio)
            y = [[self.v_X],[self.v_Y],[self.v_Theta]]

            H = [[0, 1, 0, 0, 0, 0],
                 [0, 0, 0, 1, 0, 0],
                 [0, 0, 0, 0, 0, 1]] # y = [0, x°, 0, y°, 0, theta°]' -> only the velocities
            
            R = [[6.48, 0, 0],
                 [0, 6.48, 0],
                 [0, 0, 0.615]]
            
        print("y :")
        print("H*X_estimation", np.dot(H,X_estimation))
        i = y - np.dot(H, X_estimation)
        print("i: ",i)
        S = np.dot(H, np.dot(P_estimation, np.transpose(H))) + R
        print("S: ",S)
        K = np.dot(P_estimation, np.dot(np.transpose(H),  np.linalg.inv(S)))
        print("K :",K)
        # Update of X_est and P_est
        #print("values of vX qnd vY : ", self.v_X, ' ', self.v_Y)
        #print("value of v theta : ", self.v_Theta)
        print("K*i: ", np.dot(K, i))
        self.X_est = X_estimation + np.dot(K, i)
        print("X_est:", self.X_est)
        self.X_est[4][0] = np.mod((self.X_est[4][0] + np.pi), 2*np.pi) - np.pi
        self.X_est[4][0] = np.mod(self.X_est[4][0], 2*np.pi)

        #self.theta_corr() 
        #self.X_est[4][0] = np.mod(self.X_est[4][0],2*np.pi)
        #
        #print("dans filtering", self.X_est[4][0])
        #self.angle = np.mod(self.X_est[4][0],2*np.pi)
        
        #print("X_estimation dans fct KF")
        #print(X_estimation)
        self.P_est = P_estimation - np.dot(K, np.dot(H, P_estimation))
        

        ## test en printant ces valeurs
        self.X_est_pre = self.X_est
        self.P_est_pre = self.P_est



    def odometry_update(self, Thymio, node) :

        '''Odometry calculation'''
        Thymio.getSpeeds(node)
        ## ATTENTION AUX UNITÉS DES VARIABLES à définir
        #penser au conversion des  unités
        #penser aux bornes  des angles du passage  de 0 a 2π
        #actualisée en "permanance" en fct de la pos de base (celle donnée par CV qui est actualisée en permanance aussi)

        self.speed_L = Thymio.motor_target_left
        self.speed_R = Thymio.motor_target_right

        
        self.real_speed_L = Thymio.motor_left_speed 
        self.real_speed_R = Thymio.motor_right_speed 

        #delta_R = 

        #print("speed of L and R :", self.speed_L, ' ', self.speed_R )
        
        self.wheel_dist = 95 # en mm à tester !!! 9.5 # en cm Distance between the wheel (where they touch the ground)

        
        delta_t = time.time() - self.pre_time            # time.time() to get the value of the time

        delta_S = (self.real_speed_R + self.real_speed_L) / 2                         # Forward speed
        self.v_Theta = (self.real_speed_R - self.real_speed_L) / self.wheel_dist     # Angular velocity 
        #print("values of deltaS", delta_S)
        # calculations of the variations of the speed of the Thymio
        '''self.v_X = delta_S * np.cos(Thymio.theta + self.v_Theta/2)          # SPEED in X
        self.v_Y = delta_S * np.sin(Thymio.theta + self.v_Theta/2)          # SPEED in y'''

        ##TESTER AVEC CE CALCUL
        self.v_X = delta_S * np.cos(self.X_est[4][0] + self.v_Theta/2)          # SPEED in X
        self.v_Y = delta_S * np.sin(self.X_est[4][0] + self.v_Theta/2)          # SPEED in y

        ##print("values of vX qnd vY : ", self.v_X, ' ', self.v_Y)
        ##print("value of v theta : ", self.v_Theta)
        # ou ?((theta_init + delta_Theta)/2)?? savoir lequel est juste 
        # ma version c'est celle du cours en théorie

        # update of the position (coordinates and angle/orientation) of the Thymio after time delta_t
        '''pas forcément nécessaire comme partie du code ?'''
        self.pos_X_odo = self.v_X * delta_t
        self.pos_Y_odo = self.v_Y * delta_t
        self.pos_theta_odo = self.v_Theta * delta_t


        # Update for the previous time with the current time for the next iteration
        self.pre_time = time.time()

    '''def theta_corr(self) :
        if self.X_est[4][0] >= 1.98*np.pi :
            self.X_est[4][0] = self.X_est[4][0] - 2*np.pi
            #self.X_est[4][0] =  self.X_est[4][0] - np.pi
            

        elif self.X_est[4][0] <= - 1.98*np.pi :
            self.X_est[4][0] = 2*np.pi - self.X_est[4][0] #- np.pi

        else :
            self.X_est[4][0] = self.X_est[4][0]
        #print("dans theta corr", self.X_est[4][0])'''
        
'''
        ##&&&&&& ATTTTEEENTIIIOOONNNNNN
        self.v_X =  delta_X  #récupérer les valeurs X_init et Y_init de CV
        self.v_Y = Thymio.Y_init + delta_Y * delta_t ######ATTENTIUON C?EST UNE VITESSE !!! pas une position

        self.v_Theta = Thymio.theta + delta_Theta * delta_t ## revoir calcul --> vitesse anglulaire
        ##&&&&&& ATTTTEEENTIIIOOONNNNNN

        #update du pre_time
        pre_time = time.time()

        #return  X, Y, theta # pour envoyer ces infos a MC


        # position  de base de X, Y et Theta recupérée par la CV


        #print(Thymio)

'''

'''def angle_error(self) :
        
        a = 0'''



