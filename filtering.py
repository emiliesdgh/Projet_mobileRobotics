import matplotlib.pyplot as plt
import numpy as np
import time

from classes import Classes

Ts = 0.1

class KalmanFilter :

    def __init__(self, prox_horizontal, motor_left_speed, motor_right_speed) :

        self.kalman_state = 0
        self.speed_L = motor_left_speed #motor_left_speed read only variable --> in main : get_motor_speed ?
        self.speed_R = motor_right_speed #motor_right_speed read only variable
        self.proximity_sensors = prox_horizontal #proxymity_sensors

        self.X_est = [0] * 6 # contient [x, x°, y, y°, theta, theta°]

        self.pos_X = 0
        self.v_X = 0
        self.pos_Y = 0
        self.v_Y = 0
        self.pos_Theta = 0
        self.v_Theta = 0

        self.P_est = np.eye(6)

        self.A = [[1, Ts, 0, 0, 0, 0], 
                  [0,  1, 0, 0, 0, 0],
                  [0, 0, 1, Ts, 0, 0],
                  [0, 0, 0,  1, 0, 0],
                  [0, 0, 0, 0, 1, Ts],
                  [0, 0, 0, 0, 0,  1]]
        
        self.Q = [[7.7302, 0, 0, 0, 0, 0], 
                  [0, 1000, 0, 0, 0, 0],
                  [0, 0, 7.7302, 0, 0, 0],
                  [0, 0, 0, 1000, 0, 0],
                  [0, 0, 0, 0, 0.0049, 0],
                  [0, 0, 0, 0, 0, 1000]]
         


    def filter_kalman(self, X_est_pre, P_est_pre, Thymio) :

        X_estimation = self.A @ X_est_pre 
        
        P_estimation = self.A @ (P_est_pre @ self.A.T)

        P_estimation = P_estimation + self.Q if type(self.Q) != type(None) else P_estimation
        
        if (Thymio.vision) :

            y = [[Thymio.pos_X],[self.v_X],[Thymio.pos_Y],[self.v_Y],[Thymio.theta],[self.v_Theta]] # -> x, y, theta : valeurs mesurées par la CV
                                                        # -> v_x, v_y, v_theta : vitesses

            H = np.eye(6) # => y = [x, x°, y, y°, theta, theta°]T

            R = [[7.7302, 0, 0, 0, 0, 0], 
                 [0, 6.4883, 0, 0, 0, 0],
                 [0, 0, 7.7302, 0, 0, 0],
                 [0, 0, 0, 6.4883, 0, 0],
                 [0, 0, 0, 0, 0.0049, 0],
                 [0, 0, 0, 0, 0, 0.6152]]#valeurs absurde => R : matrice de covariences des capteurs/mesures
            
            Thymio.vision = 0

        else :

            y = [[0],[self.v_X],[0],[self.v_Y],[0],[self.v_Theta]]

            H = [[0, 0, 0, 0, 0, 0], 
                 [0, 1, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0, 0],
                 [0, 0, 0, 1, 0, 0],
                 [0, 0, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0, 1]] # => y = [0, x°, 0, y°, 0, theta°]T -> uniquement les vitesses
            
            R = [[0, 0, 0, 0, 0, 0], 
                 [0, 6.4883, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0, 0],
                 [0, 0, 0, 6.4883, 0, 0],
                 [0, 0, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0, 0.6152]]
            
        i = y - H @ X_estimation

        S = H @ (P_estimation @ H.T) + R
        
        K = P_estimation @ (H.T @  np.linalg.inv(S))
        
        X_est = X_estimation + K @ i

        P_est = P_estimation - K @ (H @ P_estimation)



    def odometry_update(self, motor_left_speed, motor_right_speed, pre_time, Thymio) : # X_init, Y_init, theta_init, Thymio) : #...) : 
                                                                    # variables récupérées par CV
        ## ATTENTION AUX UNITÉS DES VARIABLES à définir
        #penser au conversion des  unités
        #penser aux bornes  des angles du passage  de 0 a 2π
        #actualisée en "permanance" en fct de la pos de base (celle donnée par CV qui est actualisée en permanance aussi)

        self.speed_L = motor_left_speed #motor_left_speed read only variable --> in main : get_motor_speed ?
        self.speed_R = motor_right_speed #motor_right_speed read only variable
        self.wheel_dist = 9.5 #8 #cm ou 9.5 ou 11 #valeur absurde => mesurer la correct valeur entre les 2 roues du Thymio

        #thymio = Thymio(pos_X, pos_Y, theta)

        delta_t = time.time() - pre_time #time.time() to get the value of the time

        delta_S = (self.speed_R + self.speed_L) / 2 #forward speed

        self.v_Theta =  (self.speed_R - self.speed_L) / self.wheel_dist # angular velocity 

        # calculations of the variations of the coordinates of the Thymio :
        self.v_X = delta_S * np.cos(Thymio.theta + self.v_Theta/2) #SPEED in X
        
         # ou ?((theta_init + delta_Theta)/2)?? savoir lequel est juste 
        # ma version c'est celle du cours en théorie
        self.v_Y = delta_S * np.sin(Thymio.theta + self.v_Theta/2) #SPEED in y

        # update of the position (coordinates and angle/orientation) of the Thymio after a time of delta_t  :
        
        #update du pre_time
        pre_time = time.time()
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


        print(Thymio)

'''

'''def angle_error(self) :
        
        a = 0'''



