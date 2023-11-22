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
        
        self.Q = np.eye(6) # valeurs de la matrice à changer
        


    def filter_kalman(self, X_est_pre, P_est_pre, Thymio) : #HT=None, HNT=None, RT=None, RNT=None) :

       #vision = 0
        """
        Estimates the current state using input sensor data and the previous state
        
        param speed: measured speed (Thymio units)
        param ground_prev: previous value of measured ground sensor
        param ground: measured ground sensor
        param pos_last_trans: position of the last transition detected by the ground sensor
        param x_est_prev: previous state a posteriori estimation
        param P_est_prev: previous state a posteriori covariance
        
        return pos_last_trans: updated if a transition has been detected
        return x_est: new a posteriori state estimation
        return P_est: new a posteriori state covariance
        """
        
        ## Prediciton through the a priori estimate
        # estimated mean of the state
        X_estimation = self.A @ X_est_pre # = A * x_est_prev
        
        # Estimated covariance of the state
        #P_estimation = np.dot(A, np.dot(P_est_pre, A.T))
        P_estimation = self.A @ (P_est_pre @ self.A.T)


        P_estimation = P_estimation + self.Q if type(self.Q) != type(None) else P_estimation
        
        ## Update         
        # y, H, and R for a posteriori estimate, depending on transition

        if (Thymio.vision) :

            y = [[Thymio.X_init],[self.v_X],[Thymio.Y_init],[self.v_Y],[Thymio.Theta_init],[self.v_Theta]] # -> x, y, theta : valeurs mesurées par la CV
                                                        # -> v_x, v_y, v_theta : vitesses

            H = np.eye(6) # => y = [x, x°, y, y°, theta, theta°]T

            R = np.eye(6) #valeurs absurde => R : matrice de covariences des capteurs/mesures

        else :

            #odometry_update(self, motor_left_speed, motor_right_speed, pre_time, Thymio)

            y = [[0],[self.v_X],[0],[self.v_Y],[0],[self.v_Theta]]

            H = [[0, 0, 0, 0, 0, 0], 
                 [0, 1, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0, 0],
                 [0, 0, 0, 1, 0, 0],
                 [0, 0, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0, 1]] # => y = [0, x°, 0, y°, 0, theta°]T -> uniquement les vitesses
            
            R = np.eye(6) #valeurs absurde => R : matrice de covariences des capteurs/mesures

        

        # innovation / measurement residual
        # estimation erreur
        # i = y - np.dot(H, x_est_a_priori)
        i = y - H @ X_estimation
        # measurement prediction covariance
        #S = np.dot(H, np.dot(P_estimation, H.T)) + R # matrice inversée -> le dessous de la division

        S = H @ (P_estimation @ H.T) + R
                
        # Kalman gain (tells how much the predictions should be corrected based on the measurements)
        #K = np.dot(P_estimation, np.dot(H.T, np.linalg.inv(S)))
        
        K = P_estimation @ (H.T @  np.linalg.inv(S))
        
        # a posteriori estimate
        X_est = X_estimation + K @ i #np.dot(K,i)
        P_est = P_estimation - K @ (H @ P_estimation) #np.dot(K,np.dot(H, P_estimation))
        
        #return pos_last_trans, X_est, P_est

        # x_est = [x,x., y, y., tetha, theta.]
        # P_est = matrice de covariance des etats -> gaussienne -> matrice presque diag
    ### SERIE 8 ###



    def odometry_update(self, motor_left_speed, motor_right_speed, pre_time, Thymio) : # X_init, Y_init, theta_init, Thymio) : #...) : 
                                                                    # variables récupérées par CV
        ## ATTENTION AUX UNITÉS DES VARIABLES à définir
        #penser au conversion des  unités
        #penser aux bornes  des angles du passage  de 0 a 2π
        #actualisée en "permanance" en fct de la pos de base (celle donnée par CV qui est actualisée en permanance aussi)

        self.speed_L = motor_left_speed #motor_left_speed read only variable --> in main : get_motor_speed ?
        self.speed_R = motor_right_speed #motor_right_speed read only variable
        self.wheel_dist = 3 #valeur absurde => mesurer la correct valeur entre les 2 roues du Thymio

        #thymio = Thymio(pos_X, pos_Y, theta)

        delta_t = time.time() - pre_time #time.time() to get the value of the time

        delta_S = (self.speed_R + self.speed_L) / 2
        delta_Theta = (self.speed_R - self.speed_L) / self.wheel_dist

        # calculations of the variations of the coordinates of the Thymio :
        delta_X = delta_S * np.cos(Thymio.Theta_init + delta_Theta/2) # ou ?((theta_init + delta_Theta)/2)?? savoir lequel est juste 
        # ma version c'est celle du cours en théorie
        delta_Y = delta_S * np.sin(Thymio.Theta_init + delta_Theta/2)

        # update of the position (coordinates and angle/orientation) of the Thymio after a time of delta_t  :
    

        ##&&&&&& ATTTTEEENTIIIOOONNNNNN
        self.v_X = Thymio.X_init + delta_X * delta_t #récupérer les valeurs X_init et Y_init de CV
        self.v_Y = Thymio.Y_init + delta_Y * delta_t ######ATTENTIUON C?EST UNE VITESSE !!! pas une position

        self.v_Theta = Thymio.Theta_init + delta_Theta ## revoir calcul --> vitesse anglulaire
        ##&&&&&& ATTTTEEENTIIIOOONNNNNN

        #update du pre_time
        pre_time = time.time()

        #return  X, Y, theta # pour envoyer ces infos a MC


        # position  de base de X, Y et Theta recupérée par la CV


        print(Thymio)


    def angle_error(self) :
        
        a = 0



