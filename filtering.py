import matplotlib.pyplot as plt
import numpy as np
import time


def odometry(self, motor_left_speed, motor_right_speed, pre_time, X_init, Y_init, theta_init) : #...) : 
                                                                    # variables récupérées par CV
    ## ATTENTION AUX UNITÉS DES VARIABLES à définir
    #penser au conversion des  unités
    #penser aux bornes  des angles du passage  de 0 a 2π
    #actualisée en "permanance" en fct de la pos de base (celle donnée par CV qui est actualisée en permanance aussi)

    self.speed_L = motor_left_speed #motor_left_speed read only variable --> in main : get_motor_speed ?
    self.speed_R = motor_right_speed #motor_right_speed read only variable
    self.wheel_dist = 3 #valeur absurde => mesurer la correct valeur entre les 2 roues du Thymio

    delta_t = time.time() - pre_time #time.time() to get the value of the time

    delta_S = (self.speed_R + self.speed_L) / 2
    delta_Theta = (self.speed_R - self.speed_L) / self.wheel_dist

    # calculations of the variations of the coordinates of the Thymio :
    delta_X = delta_S * np.cos(theta_init + delta_Theta/2) # ou ?((theta_init + delta_Theta)/2)?? savoir lequel est juste 
    # ma version c'est celle du cours en théorie
    delta_Y = delta_S * np.sin(theta_init + delta_Theta/2)

    # update of the position (coordinates and angle/orientation) of the Thymio after a time of delta_t  :
    X = X_init + delta_X * delta_t #récupérer les valeurs X_init et Y_init de CV
    Y = Y_init + delta_Y * delta_t 

    theta = theta_init + delta_Theta ## revoir calcul

    #update du pre_time
    pre_time = time.time()

    return  X, Y, theta # pour envoyer ces infos a MC


    # position  de base de X, Y et Theta recupérée par la CV


class Filtering :

    def __init__(self, prox_horizontal) : #, motor_left_speed, motor_right_speed, pre_time, X_init, Y_init, theta_init) :
                                          # besoin de ces variables ici ?

        self.proximity_sensors = prox_horizontal #proxymity_sensors
        # prox_horizontal [0,1,2,3,4] -> front sensors [5,6] -> back sensors
        self.state = 0

        self.speed_L = 0 #parce que c'est un output et pas un input => pas pris de motor_left_speed
        self.speed_R = 0
        self.wheel_dist = 3



    def kalman(self, prox_horizontal, motor_left_speed, motor_right_speed) :

        self.state = 0
        self.speed_L = motor_left_speed #motor_left_speed read only variable --> in main : get_motor_speed ?
        self.speed_R = motor_right_speed #motor_right_speed read only variable
        self.proximity_sensors = prox_horizontal #proxymity_sensors


#    def get_data():
#    thymio_data.append({"ground":list(node["prox.ground.reflected"]), 
#                        "sensor":list(node["prox.ground.reflected"]),
#                        "left_speed":node["motor.left.speed"],
#                        "right_speed":node["motor.right.speed"]})


    # Gaussian Function
    def fg(mu,sigma2,x) :
        '''f takes in a mean  and squared variance, and an input x and returns the gaussiant value'''
        coefficient = 1.0 / sqrt(2.0 * pi * sigma2)
        exponential = exp(-0.5 * (x-mu) ** 2 / sigma2)

        return coefficient * exponential
    
    # the update function
    def update(mean1, var1, mean2, var2) :
        '''this function takes in two means and two squared variance terms, and returns updated gaussian parameters'''
        
        # calculate the new parameters :
        new_mean = (var2 * mean1 + var1 * mean2)/(var1+var2)
        new_var = 1/(1/var1 + 1/var2)

        return new_mean, new_var
    
    # the motion update/predict function
    def predict(mean1, var1, mean2, var2) :
        '''this function takes in two means and two squared variance terms, and returns updated gaussian parameters, after motion'''
        # calculate the new parameters :
        new_mean = mean1 + mean2
        new_var = var1 + var2

        return new_mean, new_var


    ### SERIE 8 ###
    # Initialising the remaining constants
    # units: length [mm], time [s]
    var_speed = np.var(avg_speed[55:]/thymio_speed_to_mms)
    std_speed = np.std(avg_speed[55:]/thymio_speed_to_mms)
    q_nu = std_speed/2 # variance on speed state
    r_nu = std_speed/2 # variance on speed measurement 
    qp = 0.04 # variance on position state
    rp = 0.25 # variance on position measurement 

    A = np.array([[1, Ts], [0, 1]])
    stripe_width = 50
    Q = np.array([[qp, 0], [0, q_nu]])
    speed_conv_factor = 0.3375
    transition_thresh = 500

    def kalman_filter(speed, ground_prev, ground, pos_last_trans, x_est_prev, P_est_prev,
                      HT=None, HNT=None, RT=None, RNT=None) :
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
        x_est_a_priori = np.dot(A, x_est_prev) # = A * x_est_prev
        
        # Estimated covariance of the state
        P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T))
        P_est_a_priori = P_est_a_priori + Q if type(Q) != type(None) else P_est_a_priori
        
        ## Update         
        # y, C, and R for a posteriori estimate, depending on transition
        if ((ground_prev < transition_thresh)^(ground < transition_thresh)) : #XOR (one or the other but not both)
            if (ground>ground_prev):
                stripe_width = 44
            else:
                stripe_width = 4
            # transition detected
            pos_last_trans = pos_last_trans + stripe_width
            y = np.array([[pos_last_trans],[speed*speed_conv_factor]])
            H = np.array([[1, 0],[0, 1]])
            R = np.array([[rp, 0],[0, r_nu]])
        else:
            # no transition, use only the speed
            y = speed*speed_conv_factor
            H = np.array([[0, 1]])
            R = r_nu

        # innovation / measurement residual
        i = y - np.dot(H, x_est_a_priori)
        # measurement prediction covariance
        S = np.dot(H, np.dot(P_est_a_priori, H.T)) + R
                
        # Kalman gain (tells how much the predictions should be corrected based on the measurements)
        K = np.dot(P_est_a_priori, np.dot(H.T, np.linalg.inv(S)))
        
        
        # a posteriori estimate
        x_est = x_est_a_priori + np.dot(K,i)
        P_est = P_est_a_priori - np.dot(K,np.dot(H, P_est_a_priori))
        
        return pos_last_trans, x_est, P_est
    
    ### SERIE 8 ###



