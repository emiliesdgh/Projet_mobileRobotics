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
