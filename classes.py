import matplotlib.pyplot as plt
import numpy as np
import time

#import the classes from the other modules
from local_navigation import LocalNavigation
#from filtering 
import filtering

class Classes :

    def __init__(self) :

        self.variable = 0 


class Thymio :

    def __init__ (self) :

        self.pos_X = 0
        self.pos_Y = 0

        self.theta = 0

    def setPositions (self) :
        
        self.pos_X = 0
        self.pos_Y = 0

        self.theta = 0

