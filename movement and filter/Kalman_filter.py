import sys as _sys
import numpy as np
from filterpy.kalman import KalmanFilter
_sys.path.append("../Vision")
from vision import *

vision = Vision()
vision.update()
"""
    two cases: the camera is working or the camera is not working
"""
class Kalman_filter():
    def __init__(self):
        self.pos_x = vision.robot.center[0][0]
        self.pos_y = vision.robot.center[0][1]
        self.theta = vision.robot.orientation
        self.rho = np.sqrt(self.pos_x **2 + self.pos_y **2)
    def compute_pos_cart(self):
        self.pos_x = self.rho * np.cos(self.theta)
        self.pos_y = self.rho * np.sin(self.theta)
    def update_kalman(self):
        #mise en place du filtre ici





qp = 0.04
q_nu = 6.15
r_nu = 6.15
rp = 0.25
Q = np.array([[qp, 0], [0, q_nu]]);