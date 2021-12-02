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
        self.pos_x =
        self.pos_y =
        self.theta =



def kalman_filter()


qp = 0.04
q_nu = 6.15
r_nu = 6.15
rp = 0.25
Q = np.array([[qp, 0], [0, q_nu]]);