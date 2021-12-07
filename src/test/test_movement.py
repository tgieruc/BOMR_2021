import sys as _sys
from time import sleep
_sys.path.append("../")
import numpy as np

from movement import *

pos_prev = np.array([0, 0, 0])
position_goal = np.array([200, 0])
speed = move(pos_prev, position_goal, 0.261799)
print(speed)