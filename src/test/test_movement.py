import sys as _sys
from time import sleep
_sys.path.append("../")
import numpy as np

from movement import *

pos_prev = np.array([239,410    ,1.3644724327651792])
position_goal = np.array([304,266])
move(pos_prev, position_goal)