 # %%run_python
import numpy as np


def move(pos_prev, pos_goal, theta_prev):
    x_prev = pos_prev[1]
    y_prev = pos_prev[2]
    x_goal = pos_goal[1]
    y_goal = pos_goal[2]
    dist = np.sqrt((x_goal - x_prev)**2 + (y_goal - y_prev)**2)
    theta = theta_prev + np.arcsin((x_goal - x_prev)/dist)


