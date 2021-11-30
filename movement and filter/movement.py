
import numpy as np
import sys as _sys
_sys.path.append("../global navigation")
from test_global_navigation import *


def move(pos_prev, position_goal):
    wheel_radius = 44       # mm
    length_robot = 50       # mm
    k_rho = 20
    k_alpha = 25
    k_beta = - 5
    x_prev = pos_prev[0]
    y_prev = pos_prev[1]
    x_goal = position_goal[0]
    y_goal = position_goal[1]
    theta = pos_prev[2]
    dx = x_goal - x_prev
    dy = y_goal - y_prev
    rho = np.sqrt(dx**2 + dy**2)
    alpha = -theta + np.arctan2(dy, dx)
    beta = -theta - alpha
    v = k_rho * rho
    w = k_alpha * alpha + k_beta * beta
    speed_left = (v + w * length_robot)/wheel_radius
    speed_right = (v - w * length_robot) / wheel_radius
    speed = [int(speed_right), int(speed_right)]
    return speed

# i =1
# timer_period[0] = 100
# @onevent
# def timer0():
#     global motor_right_target, motor_left_target, i
#     pos_goal = resultat_coor[i]
#     pos =  # prendre la valeur de Kalman
#     if pos[0] == pos_goal[0] and pos[1] == pos_goal[1]:
#         i = i+1
#     speed = move(pos, pos_goal)
#     motor_right_target = speed[0]
#     motor_left_target = speed[1]



# for i in len(resultat_coor):
#     pos_goal = resultat_coor[i]
#     pos =  # prendre la valeur de Kalman
#     while pos != pos_goal:
#         pos =  # prendre la valeur de Kalman
#         speed = move(pos, pos_goal)
#         motor_right_target = speed[1]
#         motor_left_target = speed[2]
