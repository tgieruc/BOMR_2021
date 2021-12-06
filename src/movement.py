import numpy as np
from Kalman_filter import Kalman_filter


def move(pos_prev, position_goal, kalman):
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
    # speed_left = (v + w * length_robot)/wheel_radius
    speed_left = (v + w * length_robot)/kalman.thymio_to_mm_speed
    # speed_right = (v - w * length_robot) / wheel_radius
    speed_right = (v - w * length_robot) / kalman.thymio_to_mm_speed
    speed = [int(speed_right), int(speed_left)]
    return speed