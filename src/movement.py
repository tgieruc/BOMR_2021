import numpy as np
from Kalman_filter import Kalman_filter


def move(pos_prev, position_goal, kalman, min_angle, k_rho=100, k_alpha=400, k_beta=- 300, k_rot = 10):
    assert (k_rho > 0)
    assert (k_beta < 0)
    assert (k_alpha - k_rho > 0)
    wheel_radius = 44  # mm
    length_robot = 50  # mm
    dpos = position_goal - pos_prev[0:1]
    rho = np.linalg.norm(dpos)
    alpha = -pos_prev[2] + np.arctan2(pos_prev[1], pos_prev[0])
    beta = -pos_prev[2] - alpha
    if abs(alpha) > min_angle:
        return [int((k_rot * alpha * length_robot) * 0.435 / wheel_radius),
                int(-(k_rot * alpha * length_robot) * 0.435 / wheel_radius)], dpos
    else:
        v = k_rho * rho
        w = k_alpha * alpha + k_beta * beta
        # speed_left = (v + w * length_robot)/wheel_radius
        speed_left = (v + w * length_robot) * 0.435 / wheel_radius
        # speed_right = (v - w * length_robot) / wheel_radius
        speed_right = (v - w * length_robot) * 0.435 / wheel_radius
        speed = [int(speed_left), int(speed_right)]
        return speed, dpos
