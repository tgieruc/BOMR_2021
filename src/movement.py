import numpy as np
from Kalman_filter import Kalman_filter


def move(vision, pos_prev, position_goal, min_angle, k_rho, k_alpha,k_rot, is_near_checkpoint):
    print(is_near_checkpoint)
    wheel_radius = 44 * vision.mm2px
    length_robot = 50 * vision.mm2px
    dpos = position_goal - pos_prev[0:2]
    rho = np.linalg.norm(dpos)
    alpha = -pos_prev[2] + np.arctan2(dpos[1], dpos[0])
    alpha = (alpha + np.pi )% (2*np.pi) - np.pi
    beta = -pos_prev[2] - alpha
    beta =  (beta + np.pi )% (2*np.pi) - np.pi
    print(rho,alpha*180/np.pi,beta*180/np.pi)
    if (abs(alpha) > min_angle) & is_near_checkpoint:
        return [int((k_rot * alpha * length_robot) / wheel_radius),
                int(-(k_rot * alpha * length_robot) / wheel_radius)], dpos
    else:

        # w = k_alpha * alpha + k_beta * beta
        # speed_left = (v + w * length_robot)/wheel_radius
        # speed_left = (v + w * length_robot) / wheel_radius
        speed_left = (k_rho+ k_alpha * alpha)
        # speed_right = (v - w * length_robot) / wheel_radius
        speed_right = (k_rho- k_alpha * alpha )
        # speed_right = (v - w * length_robot) / wheel_radius
        speed = [int(speed_left), int(speed_right)]
        return speed, dpos
