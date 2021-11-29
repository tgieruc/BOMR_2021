 # %%run_python
import numpy as np


def move(pos_prev, pos_goal):
    wheel_radius =
    k_rho =
    k_alpha =
    k_beta =
    x_prev = pos_prev[1]
    y_prev = pos_prev[2]
    x_goal = pos_goal[1]
    y_goal = pos_goal[2]
    theta = pos_prev[3]
    dx = x_goal - x_prev
    dy = y_goal - y_prev
    rho = np.sqrt(dx**2 + dy**2)
    alpha = -theta + np.arctan2(dy, dx)
    beta = -theta - alpha
    # Rotation_matrix = np.array([[np.cos(theta), np.sin(theta), 0],[-np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    rho_dot = -k_rho * rho * np.cos(alpha)
    alpha_dot = k_rho * np.sin(alpha) - k_alpha * alpha - k_beta * beta
    beta_dot = -k_rho * np.sin(alpha)



