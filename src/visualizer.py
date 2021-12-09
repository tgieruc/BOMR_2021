from vision import Vision
from global_navigation import Path_planner
import cv2
import numpy as np

def get_visualization_kalman(vision, path_planner, kalman):
    img = get_visualization(vision, path_planner)
    img = cv2.circle(img, (int(kalman.rho_est[0]), int(kalman.rho_est[1])), 10, (0, 0, 255), -1)
    # center = np.array([int(kalman.rho_est[0]), int(kalman.rho_est[1])])
    # point = (center + np.array(
    #             [[np.cos(kalman.rho_est[2]) * 100, np.sin(kalman.rho_est[2]) * 100]])).astype(int)
    # img = cv2.line(img, (kalman.rho_est[0], kalman.rho_est[1]), (point[0, 0], point[0, 1]), (255, 0, 0), thickness=3,
    #                lineType=8)
    return img

def get_visualization(vision, path_planner):
    return path_planner.create_path_mask(vision.create_full_mask())
