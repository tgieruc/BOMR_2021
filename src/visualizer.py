from vision import Vision
from global_navigation import Path_planner
import cv2
import numpy as np

def get_visualization_vision_path_kalman(vision, path_planner, kalman):
    img = get_visualization_vision_path(vision, path_planner)
    img = cv2.circle(img, (int(kalman.rho_est[0]), int(kalman.rho_est[1])), 5, (0, 255, 0), -1)
    center = kalman.rho_est[:2].astype(int)
    point = (center + np.array([np.cos(kalman.rho_est[2]) * 100, np.sin(kalman.rho_est[2]) * 100])).astype(int)
    img = cv2.line(img, (center[0], center[1]), (point[0], point[1]), (0, 255, 0), thickness=2, lineType=8)
    return img

def get_visualization_vision_path(vision, path_planner):
    return path_planner.create_path_mask(vision.create_full_mask())
