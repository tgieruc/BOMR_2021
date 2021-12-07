from vision import Vision
from global_navigation import Path_planner
import cv2

def get_visualization_kalman(vision, path_planner, kalman):
    img = get_visualization(vision, path_planner)
    img = cv2.circle(img, (int(kalman.pos_x[0]), int(kalman.pos_y[0])), 10, (0, 0, 255), -1)
    return img

def get_visualization(vision, path_planner):
    return path_planner.create_path_mask(vision.create_full_mask())
