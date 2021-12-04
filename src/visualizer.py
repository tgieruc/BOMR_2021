from vision import Vision
from global_navigation import Path_planner

def get_visualization(vision, path_planner):
    return path_planner.create_path_mask(vision.create_full_mask())
