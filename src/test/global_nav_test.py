import sys as _sys
_sys.path.append("../")

from vision import Vision
from global_navigation import Path_planner
from visualizer import get_visualization
import matplotlib.pyplot as plt
vision = Vision("example.png")
path_planner = Path_planner()

vision.update()
path_planner.make_path(vision)
img = get_visualization(vision, path_planner)

plt.imshow(img)
plt.show()