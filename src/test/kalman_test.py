import sys as _sys
_sys.path.append("../")

from vision import Vision
from Kalman_filter import Kalman_filter

vision = Vision("example.png")
kalman = Kalman_filter(vision)