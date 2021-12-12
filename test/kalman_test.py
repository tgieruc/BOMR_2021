import sys as _sys
_sys.path.append("../")

from vision import Vision
from Kalman_filter import Kalman_filter

vision = Vision("example.png")
vision.update()
robot_speed = [100, -100]
kalman = Kalman_filter(vision)
kalman.update_kalman(vision, robot_speed, 0.1)
print(kalman.rho_est)
print(vision.robot.center, vision.robot.orientation)