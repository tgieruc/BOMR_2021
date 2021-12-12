import sys as _sys
from time import sleep
_sys.path.append("../")
import numpy as np
import cv2
from vision import *
import matplotlib.pyplot as plt
vision = Vision("example.png")
# vision.disconnect_camera()
vision.connect_camera(1)
vision.update_frame()
for i in range(10):
    sleep(0.05)
    vision.update_frame()

vision.update()
def plot(img):
    plt.imshow((img))
    plt.show()

plt.imshow(vision.create_mask_goal(vision.actual_frame.copy()))


plt.imshow(vision.create_mask_obstacles(vision.actual_frame.copy()))


plt.imshow(vision.create_mask_robot(vision.actual_frame.copy()))


plt.imshow(vision.create_full_mask())
plt.show()

