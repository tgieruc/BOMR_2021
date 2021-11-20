import sys as _sys
_sys.path.append(".")
import matplotlib.pyplot as plt
from vision import *
vision = Vision()

vision.update()


plt.imshow(vision.create_mask_aim(vision.actual_frame.copy()))


plt.imshow(vision.create_mask_obstacles(vision.actual_frame.copy()))


plt.imshow(vision.create_mask_robot(vision.actual_frame.copy()))


plt.imshow(vision.create_full_mask())
plt.show()