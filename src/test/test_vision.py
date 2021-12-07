import sys as _sys
_sys.path.append("../")

from vision import Vision
import matplotlib.pyplot as plt
vision = Vision("example.png")



vision.update()


plt.imshow(vision.create_mask_goal(vision.actual_frame.copy()))


plt.imshow(vision.create_mask_obstacles(vision.actual_frame.copy()))


plt.imshow(vision.create_mask_robot(vision.actual_frame.copy()))


plt.imshow(vision.create_full_mask())
plt.show()

