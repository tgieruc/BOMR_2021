import sys as _sys
_sys.path.append("../")

from vision import Vision
import numpy as np
import cv2

cv2.namedWindow("preview")
vision = Vision()

vision.connect_camera(0)
vision.update()
color_goal = np.array([200, 195, 53])
vision.goal.color = color_goal
while 1:
    vision.update_frame()
    vision.update()
    cv2.imshow("preview", vision.create_full_mask())
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break

vision.disconnect_camera()
cv2.destroyWindow("preview")