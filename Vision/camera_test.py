import cv2
from vision import *


cv2.namedWindow("preview")
vision = Vision()

vision.connect_camera()

while 1:
    vision.update_frame()
    vision.update()
    cv2.imshow("preview", vision.create_full_mask())
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break

vision.disconnect_camera()
cv2.destroyWindow("preview")