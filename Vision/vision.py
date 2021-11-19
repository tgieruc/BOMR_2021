import cv2
import matplotlib.pyplot as plt
import numpy as np


class Vision():
    def __init__(self):
        self.actual_frame = cv2.imread('example.png', cv2.IMREAD_COLOR)
        self.blurred_frame = cv2.medianBlur(self.actual_frame, 9)
        self.gray_frame = cv2.imread('example.png', cv2.IMREAD_GRAYSCALE)
        self.obstacles_mask = None
        self.obstacles_array = None
        self.robot_mask = None
        self.robot_array = None
        self.aim_mask = None
        self.aim_array = None

    def update(self):
        """
        Updates (obstacles robots aim)_array and _mask
        """
        self.update_obstacles()
        self.update_robot()
        self.update_aim()

    def update_obstacles(self):
        def cond_obstacles(img_color, img_grayscale, cnt):
            mask = np.zeros(img_grayscale.shape, np.uint8)
            cv2.drawContours(mask, cnt, -1, 255, -1)
            contour_mean = cv2.mean(img_color, mask)
            return np.sum(contour_mean) / 3 > 130

        self.obstacles_array = polygon_detection(self.actual_frame, self.gray_frame, 400, cond_obstacles)
        self.obstacles_mask = self.creat_mask_obstacles(self.actual_frame.copy())

    def creat_mask_obstacles(self, img):
        """
        This method creates a mask of all the obstacles

        Input:
        - img : the image on which the mask will be added

        Output:
        - the image with the mask
        """
        return create_mask(self.obstacles_array, img, [0, 0, 255])

    def update_robot(self):
        thresh = 210
        white_coeffs = np.array([0.114, 0.587, 0.229])
        white = (self.blurred_frame.astype(float) * white_coeffs).sum(axis=-1)
        white[white > thresh] = 255
        white[white <= thresh] = 0

        def cond_robot(img_color, img_grayscale, cnt):
            return 1

        self.robot_array = polygon_detection(self.actual_frame, white.astype(np.uint8), 20, cond_robot)
        self.robot_mask = self.create_mask_robot(self.actual_frame.copy())

    def create_mask_robot(self, img):
        """
        This method creates a mask of the robot

        Input:
        - img : the image on which the mask will be added

        Output:
        - the image with the mask
        """
        return create_mask(self.robot_array, img, [255, 0, 0])

    def update_aim(self):
        thresh = 200
        aim_coeffs = np.array([-0.9, 0.8, 0.8])
        aim = (self.blurred_frame.astype(float) * aim_coeffs).sum(axis=-1)

        aim[aim > thresh] = 255
        aim[aim <= thresh] = 0

        # plt.imshow(aim, cmap='Greys')

        def cond_aim(img_color, img_grayscale, cnt):
            return 1

        self.aim_array = polygon_detection(self.actual_frame, aim.astype(np.uint8), 20, cond_aim)
        self.aim_mask = self.create_mask_aim(self.actual_frame.copy())

    def create_mask_aim(self, img):
        """
        This method creates a mask of the aim

        Input:
        - img : the image on which the mask will be added

        Output:
        - the image with the mask
        """
        return create_mask(self.aim_array, img, [0, 255, 0])

    def create_full_mask(self):
        """
        This method create a mask of the obstacles, the robot and the aim, based on the latest frame

        Output:
        - the image with the mask
        """
        return self.creat_mask_obstacles(self.create_mask_robot(self.create_mask_aim(self.actual_frame.copy())))


def create_mask(contours, img_mask, color):
    """
    This method creates a mask of the contours in the given color

    Input:
    - contours : the array of contours to be added on the image
    - img_mask : the image on which the mask will be added
    - color    : [r g b] array

    Output:
    - the image with the mask
    """
    for cnt in contours:
        cv2.drawContours(img_mask, [cnt], 0, (color[2], color[1], color[0]), 7)
    return cv2.cvtColor(img_mask, cv2.COLOR_BGR2RGB)


def polygon_detection(img_color, img_grayscale, area_min, condition):
    """
    This method creates a contours array

    Input:
    - img_color : the image in color
    - img_grayscale : the image in grayscale
    - area_min    : the minimal area for a contour
    - condition   : additional condition
            Input :
                - the image in color
                - the image in grayscale
                - the contour
            Output:
                - boolean, if the contour is valid

    Output:
    - the array of contours
    """
    # Converting image to a binary image
    # (black and white only image).
    _, threshold = cv2.threshold(img_grayscale, 110, 255,
                                 cv2.THRESH_BINARY)

    # Detecting shapes in image by selecting region
    # with same colors or intensity.
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE,
                                   cv2.CHAIN_APPROX_SIMPLE)
    approx_array = []
    # Searching through every region selected to
    # find the required polygon.
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # Shortlisting the regions based on there area.
        if area > area_min:
            mask = np.zeros(img_grayscale.shape, np.uint8)
            cv2.drawContours(mask, cnt, -1, 255, -1)
            contour_mean = cv2.mean(img_color, mask)

            if condition(img_color, img_grayscale, cnt):
                approx = cv2.approxPolyDP(cnt,
                                          0.01 * cv2.arcLength(cnt, True), True)
                approx_array.append(approx)
    # Showing the image along with outlined arrow.
    return approx_array
