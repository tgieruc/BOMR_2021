import cv2
import numpy as np
from dataclasses import dataclass
from copy import deepcopy

@dataclass
class Obstacles:
    contour: list= None
    expanded_contour: np.ndarray = None
    center: np.ndarray = None

@dataclass
class Robot:
    contour: list = None
    center: np.ndarray = None
    length: float = 60

@dataclass
class Goal:
    contour: list = None
    center: np.ndarray = None


class Vision():
    def __init__(self):
        self.actual_frame = cv2.imread('example.png', cv2.IMREAD_COLOR)
        self.blurred_frame = cv2.medianBlur(self.actual_frame, 9)
        self.gray_frame = cv2.imread('example.png', cv2.IMREAD_GRAYSCALE)
        self.obstacles = Obstacles()
        self.robot = Robot()
        self.goal = Goal()

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

        self.obstacles.contour = polygon_detection(self.actual_frame, self.gray_frame, 400, cond_obstacles)
        self.obstacles.center = self.get_centroid(self.obstacles.contour)
        self.obstacles.expanded_contour = self.expand_contours()

    def create_mask_obstacles(self, img):
        """
        This method creates a mask of all the obstacles

        Input:
        - img : the image on which the mask will be added

        Output:
        - the image with the mask
        """
        def draw_dot(img):
            for cnt in self.obstacles.expanded_contour:
                cnt = cnt.astype(int)
                for dot in cnt:
                    img = cv2.circle(img, dot.flatten(), 5, (0,0,255), -1)
            return img

        return draw_dot(create_mask(self.obstacles.contour, img, [0, 0, 255]))

    def expand_contours(self):
        """
        This method expands the corners of the obstacles contours by 0.5 * the robot length
        """
        exp_cnt = deepcopy(self.obstacles.contour)
        for i, cnt in enumerate(self.obstacles.contour):
            for j, corner in enumerate(cnt):
                e1 = cnt[(j - 1) % cnt.shape[0]] - corner
                e2 = cnt[(j + 1) % cnt.shape[0]] - corner
                bissec = (e1 / np.linalg.norm(e1) + e2 / np.linalg.norm(e2))
                exp_cnt[i][j] = corner - self.robot.length / 2 * bissec / np.linalg.norm(bissec)

        return exp_cnt
    def update_robot(self):
        thresh = 210
        white_coeffs = np.array([0.114, 0.587, 0.229])
        white = (self.blurred_frame.astype(float) * white_coeffs).sum(axis=-1)
        white[white > thresh] = 255
        white[white <= thresh] = 0

        def cond_robot(img_color, img_grayscale, cnt):
            return 1

        self.robot.contour = polygon_detection(self.actual_frame, white.astype(np.uint8), 20, cond_robot)
        self.robot.center = self.get_centroid(self.robot.contour)

    def create_mask_robot(self, img):
        """
        This method creates a mask of the robot

        Input:
        - img : the image on which the mask will be added

        Output:
        - the image with the mask
        """
        return create_mask(self.robot.contour, img, [255, 0, 0])

    def update_aim(self):
        thresh = 200
        aim_coeffs = np.array([-0.9, 0.8, 0.8])
        aim = (self.blurred_frame.astype(float) * aim_coeffs).sum(axis=-1)

        aim[aim > thresh] = 255
        aim[aim <= thresh] = 0

        def cond_aim(img_color, img_grayscale, cnt):
            return 1

        self.goal.contour = polygon_detection(self.actual_frame, aim.astype(np.uint8), 20, cond_aim)
        self.goal.center = self.get_centroid(self.goal.contour)
    def create_mask_aim(self, img):
        """
        This method creates a mask of the aim

        Input:
        - img : the image on which the mask will be added

        Output:
        - the image with the mask
        """
        return create_mask(self.goal.contour, img, [0, 255, 0])

    def create_full_mask(self):
        """
        This method create a mask of the obstacles, the robot and the aim, based on the latest frame

        Output:
        - the image with the mask
        """
        return self.create_mask_obstacles(self.create_mask_robot(self.create_mask_aim(self.actual_frame.copy())))

    def get_centroid(self, contour_array):
        """
        This method returns the centroid of each contour in contour_array

        Input:
        - contour_array: an array containing all contours

        Output:
        - the centroid of all the contours
        """
        centroid = np.zeros((len(contour_array),2))
        for i, cnt in enumerate(contour_array):
            M = cv2.moments(cnt)
            centroid[i, 0] = int(M['m10']/M['m00'])
            centroid[i, 1] = int(M['m01']/M['m00'])
        return centroid


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
                                          0.02 * cv2.arcLength(cnt, True), True)
                approx_array.append(approx)
    # Showing the image along with outlined arrow.
    return approx_array
