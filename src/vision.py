import cv2
import numpy as np
from dataclasses import dataclass
from copy import deepcopy
import matplotlib.pyplot as plt


@dataclass
class Obstacles:
    contour: list = None
    expanded_contour: list = None
    center: np.ndarray = None
    color: np.ndarray = None
    thresh: int = None


@dataclass
class Robot:
    contour: list = None
    center: np.ndarray = None
    length: float = 60
    color: np.ndarray = None
    orientation: float = None
    thresh: int = None


@dataclass
class Goal:
    contour: list = None
    center: np.ndarray = None
    color: np.ndarray = None
    thresh: int = None


class Vision():
    def __init__(self, path="../Vision/5_triangle.png"):
        # self.actual_frame = cv2.imread(path, cv2.IMREAD_COLOR)
        # self.actual_frame = cv2.cvtColor(self.actual_frame, cv2.COLOR_BGR2RGB)
        # self.blurred_frame = cv2.medianBlur(self.actual_frame, 9)
        # self.gray_frame = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        self.actual_frame = None
        # self.actual_frame = cv2.cvtColor(self.actual_frame, cv2.COLOR_BGR2RGB)
        self.blurred_frame = None
        self.gray_frame = None
        self.obstacles = Obstacles()
        self.robot = Robot()
        self.goal = Goal()
        self.vc = None
        self.obstacles.color = np.array([17, 21, 20])
        self.robot.color = np.array([148, 49, 48])
        self.goal.color = np.array([78, 130, 69])
        self.status = None
        self.goal.thresh = 20
        self.robot.thresh = 20
        self.obstacles.thresh = 40
        self.mm2px = None
        self.length_edge = 115

    def set_mm2px(self):
        if not self.robot_detected():
            print("First the robot has to be detected")
        else:
            contour = self.robot.contour[0]
            d1 = np.linalg.norm(contour[0] - contour[1])
            d2 = np.linalg.norm(contour[2] - contour[1])
            d3 = np.linalg.norm(contour[0] - contour[2])

            longest_edge = max(d1, d2, d3)
            self.mm2px = longest_edge / self.length_edge
            print("mm/px ratio : ",self.mm2px)

    def set_tresh(self, go, ro, ob):
        self.goal.thresh = go
        self.robot.thresh = ro
        self.obstacles.thresh = ob

    def robot_detected(self):
        if self.robot.contour is None:
            return False
        if len(self.robot.contour) == 0:
            return False

        return True

    def set_colors(self, color_obstacles, color_robot, color_goal):
        self.obstacles.color = color_obstacles
        self.robot.color = color_robot
        self.goal.color = color_goal

    def disconnect_camera(self):
        self.vc.release()

    def connect_camera(self, camera_number):
        self.vc = cv2.VideoCapture(camera_number, cv2.CAP_DSHOW)

    def update_frame(self):
        captured = False
        while not captured:
            captured, self.actual_frame = self.vc.read()
        self.actual_frame = cv2.cvtColor(self.actual_frame, cv2.COLOR_BGR2RGB)

        self.blurred_frame = cv2.medianBlur(self.actual_frame, 9)
        self.gray_frame = cv2.cvtColor(self.actual_frame, cv2.COLOR_RGB2GRAY)

    def update(self):
        """
        Updates (obstacles robots aim)_array and _mask
        """
        self.update_robot()
        self.update_obstacles()
        self.update_goal()

    def update_obstacles(self):
        obstacle_thresh = color_compare(self.actual_frame, self.obstacles.color, self.obstacles.thresh)
        self.obstacles.contour = polygon_detection(obstacle_thresh.astype(np.uint8), 400,
                                                   0.9 * self.actual_frame.shape[0] * self.actual_frame.shape[1])
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
                    img = cv2.circle(img, dot.flatten(), 5, (255, 0, 0), -1)
            return img

        return draw_dot(create_mask(self.obstacles.contour, img, [255, 0, 0]))

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
                exp_cnt[i][j] = corner - 1.5 * self.robot.length / 2 * bissec / np.linalg.norm(bissec)

        return exp_cnt

    def get_triangle(self, contours):
        if contours == []:
            return []
        for cnt in contours:
            if cnt.shape[0] == 3:
                return [cnt]
        return []

    def update_robot(self):
        robot_thresh = color_compare(self.actual_frame, self.robot.color, self.robot.thresh)
        self.robot.contour = self.get_triangle(polygon_detection(robot_thresh.astype(np.uint8), 40,
                                                                 0.9 * self.actual_frame.shape[0] *
                                                                 self.actual_frame.shape[1]))
        self.robot.center = self.get_centroid(self.robot.contour)
        if self.robot.center.size != 0:
            _, self.robot.length = cv2.minEnclosingCircle(self.robot.contour[0])
            self.robot.length *= 2
            self.get_robot_orientation()

    def get_robot_orientation(self):
        contour = self.robot.contour[0]
        d1 = np.linalg.norm(contour[0] - contour[1])
        d2 = np.linalg.norm(contour[2] - contour[1])
        d3 = np.linalg.norm(contour[0] - contour[2])

        if (d1 < d2) & (d1 < d3):
            sommet = contour[2]
        elif (d2 < d1) & (d2 < d3):
            sommet = contour[0]
        else:
            sommet = contour[1]

        self.robot.orientation = -np.arctan2(sommet[1] - self.robot.center[0, 1], sommet[0] - self.robot.center[0, 0])

    def create_mask_robot(self, img):
        """
        This method creates a mask of the robot

        Input:
        - img : the image on which the mask will be added

        Output:
        - the image with the mask
        """
        if self.robot_detected():
            point = (self.robot.center + np.array(
                [[np.cos(self.robot.orientation) * 100, -np.sin(self.robot.orientation) * 100]])).astype(int)
            center = self.robot.center.astype(int)
            img = cv2.line(img, (center[0, 0], center[0, 1]), (point[0, 0], point[0, 1]), (0, 255, 0), thickness=3,
                           lineType=8)

        return create_mask(self.robot.contour, img, [0, 0, 255])

    def update_goal(self):
        goal_thresh = color_compare(self.blurred_frame, self.goal.color, self.goal.thresh)

        self.goal.contour = self.get_biggest_area_cnt(polygon_detection(goal_thresh.astype(np.uint8), 400,
                                                                        0.9 * self.actual_frame.shape[0] *
                                                                        self.actual_frame.shape[1]))
        self.goal.center = self.get_centroid(self.goal.contour)

    def create_mask_goal(self, img):
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
        return self.create_mask_obstacles(self.create_mask_robot(self.create_mask_goal(self.actual_frame.copy())))

    def get_centroid(self, contour_array):
        """
        This method returns the centroid of each contour in contour_array

        Input:
        - contour_array: an array containing all contours

        Output:
        - the centroid of all the contours
        """
        centroid = np.zeros((len(contour_array), 2))
        for i, cnt in enumerate(contour_array):
            M = cv2.moments(cnt)
            centroid[i, 0] = int(M['m10'] / M['m00'])
            centroid[i, 1] = int(M['m01'] / M['m00'])
        return centroid

    def get_biggest_area_cnt(self, contours):
        area = []
        if contours == []:
            return []
        for cnt in contours:
            area_tmp = cv2.contourArea(cnt)
            area.append(area_tmp)
        max_area = max(area)
        return [contours[area.index(max_area)]]


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
    if contours is not None:
        for cnt in contours:
            cv2.drawContours(img_mask, [cnt], 0, (color[0], color[1], color[2]), 2)
    return img_mask


def polygon_detection(threshold, min_area, max_area):
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
    # _, threshold = cv2.threshold(img_grayscale, 110, 255,
    #                              cv2.THRESH_BINARY)

    # Detecting shapes in image by selecting region
    # with same colors or intensity.
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE,
                                   cv2.CHAIN_APPROX_SIMPLE)
    approx_array = []
    # Searching through every region selected to
    # find the required polygon.
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
        area = cv2.contourArea(approx)
        # Shortlisting the regions based on there area.
        if min_area < area < max_area:
            approx_array.append(approx.reshape(-1, 2))
    # Showing the image along with outlined arrow.
    return approx_array


def color_compare(img, color, threshold):
    img_color = abs(img - color)
    img_gray = np.mean(img_color, axis=2)
    _, img_bin = cv2.threshold(img_gray, threshold, 10, cv2.THRESH_BINARY)
    return img_bin
