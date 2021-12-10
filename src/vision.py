import cv2
import numpy as np
from copy import deepcopy

class Vision():
    def __init__(self):
        self.actual_frame = None
        self.blurred_frame = None
        self.gray_frame = None
        self.obstacles = Obstacles()
        self.robot = Robot()
        self.target = Target()
        self.vc = None
        self.status = None
        self.mm2px = None

    def set_mm2px(self):
        if not self.robot.detected():
            print("First the robot has to be detected")
        else:
            contour = self.robot.contour[0]
            d1 = np.linalg.norm(contour[0] - contour[1])
            d2 = np.linalg.norm(contour[2] - contour[1])
            d3 = np.linalg.norm(contour[0] - contour[2])

            longest_edge = max(d1, d2, d3)
            self.mm2px = longest_edge / self.robot.length_edge
            print("mm/px ratio : ", self.mm2px)

    def set_threshold(self, target_thresh, robot_thresh, obstacles_thresh):
        self.target.thresh = target_thresh
        self.robot.thresh = robot_thresh
        self.obstacles.thresh = obstacles_thresh

    def set_colors(self, color_obstacles, color_robot, color_target):
        self.obstacles.color = color_obstacles
        self.robot.color = color_robot
        self.target.color = color_target

    def disconnect_camera(self):
        self.vc.release()

    def connect_camera(self, camera_number):
        self.vc = cv2.VideoCapture(camera_number, cv2.CAP_DSHOW)

    def update_fake_frame(self, img_path):
        self.actual_frame = cv2.imread(img_path, cv2.IMREAD_COLOR)
        self.actual_frame = cv2.cvtColor(self.actual_frame, cv2.COLOR_BGR2RGB)
        self.blurred_frame = cv2.medianBlur(self.actual_frame, 9)
        self.gray_frame = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

    def update_frame(self):
        captured = False
        while not captured:
            captured, self.actual_frame = self.vc.read()
        self.actual_frame = cv2.cvtColor(self.actual_frame, cv2.COLOR_BGR2RGB)

        self.blurred_frame = cv2.medianBlur(self.actual_frame, 9)
        self.gray_frame = cv2.cvtColor(self.actual_frame, cv2.COLOR_RGB2GRAY)

    def update(self):
        """ Updates all components """
        self.update_robot()
        self.update_obstacles()
        self.update_target()

    def update_obstacles(self):
        obstacle_thresh = color_compare(self.actual_frame, self.obstacles.color, self.obstacles.thresh)
        obstacle_thresh[0:int(self.robot.length), :] = 255
        obstacle_thresh[-int(self.robot.length):, :] = 255
        obstacle_thresh[:, :int(self.robot.length)] = 255
        obstacle_thresh[:, -int(self.robot.length):] = 255
        self.obstacles.contour = polygon_detection(obstacle_thresh.astype(np.uint8), 400,
                                                   0.9 * self.actual_frame.shape[0] * self.actual_frame.shape[1])
        self.obstacles.center = self.get_centroid(self.obstacles.contour)
        self.obstacles.expand_contours(self.robot.length)
        self.obstacles.simplify_expanded_edges(self.actual_frame)

    def get_triangle(self, contours):
        """ Returns the first triangular shaped contour"""
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

        if self.robot.detected():
            _, self.robot.length = cv2.minEnclosingCircle(self.robot.contour[0])
            self.robot.length *= 2
            self.robot.get_orientation()

    def update_target(self):
        target_thresh = color_compare(self.blurred_frame, self.target.color, self.target.thresh)

        self.target.contour = self.get_biggest_area_cnt(polygon_detection(target_thresh.astype(np.uint8), 400,
                                                                          0.9 * self.actual_frame.shape[0] *
                                                                          self.actual_frame.shape[1]))
        self.target.center = self.get_centroid(self.target.contour)

    def create_full_mask(self):
        """ This method create a mask of the obstacles, the robot and the aim, based on the latest frame """
        return self.obstacles.create_mask(self.robot.create_mask(self.target.create_mask(self.actual_frame.copy())))

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
        """ Compares a list of contours, return the one with the biggest area """
        area = []
        if contours == []:
            return []
        for cnt in contours:
            area_tmp = cv2.contourArea(cnt)
            area.append(area_tmp)
        max_area = max(area)
        return [contours[area.index(max_area)]]


class Obstacles():
    def __init__(self):
        self.contour = None
        self.expanded_contour = None
        self.center = None
        self.color = np.array([17, 21, 20])
        self.thresh = 40

    def expand_contours(self, robot_length):
        """
        This method expands the corners of the obstacles contours by 0.5 * the robot length
        """
        exp_cnt = deepcopy(self.contour)
        for i, cnt in enumerate(self.contour):
            for j, corner in enumerate(cnt):
                e1 = cnt[(j - 1) % cnt.shape[0]] - corner
                e2 = cnt[(j + 1) % cnt.shape[0]] - corner
                bisector = (e1 / np.linalg.norm(e1) + e2 / np.linalg.norm(e2))
                exp_cnt[i][j] = corner - 0.8 * robot_length * bisector / np.linalg.norm(bisector)

        self.expanded_contour = exp_cnt

    def simplify_expanded_edges(self, actual_frame):
        img = 255 * np.ones_like(actual_frame[:, :, 0])
        if self.expanded_contour is not None:
            for cnt in self.expanded_contour:
                cv2.drawContours(img, [cnt], 0, 0, thickness=cv2.FILLED)
            self.expanded_contour = polygon_detection(img.astype(np.uint8), 400, 0.9 * actual_frame.shape[0] *
                                                      actual_frame.shape[1])
            img = 255 * np.ones_like(actual_frame[:, :, 0])

            for contour in self.expanded_contour:
                convexHull = cv2.convexHull(contour)
                cv2.drawContours(img, [convexHull], -1, 0, 2)
            self.expanded_contour = polygon_detection(img.astype(np.uint8), 400, 0.9 * actual_frame.shape[0] *
                                                      actual_frame.shape[1])

    def create_mask(self, img):
        """
        This method creates a mask of all the obstacles
        """

        for cnt in self.expanded_contour:
            cnt = cnt.astype(int)
            for dot in cnt:
                img = cv2.circle(img, dot.flatten(), 5, (255, 0, 0), -1)

        return create_mask(self.contour, img, [255, 0, 0])


class Robot:
    def __init__(self):
        self.contour = None
        self.center = None
        self.length = 60
        self.color = np.array([148, 49, 48])
        self.orientation = None
        self.thresh = 20
        self.length_edge = 115

    def detected(self):
        if self.contour is None:
            return False
        if len(self.contour) == 0:
            return False

        return True

    def get_orientation(self):
        """ Get the robot orientation, stores it in self.robot.orientation"""
        contour = self.contour[0]
        d1 = np.linalg.norm(contour[0] - contour[1])
        d2 = np.linalg.norm(contour[2] - contour[1])
        d3 = np.linalg.norm(contour[0] - contour[2])

        if (d1 < d2) & (d1 < d3):
            peak = contour[2]
        elif (d2 < d1) & (d2 < d3):
            peak = contour[0]
        else:
            peak = contour[1]

        self.orientation = np.arctan2(peak[1] - self.center[0, 1], peak[0] - self.center[0, 0])

    def create_mask(self, img):
        """
        This method creates a mask of the robot

        Input:
        - img : the image on which the mask will be added
        Output:
        - the image with the mask
        """
        if self.detected():
            point = (self.center + np.array([[np.cos(self.orientation) * 100, np.sin(self.orientation) * 100]])).astype(int)
            center = self.center.astype(int)
            img = cv2.line(img, (center[0, 0], center[0, 1]), (point[0, 0], point[0, 1]), (0, 0, 255), thickness=3, lineType=8)

        return create_mask(self.contour, img, [0, 0, 255])


class Target:
    def __init__(self):
        self.contour = None
        self.center = None
        self.color = np.array([78, 130, 69])
        self.thresh = 20

    def create_mask(self, img):
        """ This method creates a mask of the aim """
        return create_mask(self.contour, img, [0, 255, 0])





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
    This method creates a list of contours

    Input:
    - threshold   : the threshold for which the contour will be selected
    - area_min    : the minimal area for a contour
    - area_max    : the maximal area for a contour

    Output:
    - the list of contours
    """
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    approx_array = []

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
        area = cv2.contourArea(approx)
        if min_area < area < max_area:
            approx_array.append(approx.reshape(-1, 2))

    return approx_array


def color_compare(img, color, threshold):
    img_color = abs(img - color)
    img_gray = np.mean(img_color, axis=2)
    _, img_bin = cv2.threshold(img_gray, threshold, 10, cv2.THRESH_BINARY)
    return img_bin
