from vision import Vision
import numpy as np


class Kalman_filter():
    def __init__(self, vision):
        self.pos_x = vision.robot.center[0][0]
        self.pos_y = vision.robot.center[0][1]
        self.theta = vision.robot.orientation
        self.rho = np.sqrt(self.pos_x ** 2 + self.pos_y ** 2)
        self.alpha = np.arctan2(self.pos_y, self.pos_x)
        self.P_est = 1000 * np.ones(2)
        self.state = "camera_on"
        self.time = 100                                 #"""en ms"""
        self.thymio_to_mm_speed = 0.435                 #"""taken from exercise 8"""
        self.speed = 0
        self.rho_est = np.array([[self.rho],[self.speed]])
        self.qp = 0.04
        self.q_nu = 6.15
        self.Q = np.array([[self.qp, 0], [0, self.q_nu]])
        self.A = np.array([[1, self.time/1000], [0, 1]])

    def compute_pos_cart(self):
        self.pos_x = self.rho_est[0] * np.cos(self.alpha)
        self.pos_y = self.rho_est[0] * np.sin(self.alpha)

    def update_value(self, vision, robot_speed):
        self.pos_x = vision.robot.center[0][0]
        self.pos_y = vision.robot.center[0][1]
        self.theta = vision.robot.orientation
        if vision.robot_detected:
            self.state = "camera_on"
        else:
            self.state = "camera_not_on"
        self.rho = np.sqrt(self.pos_x ** 2 + self.pos_y ** 2)
        self.speed = vision.mm2px * self.thymio_to_mm_speed * (robot_speed[0] + robot_speed[1]) / 2

    def update_kalman(self, vision, robot_speed):
        self.update_value(vision, robot_speed)
        """" mise en place du filtre ici """
        rho_est_a_priori = np.dot(self.A, self.rho_est)
        p_est_a_priori = np.dot(self.A, np.dot(self.P_est, self.A.T)) + self.Q
        r_nu = 6.15
        if self.state == "camera_on":
            """
            kalman avec camera
            """
            rp = 0.25

            R = np.array([[rp, 0], [0, r_nu]])
            H = np.array([[1, 0], [0, 1]])
            y = np.array([[self.rho], [self.speed]])
        elif self.state == "camera_not_on":
            """kalman avec uniquement les vitesses"""
            y = self.speed
            H = np.array([0, 1])
            R = r_nu

        i = y - np.dot(H, rho_est_a_priori)
        S = np.dot(H, np.dot(p_est_a_priori, H.T)) + R
        K = np.dot(p_est_a_priori, np.dot(H.T, np.linalg.inv(S)))

        self.rho_est = rho_est_a_priori + np.dot(K, i)
        self.P_est = p_est_a_priori - np.dot(K, np.dot(H, p_est_a_priori))
        self.compute_pos_cart()
