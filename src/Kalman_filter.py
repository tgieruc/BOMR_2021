from vision import Vision
import numpy as np


class Kalman_filter():
    def __init__(self, vision):
        self.pos_x = vision.robot.center[0][0]
        self.pos_y = vision.robot.center[0][1]
        self.theta = vision.robot.orientation
        # self.rho = np.sqrt(self.pos_x ** 2 + self.pos_y ** 2)
        self.phi = np.arctan2(self.pos_y, self.pos_x)
        self.P_est = 1000 * np.ones(3)
        self.time = 0.01                                #"""en s"""
        self.thymio_to_mm_speed = 0.435                 #"""taken from exercise 8"""
        self.rho_est = np.array([[self.pos_x], [self.pos_y], [self.theta]])
        self.R = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])
        self.q1 = 0.1
        self.q2 = 0.1
        self.v = 0
        self.w = 0
        self.alpha = self.time * self.w /2
        self.Q = np.array([[self.q1, 0], [0, self.q2]])
        self.A = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.B = np.array([[self.time * np.cos(self.alpha), 0], [self.time * np.sin(self.alpha), 0], [0, self.time]])
        self.C = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.u = np.array([[self.v], [self.w]])

    # def compute_pos_cart(self):
    #     self.pos_x = self.rho_est[0] * np.cos(self.phi)
    #     self.pos_y = self.rho_est[0] * np.sin(self.phi)
    def update_values(self, vision, robot_speed, time):
        self.time = time
        self.v = self.thymio_to_mm_speed * (robot_speed[0] + robot_speed[1]) / 2
        self.w = self.thymio_to_mm_speed * (robot_speed[0] - robot_speed[1]) / 2
        self.u = np.array([[self.v], [self.w]])
        self.alpha = self.time * self.w / 2
        self.B = np.array([[self.time * np.cos(self.alpha), 0], [self.time * np.sin(self.alpha), 0], [0, self.time]])

    def update_kalman(self, vision, robot_speed, time):
        """" mise en place du filtre ici """
        self.update_values(vision, robot_speed, time)
        rho_est_a_priori = np.dot(self.A, self.rho_est) + np.dot(self.B, self.u)
        p_est_a_priori = np.dot(self.A, np.dot(self.P_est, self.A.T)) + np.dot(self.B, np.dot(self.Q, self.B.T))
        if vision.robot_detected():
            """
            kalman avec camera
            """
            self.pos_x = vision.robot.center[0][0]
            self.pos_y = vision.robot.center[0][1]
            self.theta = vision.robot.orientation

            S = np.dot(self.C, np.dot(p_est_a_priori, self.C.T)) + self.R
            K = np.dot(p_est_a_priori, np.dot(self.C.T, np.linalg.inv(S)))
            print(K.shape)

        else:
            """kalman avec uniquement les vitesses"""
            K = 0

        y = np.array([[self.pos_x], [self.pos_y], [self.theta]])
        i = y - np.dot(self.C, rho_est_a_priori)
        print(i.shape)
        self.rho_est = rho_est_a_priori + K * i
        self.P_est = np.dot((np.identity(3) - np.dot(K, self.C)), p_est_a_priori)
