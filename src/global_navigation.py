import sys as _sys
import numpy as np
import math as _math
from dataclasses import dataclass
import cv2

from sympy import Point, Polygon, Line
from dijkstar import Graph, find_path

import matplotlib.pyplot as plt

from vision import Vision


class Path_planner():
    def __init__(self):
        self.path = None

    def make_path_stub(self, vision):
        '''
        Stub fct
        '''
        out = [np.array([132, 589]), np.array([221, 399]), np.array([523, 281]), np.array([589, 276]),
               np.array([583, 182])]
        self.path = np.array(out)


    def make_path(self, vision):
        obstacle = vision.obstacles.expanded_contour
        robot_start = vision.robot.center.astype(int)[0]
        robot_goal = vision.goal.center.astype(int)[0]

        distance, start_dijk, goal_dijk, contours_obstacles = self.create_path(obstacle, robot_start,
                                                                               robot_goal)

        self.path = self.execution_dijk(distance, start_dijk, goal_dijk, robot_start, robot_goal,
                                        obstacle)

    def create_path(self, obstacle, robot_start, robot_goal):
        start_chemin = []
        goal_chemin = []
        distance = []
        # marge = 0
        nom_dijk = ['S', 'G']
        start_dijk = []
        goal_dijk = []
        contours_obstacles = []

        for i in range(len(obstacle)):  # parcours tous les obstacles
            for j in range(len(obstacle[i])):  # parcours tous les angles
                nom_dijk.append(j * len(obstacle) + i)
                # print(i, ',', j, ',', j*len(obstacle)+i, obstacle[i][j])
                if j < len(obstacle[i]) - 1:
                    contours_obstacles.append([obstacle[i][j], obstacle[i][j + 1]])
                    distance.append(_math.dist(obstacle[i][j], obstacle[i][j + 1]))
                    start_dijk.append(j * len(obstacle) + i)
                    goal_dijk.append((j + 1) * len(obstacle) + i)
                    distance.append(_math.dist(obstacle[i][j + 1], obstacle[i][j]))
                    start_dijk.append((j + 1) * len(obstacle) + i)
                    goal_dijk.append(j * len(obstacle) + i)
                else:
                    contours_obstacles.append([obstacle[i][j], obstacle[i][0]])
                    distance.append(_math.dist(obstacle[i][j], obstacle[i][0]))
                    start_dijk.append(j * len(obstacle) + i)
                    goal_dijk.append(i)
                    distance.append(_math.dist(obstacle[i][0], obstacle[i][j]))
                    start_dijk.append(i)
                    goal_dijk.append(j * len(obstacle) + i)

        pt_obstacle = []
        poly = []

        for i in range(len(obstacle)):
            pt_obstacle.append(list(map(Point, obstacle[i])))
            poly.append(Polygon(*pt_obstacle[i]))

        for i in range(len(obstacle)):  # parcours tous les obstacles
            # print(obstacle[i])
            for j in range(len(obstacle[i])):  # parcours tous les angles
                # print(obstacle[i][j])
                for k in range(len(obstacle) - i - 1):  # parcours tous les obstacles plus loin dans la liste
                    # print(obstacle[i])
                    for l in range(len(obstacle[k + i + 1])):  # parcours tous les angles

                        start_tmp, goal_tmp = map(Point, [obstacle[i][j], obstacle[k + i + 1][l]])
                        path = Line(start_tmp, goal_tmp)

                        maxIntersection = 0
                        intersection = 0

                        for m in range(len(obstacle)):
                            isIntersection = poly[m].intersection(path)

                            intersection = len(isIntersection)

                            for n in range(len(isIntersection)):
                                if (start_tmp[0] > isIntersection[n][0] < goal_tmp[0] or start_tmp[0] <
                                        isIntersection[n][0] >
                                        goal_tmp[0]):
                                    intersection = intersection - 1

                            if intersection > maxIntersection:
                                maxIntersection = intersection

                        if maxIntersection == 1:
                            start_chemin.append(obstacle[i][j])
                            goal_chemin.append(obstacle[k + i + 1][l])

                            distance.append(_math.dist(obstacle[i][j], obstacle[k + i + 1][l]))
                            start_dijk.append(j * len(obstacle) + i)
                            goal_dijk.append(l * len(obstacle) + (k + i + 1))
                            distance.append(_math.dist(obstacle[i][j], obstacle[k + i + 1][l]))
                            start_dijk.append(l * len(obstacle) + (k + i + 1))
                            goal_dijk.append(j * len(obstacle) + i)

                # lien obstacles robot

                start_tmp, goal_tmp = map(Point, [obstacle[i][j], robot_start])
                path = Line(start_tmp, goal_tmp)

                maxIntersection = 0
                intersection = 0

                for m in range(len(obstacle)):
                    isIntersection = poly[m].intersection(path)

                    intersection = len(isIntersection)

                    for n in range(len(isIntersection)):
                        if (start_tmp[0] > isIntersection[n][0] < goal_tmp[0] or start_tmp[0] < isIntersection[n][0] >
                                goal_tmp[0]):
                            intersection = intersection - 1

                    if intersection > maxIntersection:
                        maxIntersection = intersection

                if maxIntersection == 1:
                    start_chemin.append(obstacle[i][j])
                    goal_chemin.append(robot_start)
                    distance.append(_math.dist(start_chemin[-1], goal_chemin[-1]))

                    start_dijk.append('S')
                    goal_dijk.append(j * len(obstacle) + i)

                # lien goal obstacles

                start_tmp, goal_tmp = map(Point, [obstacle[i][j], robot_goal])
                path = Line(start_tmp, goal_tmp)

                maxIntersection = 0
                intersection = 0

                for m in range(len(obstacle)):
                    isIntersection = poly[m].intersection(path)

                    intersection = len(isIntersection)

                    for n in range(len(isIntersection)):
                        if (start_tmp[0] > isIntersection[n][0] < goal_tmp[0] or start_tmp[0] < isIntersection[n][0] >
                                goal_tmp[0]):
                            intersection = intersection - 1

                    if intersection > maxIntersection:
                        maxIntersection = intersection

                if maxIntersection == 1:
                    start_chemin.append(obstacle[i][j])
                    goal_chemin.append(robot_goal)
                    distance.append(_math.dist(start_chemin[-1], goal_chemin[-1]))

                    start_dijk.append(j * len(obstacle) + i)
                    goal_dijk.append('G')

                # lien robot goal

                start_tmp, goal_tmp = map(Point, [robot_start, robot_goal])
                path = Line(start_tmp, goal_tmp)

                maxIntersection = 0
                intersection = 0

                for m in range(len(obstacle)):
                    isIntersection = poly[m].intersection(path)

                    intersection = len(isIntersection)

                    for n in range(len(isIntersection)):
                        if (start_tmp[0] > isIntersection[n][0] < goal_tmp[0] or start_tmp[0] < isIntersection[n][0] >
                                goal_tmp[0]):
                            intersection = intersection - 1

                    if intersection > maxIntersection:
                        maxIntersection = intersection

                if maxIntersection == 0:
                    start_chemin.append(robot_start)
                    goal_chemin.append(robot_goal)
                    distance.append(_math.dist(start_chemin[-1], goal_chemin[-1]))

                    start_dijk.append('S')
                    goal_dijk.append('G')

        return distance, start_dijk, goal_dijk, contours_obstacles

    # recherche du chemin

    # graph = Graph()
    # graph.add_edge(1, 2, 110)
    # graph.add_edge(2, 3, 125)
    # graph.add_edge(3, 4, 108)
    # find_path(graph, 1, 4)

    def execution_dijk(self, distance, start_dijk, goal_dijk, robot_start, robot_goal, obstacle):

        graph = Graph()

        for i in range(len(distance)):
            graph.add_edge(start_dijk[i], goal_dijk[i], distance[i])

        resultat = find_path(graph, 'S', 'G')

        resultat_coor = []

        for i in range(len(resultat.nodes)):
            if resultat.nodes[i] == 'S':
                resultat_coor.append(robot_start)
                # print('resultats', resultat.nodes[i], resultat_coor[i])
            elif resultat.nodes[i] == 'G':
                resultat_coor.append(robot_goal)
                # print('resultats',resultat.nodes[i],resultat_coor[i])
            else:
                reste = int(resultat.nodes[i] % len(obstacle))
                resultat_coor.append(obstacle[reste][int((resultat.nodes[i] - reste) / len(obstacle))])
                # print('resultats',resultat.nodes[i],resultat_coor[i])

        return resultat_coor

    def create_path_mask(self, img):
        for i in range(len(self.path) - 1):
            img = cv2.line(img, self.path[i], self.path[i + 1], (148, 0, 211), 2)

        return img

