import numpy as np
import math as _math
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
        robot_target = vision.target.center.astype(int)[0]

        if len(obstacle) == 0:
            self.path = [robot_start, robot_target]
        else :
            distance, start_dijk, target_dijk, contours_obstacles = self.create_path(obstacle, robot_start, robot_target)

            self.path = self.execution_dijk(distance, start_dijk, target_dijk, robot_start, robot_target, obstacle)

    def create_path(self, obstacle, robot_start, robot_target):
        start_path = []
        target_path = []
        distance = []
        nom_dijk = ['S', 'G']
        start_dijk = []
        target_dijk = []
        contours_obstacles = []

        for i in range(len(obstacle)):  # parcours tous les obstacles
            for j in range(len(obstacle[i])):  # parcours tous les angles
                nom_dijk.append(j * len(obstacle) + i)
                # print(i, ',', j, ',', j*len(obstacle)+i, obstacle[i][j])
                if j < len(obstacle[i]) - 1:
                    contours_obstacles.append([obstacle[i][j], obstacle[i][j + 1]])
                    distance.append(_math.dist(obstacle[i][j], obstacle[i][j + 1]))
                    start_dijk.append(j * len(obstacle) + i)
                    target_dijk.append((j + 1) * len(obstacle) + i)
                    distance.append(_math.dist(obstacle[i][j + 1], obstacle[i][j]))
                    start_dijk.append((j + 1) * len(obstacle) + i)
                    target_dijk.append(j * len(obstacle) + i)
                else:
                    contours_obstacles.append([obstacle[i][j], obstacle[i][0]])
                    distance.append(_math.dist(obstacle[i][j], obstacle[i][0]))
                    start_dijk.append(j * len(obstacle) + i)
                    target_dijk.append(i)
                    distance.append(_math.dist(obstacle[i][0], obstacle[i][j]))
                    start_dijk.append(i)
                    target_dijk.append(j * len(obstacle) + i)

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

                        start_tmp, target_tmp = map(Point, [obstacle[i][j], obstacle[k + i + 1][l]])
                        path = Line(start_tmp, target_tmp)

                        maxIntersection = 0
                        intersection = 0

                        for m in range(len(obstacle)):
                            isIntersection = poly[m].intersection(path)

                            intersection = len(isIntersection)

                            for n in range(len(isIntersection)):
                                if (start_tmp[0] > isIntersection[n][0] < target_tmp[0] or start_tmp[0] <
                                        isIntersection[n][0] >
                                        target_tmp[0]):
                                    intersection = intersection - 1

                            if intersection > maxIntersection:
                                maxIntersection = intersection

                        if maxIntersection == 1:
                            start_path.append(obstacle[i][j])
                            target_path.append(obstacle[k + i + 1][l])

                            distance.append(_math.dist(obstacle[i][j], obstacle[k + i + 1][l]))
                            start_dijk.append(j * len(obstacle) + i)
                            target_dijk.append(l * len(obstacle) + (k + i + 1))
                            distance.append(_math.dist(obstacle[i][j], obstacle[k + i + 1][l]))
                            start_dijk.append(l * len(obstacle) + (k + i + 1))
                            target_dijk.append(j * len(obstacle) + i)

                # lien obstacles robot

                start_tmp, target_tmp = map(Point, [obstacle[i][j], robot_start])
                path = Line(start_tmp, target_tmp)

                maxIntersection = 0
                intersection = 0

                for m in range(len(obstacle)):
                    isIntersection = poly[m].intersection(path)

                    intersection = len(isIntersection)

                    for n in range(len(isIntersection)):
                        if (start_tmp[0] > isIntersection[n][0] < target_tmp[0] or start_tmp[0] < isIntersection[n][0] >
                                target_tmp[0]):
                            intersection = intersection - 1

                    if intersection > maxIntersection:
                        maxIntersection = intersection

                if maxIntersection == 1:
                    start_path.append(obstacle[i][j])
                    target_path.append(robot_start)
                    distance.append(_math.dist(start_path[-1], target_path[-1]))

                    start_dijk.append('S')
                    target_dijk.append(j * len(obstacle) + i)

                # lien target obstacles

                start_tmp, target_tmp = map(Point, [obstacle[i][j], robot_target])
                path = Line(start_tmp, target_tmp)

                maxIntersection = 0
                intersection = 0

                for m in range(len(obstacle)):
                    isIntersection = poly[m].intersection(path)

                    intersection = len(isIntersection)

                    for n in range(len(isIntersection)):
                        if (start_tmp[0] > isIntersection[n][0] < target_tmp[0] or start_tmp[0] < isIntersection[n][0] >
                                target_tmp[0]):
                            intersection = intersection - 1

                    if intersection > maxIntersection:
                        maxIntersection = intersection

                if maxIntersection == 1:
                    start_path.append(obstacle[i][j])
                    target_path.append(robot_target)
                    distance.append(_math.dist(start_path[-1], target_path[-1]))

                    start_dijk.append(j * len(obstacle) + i)
                    target_dijk.append('G')

                # lien robot target

                start_tmp, target_tmp = map(Point, [robot_start, robot_target])
                path = Line(start_tmp, target_tmp)

                maxIntersection = 0
                intersection = 0

        for m in range(len(obstacle)):
            isIntersection = poly[m].intersection(path)

            intersection = len(isIntersection)

            for n in range(len(isIntersection)):
                if (start_tmp[0] > isIntersection[n][0] < target_tmp[0] or start_tmp[0] < isIntersection[n][0] >
                        target_tmp[0]):
                    intersection = intersection - 1

            if intersection > maxIntersection:
                maxIntersection = intersection

        if maxIntersection == 0:
            start_path.append(robot_start)
            target_path.append(robot_target)
            distance.append(_math.dist(start_path[-1], target_path[-1]))

            start_dijk.append('S')
            target_dijk.append('G')

        return distance, start_dijk, target_dijk, contours_obstacles

    def execution_dijk(self, distance, start_dijk, target_dijk, robot_start, robot_target, obstacle):
        '''path searching'''

        graph = Graph()

        for i in range(len(distance)):
            graph.add_edge(start_dijk[i], target_dijk[i], distance[i])

        resultat = find_path(graph, 'S', 'G')

        resultat_coor = []

        for i in range(len(resultat.nodes)):
            if resultat.nodes[i] == 'S':
                resultat_coor.append(robot_start)
            elif resultat.nodes[i] == 'G':
                resultat_coor.append(robot_target)
            else:
                reste = int(resultat.nodes[i] % len(obstacle))
                resultat_coor.append(obstacle[reste][int((resultat.nodes[i] - reste) / len(obstacle))])

        return resultat_coor

    def create_path_mask(self, img):
        for i in range(len(self.path) - 1):
            img = cv2.line(img, self.path[i], self.path[i + 1], (148, 0, 211), 2)

        return img

