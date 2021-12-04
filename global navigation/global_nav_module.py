import sys as _sys
import numpy as np
import math as _math
from dataclasses import dataclass
import cv2

from sympy import Point, Polygon, Line

from dijkstar import Graph, find_path

import matplotlib.pyplot as plt

# from BOMR_2021.Vision.vision import Vision

_sys.path.append("../Vision")

from vision import Vision

# @dataclass
# class Road:
#     start_chemin: np.ndarray = None
#     goal_chemin: np.ndarray = None
#     distance: list = None


vision = Vision()

vision.update()

image = vision.actual_frame.copy()

plt.imshow(vision.create_mask_obstacles(vision.actual_frame.copy()))

plt.imshow(vision.create_mask_robot(vision.actual_frame.copy()))

plt.imshow(vision.create_full_mask())

# print(vision.obstacles.expanded_contour[0][0,0])


obstacle_test = vision.obstacles.expanded_contour
robot_start_test = np.array([int(vision.robot.center[0][0]), int(vision.robot.center[0][1])])
# robot_start_test = np.array([650, 50])
print(robot_start_test)
robot_goal_test = np.array([int(vision.goal.center[0][0]), int(vision.goal.center[0][1])])
# robot_goal_test = np.array([500, 300])
print(robot_goal_test)

# relier tous les angles des obstacles

# num = 0
# start_chemin.append(obstacle[i][j])
# goal_chemin.append(obstacle[k + i + 1][l])


def chemin_creation(obstacle, robot_start, robot_goal):

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
                    chemin = Line(start_tmp, goal_tmp)

                    maxIntersection = 0
                    intersection = 0

                    for m in range(len(obstacle)):
                        isIntersection = poly[m].intersection(chemin)

                        intersection = len(isIntersection)

                        for n in range(len(isIntersection)):
                            if (start_tmp[0] > isIntersection[n][0] < goal_tmp[0] or start_tmp[0] < isIntersection[n][0] >
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
            chemin = Line(start_tmp, goal_tmp)

            maxIntersection = 0
            intersection = 0

            for m in range(len(obstacle)):
                isIntersection = poly[m].intersection(chemin)

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
            chemin = Line(start_tmp, goal_tmp)

            maxIntersection = 0
            intersection = 0

            for m in range(len(obstacle)):
                isIntersection = poly[m].intersection(chemin)

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
            chemin = Line(start_tmp, goal_tmp)

            maxIntersection = 0
            intersection = 0

            for m in range(len(obstacle)):
                isIntersection = poly[m].intersection(chemin)

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

def execution_dijk(distance, start_dijk, goal_dijk, robot_start, robot_goal, obstacle):

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


    # for i in range(len(goal_chemin)):
    #     plt.imshow(cv2.line(image, start_chemin[i], goal_chemin[i], (0, 100, 255), 5))


distance, start_dijk, goal_dijk, contours_obstacles = chemin_creation(obstacle_test, robot_start_test, robot_goal_test)

resultat_coor = execution_dijk(distance, start_dijk, goal_dijk, robot_start_test, robot_goal_test, obstacle_test)

for i in range(len(contours_obstacles)):
    plt.imshow(cv2.line(image, contours_obstacles[i][0], contours_obstacles[i][1], (0, 255, 255), 5))


i = 1

while i in range(len(resultat_coor)):
    plt.imshow(cv2.line(image, resultat_coor[i - 1], resultat_coor[i], (255, 0, 0), 2))
    i = i + 1

# cv2.imshow('contours',image)
plt.show()

print('fin')