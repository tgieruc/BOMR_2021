import sys as _sys
import numpy as np
import math as _math
from dataclasses import dataclass
import cv2

from sympy import Point, Polygon, Line

from dijkstar import Graph, find_path

import matplotlib.pyplot as plt

from BOMR_2021.Vision.vision import Vision

_sys.path.append("../Vision")

from vision import *

# @dataclass
# class Road:
#     start_chemin: np.ndarray = None
#     goal_chemin: np.ndarray = None
#     distance: list = None
#
#     def __init__(self):
#         self.start_chemin = [1,1]
#         self.goal_chemin = [2,2]
#         self.distance = [1]

vision = Vision()

vision.update()

image = vision.actual_frame.copy()


plt.imshow(vision.create_mask_obstacles(vision.actual_frame.copy()))


plt.imshow(vision.create_mask_robot(vision.actual_frame.copy()))


plt.imshow(vision.create_full_mask())

# print(vision.obstacles.expanded_contour[0][0,0])


obstacle = vision.obstacles.expanded_contour
robot_start = np.array([int(vision.robot.center[0][0]), int(vision.robot.center[0][1])])
# robot_start = np.array([100, 350])
print(robot_start)
robot_goal = np.array([int(vision.goal.center[0][0]),int(vision.goal.center[0][1])])
# robot_goal = np.array([500,300])
print(robot_goal)
# print(obstacle[0][0,0])

start_chemin = []
goal_chemin = []
distance = []
marge = 0
nom_dijk = ['S', 'G']
start_dijk = []
goal_dijk = []

pt_obstacle = []
poly = []

for i in range(len(obstacle)):
    pt_obstacle.append(list(map(Point, obstacle[i])))
    poly.append(Polygon(*pt_obstacle[i]))

# relier tous les angles des obstacles

contours_obstacles = []


num = 0
# start_chemin.append(obstacle[i][j])
# goal_chemin.append(obstacle[k + i + 1][l])

for i in range(len(obstacle)): #parcours tous les obstacles
    for j in range(len(obstacle[i])): #parcours tous les angles
        inside = False
        maxIntersection = 0
        intersection = 0
        if j < len(obstacle[i])-1:
            contours_obstacles.append([obstacle[i][j], obstacle[i][j + 1]])
            start_tmp, goal_tmp = map(Point, [obstacle[i][j], obstacle[i][j + 1]])
            chemin = Line(start_tmp, goal_tmp)

            maxIntersection = 0
            intersection = 0
            inside = False

            for m in range(len(obstacle)):
                if m != i:
                    isIntersection = poly[m].intersection(chemin)

                    intersection = len(isIntersection)

                    for n in range(len(isIntersection)):
                        if (start_tmp[0] > isIntersection[n][0] < goal_tmp[0] or start_tmp[0] < isIntersection[n][0] >
                                goal_tmp[0]):
                            intersection = intersection - 1

                    if intersection > maxIntersection:
                        maxIntersection = intersection

            for a in range(len(obstacle)):
                if poly[a].encloses_point(pt_obstacle[i][j]):
                    inside = True
                    break
            if maxIntersection == 1 and not inside:
                nom_dijk.append(j * len(obstacle) + i)
                distance.append(_math.dist(obstacle[i][j], obstacle[i][j+1]))
                start_dijk.append(j * len(obstacle) + i)
                goal_dijk.append((j+1) * len(obstacle) + i)
                distance.append(_math.dist(obstacle[i][j+1], obstacle[i][j]))
                start_dijk.append((j+1) * len(obstacle) + i)
                goal_dijk.append(j * len(obstacle) + i)
        else:
            contours_obstacles.append([obstacle[i][j], obstacle[i][0]])

            contours_obstacles.append([obstacle[i][j], obstacle[i][0]])
            start_tmp, goal_tmp = map(Point, [obstacle[i][j], obstacle[i][0]])
            chemin = Line(start_tmp, goal_tmp)

            maxIntersection = 0
            intersection = 0
            inside = False

            for m in range(len(obstacle)):
                if m != i:
                    isIntersection = poly[m].intersection(chemin)

                    intersection = len(isIntersection)

                    for n in range(len(isIntersection)):
                        if (start_tmp[0] > isIntersection[n][0] < goal_tmp[0] or start_tmp[0] < isIntersection[n][0] >
                                goal_tmp[0]):
                            intersection = intersection - 1

                    if intersection > maxIntersection:
                        maxIntersection = intersection
            for a in range(len(obstacle)):
                if poly[a].encloses_point(pt_obstacle[i][j]):
                    inside = True
                    break
            if maxIntersection == 1 and not inside:
                nom_dijk.append(j * len(obstacle) + i)
                distance.append(_math.dist(obstacle[i][j], obstacle[i][0]))
                start_dijk.append(j * len(obstacle) + i)
                goal_dijk.append(i)
                distance.append(_math.dist(obstacle[i][0], obstacle[i][j]))
                start_dijk.append(i)
                goal_dijk.append(j * len(obstacle) + i)


for i in range(len(obstacle)): # parcours tous les obstacles
    # print(obstacle[i])
    for j in range(len(obstacle[i])): #parcours tous les angles
        # print(obstacle[i][j])
        for k in range(len(obstacle)-i-1): #parcours tous les obstacles plus loin dans la liste
            # print(obstacle[i])
            for l in range(len(obstacle[k+i+1])): #parcours tous les angles

                start_tmp, goal_tmp = map(Point, [obstacle[i][j], obstacle[k+i+1][l]])
                chemin = Line(start_tmp, goal_tmp)

                maxIntersection = 0
                intersection = 0
                inside = False

                for m in range(len(obstacle)):
                    isIntersection = poly[m].intersection(chemin)

                    intersection = len(isIntersection)

                    for n in range(len(isIntersection)):
                        if (start_tmp[0] > isIntersection[n][0] < goal_tmp[0] or start_tmp[0] < isIntersection[n][0] >
                             goal_tmp[0]):
                            intersection = intersection - 1

                    if intersection > maxIntersection:
                        maxIntersection = intersection

                for a in range(len(obstacle)):
                    if poly[a].encloses_point(start_tmp) or poly[a].encloses_point(goal_tmp):
                        inside = True

                if maxIntersection == 1 and (not inside):
                    start_chemin.append(obstacle[i][j])
                    goal_chemin.append(obstacle[k + i + 1][l])

                    distance.append(_math.dist(obstacle[i][j], obstacle[k + i + 1][l]))
                    start_dijk.append(j*len(obstacle)+i)
                    goal_dijk.append(l*len(obstacle)+(k + i + 1))
                    distance.append(_math.dist(obstacle[i][j], obstacle[k + i + 1][l]))
                    start_dijk.append(l*len(obstacle)+(k + i + 1))
                    goal_dijk.append(j*len(obstacle)+i)


        # lien obstacles robot

        start_tmp, goal_tmp = map(Point, [obstacle[i][j], robot_start])
        chemin = Line(start_tmp, goal_tmp)

        maxIntersection = 0
        intersection = 0
        inside = False

        for m in range(len(obstacle)):
            isIntersection = poly[m].intersection(chemin)

            intersection = len(isIntersection)

            for n in range(len(isIntersection)):
                if (start_tmp[0] > isIntersection[n][0] < goal_tmp[0] or start_tmp[0] < isIntersection[n][0] >
                        goal_tmp[0]):
                    intersection = intersection - 1

            if intersection > maxIntersection:
                maxIntersection = intersection

            if poly[m].encloses_point(start_tmp) or poly[m].encloses_point(goal_tmp):
                inside = True

        if maxIntersection == 1 and (not inside):
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
        inside = False

        for m in range(len(obstacle)):
            isIntersection = poly[m].intersection(chemin)

            intersection = len(isIntersection)

            for n in range(len(isIntersection)):
                if (start_tmp[0] > isIntersection[n][0] < goal_tmp[0] or start_tmp[0] < isIntersection[n][0] >
                        goal_tmp[0]):
                    intersection = intersection - 1

            if intersection > maxIntersection:
                maxIntersection = intersection

            if poly[m].encloses_point(start_tmp) or poly[m].encloses_point(goal_tmp):
                inside = True

        if maxIntersection == 1 and (not inside):
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



            
# recherche du chemin


# graph = Graph()
# graph.add_edge(1, 2, 110)
# graph.add_edge(2, 3, 125)
# graph.add_edge(3, 4, 108)
# find_path(graph, 1, 4)

graph = Graph()

for i in range(len(distance)):
    graph.add_edge(start_dijk[i], goal_dijk[i], distance[i])

resultat = find_path(graph, 'S', 'G')

resultat_coor = []

for i in range(len(resultat.nodes)):
    if resultat.nodes[i] == 'S':
        resultat_coor.append(robot_start)
        print('resultats', resultat.nodes[i], resultat_coor[i])
    elif resultat.nodes[i] == 'G':
        resultat_coor.append(robot_goal)
        print('resultats',resultat.nodes[i],resultat_coor[i])
    else:
        reste = int(resultat.nodes[i] % len(obstacle))
        resultat_coor.append(obstacle[reste][int((resultat.nodes[i] - reste) / len(obstacle))])
        print('resultats',resultat.nodes[i],resultat_coor[i])


for i in range(len(contours_obstacles)):
    plt.imshow(cv2.line(image, contours_obstacles[i][0], contours_obstacles[i][1], (0, 255, 255), 5))

for i in range(len(goal_chemin)):
    plt.imshow(cv2.line(image, start_chemin[i], goal_chemin[i], (0, 100, 255), 5))

i = 1

while i in range(len(resultat_coor)):
    plt.imshow(cv2.line(image, resultat_coor[i-1], resultat_coor[i], (255, 0, 0), 2))
    i = i+1


# cv2.imshow('contours',image)
plt.show()

print('fin')