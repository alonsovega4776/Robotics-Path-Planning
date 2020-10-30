"""
Alonso Vega
October 29, 2020
Environment Class
"""
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import collections as pltC


class Environment:
    __slots__ = '_xMin', '_xMax', '_yMin', '_yMax', '_obstacleList', '_goal', '_start'

    def __init__(self, X, Y, obstacle_list, start, goal):
        self._xMin = X[0]
        self._xMax = X[1]

        self._yMin = Y[0]
        self._yMax = Y[1]

        self._obstacleList = obstacle_list

        self._goal = goal
        self._start = start

    def x_max(self):
        return self._xMax

    def y_max(self):
        return self._yMax

    def x_min(self):
        return self._xMin

    def y_min(self):
        return self._yMin

    def goal(self):
        return self._goal

    def start(self):
        return self._start

    def obstacles(self):
        return self._obstacleList

    def print_environment(self):
        fig, axis = plt.subplots()

        points = []
        i_1 = 0
        for obstacle in self._obstacleList:
            O_points = []
            for v in obstacle.boundary_vertices():
                x = v.x_value()
                y = v.y_value()
                points.append([x, y])
                O_points.append([x, y])
                axis.annotate('v_' + f'{i_1}' + '' + f'{v.id()}', (x, y),
                              (0.65*x + 0.35*obstacle.centroid()[0], 0.65*y + 0.35*obstacle.centroid()[1]))
            O_points = np.array(O_points)
            plt.fill(O_points[:, 0], O_points[:, 1], color=(0, 0, 0, 0.15))
            i_1 = i_1 + 1
        points = np.array(points)
        axis.scatter(points[:, 0], points[:, 1], s=100.0, color=(1, 0, 0, 0.5))

        points = []
        i_1 = 0
        for obstacle in self._obstacleList:
            O_cent = obstacle.centroid()
            points.append(O_cent)
            axis.annotate('O_' + f'{i_1}', (O_cent[0], O_cent[1]))
            i_1 = i_1 + 1
        points = np.array(points)
        axis.scatter(points[:, 0], points[:, 1],
                     s=5.0, color=(0, 0, 0, 0.4), marker='h')

        points = []
        for obstacle in self._obstacleList:
            for e in obstacle.edges():
                (v_1, v_2) = e.end_vertices()
                points.append([(v_1.x_value(), v_1.y_value()), (v_2.x_value(), v_2.y_value())])

        segments = pltC.LineCollection(points, linewidths=0.75, colors=(1, 0, 0, 1))
        axis.add_collection(segments)


        plt.show()

    def collision(self, point):
        point = np.array(point)
        for obstacle in self._obstacleList:
            col_vect = (np.matmul(obstacle.boundary()['N'], point) + obstacle.boundary()['b'] <= 0)
            for collision in col_vect:
                if collision:
                    return True
        return False






