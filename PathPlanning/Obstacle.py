"""
October 27, 2020
Graph Class
"""
import SpatialGraph
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import numpy as np
import matplotlib.pyplot as plt


class Obstacle(SpatialGraph.Graph):
    __slots__ = '_convex', '_delO', '_boundaryVertices', '_centroid'

    def __init__(self, point_list, convex=True):
        super().__init__(directed=False)

        self._convex = convex

        if convex:
            hull = ConvexHull(np.array(point_list), incremental=False)
            self._delO = {'N': hull.equations[:, 0:2], 'b': hull.equations[:, 2]}
            self._boundaryVertices = hull.vertices

            k = len(hull.vertices)
            p_sum = np.matmul(np.ones(k), hull.points[hull.vertices])
            self._centroid = np.multiply(1 / k, p_sum)
            self._centroid = [self._centroid[0], self._centroid[1]]

            for point in point_list:
                super().insert_vertex(point[0], point[1])

            facet_indices = hull.simplices
            i_1 = 0
            for e in facet_indices:
                super().insert_edge(super().get_vertex(e[0]),
                                    super().get_vertex(e[1]),
                                    {'N': hull.equations[i_1, 0:2], 'b': hull.equations[i_1, 2]})
                i_1 = i_1 + 1


        else:
            self._delO = None

    def is_convex(self):
        return self._convex

    def boundary(self):
        return self._delO

    def insert_vertex(self, x, y, element=None):
        print('\n ERROR: cannot insert vertex for now. \n')

    def insert_edge(self, v_1, v_2, x=None):
        print('\n ERROR: cannot insert edges. \n')

    def boundary_vertices(self):
        list_convex_vertices = []
        for k in self._boundaryVertices:
            list_convex_vertices.append(self._vertices[k])
        return list_convex_vertices

    def centroid(self):
        return self._centroid










