"""
Alonso Vega
October 27, 2020
Graph Class
"""
import SpatialGraph
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import numpy as np
import matplotlib.pyplot as plt


class Obstacle(SpatialGraph.Graph):
    __slots__ = '_convex', '_delO'

    def __init__(self, point_list, convex=True):
        SpatialGraph.Graph.__init__(self, directed=False)
        super().__init__(directed=False)

        self._convex = convex

        if convex:
            hull = ConvexHull(np.array(point_list), incremental=False)
            self._delO = {'N': hull.equations[:, 0:2], 'b': hull.equations[:, 2]}
            vertex_indices = hull.points[hull.vertices]
            facet_indices = hull.simplices

            for point in vertex_indices:
                super().insert_vertex(point[0], point[1])

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








