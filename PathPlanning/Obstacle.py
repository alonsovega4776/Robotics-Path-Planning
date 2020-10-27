"""
Alonso Vega
October 27, 2020
Graph Class
"""
import SpatialGraph


class Obstacle(SpatialGraph.Graph):
    ID_count = 0
    __slots__ = '_id', '_convex'

    def __init__(self, vertex_list_1, vertex_list_2=[]):
        self.ID_count = self.ID_count + 1
        self._id = self.ID_count

        if len(vertex_list_2 == 0):
            self._convex = False
        else:
            self._convex = True





