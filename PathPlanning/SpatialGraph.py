"""
Alonso Vega
October 21, 2020
Graph Class
"""
import numpy as np


# ------------------------------------------------Vertex----------------------------------------------------------------
class Vertex:
    __slots__ = '_element', '_id', '_x', '_y'

    def __init__(self, x, y, element, n):
        self._x = x
        self._y = y
        self._element = element
        self._id = n

    def element(self):
        return self._element

    def id(self):
        return self._id

    def x_value(self):
        return self._x

    def __hash__(self):
        return hash(id(self))

    def y_value(self):
        return self._y
# ------------------------------------------------Vertex----------------------------------------------------------------


# --------------------------------------------------Edge----------------------------------------------------------------
class Edge:
    __slots__ = '_origin', '_destination', '_element'

    def __init__(self, v_1, v_2, e):
        self._origin = v_1
        self._destination = v_2
        self._element = e

    def end_vertices(self):
        return self._origin, self._destination

    def adjacent(self, v):
        return self._destination if v is self._origin else self._origin

    def element(self):
        return self._element

    def __hash__(self):
        return hash((self._origin, self._destination))
# --------------------------------------------------Edge----------------------------------------------------------------


# ------------------------------------------------Graph-----------------------------------------------------------------
class Graph:
    __slots__ = '_vertices', '_edges', '_incidenceFunction',\
                '_I_plus_list', '_I_minus_list',\
                '_directed'

    def __init__(self, directed=False):
        self._directed = directed

        self._vertices = []
        self._edges = []
        self._incidenceFunction = {}

        self._I_plus_list = []
        self._I_minus_list = [] if directed else self._I_plus_list

    def is_directed(self):
        return self._directed

    def num_vertices(self):
        return len(self._vertices)

    def vertices(self):
        return self._vertices

    def num_edges(self):
        return len(self._edges)

    def edges(self):
        return self._edges

    def get_vertex(self, n):
        return self._vertices[n]

    def get_edge(self, v_1, v_2):
        i_1 = v_1.id()
        return self._I_plus_list[i_1][v_2]

    def degree(self, v, direction='+'):
        incident_map_list = self._I_plus_list if (direction == '+') else self._I_minus_list
        return len(incident_map_list[v.id()])

    def incident_edges(self, v, direction='+'):
        incident_map_list = self._I_plus_list if (direction == '+') else self._I_minus_list
        edge_list = []

        for e_i in incident_map_list[v.id()].values():
            edge_list.append(e_i)

        return edge_list

    def insert_vertex(self, x, y, element=None):
        next_id = len(self._vertices)

        v = Vertex(x, y, element, next_id)
        self._vertices.append(v)

        self._I_minus_list.append({})
        if self.is_directed():
            self._I_plus_list.append({})
        else:
            self._I_plus_list = self._I_minus_list

        return v

    def insert_edge(self, v_1, v_2, x=None):
        e = Edge(v_1, v_2, x)
        self._incidenceFunction[(v_1, v_2)] = e
        self._edges.append(e)

        i_1 = v_1.id()
        i_2 = v_2.id()

        self._I_plus_list[i_1][v_2] = e
        self._I_minus_list[i_2][v_1] = e

        return e

    def distance(self, v_1, v_2, norm_order=2):
        p_1 = [v_1.x_value(), v_1.y_value()]
        p_2 = [v_2.x_value(), v_2.y_value()]

        diff = np.subtract(p_1, p_2)
        return np.linalg.norm(diff, ord=norm_order)
# ------------------------------------------------Graph-----------------------------------------------------------------
