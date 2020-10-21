"""
Alonso Vega
October 21, 2020
Graph Class
"""


class Vertex:
    __slots__ = '_element'

    def __init__(self, x):
        self._element = x

    def element(self):
        return self._element

    def __hash__(self):
        return hash(id(self))


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


class Graph:
    __slots__ = '_vertices', '_edges', '_outgoing', '_incoming'

    def __init__(self, directed=False):
        self._vertices = []
        self._edges = {}

        self._outgoing = {}
        self._incoming = {} if directed else self._outgoing

    def is_directed(self):
        return self._incoming is not self._outgoing

    def num_vertices(self):
        return len(self._vertices)

    def vertices(self):
        return self._vertices

    def num_edges(self):
        return len(self._edges)

    def edges(self):
        return self._edges

    def get_edge(self, v_1, v_2):
        return self._edges[(v_1, v_2)]

    def degree(self, v, direction='+'):
        incident_map = self._outgoing if (direction == '+') else self._incoming
        return len(incident_map[v])

    def incident_edges(self, v, direction='+'):
        incident_map = self._outgoing if (direction == '+') else self._incoming
        edge_list = []
        for e_i in incident_map[v].values():
            edge_list.append(e_i)

        return edge_list

    def insert_vertex(self, x=None):
        v = self.Vertex(x)
        self._vertices.append(v)

        self._outgoing[v] = {}
        if self.is_directed():
            self._incoming[v] = {}

    def insert_edge(self, v_1, v_2, x=None):
        e = self.Edge(v_1, v_2, x)
        self._edges[(v_1, v_2)] = e

        self._outgoing[v_1][v_2] = e
        self._incoming[v_2][v_1] = e

