"""
Alonso Vega
October 21, 2020
Graph Class
"""


# ------------------------------------------------Vertex----------------------------------------------------------------
class Vertex:
    __slots__ = '_element', '_id'

    def __init__(self, x, n):
        self._element = x
        self._id = n

    def element(self):
        return self._element

    def id(self):
        return self._id

    def __hash__(self):
        return hash(id(self))
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

    def get_edge(self, v_1, v_2):
        i_1 = v_1.id()
        return self._I_plus_list[i_1][v_2]

    def degree(self, v, direction='+'):
        incident_map_list = self._I_plus_list if (direction == '+') else self._I_minus_list
        return len(incident_map_list[v])

    def incident_edges(self, v, direction='+'):
        incident_map_list = self._I_plus_list if (direction == '+') else self._I_minus_list
        edge_list = []

        for e_i in incident_map_list[v].values():
            edge_list.append(e_i)

        return edge_list

    def insert_vertex(self, x=None):
        next_id = len(self._vertices)

        v = self.Vertex(x, next_id)
        self._vertices.append(v)

        self._I_minus_list.append({})
        if self.is_directed():
            self._I_plus_list.append({})
        else:
            self._I_plus_list = self._I_minus_list

    def insert_edge(self, v_1, v_2, x=None):
        e = self.Edge(v_1, v_2, x)
        self._incidenceFunction[(v_1, v_2)] = e

        i_1 = v_1.id()
        i_2 = v_2.id()

        self._I_plus_list[i_1][v_2] = e
        self._I_minus_list[i_2][v_1] = e
# ------------------------------------------------Graph-----------------------------------------------------------------
