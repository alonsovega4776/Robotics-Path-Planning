"""
Alonso Vega
November 14, 2020
Tree Class
"""
import SpatialGraph


# ------------------------------------------------Vertex----------------------------------------------------------------
class Tvertex(SpatialGraph.Vertex):
    __slots__ = '_parent', '_children'

    def __init__(self, x, y, element, id_num, parent=None):
        super().__init__(x=x, y=y, element=element, n=id_num)

        self._parent = parent
        self._children = []

    def get_parent(self):
        return self._parent

    def get_children(self):
        return self._children

    def insert_kid(self, v):
        self._children.append(v)
# ------------------------------------------------Vertex----------------------------------------------------------------


# -------------------------------------------------Tree-----------------------------------------------------------------
class Tree(SpatialGraph.Graph):
    __slots__ = '_root'

    def __init__(self, x_inital):
        super().__init__(directed=False)

        self._root = self.insert_vertex(x_inital[0], x_inital[1], padre=None, element=None)

    def get_root(self):
        return self._root

    def insert_vertex(self, x, y, padre, element=None):
        next_id = len(super()._vertices)

        v = Tvertex(x, y, element, next_id, parent=padre)
        super().vertices().append(v)

        super().I_minus_list().append({})
        if super().is_directed():
            super().I_plus_list().append({})
        else:
            super().set_I_plus_list(super().I_minus_list())

        if padre != None:
            padre.insert_kid(v)
            super().insert_edge(padre, v)

        return v
# -------------------------------------------------Tree-----------------------------------------------------------------
