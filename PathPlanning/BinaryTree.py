"""
Alonso Vega
November 24, 2020
Binary Tree Class
"""
import SpatialGraph
import Utility as util


# ------------------------------------------------Vertex----------------------------------------------------------------
class KdTvertex():
    __slots__ = '_parent', '_left', '_right', '_id'

    def __init__(self, id_num, parent=None, left_node=None, right_node=None):
        self._id = id_num
        self._parent = parent
        self._left = left_node
        self._right = right_node

    def get_parent(self):
        return self._parent

    def right_child(self):
        return self._right

    def left_child(self):
        return self._left

    def insert_kid(self, v, kid):
        if kid == 'left':
            if self._left is not None:
                print('\nERROR: already have a left kid.\n')
                exit()
            self._left = v
        if kid == 'right':
            if self._right is not None:
                print('\nERROR: already have a right kid.\n')
                exit()
            self._right = v

    def __hash__(self):
        return hash(id(self))
# ------------------------------------------------Vertex----------------------------------------------------------------


class KdTree(SpatialGraph.Graph):
    __slots__ = '_root'

    def __init__(self):
        super().__init__(directed=False)
        self._root = self.insert_vertex(padre=None, side=None)

    def get_root(self):
        return self._root

    def insert_vertex(self, padre, side):
        next_id = len(super().vertices())

        v = KdTvertex(next_id, parent=padre)
        super().vertices().append(v)

        super().I_minus_list().append({})
        if super().is_directed():
            super().I_plus_list().append({})
        else:
            super().set_I_plus_list(super().I_minus_list())

        if padre != None:
            padre.insert_kid(v, side)
            super().insert_edge(padre, v)

        return v

    def is_leaf(self, v):
        return (v.right_child() is None) & (v.left_child() is None)
