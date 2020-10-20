import pdb
import numpy as np

import boostrtrees as bsrt


class Tree(object):
    trees = []
    knn_count = 0
    nbors_in_cube_count = 0
    add_to_index_count = 0
    rm_from_index_count = 0
    mark_cost_change_count = 0

    def __init__(self):
        self.idx = bsrt.RTree3D()
        self.vertices = set()

        self._add_vertices = []
        self._rm_vertices = []

        self.E = {}
        self._p2c_ = {}
        self.num_nodes = 0
        self.robot = None
        self.index = len(Tree.trees)
        Tree.trees.append(self)

        Node.treeClass = Tree
        Tree.nodeClass = Node

        self.del_nodes = set()

    def add_vertex(self, node):
        """
        Add vertex to corresponding tree
        :param v: tuple, vertex to add
        """
        self.vertices.add(node)
        self.add_to_index(node)
        self.num_nodes += 1

    def new_vertex(self, p, t=0.0):
        """
        Create and node to corresponding tree
        :param v: tuple, vertex to add 
        """
        node = Node(p, t, tree_ix=self.index)
        self.add_vertex(node)
        return node

    def add_edge(self, child, parent):
        """
        Add edge from parent to child
        :param child: tuple, child vertex
        :param parent: tuple, parent vertex
        """
        if self.edge_exists_between(child, parent):
            pdb.set_trace()

        self.E[child] = parent

        self._p2c_.setdefault(parent, set())
        self._p2c_[parent].add(child)

        # child.set_parent(parent)
        # parent.add_child(child)
        # child.all_parents.add(parent)

    def rm_edge(self, child, parent):
        """
        Remove edge to child from parent
        :param child: tuple, child vertex
        :param parent: tuple, parent vertex
        """
        self.E.pop(child)
        self._p2c_[parent].discard(child)

        child.need_recalc = True

    def rm_vertex(self, node, return_children=False):
        parent = self.E.pop(node, None)
        if parent:
            self._p2c_[parent].discard(node)
        # def remove_points(self, np.ndarray[double, ndim=2, mode="c"] points not None):

        children = None
        if not return_children:
            all_des = self.get_descendants(node)
            self.vertices.difference_update(all_des)
            for tdes in all_des:
                self.E.pop(tdes, None)
                self._p2c_.pop(tdes, None)

                self.rm_from_index(tdes)

                self.del_nodes.add(tdes)
                del tdes

            orphans = self.robot.orphans
            orphans.discard(node)
            # if bool(orphans):
            # print('Orphans @rm_vertex({}): {}'.format(node.pos, [t.pos for t in orphans]))

        else:
            children = self._p2c_.get(node, set()).copy()
            self._p2c_.pop(node, None)
            if children:
                for child in children:
                    # child.parent = None
                    self.E.pop(child)

        self.vertices.discard(node)

        self.rm_from_index(node)

        self.del_nodes.add(node)
        del node

        return children

    def get_descendants_old(self, node, des=set()):
        children = self.get_children(node)
        des = des | children
        for child in children:
            des |= self.get_descendants_old(child, des)
        return des

    def get_descendants(self, node):
        open_set = {node}
        closed_set = set()
        while bool(open_set):
            curr_node = open_set.pop()
            children = self._p2c_.get(curr_node, set())
            open_set |= children
            closed_set.add(curr_node)
        return closed_set

    def get_ancestors(self, node, anc=set()):
        parent = self.get_parent(node)
        if parent is not None:
            anc.add(parent)
            anc |= self.get_ancestors(parent, anc)
        return anc

    def get_children(self, node):
        return self._p2c_.get(node, set())

    def get_parent(self, node):
        return self.E.get(node, None)

    def print_nodes(self, nodeset):
        print([t.pos for t in nodeset])

    def nearby(self, x, n):
        """
        Return nearby vertices
        :param x: tuple, vertex around which searching
        :param n: int, max number of neighbors to return
        :return: list of nearby vertices
        """
        nearby_indices = set(self.idx.knn_np(x.reshape(-1, 3), n))
        # nearby_indices = set(self.idx.knn(x[0], x[1], x[2], n))
        nearby_nodes = {self._add_vertices[ix] for ix in nearby_indices}
        nearby_nodes.difference_update(self._rm_vertices)

        # print("nearby node list:")
        # print([node.pos for node in nearby_nodes])

        # ghosts = self.find_ghosts(nearby_nodes)
        # if bool(ghosts):
        #     print('Ghosts! -- {}'.format([t.pos for t in ghosts]))

        return nearby_nodes

    def nearest(self, x):
        """
        Return vertex nearest to x
        :param x: tuple, vertex around which searching
        :return: tuple, nearest vertex to x
        """
        return self.nearby(x, 1)[0]

    def nbors_in_cube(self, node_pos, cu_side):
        """
        return neighbors within a cube of side `cu_side`
        with `node_pos` at its center
        :param node_pos: numpy array(3), position in 3D
        :param cu_side: float, side of square
        """
        Tree.nbors_in_cube_count += 1
        corners = np.tile(node_pos.reshape(1, -1), (2, 1))
        half_s = float(cu_side) / 2

        corners[0, :] -= half_s
        corners[1, :] += half_s
        # boundaries
        # left = cx - half_s
        # bottom = cy - half_s
        # right = cx + half_s
        # top = cy + half_s
        # back = cz - half_s
        # front = cz + half_s
        nbors_indices = set(self.idx.intersection(corners))
        nbors_nodes = {self._add_vertices[ix] for ix in nbors_indices}

        # ghosts = self.find_ghosts(nbors_nodes)
        # if bool(ghosts):
        #     print('Ghosts! -- {}'.format([t.pos for t in ghosts]))

        return list(nbors_nodes)

    def has_cost_changed(self, node):
        return node.need_recalc
        # # If this node obviously needs to recompute its cost
        # if node.need_recalc:
        #     return True
        # elif self.E.get(node, None) is None:
        #     return False
        # else:
        #     ret_val = False
        #     temp_node = node
        #     while temp_node is not None:
        #         ret_val |= temp_node.need_recalc
        #         temp_node = self.E.get(temp_node, None)
        #     # node.need_recalc = self.has_cost_changed(self.E[node])
        #     node.need_recalc = ret_val
        #     return node.need_recalc

    def mark_cost_change(self, node):
        node.need_recalc = True
        all_des = self.get_descendants(node)
        for tdes in all_des:
            tdes.need_recalc = True

    def find_ghosts(self, nodelist):
        return [tnode for tnode in nodelist if tnode not in self.vertices]

    def add_to_index(self, arg_node):
        Tree.add_to_index_count += 1
        # print('insert_points: {} -- {}'.format(arg_node.pos.reshape(-1, 3), Tree.add_to_index_count))
        pt = np.array([np.append(arg_node.pos.reshape(-1,3),self.num_nodes)])
        # self.idx.insert_points(arg_node.pos.reshape(-1, 3))
        self.idx.insert_points(pt)
        self._add_vertices.append(arg_node)

    def rm_from_index(self, arg_node):
        Tree.rm_from_index_count += 1
        # print('remove_points: {} -- {}'.format(arg_node.pos.reshape(-1, 3), Tree.rm_from_index_count))
        self.idx.remove_points(arg_node.pos.reshape(-1, 3))
        self._rm_vertices.append(arg_node)

    def edge_exists_between(self, nodeA, nodeB):
        parentA = self.get_parent(nodeA)
        if parentA and nodeB == parentA:
            return True
        parentB = self.get_parent(nodeB)
        if parentB and nodeA == parentB:
            return True
        return False


class Node(object):
    nextNodeIdx = 0
    treeClass = None

    def __init__(self, pos, t=0.0, cost=float('inf'), tree_ix=0):
        """
        Init a node
        :param pos: np.array(3), xyz coordinates
        :param t: float, time since t0
        """
        self.index = Node.nextNodeIdx
        self.pos = pos
        self.x, self.y, self.z = tuple(self.pos)
        self.t = t

        # self.parent = parent
        # self.children = children
        self.cost = cost
        self.need_recalc = True
        self.tree_ix = tree_ix
        self.hash_tuple = self.index

        Node.nextNodeIdx += 1
        # debug
        # self.all_parents = set()

    def __hash__(self):
        return hash(self.hash_tuple)

    def __eq__(self, other):
        return self.hash_tuple == other.hash_tuple

    def __ne__(self, other):
        return not (self == other)

    def __str__(self):
        return '\n\t#{} @{} |=>{}'.format(self.tree_ix, self.pos, self.lsp.pos if self.lsp is not None else 'None')

    @property
    def parent(self):
        return Node.treeClass.trees[self.tree_ix].E.get(self, None)

    @property
    def children(self):
        return Node.treeClass.trees[self.tree_ix]._p2c_.get(self, set())

    def get_parent(self):
        return Node.treeClass.trees[self.tree_ix].E.get(self, None)

    def get_children(self):
        return Node.treeClass.trees[self.tree_ix]._p2c_.get(self, set())

    def set_parent(self, parent):
        Node.treeClass.trees[self.tree_ix].E[self] = parent

    def rm_parent(self):
        parent = self.get_parent()
        if parent:
            Node.treeClass.trees[self.tree_ix].E.pop(self)
            Node.treeClass.trees[self.tree_ix]._p2c_[parent].discard(self)
        else:
            raise EnvironmentError

    def add_child(self, child):
        child.set_parent(self)
        Node.treeClass.trees[self.tree_ix]._p2c_[self].setdefault(set())
        Node.treeClass.trees[self.tree_ix]._p2c_[self].add(child)

    def rm_child(self, child):
        self.get_children().discard(child)

    @property
    def lsp(self):
        return Node.treeClass.trees[self.tree_ix].E.get(self, None)

    @property
    def lsc(self):
        return list(Node.treeClass.trees[self.tree_ix]._p2c_.get(self, set()))

    @property
    def prp(self):
        parent = Node.treeClass.trees[self.tree_ix].E.get(self, None)
        return 'None' if parent is None else parent.pos

    @property
    def prc(self):
        return [t.pos for t in Node.treeClass.trees[self.tree_ix]._p2c_.get(self, set())]


class mySet(set):
    def __init__(self, *args):
        super(mySet, self).__init__(*args)

    def discard(self, *args):
        print(*args)
        pdb.set_trace()
        super(mySet, self).discard(*args)
