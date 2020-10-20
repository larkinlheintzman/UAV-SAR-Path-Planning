import rtree

class Tree(object):
    def __init__(self):

        props = rtree.index.Property()
        props.dimension = 3
        self.idx = rtree.index.Index(properties=props)
        self.vertices = set()
        self.E = {}
        self._p2c_ = {}
        self.num_nodes = 0

    def add_vertex(self, node):
        """
        Add vertex to corresponding tree
        :param v: tuple, vertex to add
        """
        p = tuple(node.pos)
        self.idx.insert(0, p + p, node)
        self.vertices.add( node )
        self.num_nodes += 1

    def new_vertex(self, p, t=0.0):
        """
        Create and node to corresponding tree
        :param v: tuple, vertex to add
        """
        node = Node(p, t)
        self.add_vertex( node )
        return node

    def add_edge(self, child, parent):
        """
        Add edge from parent to child
        :param child: tuple, child vertex
        :param parent: tuple, parent vertex
        """
        self.E[child] = parent
        
        self._p2c_.setdefault( parent, set() )
        self._p2c_[parent].add(child)

        child.set_parent(parent)
        parent.add_child(child)

    def rm_edge(self, child, parent):
        """
        Remove edge to child from parent
        :param child: tuple, child vertex
        :param parent: tuple, parent vertex
        """
        self.E.pop( child )
        self._p2c_[parent].remove(child)

        child.rm_parent()
        parent.rm_child(child)

    def nearby(self, x, n):
        """
        Return nearby vertices
        :param x: tuple, vertex around which searching
        :param n: int, max number of neighbors to return
        :return: list of nearby vertices
        """
        return list( self.idx.nearest(x, num_results=n, objects="raw") )

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
        cx, cy, cz = tuple(node_pos)
        half_s = float(cu_side) / 2

        # boundaries
        left = cx - half_s
        right = cx + half_s
        bottom = cy - half_s
        top = cy + half_s
        back = cz - half_s
        front = cz + half_s

        return list( self.idx.intersection((left, bottom, back, right, top, front), objects="raw") )


    # def rewire(self, robot, x, L):
    #     x.cost = robot.path_cost(x)

    #     for dist, nbor in L:
    #         if nbor.cost == float('inf'):
    #             nbor.cost = robot.path_cost(nbor)

    #         if nbor.cost > x.cost + dist:
    #             if robot.check_edge_between( x, nbor ) == True:
    #                 self.rm_edge( nbor, self.E[nbor] )
    #                 self.add_edge( nbor, x )

class Node(object):
    def __init__(self, pos, t=0.0, cost=float('inf'), parent=None, children=set() ):
        """
        Init a node
        :param pos: np.array(3), xyz coordinates
        :param t: float, time since t0
        """
        self.pos = pos
        self.x, self.y, self.z = tuple(self.pos)
        self.t = t
        self.hash_tuple = tuple(pos) + tuple([self.t])

        self.parent = parent
        self.children = children
        self.cost = cost

    def __hash__(self):
        return hash(self.hash_tuple)

    def __eq__(self, other):
        return self.hash_tuple == other.hash_tuple

    def __ne__(self, other):
        return not(self==other)

    def set_parent(self, parent):
        self.parent = parent
        parent.children.add(self)

    def rm_parent(self):
        if self.parent is not None:
            self.parent.children.remove(self)
        self.parent = None

    def add_child(self, child):
        self.children.add(child)
        child.parent = self

    def rm_child(self, child):
        if child in self.children:
            self.children.remove(child)
            child.parent = None
        else:
            raise EnvironmentError