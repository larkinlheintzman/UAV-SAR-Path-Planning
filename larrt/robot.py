import sys, os, time, random

# up1 = os.path.abspath('..')
# if up1 not in sys.path:
#     sys.path.insert(0, up1)

import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from operator import itemgetter

from mrmh_model.params import Default

from . import searchspace
from . import tree_boost

from tqdm import tqdm
import pdb


class Robot(object):

    max_steer_dist = 20 # should vary this dep on env size
    steer_line_step = 2 #1
    steer_line_search = tuple( np.arange(steer_line_step, max_steer_dist+steer_line_step, steer_line_step) )
    max_goal_connect_dist = max_steer_dist * 3
    
    space = None
    planner = None
    plot_fig = None
    plot_ax_3d = None

    num_cost_queries = 0
    queried_points = set()

    __all_robots__ = []
    num_robots = 0
    colors = 'bgcrmyk'

    _first_init = True
    def __init__(self, init_pos=[0.0, 0.0, 0.0], goal_pos=None):
        super(Robot).__init__

        self.init = tree_boost.Node( np.array( init_pos, dtype=float ), cost=0 )
        self.goal = tree_boost.Node( np.array( goal_pos, dtype=float ), cost=float('inf') )
        self.root = self.init
        self.path = []
        self.path_nodes = set()
        self._initpath = []
        self._finpath = []
        self.tree = tree_boost.Tree()
        self.tree.robot = self

        self.found_goal = False
        self.run_done = False

        self.goal_has_nbors = False
        self.goal_free_nbors = set()
        self.goal_obs_nbors = set()
        self.goal_nbors_ancestors = set()
        self.goal_nbors_cousins = set()

        self.add_vertex(self.init)

        self.index = Robot.num_robots
        self.pbar = None

        if self not in Robot.__all_robots__:
            Robot.num_robots += 1
            Robot.__all_robots__.append(self)

        Robot.max_steer_dist = self.planner.rewire_radius / 5
        Robot.max_goal_connect_dist = Robot.max_steer_dist * 2

        if Robot._first_init:
            Robot._first_init = False
            print('\nRobot :: Max steer: {}\tMax goal connect: {}'.format(Robot.max_steer_dist, Robot.max_goal_connect_dist))

        # Robot.plot_fig = self.planner.space.terrain.paths_plot.number

    def planning_done(self):
        # if self.found_goal:
        #     return True
        # # elif self.tree.E.get( self.goal, None ) is not None:
        # elif self.goal.parent is not None:
        #     self.found_goal = True
        #     self.find_path()
        # else:

        return self.found_goal and (self.num_nodes() >= self.planner.max_nodes)

    def sort_by_dist(self, nodes, x_pos):
        L = [ (self.space.dist(tnode.pos, x_pos), tnode) for tnode in nodes ]
        L.sort(key=itemgetter(0))
        return L

    def sort_by_total_cost(self, nodes, x_pos):
        L = [ (self.path_cost(tnode) + self.space.dist(tnode.pos, x_pos), tnode) for tnode in nodes ]
        L.sort(key=itemgetter(0))
        return L

    def sort_by_astar_cost(self, nodes, x_pos, goal_pos=None):
        if goal_pos is None: goal_pos = self.goal.pos
        L = [ (self.path_cost(tnode) + self.space.dist(tnode.pos, x_pos) + self.space.dist(x_pos, goal_pos), tnode) for tnode in nodes ]
        L.sort(key=itemgetter(0))
        return L

    def eval_node(self, node):
        return self.path_cost(node) / (1+ self.other_cost_factor(node) )

    def other_cost_factor(self, node):
        return 0

    def steer(self, x0, x1):
        x0 = np.array(x0)
        x1 = np.array(x1)

        vec = x1 - x0
        uvec = vec / Robot.space.dist(x0, x1)

        x_new = None
        for tstep in Robot.steer_line_search:
            x_temp = x0 + uvec * tstep
            if Robot.space.obstacle_free( x_temp ):
                x_new = x_temp
            else:
                break
        return x_new

    def check_edge_between(self, x0, x1):
        if self.edge_exists_between(x0, x1):
            return False

        dist = self.space.dist(x0.pos, x1.pos)

        vec = x1.pos - x0.pos
        step_vec = vec / dist * Robot.steer_line_step

        checked_dist = 0
        temp = x0.pos
        while checked_dist < dist + Robot.steer_line_step:
            if not self.space.obstacle_free(temp):
                return False

            temp += step_vec
            checked_dist += Robot.steer_line_step            
        return True

    def rewire(self, x, L, force_recompute=False):
        any_rewired = False
        x.cost = self.path_cost(x)
        x.need_recalc = False

        for dist, nbor in L:
            if nbor == x:
                continue
            if nbor.cost == float('inf') or force_recompute:
                nbor.cost = self.path_cost(nbor)
                nbor.need_recalc = False

            if nbor.cost > x.cost + dist:
                if self.check_edge_between( x, nbor ) == True:
                    if nbor.parent is None:
                        pdb.set_trace()
                    self.rm_edge( nbor, self.tree.E[nbor] )
                    # self.rm_edge( nbor, nbor.parent )
                    self.add_edge( nbor, x )
                    self.tree.mark_cost_change(nbor)
                    # nbor.need_recalc = True
                    any_rewired = True
        return any_rewired

    def can_connect_to_goal(self):
        if self.tree.E.get(self.goal, None) is not None:
        # if self.goal.parent is not None:
            return True
        x_near = self.nearest(self.goal.pos)
        goal_dist = Robot.space.dist(self.goal.pos, x_near.pos)
        
        if goal_dist > Robot.max_goal_connect_dist:
            return False
        
        vec = self.goal.pos - x_near.pos
        uvec = vec / goal_dist

        goal_line_search = tuple(   np.arange(Robot.steer_line_step, \
                                    goal_dist+Robot.steer_line_step, \
                                    Robot.steer_line_step) \
                                )
        temp = x_near.pos
        for tstep in goal_line_search:
            temp = x_near.pos + tstep * uvec
            if not Robot.space.obstacle_free(temp):
                return False

        self.add_edge( self.goal, x_near )
        self.found_goal = True
        return True

    def new_can_connect_to_goal(self):
        if self.tree.E.get(self.goal, None) is not None:
        # if self.goal.parent is not None:
            self.found_goal = True
            return True

        n = min(Robot.planner.rewire_nbors, self.tree.num_nodes)
        nodes_nearby = self.nearby( self.goal.pos, n )
        L_near = self.sort_by_dist(nodes_nearby, self.goal.pos)
        
        for dist, x_near in L_near:
            if dist > self.max_goal_connect_dist:
                continue

            if self.check_edge_between(x_near, self.goal):
                self.add_edge( self.goal, x_near )
                self.found_goal = True
                self.find_path()
                print('R{}: Connected to goal using {} nodes'.format( self.index, self.tree.num_nodes ))
                return True
        return False

    def check_goal_has_nbors_in_cube(self, N=10, cu_side=5):
        if self.goal_has_nbors:
            return self.goal_has_nbors
        
        nbors = set(self.nbors_in_cube(self.goal.pos, cu_side)) | self.goal_free_nbors
        
        for t_nbor in nbors:
            if not self.check_edge_between(t_nbor, self.goal):
                self.goal_obs_nbors.add(t_nbor)

        nbors.difference_update(self.goal_obs_nbors)

        nbors_ancestors = set()
        nbors_ancestors |= self.goal_nbors_ancestors
        for t_nbor in nbors:
            nbors_ancestors.update( self.tree.get_ancestors(t_nbor) )
        nbors.difference_update(nbors_ancestors)

        self.goal_nbors_ancestors |= nbors_ancestors
        self.goal_free_nbors |= nbors

        if len(nbors) >= N:
            self.goal_has_nbors = True

        # if random.random() < 0.01:
        # print('Robot #{} -- nbors: {} -- {}'.format(self.index, len(nbors), [t.pos for t in nbors]))

        return self.goal_has_nbors

    def check_goal_has_nbors(self):
        return self.goal_has_nbors and (self.num_nodes() >= self.planner.max_nodes)

    def path_cost(self, _x_leaf=None, _x_init=None, force_recompute=False, recur_count=0):
        x_init = self.root if _x_init is None else _x_init
        x_leaf = self.goal if _x_leaf is None else _x_leaf

        if recur_count > 50:
            print('Recursion depth > 50')
            pdb.set_trace()

        cost = 0
        if x_leaf == x_init:
            cost = 0

        elif self.tree.E.get(x_leaf, None) is None:
            print('Path cost from {} to {} :: x_leaf has no parent!'.format(x_init.pos, x_leaf.pos))
            pdb.set_trace()

        elif self.tree.E[x_leaf] == x_init:
            cost = self.space.dist(x_leaf.pos, self.tree.E[x_leaf].pos)

        elif x_leaf.cost is not float('inf') and x_init.cost is not float('inf') and not force_recompute and not self.tree.has_cost_changed(x_leaf):
            cost = x_leaf.cost - x_init.cost

        else:
            if _x_init is not None:
                x_init.cost = self.path_cost( self.tree.E[x_init], force_recompute=force_recompute, recur_count=recur_count+1 ) + \
                            self.space.dist(x_init.pos, self.tree.E[x_init].pos)
                x_init.need_recalc = False
            x_leaf.cost = self.path_cost( self.tree.E[x_leaf], force_recompute=force_recompute, recur_count=recur_count+1 ) + \
                            self.space.dist(x_leaf.pos, self.tree.E[x_leaf].pos)
            x_leaf.need_recalc = False

            cost = x_leaf.cost - x_init.cost

        # if recur_count == 0 and x_init == self.root:
        #     if x_leaf not in Robot.queried_points:
        #         Robot.queried_points.add(x_leaf)
        #         Robot.num_cost_queries += 1

        return cost

    def prune(self, num_prune=700):
        leaves = self.tree.vertices - self.tree._p2c_.keys() - self.path_nodes
        num_leaves = len(leaves)
        if num_leaves > num_prune:
            L_leaves = self.sort_by_astar_cost(leaves, self.goal.pos)
            for _, node in L_leaves[ num_prune: ]:
                self.rm_vertex( node, return_children=False )
        
        if bool(self.orphans):
            print('Orphans @prune: {}'.format([t.pos for t in self.orphans]))
            pdb.set_trace()

        leaves = self.tree.vertices - self.tree._p2c_.keys() - self.path_nodes

    def find_path(self, root=None):
        root = self.root if root is None else root
        if self.tree.E.get(self.goal, None) is None:
            return []

        self.path = []
        temp = self.goal
        while temp != root:
            self.path.append( temp )
            temp = self.tree.E[temp]
            # temp = temp.parent
        self.path.append( root )
        self.path.reverse()
        self.path_nodes = set(self.path)
        return self.path

    def get_path_to_root(self, node):
        path = []
        temp = node
        while temp is not None and temp != self.root:
            path.append(temp)
            temp = self.tree.E.get(temp, None)

        if temp is not None:
            path.append( self.root )
            path.reverse()
            return path
        else:
            print('No route to root!')
            pdb.set_trace()
            return []

    def get_robot_path_array(self, leaf=None):
        if self.found_goal:
            if not self.path:
                tpath = self.find_path()
            tpath = [tnode.pos for tnode in tpath]
            return np.stack(tpath, axis=0)
        elif leaf is not None:
            tpath = self.get_path_to_root(leaf)
            tpath = [tnode.pos for tnode in tpath]
            return np.stack(tpath, axis=0)
        else:
            return None

    def connect_to_nborhood(self, node, sort_fn, num_nbors=15, orphans_to_avoid=None):

        num_nbors = int( min(num_nbors, self.tree.num_nodes) )

        nodes_nearby = set(self.nearby( node.pos, num_nbors ))
        nodes_nearby.difference_update( orphans_to_avoid )
        nodes_nearby.intersection_update(self.tree.vertices)
        # nodes_nearby = list(nodes_nearby)

        L_near = sort_fn(nodes_nearby, node.pos)

        for _, x_near in L_near:
            if self.check_edge_between(node, x_near):
                self.add_edge(node, x_near)
                return True
        return False

    def update_root(self, newroot=None):
        if len(self.path)<2:
            self._finpath += [ t.pos for t in self.path ]
            return None

        oldroot = self.path.pop(0)
        self._finpath.append(oldroot.pos)
        if newroot is None:
            newroot = self.path[0]
        self.root = newroot

        orphans = self.orphans
        orphans.discard(oldroot)
        if bool(orphans):
            print('Orphans BEFORE update_root: {}'.format([t.pos for t in orphans]))

        orphaned_children = self.rm_vertex(oldroot, return_children=True)
        if newroot in orphaned_children:
            orphaned_children.discard(newroot)

        orphaned_descendants = dict()
        for tnode in orphaned_children:
            orphaned_descendants[tnode] = self.tree.get_descendants(tnode)
            orphaned_descendants[tnode].add(tnode)

        while bool(orphaned_children):
            orphans_to_avoid = set()
            for ttchild in orphaned_children:
                orphans_to_avoid |= orphaned_descendants[ttchild]

            tnode = orphaned_children.pop()
            rewired = self.connect_to_nborhood(tnode, self.sort_by_dist, Robot.planner.rewire_nbors, orphans_to_avoid)
            if rewired:
                # tnode.need_recalc = True
                self.tree.mark_cost_change(tnode)
            else:
                self.rm_vertex(tnode)

        if bool(self.orphans):
            print('Orphans @update_root: {}'.format([t.pos for t in self.orphans]))
            pdb.set_trace()

        newroot.cost = 0
        newroot.need_recalc = False

        return newroot

    def add_vertex(self, node):
        self.tree.add_vertex(node)

    def rm_vertex(self, node, return_children=False):
        return self.tree.rm_vertex(node, return_children)

    def new_vertex(self, p, t=0.0):
        return self.tree.new_vertex( p, t )

    def add_edge(self, child, parent):
        self.tree.add_edge( child, parent )

    def rm_edge(self, child, parent):
        self.tree.rm_edge( child, parent )

    def nearby(self, x, n):
        return self.tree.nearby( x, n )

    def nearest(self, x):
        return self.tree.nearest( x )

    def edge_exists_between(self, nodeA, nodeB):
        return self.tree.edge_exists_between( nodeA, nodeB )

    def nbors_in_cube(self, node_pos, cu_side):
        return self.tree.nbors_in_cube( node_pos, cu_side )

    def num_nodes(self):
        return self.tree.num_nodes

    @property
    def orphans(self):
        orphans = self.tree.vertices.difference( self.tree.E.keys() )
        orphans.discard(self.root)
        return orphans

    @property
    def lsorphans(self):
        orphans = self.orphans
        return [ t.pos for t in orphans ]

    def draw_plan(self):
        if Robot.plot_fig is None:
            Robot.plot_fig = plt.figure().number
        else:
            plt.figure(Robot.plot_fig)

        # plt.plot(rnd[0], rnd[1], "^k")
        for c, p in self.tree.E.items():
            cx, cy, _ = tuple(c.pos)
            px, py, _ = tuple(p.pos)
            plt.plot( np.array([cx, px]), np.array([cy, py]), color=Robot.colors[self.index] )

        # for (ox, oy, size) in self.obstacleList:
        #     plt.plot(ox, oy, "ok", ms=30 * size)

        sx, sy, _ = tuple(self.init.pos)
        plt.plot(sx, sy, 'xm')

        gx, gy, _ = tuple(self.goal.pos)
        plt.plot(gx, gy, 'xr')

        # plt.axis([ Robot.space.xmin, Robot.space.xmax, Robot.space.ymin, Robot.space.ymax ])
        plt.grid(True)

        # plt.plot(rnd[0], rnd[1], "^k")

    def draw_path(self):
        if Robot.plot_fig is None:
            Robot.plot_fig = plt.figure().number
        else:
            plt.figure(Robot.plot_fig)
        
        if self.run_done:
            self.path = self._initpath
        elif self.found_goal:
            self.find_path()

        if len(self.path) > 0:
            self.path_np = np.vstack([ t.pos for t in self.path ])
            plt.plot( self.path_np[:,0], self.path_np[:,1], "-y")

            # for ix in range( len(self.path) -1 ):
            #     cx, cy, _ = tuple( self.path[ix].pos )
            #     px, py, _ = tuple( self.path[ix+1].pos )
            #     plt.plot( np.array([cx, px]), np.array([cy, py]), "-y")

        if len(self._finpath) > 0:
            self._finpath_np = np.vstack(self._finpath)
            plt.plot( self._finpath_np[:,0], self._finpath_np[:,1], "-r")
            # for ix in range( len(self._finpath) -1 ):
            #     cx, cy, _ = tuple( self._finpath[ix] )
            #     px, py, _ = tuple( self._finpath[ix+1] )
            #     print('({xs}, {ys})'.format( xs=np.array([cx, px]), ys=np.array([cy, py]) ))
            #     plt.plot( np.array([cx, px]), np.array([cy, py]), "-r")

    def draw_goal_nbor_paths(self, highlight=None):
        if Robot.plot_fig is None:
            Robot.plot_fig = plt.figure().number
        else:
            plt.figure(Robot.plot_fig)

        for tnode in self.goal_free_nbors:
            tpath_np = self.get_robot_path_array(leaf=tnode)
            plt.plot( tpath_np[:,0], tpath_np[:,1], "-y")

        if highlight is not None:
            tpath_np = self.get_robot_path_array(leaf=highlight)
            plt.plot( tpath_np[:,0], tpath_np[:,1], "-r")

    def draw_plan_3d(self):
        if Robot.plot_ax_3d is None:
            fig = plt.figure()
            Robot.plot_ax_3d = fig.gca(projection='3d')

        ax = Robot.plot_ax_3d

        # plt.plot(rnd[0], rnd[1], "^k")
        for c, p in self.tree.E.items():
            cx, cy, cz = tuple(c.pos)
            px, py, pz = tuple(p.pos)
            ax.plot( np.array([cx, px]), np.array([cy, py]), np.array([cz, pz]), color=Robot.colors[self.index], alpha=0.5 )

        # for (ox, oy, size) in self.obstacleList:
        #     plt.plot(ox, oy, "ok", ms=30 * size)

        sx, sy, sz = tuple(self.init.pos)
        ax.scatter(sx, sy, sz, c='magenta')

        gx, gy, gz = tuple(self.goal.pos)
        ax.scatter(gx, gy, gz, c='red')

        # plt.axis([ Robot.space.xmin, Robot.space.xmax, Robot.space.ymin, Robot.space.ymax ])
        # plt.plot(rnd[0], rnd[1], "^k")

    def print_stats(self):
        # print('R{}: nodes: ')
        pass

    def draw_path_3d(self):
        if Robot.plot_ax_3d is None:
            fig = plt.figure()
            Robot.plot_ax_3d = fig.gca(projection='3d')

        ax = Robot.plot_ax_3d

        if self.run_done:
            self.path = self._initpath
        elif self.found_goal:
            self.find_path()

        if len(self.path) > 0:
            self.path_np = np.vstack([ t.pos for t in self.path ])
            plt.plot( xs=self.path_np[:, 0], ys=self.path_np[:, 1], zs=self.path_np[:, 2], color="yellow")
            # for ix in range( len(self.path) -1 ):
            #     cx, cy, cz = tuple( self.path[ix].pos )
            #     px, py, pz = tuple( self.path[ix+1].pos )
            #     ax.plot( xs=np.array([cx, px]), ys=np.array([cy, py]), zs=np.array([cz, pz]), color="yellow" )

        if len(self._finpath) > 0:
            self._finpath_np = np.vstack(self._finpath)
            ax.plot( xs=self._finpath_np[:, 0], ys=self._finpath_np[:, 1], zs=self._finpath_np[:, 2], color="red")
            # for ix in range( len(self._finpath) -1 ):
            #     cx, cy, cz = tuple( self._finpath[ix] )
            #     px, py, cz = tuple( self._finpath[ix+1] )
            #     print('({xs}, {ys}, {zs})'.format( xs=np.array([cx, px]), ys=np.array([cy, py]), zs=np.array([cz, pz]) ))
            #     ax.plot( xs=np.array([cx, px]), ys=np.array([cy, py]), zs=np.array([cz, pz]), color="red" )

    @staticmethod
    def all_done_planning():
        return all([ trobot.planning_done() for trobot in Robot.get_all() ])

    @staticmethod
    def all_run_done():
        return all([ trobot.run_done for trobot in Robot.get_all() ])

    @staticmethod
    def get_paths_array(interp_res=10):
        _all_paths = []
        _all_path_len = []
        _all_fixed_points = []
        for trobot in Robot.__all_robots__:
            if trobot.found_goal:
                if not trobot.path:
                    trobot.find_path()
                _tpath = [tnode.pos for tnode in trobot.path]
                if bool(_tpath):
                    if interp_res > 0:
                        _tpath = interpolate_path(_tpath, interp_res=interp_res)
                    _all_paths += _tpath
                    _all_path_len.append(len(_tpath))

                    _all_fixed_points.append(_tpath[0])
                    _all_fixed_points.append(_tpath[-1])
        return np.asarray(np.stack(_all_fixed_points, axis=0), dtype=np.float64), np.asarray(np.stack(_all_paths, axis=0), dtype=np.float64), _all_path_len

    @staticmethod
    def get_all():
        return Robot.__all_robots__

    @staticmethod
    def make_sweep_paths(sweep_num = 8, interp_res = 10):

        _ymin = Robot.space.ymin
        _ymax = Robot.space.ymax
        _yrange = Robot.space._yrange

        _xmin = Robot.space.xmin
        _xmax = Robot.space.xmax
        _xrange = Robot.space._xrange

        _num = Robot.num_robots

        # parameters for sweep paths
        sx = np.int(_yrange/20) # bound from sides of environment (x axis)
        sy = _yrange/(sweep_num*2*_num)
        sw = _num*sy*2 # sweep width along y axis for drone team
        path_res = 10 # points along each sweep (along y axis)

        for ix, trobot in enumerate(Robot.__all_robots__):
            flip_flag = True
            sweep_x = np.array([]) # will need to come back and delete this row
            sweep_y = np.array([])
            sweep_z = np.array([])
            for sn in range(sweep_num):
                y_vals = np.linspace(_ymin + sn*sw + 2*ix*sy + sy,_ymin + sn*sw + 2*ix*sy + sy,path_res)
                if flip_flag:
                    x_vals = np.linspace(_xmin + sx, _xmax - sx,path_res)
                else:
                    x_vals = np.linspace(_xmax - sx, _xmin + sx,path_res)
                # query terrain at interp'd path points
                z_vals = Robot.space.terrain.h_smooth.ev(x_vals, y_vals) + Robot.space.params['sweep_height']
                sweep_y = np.concatenate([sweep_y,y_vals])
                sweep_x = np.concatenate([sweep_x,x_vals])
                sweep_z = np.concatenate([sweep_z,z_vals])
                flip_flag = not flip_flag
            sweep_guides = np.vstack( [sweep_x, sweep_y, sweep_z] ).T

            sweep_guides_list = np.split(sweep_guides, sweep_guides.shape[0], axis=0)
            sweep_guides_list = [ _tpos.reshape(-1) for _tpos in sweep_guides_list ]
            trobot.sweep_path = interpolate_path(sweep_guides_list, interp_res = interp_res)

    @staticmethod
    def make_rc_paths(searcher_paths, interp_res = 10):
        # generate paths for drones that are a set heigh above ground searchers
        interp_rc = True
        if len(searcher_paths) >= Robot.num_robots:
            # more humans, some humans go without
            for ix, trobot in enumerate(Robot.__all_robots__):
                trobot.sweep_path = searcher_paths[ix]
                if interp_rc:
                    trobot.sweep_path = interpolate_path(searcher_paths[ix], interp_res = interp_res)
                else:
                    trobot.sweep_path = searcher_paths[ix]
        else:
            # more robots, group up on some humans (entirely overthought)
            ratio = np.ceil(Robot.num_robots/len(searcher_paths)).astype(np.int)
            extras = np.mod(Robot.num_robots,ratio)
            groups = np.floor_divide(Robot.num_robots,ratio)
            r_counter = 0 # counter for robots that have been assigned
            s_counter = 0 # counter for searchers that have been assigned
            group_spacer = np.linspace(-30,30,groups*ratio) # distance grouped drones from searcher path
            extra_spacer = np.linspace(-30,30,extras) # distance drones from searcher path
            # sort out groups first
            for g in range(groups):
                for r in range(ratio):
                    # adjust x-y placement to avoid overlap
                    temp_spacer = np.array([group_spacer[r_counter], 0, 0])
                    robot_path = [srch_path + temp_spacer for srch_path in searcher_paths[s_counter]]
                    if interp_rc:
                        Robot.__all_robots__[r_counter].sweep_path = interpolate_path(robot_path,interp_res = interp_res)
                    else:
                        Robot.__all_robots__[r_counter].sweep_path = robot_path
                    r_counter = r_counter + 1
                s_counter = s_counter + 1

            # assign extras
            for e in range(extras):
                temp_spacer = np.array([extra_spacer[r_counter-(groups*ratio)], 0, 0])
                robot_path = [srch_path + temp_spacer for srch_path in searcher_paths[s_counter]]
                if interp_rc:
                    Robot.__all_robots__[r_counter].sweep_path = interpolate_path(robot_path,interp_res = interp_res)
                else:
                    Robot.__all_robots__[r_counter].sweep_path = robot_path
                r_counter = r_counter + 1
        # print("done making rc paths")


    @staticmethod
    def get_sweep_paths_array():
        _all_paths = []
        _all_path_len = []
        _all_fixed_points = []
        for trobot in Robot.__all_robots__:
            _all_paths += trobot.sweep_path
            _all_path_len.append(len(trobot.sweep_path))

            start_pt = trobot.sweep_path[0].copy()
            start_pt[-1] = Robot.space.terrain.h_smooth.ev( start_pt[0], start_pt[1] ) + 2
            _all_fixed_points.append(start_pt)

            end_pt = trobot.sweep_path[-1].copy()
            end_pt[-1] = Robot.space.terrain.h_smooth.ev( end_pt[0], end_pt[1] ) + 2
            _all_fixed_points.append(end_pt)

        return np.asarray(np.stack(_all_fixed_points, axis=0), dtype=np.float64), np.asarray(np.stack(_all_paths, axis=0), dtype=np.float64), _all_path_len

def interpolate_path(path, interp_res = 10):

    # assuming path comes in [array([1,2,3]),...] form
    x_pts = np.array([pt[0] for pt in path])
    y_pts = np.array([pt[1] for pt in path])
    z_pts = np.array([pt[2] for pt in path])
    # get total length of path
    # total_len = np.sum(np.sqrt(np.sum(np.diff(np.array([x_pts, y_pts, z_pts]).T, axis=0) ** 2, axis=1)))

    tck, u = interpolate.splprep([x_pts, y_pts, z_pts], s=0, k=1)  # parametric interpolation
    u_new = np.linspace(0, 1, interp_res)  # scaled discretization
    pts_interp = interpolate.splev(u_new, tck)

    # re format into path list
    ret_path = np.split(np.array(pts_interp).T,u_new.shape[0],axis=0)
    ret_path = [pt.reshape(-1) for pt in ret_path]
    return ret_path

def _dist(x0, x1):
    return np.linalg.norm( x1-x0, ord=2 )