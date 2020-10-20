import sys, os, time
import numpy as np
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mrmh_model import params
from larrt.robot import interpolate_path # upgraded interpolator
from . import searchspace
from . import robot
from . import tree_boost
from tqdm import tqdm
import pdb
import itertools

#RRT modes
rrtNAIVE=0
rrtTOTALDIST=1
rrtSTAR=2

planner_modes = { 'NAIVE':rrtNAIVE, 'TOTALDIST':rrtTOTALDIST, 'RRTSTAR':rrtSTAR }

class Planning(params.Default):
    prc = 0.01
    rewire_radius = 45 #75
    rewire_nbors = 5 #20
    replan_horizon = 4
    replan_count = 0

    def __init__(self, params, on_terrain=None, mode='NAIVE'):
        super().__init__(params)
        self.space = searchspace.SearchSpace(params, on_terrain)
        self.max_nodes = self.params.get('max_plan_nodes', 2500)
        robot.Robot.space = self.space
        robot.Robot.planner = self

        self.robots = self.space.init_robots()
        self.robotClass = robot.Robot
        self.space.add_obstacles( self.params.get('obstacles', []) )

        self.mode = planner_modes.get(mode, rrtNAIVE)

    def plan(self):
        stime = time.time()
        for trobot in robot.Robot.get_all():
            self.init_plan_robot(trobot)
        print( '\nPlanning took {} secs.'.format( time.time()-stime ) )

    def replan(self, trobot, root=None, num_nodes=70):
        if root is None:
            root = trobot.root

        # print('Replanning for robot #{} @{}'.format(trobot.index, root.pos))

        if len(trobot.path) > Planning.replan_horizon+1:
            horend_node = trobot.path[Planning.replan_horizon]
        elif len(trobot.path) > 2:
            horend_node = trobot.path[-1]
        else:
            return False

        curr_nodes = 0
        any_rewired = False
        while curr_nodes < num_nodes:

            if bool(trobot.orphans):
                pdb.set_trace()

            x_rand_pos = self.space.sample_between(trobot.root.pos, horend_node.pos)
            if self.space.obstacle_free(x_rand_pos):
                n = int( min(Planning.rewire_nbors, trobot.tree.num_nodes) )
                nodes_nearby = set(trobot.nearby( x_rand_pos, n ))
                nodes_nearby.intersection_update(trobot.tree.vertices)

                L_near = trobot.sort_by_astar_cost(nodes_nearby, x_rand_pos, goal_pos=horend_node.pos)
                x_new, x_new_pos = None, None
                for _, x_near in L_near:
                    x_new_pos = trobot.steer( x_near.pos, x_rand_pos )
                    if x_new_pos is not None:
                        x_new = trobot.new_vertex(x_new_pos)
                        trobot.add_edge( x_new, x_near )
                        curr_nodes += 1
                        break
                
                if x_new is not None:
                    nodes_nearby = set(trobot.nbors_in_cube(x_new_pos, Planning.rewire_radius))
                    # nodes_nearby = set(trobot.nearby( x_new_pos, n ))
                    nodes_nearby.intersection_update(trobot.tree.vertices)
                    nodes_nearby.discard(x_new)
                    new_parent = trobot.tree.get_parent(x_new)
                    nodes_nearby.discard(new_parent)
                    nodes_nearby.discard(trobot.root)

                    L_near = trobot.sort_by_astar_cost(nodes_nearby, x_new_pos, goal_pos=horend_node.pos)
                    if bool(trobot.orphans):
                        pdb.set_trace()

                    Planning.replan_count += 1
                    any_rewired = trobot.rewire( x_new, L_near ) or any_rewired
                    trobot.prune()
        if any_rewired:
            trobot.find_path()

    def init_plan_robot(self, trobot):
        # trobot.pbar = tqdm(desc='Init R{}'.format(trobot.index), total=self.max_nodes)
        iter_counter = 0
        while not trobot.planning_done():
            iter_counter = iter_counter + 1
            if np.mod(iter_counter,1000) == 0:
                print("initial planning, iteration {}".format(iter_counter))

            x_rand_pos = self.space.sample()
            if self.space.obstacle_free(x_rand_pos):

                n = int( min(Planning.rewire_nbors, trobot.tree.num_nodes) )
                nodes_nearby = trobot.nearby( x_rand_pos, n )
                nodes_nearby.intersection_update(trobot.tree.vertices)
                
                if self.mode == rrtNAIVE:
                    L_near = trobot.sort_by_dist(nodes_nearby, x_rand_pos)
                elif self.mode == rrtTOTALDIST:
                    L_near = trobot.sort_by_total_cost(nodes_nearby, x_rand_pos)
                elif self.mode == rrtSTAR:
                    L_near = trobot.sort_by_astar_cost(nodes_nearby, x_rand_pos)
                else:
                    raise Exception

                x_new, x_new_pos = None, None
                for _, x_near in L_near:
                    x_new_pos = trobot.steer( x_near.pos, x_rand_pos )
                    if x_new_pos is not None:
                        x_new = trobot.new_vertex(x_new_pos)
                        trobot.add_edge( x_new, x_near )
                        break

                if x_new is not None and self.mode == rrtSTAR:
                #     # trobot.pbar.update(1)
                    nodes_nearby = set(trobot.nbors_in_cube( x_new.pos, Planning.rewire_radius ))
                    nodes_nearby = set(trobot.nearby( x_new_pos, n ))
                    nodes_nearby.intersection_update(trobot.tree.vertices)
                    nodes_nearby.discard(trobot.root)
                    nodes_nearby.discard(x_new)

                    L_near = trobot.sort_by_astar_cost(nodes_nearby, x_new_pos)

                    trobot.rewire( x_new, L_near )

            if random.random() < Planning.prc:
                trobot.new_can_connect_to_goal()
                # trobot.prune()

    def get_paths_array(self, interp_res = 10):
        _all_paths = []
        _all_path_len = []
        _all_fixed_points = []
        for trobot in self.robots:
            if trobot.found_goal:
                if not trobot.path:
                    trobot.find_path()
                _tpath = [tnode.pos for tnode in trobot.path]
                if bool(_tpath):
                    if interp_res > 0:
                        _tpath = interpolate_path(_tpath, interp_res = interp_res)
                    _all_paths += _tpath
                    _all_path_len.append(len(_tpath))

                    _all_fixed_points.append(_tpath[0])
                    _all_fixed_points.append(_tpath[-1])
        return np.asarray(np.stack(_all_fixed_points, axis=0), dtype=np.float64), np.asarray(np.stack(_all_paths, axis=0), dtype=np.float64), _all_path_len

    def run(self):
        """
        Update root to one of root's children
        Delete root
        Rewire other children
        """
        stime = time.time()
        for trobot in self.robots:
            # print('R{}'.format(trobot.index))
            trobot._initpath = trobot.find_path()
        
        while not robot.Robot.all_run_done():
            for trobot in self.robots:
                if len(trobot.path)<2:
                    trobot.run_done = True
                    continue

                nextDest = trobot.update_root()
                if bool(trobot.orphans):
                    pdb.set_trace()
                if nextDest is not None:
                    self.replan(trobot=trobot, root=nextDest)
                if bool(trobot.orphans):
                    pdb.set_trace()
        print( '\nLocal replanning took {} secs.'.format( time.time()-stime ) )

    def draw(self, _draw_plan=True, _draw_path=True, _draw_obs=True):
        stime = time.time()
        # plt.clf()
        if _draw_plan:
            for trobot in self.robots:
                trobot.draw_plan()

        if _draw_path:
            for trobot in self.robots:
                trobot.draw_path()

        if _draw_obs:
            plot_ix = self.robotClass.plot_fig
            for tobs in self.space.obstacles:
                tobs.draw(plot_ix)
        print( '\n2D Visualization took {} secs.'.format( time.time()-stime ) )

    def draw_3d(self, _draw_plan=True, _draw_path=True):
        stime = time.time()
        # plt.clf()
        if _draw_plan:
            for trobot in self.robots:
                trobot.draw_plan_3d()

        if _draw_path:
            for trobot in self.robots:
                trobot.draw_path_3d()
        print( '\n3D Visualization took {} secs.'.format( time.time()-stime ) )

            # if trobot.path:
            #     print('Path: {}'.format( '[ ' + ', '.join( str(tuple(t.pos)) for t in trobot.path ) + ' ]' ))
            #     print('Cost: {}'.format( trobot.path_cost() ))


# def _interpolate_path(path, _num = 10, _res=10.0, _use_num=True):
#     ret_path = []
#     for tix in range(len(path[:-1])):
#         ret_path.append(path[tix])
#         ret_path += _interpolate_between(path[tix], path[tix+1], _num, _res, _use_num=_use_num)
#     # ret_path.append(path[-1])
#     return ret_path
#
# def _interpolate_between(ptA, ptB, _num = 10, _res=10.0, _use_num=True):
#     ret_pts = []
#     tdist = _dist(ptA, ptB)
#
#     if _use_num:
#         num_pts = _num
#         ulen = tdist / num_pts
#     else:
#         num_pts = int(max(tdist/_res -1, 0))
#         ulen = _res
#
#     uvect = (ptB - ptA) / tdist * ulen
#
#     temp_pt = ptA
#     for _ in range(num_pts):
#         temp_pt = temp_pt.copy() + uvect
#         ret_pts.append(temp_pt)
#     return ret_pts
#
#     # new_t = np.arange(0, t[-1], step=self.terrain.res, dtype=np.float64).tolist()
#     # for ix in range(len(new_t)):
#     #     self.smd_history[0].append(float( x_smoothhist( new_t[ix] ) ))
#     #     self.smd_history[1].append(float( y_smoothhist( new_t[ix] ) ))
#     #     self.smd_history[2].append(float( self.terrain.get_altitude( self.smd_history[0][ix], self.smd_history[1][ix] ) + 1 ))

def _dist(x0, x1):
    return np.linalg.norm( x1-x0, ord=2 )