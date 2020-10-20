import sys, os, time, math

# up1 = os.path.abspath('..')
# if up1 not in sys.path:
#     sys.path.insert(0, up1)

import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from mrmh_model import params

from . import searchspace
from . import robot
from . import tree_boost

from tqdm import tqdm

import pdb
import itertools

# def debug_signal_handler(signal, frame):
#     import pdb
#     pdb.set_trace()
# import signal
# signal.signal(signal.SIGINT, debug_signal_handler)

class Planning(params.Default):
    prc = 0.01
    rewire_radius = 15
    rewire_nbors = 20
    replan_horizon = 4
    replan_count=0

    def __init__(self, params, on_terrain=None):
        super().__init__(params)
        self.space = searchspace.SearchSpace(params, on_terrain)
        self.max_nodes = self.params.get('max_plan_nodes', 2500)
        self.robotClass = robot.Robot
        robot.Robot.space = self.space
        robot.Robot.planner = self
        self.robots = self.space.init_robots()
        self.space.add_obstacles( self.params.get('obstacles', []) )

        Planning.rewire_radius = int((math.sqrt(self.space._yrange**2 + self.space._xrange**2)/25))

    def plan(self):
        stime = time.time()
        for trobot in robot.Robot.get_all():
            self.init_plan_robot(trobot)
        print( 'Planning took {} secs.'.format( time.time()-stime ) )

    def init_plan_robot(self, trobot):
        # trobot.pbar = tqdm(desc='Init R{}'.format(trobot.index), total=self.max_nodes)
        while not trobot.planning_done():
            x_rand_pos = self.space.sample()
            if self.space.obstacle_free(x_rand_pos):

                n = int( min(Planning.rewire_nbors, trobot.tree.num_nodes) )
                nodes_nearby = trobot.nearby( x_rand_pos, n )
                nodes_nearby.intersection_update(trobot.tree.vertices)
                L_near = trobot.sort_by_dist(nodes_nearby, x_rand_pos)

                x_new, x_new_pos = None, None
                for _, x_near in L_near:
                    x_new_pos = trobot.steer( x_near.pos, x_rand_pos )
                    if x_new_pos is not None:
                        x_new = trobot.new_vertex(x_new_pos)
                        trobot.add_edge( x_new, x_near )
                        break

            # trobot.pbar.update(1)
            if random.random() < Planning.prc:
                trobot.new_can_connect_to_goal()

    def draw_candidate_paths(self):
        stime = time.time()
        for trobot in self.robots:
            trobot.draw_goal_nbor_paths()
        plt_ix = plt.gcf().number
        for tobs in self.space.obstacles:
            tobs.draw(plt_ix)
        print( 'Visualizing candidate paths in 2D took {} secs.'.format( time.time()-stime ) )

    def draw(self, _draw_plan=True, _draw_path=True):
        stime = time.time()
        # plt.clf()
        if _draw_plan:
            for trobot in self.robots:
                trobot.draw_plan()

        if _draw_path:
            for trobot in self.robots:
                trobot.draw_path()
        print( '2D Visualization took {} secs.'.format( time.time()-stime ) )

    def draw_3d(self, _draw_plan=True, _draw_path=True):
        stime = time.time()
        # plt.clf()
        if _draw_plan:
            for trobot in self.robots:
                trobot.draw_plan_3d()

        if _draw_path:
            for trobot in self.robots:
                trobot.draw_path_3d()
        print( '3D Visualization took {} secs.'.format( time.time()-stime ) )

            # if trobot.path:
            #     print('Path: {}'.format( '[ ' + ', '.join( str(tuple(t.pos)) for t in trobot.path ) + ' ]' ))
            #     print('Cost: {}'.format( trobot.path_cost() ))

class PathCollection:
    planner = None
    def __init__(self, leaves):
        pass