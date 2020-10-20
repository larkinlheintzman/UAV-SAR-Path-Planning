import sys, os, time

# up1 = os.path.abspath('..')
# if up1 not in sys.path:
#     sys.path.insert(0, up1)

import random
import uuid

import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
from rtree import index
from scipy import ndimage, misc, signal, interpolate

from . import robot
from mrmh_model import terrain

class SearchSpace(terrain.Terrain):
    def __init__(self, params, onTerrain=None):
        super().__init__(params)
        self.params = params
        self.set_defaults()

        self.on_terrain(onTerrain)
        self.sanity_checks()

        self.obstacles = []

        self.sample_dim = 3

        props = index.Property()
        props.dimension = self.sample_dim
        self.obs_check = index.Index(interleaved=True, properties=props)

    def init_robots(self):
        robot.Robot.max_nodes = self.params['max_plan_nodes']
        for ix in range(self.num_robots):
            xpos = self.xmin + (2*ix+1)*self._xrange/(2*self.num_robots)
            ypos = self.ymin
            zpos = self.h_smooth.ev(xpos, ypos) + 1

            xgoal = xpos
            ygoal = self.ymax
            zgoal = self.h_smooth.ev(xgoal, ygoal) + 1

            trobot = robot.Robot( init_pos=(xpos, ypos, zpos), goal_pos=(xgoal, ygoal, zgoal) )
            self.robots.append( trobot )
        return self.robots

    def sample(self):
        ret_sample = np.zeros((3), dtype=float)
        ret_sample[0] = np.random.uniform(low=self.xmin, high=self.xmax)
        ret_sample[1] = np.random.uniform(low=self.ymin, high=self.ymax)

        h_at_xy = self.h_smooth.ev( ret_sample[0], ret_sample[1] )
        ret_sample[2] = np.random.uniform(low=h_at_xy+self.params['planning_min_height'], high=h_at_xy+self.params['planning_max_height'])

        # # TODO: just change lower limit for uniform dist?
        # while True:
        #     z_sample = np.random.uniform(low=h_at_xy, high=self.hmax)
        #     if( z_sample > h_at_xy ):
        #         break
        # sample[2] = z_sample
        return ret_sample

    def sample_between(self, posA, posB):
        xmin, xmax = minmax_np(posA, posB, 0)
        ymin, ymax = minmax_np(posA, posB, 1)
        hmin, hmax = minmax_np(posA, posB, 2)

        ret_sample = np.zeros((3), dtype=float)
        ret_sample[0] = np.random.uniform(low=xmin, high=xmax)
        ret_sample[1] = np.random.uniform(low=ymin, high=ymax)

        h_at_xy = self.h_smooth.ev( ret_sample[0], ret_sample[1] )
        hmin = max(h_at_xy, hmin)
        ret_sample[2] = np.random.uniform(low=hmin, high=hmax)

        return ret_sample

    def dist(self, x0, x1):
        return np.linalg.norm( x1-x0, ord=2 )

    def add_obstacles(self, obs=[]):
        for t_obs in obs:
            self.obstacles.append( Obstacle(t_obs) )

    def obstacle_free(self, x):
        if x[2] < self.h_smooth.ev( x[0], x[1] ):
            return False
        obs_check = not any([ t_obs.collision_check(x) for t_obs in self.obstacles ])
        return obs_check

    def on_terrain(self, onTerrain):
        if onTerrain:
            self.h = onTerrain.h
            self.h_smooth = onTerrain.h_smooth
            self.terrain = onTerrain

    def set_defaults(self):
        super().set_defaults()
        self.num_robots = self.params.get('num_robots', 2)
        self.robots = []

    def envelope(self, a, b):
        lims = []
        for tdim in range(self.sample_dim):
            lims.append(minmax_np(a, b, tdim))
        return lims

class Obstacle:
    types = { 'Sphere', 'InfCylinder' }
    def __init__( self, obs_params ):

        if obs_params['type'] is 'InfCylinder':
            cx, cy = obs_params['center']
            r = obs_params['radius']
            self.type = 'InfCylinder'
            self.cx, self.cy = cx, cy
            self.c = np.array([cx, cy], dtype=float)
            self.r = r
            # print( 'InfCylinder: {}, {}'.format( (cx, cy), r ) )

        elif obs_params['type'] is 'Sphere':
            cx, cy, cz = obs_params['center']
            r = obs_params['radius']
            self.type = 'Sphere'
            self.cx, self.cy, self.cz = cx, cy, cz
            self.c = np.array( [cx, cy, cz], dtype=float)
            self.r = r
            # print( 'Sphere: {}, {}'.format( (cx, cy, cz), r ) )

    def draw(self, plt_ix=None, _fill=False):
        if plt_ix is not None:
            plt.figure(plt_ix)
        self.proj = plt.Circle((self.cx, self.cy), radius=self.r, fill=_fill, color='yellow')
        ax = plt.gca()
        ax.add_patch(self.proj)

    def collision_check(self, x ):
        if self.type is 'Sphere':
            return self.check_sphere_collision(x)
        else:
            return self.check_infcyl_collision(x)

    def check_infcyl_collision(self, pos):
        return np.linalg.norm( pos[:2]-self.c, ord=2 ) <= self.r

    def check_sphere_collision(self, pos):
        return np.linalg.norm( pos-self.c, ord=2 ) <= self.r

def minmax_np(a, b, idx):
    lo = a[idx]
    if b[idx] > lo:
        hi = b[idx]
    else:
        lo = b[idx]
        hi = a[idx]
    return tuple([lo, hi])