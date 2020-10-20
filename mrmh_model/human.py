from __future__ import print_function

import numpy as np
import random

import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

from scipy import ndimage, misc, signal

from . import terrain as terrain

class Human():

    a = np.array([1e-5, 1e-6])
    b = np.array([1e-5, 1e-3])
    m = 70
    alpha = np.array([-7e-0, -5e-0])
    beta = np.array([3e-1, 1.1e-1])

    # gamma = np.array([-5e-5, -3e-5])
    gamma = np.array([-5e+1, -7e+1])

    terrain = None

    num_humans = 0
    humans_list = []
    params = {}

    fatigueFactor = 1.0
    fatigueCount = 0

    def __init__( self, params={} ):

        if self.terrain is None:
            raise Exception('Need a terrain before you can create humans!')

        # Initial position - x and y coordinates
        if params.get('random_pos', False):
            _xrange = self.terrain.xmax - self.terrain.xmin
            _yrange = self.terrain.ymax - self.terrain.ymin
            self.x = np.array([  np.random.normal(0, _xrange/4 ), \
                                 np.random.normal(0, _yrange/4 ) ])
        else:
            self.x = np.array( params.get('x', [0.0, 0.0] ), dtype=float )

        if params.get('random_motion_init', False):
            self.xdot = np.array( [ (random.random()-random.random()), (random.random()-random.random()) ], dtype=float )
            self.xddot = np.array( [ (random.random()-random.random()), (random.random()-random.random()) ], dtype=float )
        else:
            self.xdot = np.array( params.get('xdot', [0.0, 0.0] ), dtype=float )
            self.xddot = np.array( params.get('xddot', [0.0, 0.0] ), dtype=float )

        Human.params = params
        self.history = [ [], [], [] ]

        self.out_of_bounds = False

        Human.num_humans += 1
        Human.humans_list.append(self)

    def get_pos(self):
        return (self.x[0], self.x[1])

    def next_x(self, dt=1.0):

        Human.fatigueCount += 1
        if Human.fatigueCount%25 == 0:
            Human.fatigueFactor *= 0.99
        self.xdot *= Human.fatigueFactor

        self.x = self.x + self.xdot * dt
        pass
    
    def next_xdot(self, dt=1.0):
        self.xdot = self.xdot + self.xddot * dt
        pass

    def update_xddot(self):
        # FI_sum = sum( [ self.F_interx(thuman) for thuman in Human.humans_list if thuman != self ] )
        self.xddot =  ( 0 \
                    # + Human.alpha * self.F_rand1() \
                    # + Human.beta * FI_sum \
                    + Human.gamma * self.F_rand2() * self.F_rand1() \
                    + (Human.a * abs(self.xdot) + Human.b) * self.xdot \
                    + Human.alpha * random.random() \
                    ) / Human.m

    def do_step(self, dt=1.0):
        if(not self.out_of_bounds):
            self.update_xddot()
            self.next_xdot(dt=dt)
            self.next_x(dt=dt)

        self.history[0].append( self.x[0] )
        self.history[1].append( self.x[1] )
    
        if(self.terrain is not None):
            self.history[2].append( self.terrain.get_altitude( self.x[0], self.x[1] ) )

    def F_rand1(self):
            # return np.asarray( self.terrain.get_altitude( self.x[0], self.x[1] ) + np.random.rand( *self.x.shape ) , dtype=float )
            return self.terrain.get_gradient( self.x[0], self.x[1] )
            # return np.array([ random.random(), random.random() ]) + np.array([ random.random(), random.random() ]) * self.x

    def F_rand2(self):
            tdir = self.terrain.get_gradient_dir( self.x[0], self.x[1] )
            return np.array([ np.cos(tdir), np.sin(tdir) ] )
            # return random.random()

    def F_interx(self, thuman):
        # return ( 1 ) / ( 1e-4 + ( self.x - thuman.x )**2 ) + random.random()
        try:
            return ( (self.x - thuman.x)**2 + 1)
        except FloatingPointError:
            print(self.x)
            print(thuman.x)
            raise Exception
        # return 0

    def __str__(self):
        return 'x: {}\tdx: {}\td2x: {}'.format(self.x, self.xdot, self.xddot)

    @staticmethod
    def step_all( dt=1.0):
        for thuman in Human.humans_list:
            thuman.do_step(dt=dt)

    @staticmethod
    def print_all():
        for thuman in Human.humans_list:
            print(thuman)

    @staticmethod
    def plot_all():
        if(Human.terrain is not None and Human.terrain.surface_plot is not None):
            ax = Human.terrain.surface_plot
        else:
            fig = plt.gcf()
            ax = fig.gca(projection='3d')
        for thuman in Human.humans_list:
            ax.plot( xs=thuman.history[0], ys=thuman.history[1], zs=thuman.history[2] )
            plt.title('Lost persons\' paths - 3D')
            # plt.scatter(thuman.history[0], thuman.history[1])
        plt.show()

    @staticmethod
    def plot_all_2d():
        fig = plt.figure()
        plt.title('Lost persons\' paths - 2D')
        ax = fig.gca()
        for thuman in Human.humans_list:
            ax.plot( thuman.history[0], thuman.history[1])
            # plt.scatter(thuman.history[0], thuman.history[1])
        plt.draw()
    
    @staticmethod
    def go_forth( params={} ):
        if(params):
            Human.params = params
        
        num_humans = params.get('num_humans', 1)
        for _ in range(num_humans):
            Human( Human.params )

        return Human.humans_list

    @staticmethod
    def reset( params={} ):
        num_humans = Human.num_humans
        
        Human.fatigueFactor = 1.0
        Human.fatigueCount = 0
        
        if(num_humans > 0):
            Human.num_humans = 0
            Human.humans_list = []

            for _ in range(num_humans):
                Human( Human.params )
        elif( params.get('num_humans', 0) ):
            Human.go_forth(params=params)
        else:
            Human.go_forth()

        return Human.humans_list