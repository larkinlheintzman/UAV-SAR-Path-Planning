from __future__ import print_function

import numpy as np
import random

import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

from scipy import ndimage, misc, signal

from . import terrain as terrain

import pdb

class Searcher():

    # a = np.array([1e-5, 1e-6])
    # b = np.array([1e-5, 1e-3])
    # m = 70

    alpha = np.array([7e-3, 5e-3])
    beta = np.array([3e-2, 1.1e-2])
    gamma = np.array([5e-1, 3e-1])

    terrain = None

    num_searchers = 0
    searchers_list = []
    params = {}

    x_sweep_width = 1
    y_sweep_freq = 1

    def __init__( self, params={} ):

        if self.terrain is None:
            raise Exception('Need a terrain before you can create searchers!')

        # Initial position - x and y coordinates
        self.x = np.array( params.get('sx', [0.0, 0.0] ), dtype=float )
        self.xdot = np.array( params.get('sxdot', [0.0, 0.0] ), dtype=float )
        self.xddot = np.array( params.get('sxddot', [0.0, 0.0] ), dtype=float )

        self.T = 0.0
        self.phase_offset = np.random.rand(1)

        self._init_x_ = self.x.copy()
        self.xdot_temp = self.xdot.copy()

        Searcher.params = params
        self.history = [ [], [], [] ]

        Searcher.num_searchers += 1
        Searcher.searchers_list.append(self)

    def get_pos(self):
        return (self.x[0], self.x[1])

    def next_x(self, dt=1.0):
        self.T += dt
        self.x = self.x + self.xdot * dt

    def next_xdot(self, dt=1.0):
        self.xdot = self.xdot - self.xdot_temp + self.xddot * dt #np.array([0.1, 0.0]) 

        self.xdot_temp[1] = 0 #( ( self._init_x_[1] + Searcher.x_sweep_width * np.sin(self.T *dt) ) - self.x[1] )
        self.xdot_temp[0] = 0 #dt

        self.xdot += self.xdot_temp

    def update_xddot(self):
        # FI_sum = sum( [ self.F_interx(tsearcher) for tsearcher in Searcher.searchers_list if tsearcher != self ] )
        # self.xddot = np.array( [ 0, Searcher.x_sweep_width * -np.cos((self.T * Searcher.y_sweep_freq + 1e-5 )) ] )
        self.xddot = Searcher.gamma * self.F_rand2() * self.F_rand1() - (self.xdot - self.xdot_temp)
        # self.xddot *= 0

    def do_step(self, dt=1.0):
        self.next_x(dt=dt)
        self.next_xdot(dt=dt)
        self.update_xddot()

        self.history[0].append( self.x[0] )
        self.history[1].append( self.x[1] )
    
        if(self.terrain is not None):
            self.history[2].append( self.terrain.get_altitude( self.x[0], self.x[1] ) )

    def F_rand1(self):
        # return np.asarray( self.terrain.get_altitude( self.x[0], self.x[1] ) + np.random.rand( *self.x.shape ) , dtype=float )
        return self.terrain.get_gradient( self.x[0], self.x[1] )

    def F_rand2(self):
        tdir = self.terrain.get_gradient_dir(self.x[0], self.x[1])
        return np.array([ np.cos(tdir), np.sin(tdir) ])

    def F_interx(self, tsearcher):
        # return ( 1 ) / ( 1e-4 + ( self.x - tsearcher.x )**2 ) + random.random()
        try:
            return ( (self.x - tsearcher.x)**2 + 1 )
        except FloatingPointError:
            print(self.x)
            print(tsearcher.x)
            raise Exception
        # return 0

    def __str__(self):
        return 'x: {}\tdx: {}\td2x: {}'.format(self.x, self.xdot, self.xddot)

    @staticmethod
    def step_all( dt=1.0):
        for tsearcher in Searcher.searchers_list:
            tsearcher.do_step(dt=dt)

    @staticmethod
    def print_all():
        for tsearcher in Searcher.searchers_list:
            print(tsearcher)

    @staticmethod
    def plot_all():
        if(Searcher.terrain is not None and Searcher.terrain.surface_plot is not None):
            ax = Searcher.terrain.surface_plot
        else:
            fig = plt.gcf()
            ax = fig.gca(projection='3d')
        for tsearcher in Searcher.searchers_list:
            ax.plot( xs=tsearcher.history[1], ys=tsearcher.history[0], zs=tsearcher.history[2] )
            # plt.scatter(tsearcher.history[0], tsearcher.history[1])
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

    @staticmethod
    def plot_all_2d():
        fig = plt.figure()
        print('XXYYZZsearch.old')
        ax = fig.gca()
        for tsearcher in Searcher.searchers_list:
            ax.plot( tsearcher.history[1], tsearcher.history[0] )
        plt.xlim( Searcher.terrain.xmin, Searcher.terrain.xmax )
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()
    
    @staticmethod
    def go_forth( params={} ):
        if(params):
            Searcher.params = params
        
        num_searchers = params.get('num_searchers', 1)

        if Searcher.terrain is None:
            raise Exception('Need a terrain before you can create searchers!')

        _ymin = Searcher.terrain.ymin
        _ymax = Searcher.terrain.ymax
        _yrange = _ymax - _ymin

        _xmin = Searcher.terrain.xmin
        _xmax = Searcher.terrain.xmax
        _xrange = _xmax - _xmin

        Searcher.x_sweep_width = ( (_xrange) / (2*num_searchers) ) * 0.97
        Searcher.y_sweep_freq = ( 13/_yrange )

        print(Searcher.x_sweep_width, Searcher.y_sweep_freq)
        print(_xmin, _xmax, _xrange)

        for ix in range(num_searchers):
            params['sx'] = [ _ymin, _xmin + (2*ix+1)*_xrange/(2*num_searchers) ]
            print(ix, params['sx'])
            Searcher( Searcher.params )

        return Searcher.searchers_list

    @staticmethod
    def reset( params={} ):
        num_searchers = Searcher.num_searchers

        if(num_searchers > 0):
            Searcher.num_searchers = 0
            Searcher.searchers_list = []

            for _ in range(num_searchers):
                Searcher( Searcher.params )
        elif( params.get('num_searchers', 0) ):
            Searcher.go_forth(params=params)
        else:
            Searcher.go_forth()

        return Searcher.searchers_list