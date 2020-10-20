from __future__ import print_function

import numpy as np
import random

import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

# import plotly.plotly as ply
from scipy import ndimage, misc, signal, interpolate
import copy, enum

from . import terrain as terrain
from . import params as params

import pdb

# Searcher Modes
SWEEP = 0
GRADIENT = 1

# ply.sign_in('bgavin', 'WdoigPPhaOBZU0AZsPs3')
np.set_printoptions(precision=3, floatmode='fixed', suppress=True)

class Searcher(params.Default):

    terrain = None
    num_searchers = 0
    searchers_list = []
    params = {}

    x_sweep_width = 1
    y_sweep_lambda = 1

    # gamma = np.array([-2e-1, -7e-1])
    # gamma = np.array([ 0.8, 0.6 ]) * 7e-1
    gamma = np.array([ 100, 100 ]) # make this not weird?

    fatigueFactor = 1.0
    fatigueCount = 0

    # grad_ON_init = 2000.0
    # grad_OFF_init = 1900.0 # no more drunk searchers
    # grad_ON_init = 9.5
    # grad_OFF_init = 9.0 # original values
    grad_ON_init = 20
    grad_OFF_init = 19.5 # tweaked values

    def __init__( self, params={} ):
        super().__init__(params)

        if self.terrain is None:
            raise Exception('Need a terrain before you can create searchers!')

        # Initial position - x and y coordinates
        self.x = np.array( params.get('sx', [0.0, 0.0] ), dtype=float )
        self.xdot = np.array( params.get('sxdot', [0.0, 0.0] ), dtype=float )
        self.xddot = np.array( params.get('sxddot', [0.0, 0.0] ), dtype=float )

        self.T = 0.0

        params.setdefault( 'dt', 0.25 )
        self.dt = params['dt']

        self.phase_offset = np.random.rand(1)
        self._init_x_ = self.x.copy()
        self.xdot_temp = self.xdot.copy()

        Searcher.params = params
        self.history = [ [], [], [], [] ]

        self.mode = SWEEP
        self.sweep_done = False


        if params['search_method'] == 'sweep':
            self.num_ticks = params['searcher_num_ticks']
            sweep_xlim, sweep_ylim = params['sweep_bounds']
            # sweep_xlim, sweep_ylim = [400/3, 400/10]
            sweep_x = np.array(sweep_xlim, dtype=float)
            sweep_x = np.hstack( [ sweep_x, np.flip( sweep_x.copy() ) ] )
            sweep_x = np.tile(sweep_x, int(self.num_ticks/4) +1 )
            sweep_x = sweep_x[0:self.num_ticks]

            sweep_y = np.linspace( sweep_ylim[0], sweep_ylim[1], num=self.num_ticks+1, endpoint=True, dtype=float )
            sweep_y = sweep_y[1:self.num_ticks+1]

        elif params['search_method'] == 'grouped':
            sweep_x = params['grouped_x'] # remember: these parameters are set by go_forth method
            sweep_y = params['grouped_y']
            self.num_ticks = sweep_x.shape[0]
        else:
            sweep_x = params['grouped_x']
            sweep_y = params['grouped_y']

        self.sweep_guides = np.vstack( [sweep_x, sweep_y] )
        self.target_ix = 0
        self.curr_target = self.sweep_guides[ :, self.target_ix ]
        # self.target_dist = dist( self.sweep_guides[:, 0], self.sweep_guides[:, 1] ) * 0.1
        self.target_dist = 4 # constant is baddddd, but its better than what was there before
        self.target_count = 0

        self.grad_ON = Searcher.grad_ON_init
        self.grad_OFF = Searcher.grad_OFF_init

        self.index = Searcher.num_searchers
        Searcher.num_searchers += 1
        Searcher.searchers_list.append(self)

    def get_pos(self):
        return (self.x[0], self.x[1])

    def next_x(self, dt=1.0):
        self.T += dt
        self.x = self.x + self.xdot * dt
        # re-map x vals back to terrain
        if self.x[0] <= self.params['xlims'][0]:
            self.x[0] = self.params['xlims'][0] + 1 # shift back onto terrain
        elif self.x[0] >= self.params['xlims'][1]:
            self.x[0] = self.params['xlims'][1] - 1 # shift back onto terrain
        # same for y vals
        if self.x[1] <= self.params['ylims'][0]:
            self.x[1] = self.params['ylims'][0] + 1 # shift back onto terrain
        elif self.x[1] >= self.params['ylims'][1]:
            self.x[1] = self.params['ylims'][1] - 1 # shift back onto terrain

    def next_xdot(self, dt=1.0):
        if self.mode == GRADIENT:
            self.xdot += self.xddot * dt # TODO: swapping additions around! (test)
        if not self.sweep_done:
            self.xdot += self.curr_target - self.x
            # self.xdot = np.array([ 0.916515, 0.4 ]) * self.xdot / np.sqrt( np.sum(self.xdot**2) )
            # original code:
            # self.xdot = np.array([ 0.8, 0.6 ]) * self.xdot / np.sqrt( np.sum(self.xdot**2) )
            # jitter to bump out of local minima
            self.xdot = self.xdot / np.sqrt( np.sum(self.xdot**2) )
            # self.xdot = 0.1*np.random.rand(2)
            # self.xdot = self.xdot / np.sqrt( np.sum(self.xdot**2) )

    def update_xddot(self):
        if self.mode == GRADIENT:
            # self.xddot = Searcher.gamma * self.F_rand2() * self.F_rand1()
            self.xddot = Searcher.gamma * self.F_rand2() #* self.F_rand1()

    def map_lim_check(self):
        if self.x[1] >= Searcher.terrain.ymax: # or self.x[1] <= Searcher.terrain.ymin:
            self.sweep_done = True
            print( 'Searcher {0}: sweep done at {1:02.2f}!'.format(self.index, self.T) )

    def sweep_cb(self):
        target_reached = dist( self.x, self.curr_target ) <= self.target_dist
        if target_reached:
            if self.target_ix < self.num_ticks-1:
                self.target_ix += 1
                self.curr_target = self.sweep_guides[:, self.target_ix]

                self.target_count = 0
                self.grad_ON = Searcher.grad_ON_init
                self.grad_OFF = Searcher.grad_OFF_init
            else:
                self.sweep_done = True
        else:
            self.target_count += 1

    @staticmethod
    def all_done():
        return all([ tsearcher.sweep_done for tsearcher in Searcher.searchers_list ])

    def do_step(self, dt=1.0):
        if self.sweep_done:
            return
        self.update_mode() # resets and selects which way to move the searcher based on terrain
        self.sweep_cb()

        self.update_xddot()
        self.next_xdot(dt=dt)
        self.next_x(dt=dt)
        self.map_lim_check()

        self.history[0].append( self.x[0] )
        self.history[1].append( self.x[1] )
        self.history[2].append( self.terrain.get_altitude( self.x[0], self.x[1] ) + 1 )
        self.history[3].append( self.T )

    def update_mode(self):
        grad = abs( self.terrain.get_gradient(self.x[0], self.x[1]) )
        # print('Searcher {}: gradient: {}'.format(self.index,grad))
        # print(self.target_count)
        if( self.target_count > 100 and self.target_count%5==0 ):
            self.grad_ON += 0.5
            self.grad_OFF += 0.5

        if grad > self.grad_ON and self.mode == SWEEP:
            self.xdot = np.array([0.0, 0.0])
            self.xddot = np.array([0.0, 0.0])
            self.mode = GRADIENT
            # print('Searcher {}: Switch to GRADIENT mode'.format(self.index))
        elif grad < self.grad_OFF and self.mode == GRADIENT:
            self.xdot = np.array([0.0, 0.0])
            self.xddot = np.array([0.0, 0.0])
            self.mode = SWEEP
            # print('Searcher {}: Switch to SWEEP mode'.format(self.index))

    def F_rand1(self):
        return self.terrain.get_gradient( self.x[0], self.x[1] )

    def F_rand2(self):
        tdir = self.terrain.get_gradient_dir(self.x[0], self.x[1])
        # play with gradient direction
        return np.array([ np.cos(tdir + (np.pi)), np.sin(tdir + (np.pi)) ])
        # return np.array([ np.cos(tdir), np.sin(tdir) ])

    def F_interx(self, tsearcher):
        try:
            return ( (self.x - tsearcher.x)**2 + 1 )
        except FloatingPointError:
            print(self.x)
            print(tsearcher.x)
            raise Exception
        # return 0

    def __str__(self):
        modeStr = 'SWP' if self.mode == SWEEP else 'GRD'
        currDist = vertical_abs_dist(self.curr_target, self.x)
        return '{} => {} x: {}\tdx: {}\td2x: {}\t tgt: {}\t tgt_ix: {}\t distToTgt: {}\t distThreshold: {} :: {}'.format(
            self.index, modeStr, self.x, self.xdot, self.xddot, self.curr_target, self.target_ix, currDist, self.target_dist, self.sweep_done)

    def smooth_traj(self):
        # if self.sweep_done:
        x = self.history[0]
        y = self.history[1]
        z = self.history[2]
        t = self.history[3]

        tck, u = interpolate.splprep([x, y, z], s=0, k=3)  # parametric interpolation
        u_new = np.linspace(0, 1, self.params['searcher_path_num'])  # scaled discretization
        pts_interp = interpolate.splev(u_new, tck)
        self.smd_history = copy.deepcopy( self.history )
        self.smd_history[0] = pts_interp[0].tolist() # x axis
        self.smd_history[1] = pts_interp[1].tolist() # y axis
        self.smd_history[2] = pts_interp[2].tolist() # z axis
        # x_smoothhist = interpolate.UnivariateSpline( t, x, k=5 )
        # y_smoothhist = interpolate.UnivariateSpline( t, y, k=5 )
        # z_smoothhist = interpolate.UnivariateSpline( t, z, k=5 )
        # self.smd_history = copy.deepcopy( self.history )
        # for ix in range(len(x)):
        #     self.smd_history[0][ix] = x_smoothhist( t[ix] )
        #     self.smd_history[1][ix] = y_smoothhist( t[ix] )
        #     self.smd_history[2][ix] = z_smoothhist( t[ix] )

    @staticmethod
    def step_all( dt=1.0):
        # Searcher.print_all()
        for tsearcher in Searcher.searchers_list:
            tsearcher.do_step(dt=dt)
        Searcher.fatigueCount += 1

    @staticmethod
    def print_all():
        print('\n{} :: {}'.format( Searcher.fatigueCount, Searcher.searchers_list[0].T ))
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
            ax.plot( xs=tsearcher.history[0], ys=tsearcher.history[1], zs=tsearcher.history[2] )
            # plt.scatter(tsearcher.history[0], tsearcher.history[1])
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.draw()

        if Searcher.searchers_list[0].params.get('pushToPlotly', False):
            raise NotImplementedError
            fig = plt.gcf()
            # plot_url = ply.plot_mpl(fig)
            # print('Plot.ly url: {}'.format(plot_url))

    @staticmethod
    def smooth_traj_all():
        for tsearcher in Searcher.searchers_list:
            tsearcher.smooth_traj()

    @staticmethod
    def plot_all_2d():

        if(Searcher.terrain is not None and Searcher.terrain.paths_plot is not None):
            fig = Searcher.terrain.paths_plot
            plt.figure(fig.number)
            plt.title('Heatmap - Lost person | Searchers\' paths')
        else:
            fig = plt.figure() #plt.gcf()
            plt.title('Searchers\' paths')
            # ax = fig.gca()
            Searcher.terrain.paths_plot = fig

        for tsearcher in Searcher.searchers_list:
            plt.plot( tsearcher.history[0], tsearcher.history[1], color='xkcd:light grey' )

        plt.xlim( Searcher.terrain.xmin, Searcher.terrain.xmax )
        plt.ylim( Searcher.terrain.ymin, Searcher.terrain.ymax )

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.draw()

    @staticmethod
    def plot_all_smooth_2d():

        if(Searcher.terrain is not None and Searcher.terrain.paths_plot is not None):
            fig = Searcher.terrain.paths_plot
            plt.figure(fig.number)
            plt.title('Heatmap - Lost person | Searchers\' paths')
        else:
            fig = plt.figure() #plt.gcf()
            plt.title('Searchers\' paths')
            # ax = fig.gca()
            Searcher.terrain.paths_plot = fig

        for tsearcher in Searcher.searchers_list:
            plt.plot( tsearcher.smd_history[0], tsearcher.smd_history[1], 'xkcd:blue grey' )

        plt.xlim( Searcher.terrain.xmin, Searcher.terrain.xmax )
        plt.ylim( Searcher.terrain.ymin, Searcher.terrain.ymax )

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.draw()

    @staticmethod
    def go_forth( params={} ):
        if(params):
            Searcher.params = params
        
        num_searchers = params.get('num_searchers', 1)

        if Searcher.terrain is None:
            raise Exception('Need a terrain before you can create searchers!')

        _ymin = Searcher.terrain.ymin # pulled directly from terrain data, not parameters...
        _ymax = Searcher.terrain.ymax
        _yrange = Searcher.terrain._yrange

        _xmin = Searcher.terrain.xmin
        _xmax = Searcher.terrain.xmax
        _xrange = Searcher.terrain._xrange

        dt = params.get('dt', 1)

        Searcher.x_sweep_width = ( (_xrange) / (2*num_searchers) ) * 1.2 # whats the deal with these constants?
        Searcher.y_sweep_lambda = float(dt) *  (_yrange) * 6.7e-2

        '''
        Major additions incomming!
        - params['grouped_x']
        - params['grouped_y']
        - custom sweep width
        - grouped search method
        '''
        sweep_num = params['sweep_num'] # number of lengths of area to perform (for 'grouped' case)
        s_x = _xrange/(sweep_num*2*num_searchers) # adjusted sweep length (2*s per searcher)
        sw = 2*num_searchers*s_x
        s_y = np.int(_yrange/20) # y axis bound need not be calculated

        # build waypoints for 'grouped' search method while we're at it
        for ix in range(num_searchers):
            y_flipper = True
            grouped_x = []
            grouped_y = []
            for sn in range(sweep_num):
                grouped_x.append(_xmin + sn*sw + 2*ix*s_x + s_x)
                grouped_x.append(_xmin + sn*sw + 2*ix*s_x + s_x)
                if y_flipper:
                    grouped_y.append(_ymin + s_y)
                    grouped_y.append(_ymax - s_y)
                else:
                    grouped_y.append(_ymax - s_y)
                    grouped_y.append(_ymin + s_y)
                y_flipper = not y_flipper

            if params['search_method'] == 'grouped':
                params['sx'] = [ grouped_x[0], grouped_y[0] ]
            elif params['search_method'] == 'sweep':
                params['sx'] = [ _xmin + (2*ix+1)*_xrange/(2*num_searchers), _ymin ] # starting position
            else:
                params['sx'] = [ _xmin + (2*ix+1)*_xrange/(2*num_searchers), _ymin ] # default to sweep method

            params['sweep_bounds'] = [ ( _xmin+ ix*(_xrange/num_searchers), _xmin+ (ix+1)*(_xrange/num_searchers) ), (_ymin, _ymax) ] # sweep bounds
            params['grouped_x'] = np.array(grouped_x)
            params['grouped_y'] = np.array(grouped_y)
            Searcher( Searcher.params ) # wait. so it instantiates a class within the for loop? and doesnt assign it?
            # print(ix, params['sx'], params['sweep_bounds'])

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

    def set_defaults(self):
        pass

def dist( a, b ):
    return np.sqrt(np.sum( (a - b)**2 ))

def vertical_dist( a, b ):
    return (a[1] - b[1] )**2

def vertical_abs_dist( a, b ):
    return abs(a[1] - b[1] )