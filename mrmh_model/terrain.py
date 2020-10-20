from __future__ import print_function

import numpy as np
import random
import traceback

import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import hashlib, sys, os

from scipy import ndimage, misc, signal, interpolate

# import pdb
from . import space as space
from . import human as human
from . import params as params

from arcgis_terrain import get_terrain_map


class Terrain(space.Space):
    def __init__(self, params={}):
        super().__init__(params)

        self.generate_terrain(terrainType=params.get('terrainType', 'random'))
        # key error: 'terrain' is not right. should be 'terrainType'
        self.compute_gradient()

    def generate_terrain(self, terrainType='random', terrainLocation=None):

        # print('Generating terrain')
        # terrainLocation = [[[37.2296, -80.4139],[37.22965, -80.41390],[37.22960, -80.41395]]] # terrain is placed on the centroid of these points ('swhy there's three not four)
        # terrainLocation = [[[37.227290, -80.406481],[37.227338, -80.406538],[37.227242, -80.406543]]]
        if terrainType == 'random':
            self.h = random.random() * 3e-01 * np.cos(random.random() + self.x * (random.random())) + \
                     random.random() * 1e-2 * np.cos(random.random() + self.y * (random.random())) + \
                     random.random() * 3e-1 * np.sin(self.y * (random.random())) + \
                     random.random() * 1e-3 * np.sin(self.x * (random.random()))
            self.h += 50 * np.random.rand(self._xnum, self._ynum) - 50 * np.random.rand(self._xnum, self._ynum) \
                      + 100 * np.random.rand(self._xnum, self._ynum) - 100 * np.random.rand(self._xnum, self._ynum)
            self.h *= 2

            self.h = ndimage.gaussian_filter(self.h, sigma=6, mode='reflect')

        elif terrainType == 'loadNumPy':
            nameStr = 'terrain_' + '{}{}{}{}'.format(self.params['xlims'], self.params['ylims'], self.params['zlims'],
                                                     self.params['res']) + '.npy'
            if os.path.isfile(nameStr):
                print('Loading saved terrain: {}'.format(nameStr))
                self.h = np.load(nameStr)
            else:
                raise FileNotFoundError(nameStr)

        elif terrainType == 'flat':
            self.h = np.zeros((self._xnum, self._ynum), dtype=float)

        elif terrainType == 'yslope':
            self.h = np.ones((self._xnum, self._ynum), dtype=float)
            self.h *= np.linspace(50.0, -50.0, self._ynum)

        elif terrainType == 'real':
            # load actual terrain from gps points

            print("collecting terrain data ...")
            terrain_location = self.params.get('anchor_point', False)
            [e,x,y,data,cen_pt] = get_terrain_map(terrain_location, sample_dist = 2*self.res,
                                      extent = self._xrange, heading=self.params.get('heading'), show_plot=False, verbosity=False)
            e = e - np.min(e)

            # interpolate terrain to match size/resolution of other layers
            x_temp = np.linspace(0,self._xrange,np.int(self._xrange/(2*self.res))) # get correct size of terrain map
            y_temp = np.linspace(0,self._xrange,np.int(self._xrange/(2*self.res)))
            f = interpolate.interp2d(x_temp, y_temp, e, kind='quintic')
            x_temp = np.linspace(0,self._xrange,np.int(self._xrange/(1*self.res)))
            y_temp = np.linspace(0,self._xrange,np.int(self._xrange/(1*self.res)))
            e_interp = f(x_temp, y_temp)

            self.h = e_interp

            self.terrain_data = [e_interp, x_temp, x_temp, data, cen_pt]
            print("terrain data collected!")

        if self.params.get('save_terrain', False):
            nameStr = 'terrain_' + '{}{}{}{}'.format(self.params['xlims'], self.params['ylims'], self.params['zlims'],
                                                     self.params['res']) + '.npy'
            np.save(nameStr, self.h)

        self.h_smooth = interpolate.RectBivariateSpline(self._x, self._y, self.h)

    def get_altitude(self, xval=0.0, yval=0.0):
        if self.is_within_bounds(xval, yval):
            return self.h_smooth.ev(xval, yval)
        else:
            return 0

    def compute_gradient(self):
        dy = ndimage.sobel(self.h, axis=1, mode='nearest')
        dx = ndimage.sobel(self.h, axis=0, mode='nearest')

        self.grad_mag = np.hypot(dy, dx)
        self.grad_mag_interp = interpolate.interp2d(self._x, self._y, self.grad_mag.transpose(), kind='cubic')

        self.grad_dir = np.arctan2(dy, dx)

    def get_gradient(self, xval=0.0, yval=0.0):
        if self.is_within_bounds(xval, yval):
            return self.grad_mag_interp(xval, yval)
        else:
            return 0

    def get_gradient_dir(self, xval=0.0, yval=0.0):
        xidx, yidx = self.idx_from_coords(xval, yval)
        return self.grad_dir[xidx, yidx]

    def _print_debug_init(self):
        print(
            '_x :: min:{}, max:{}, range:{}, res:{}, _shape:{}'.format(self._x[0], self._x[-1], self._xrange, self.res,
                                                                       self._x.shape))
        print(
            '_y :: min:{}, max:{}, range:{}, res:{}, _shape:{}'.format(self._y[0], self._y[-1], self._yrange, self.res,
                                                                       self._y.shape))
        print('h :: min:{}, max:{}, range:{}, res:{}, _shape:{}'.format(self.hmin, self.hmax, self._zrange, self.res,
                                                                        self.h.shape))
        print('\n')
        print('x :: {} to {} \tshape:{}'.format(self.x[0, 0], self.x[-1, 0], self.x.shape))
        print('y :: {} to {} \tshape:{}'.format(self.y[0, 0], self.y[0, -1], self.y.shape))

    def plot(self):
        print('Plotting terrain')
        fig = plt.figure()
        self.ax = fig.add_subplot(1, 1, 1, projection='3d')
        plt.title('Terrain')
        # self.ax = fig.add_subplot(111, projection='3d')

        # self.ax.plot_surface( self.x, self.y, self.h, cmap=cm.coolwarm )
        self.ax.plot_wireframe(self.x, self.y, self.h, color='gray')

        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.set_zlim(self.hmin, self.hmax)

        self.surface_plot = self.ax
        plt.draw()
        plt.xlabel('X')
        plt.ylabel('Y')

    def plot_grad(self):
        print('Plotting terrain gradient')
        fig = plt.figure()
        plt.title('Terrain gradient')

        self.ax = fig.add_subplot(111, projection='3d')
        self.ax.plot_surface(self.x, self.y, self.grad_mag)
        # self.ax.plot_wireframe( self.x, self.y, self.grad )

        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.set_zlim(self.hmin, self.hmax)

        self.gradient_plot = self.ax
        plt.draw()
        plt.xlabel('X')
        plt.ylabel('Y')
