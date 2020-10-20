import math
import random
import uuid

import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import hashlib, sys, os

import numpy as np
from scipy import ndimage, misc, signal, interpolate

from . import params as params


class Space(params.Default):
    def __init__(self, params):
        super().__init__(params)
        self.set_defaults()

        self.sanity_checks()

        self._x = np.linspace( self.xmin, self.xmax, self._xnum )
        self._y = np.linspace( self.ymin, self.ymax, self._ynum )
        self.y, self.x = np.meshgrid(self._y, self._x)

        self.sample_dim = 2

    def is_within_bounds(self, xval=0.0, yval=0.0 ):
        return (xval > self.xmin and xval < self.xmax and yval > self.ymin and yval < self.ymax)

    def idx_from_coords(self, xval=0.0, yval=0.0, saturate=True ):
        if(saturate):
            xval = max(self.xmin + self.res, xval)
            xval = min(self.xmax - self.res, xval)
            yval = max(self.ymin + self.res, yval)
            yval = min(self.ymax - self.res, yval)

        xidx = int( (xval - self.xmin) / float( self.res ) )
        yidx = int( (yval - self.ymin) / float( self.res ) )
        return (xidx, yidx)

    def sanity_checks(self):
        assert  self._xrange > 0 and \
                self._yrange > 0 and \
                self._zrange > 0, \
                    'Limits must be specified in the form (min,max)'

    def set_defaults(self):
        super().set_defaults()
        self.xmin, self.xmax = self.params['xlims']
        self.ymin, self.ymax = self.params['ylims']
        self.hmin, self.hmax = self.params['zlims']
        self.res = self.params['res']

        self._xrange = int(self.xmax - self.xmin)
        self._yrange = int(self.ymax - self.ymin)
        self._zrange = int(self.hmax - self.hmin)

        self._xnum = int(self._xrange/self.res)
        self._ynum = int(self._yrange/self.res)

        self.h = None
        self.h_smooth = None

        self.surface_plot = None
        self.gradient_plot = None
        self.paths_plot = None
