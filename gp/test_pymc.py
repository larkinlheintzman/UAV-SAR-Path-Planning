import sys, os, time
import json

import numpy as np
import pymc3 as pm

import theano
import theano.tensor as tt

import pdb

zero_func = pm.gp.mean.Zero()

ls_z = np.array([3.0]).astype('float64')

t = np.linspace(-5, 5, 20)
tpos = np.linspace(0, 5, 10)
x, y, z = np.meshgrid(t, t, tpos)
print(x.shape, y.shape, z.shape, t.shape)

X = np.vstack( (x.flatten(), y.flatten(), z.flatten()) ).transpose()
# X = np.asarray(X, dtype=float)
print(X.shape)

ls1 = 0.05
ls2 = 0.6
w = 0.3
def tanh_func(x, ls1, ls2, w):
    return tt.tanh(x)

def ls_func(x, ls_z):
    return theano.shared( np.asmatrix( np.array([ x[2], x[2], ls_z ]) ) ) #.astype('float64'))

# cov = pm.gp.cov.Gibbs(1, ls_func, ls_z)
cov = pm.gp.cov.Gibbs(3, tanh_func, (ls1, ls2, w))
K = cov(X).eval()

    # return np.array( [2]* np.shape(x)[-1] )

    # ls_vec = tt.ones((3))
    # ls_vec[0] = x[2]
    # ls_vec[1] = ls_vec[0]
    # ls_vec[2] = ls_z