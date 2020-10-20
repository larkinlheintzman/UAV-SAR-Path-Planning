import rtree
from scipy.spatial import KDTree, cKDTree

import torch
import pyro
import pyro.contrib.gp as gp

import numpy as np
from scipy import ndimage, misc, signal, interpolate

import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

from mrmh_model import params

from tqdm import tqdm, trange

import time
import pdb

class SearcherGP:
    def __init__(self, mc_handle):
        self.mc_handle = mc_handle
        self.searcherClass = mc_handle.searcherClass

        props = rtree.index.Property()
        props.dimension = 2
        self.idx = rtree.index.Index(properties=props)
        self.p_interp = mc_handle.p_interp
        
        self.sgpr = None
        self.init_GP()

    def p_at_posxy(self, pos_xy):
        return self.p_interp.ev(*pos_xy)

    def init_GP(self):
        kernel = gp.kernels.RBF(input_dim=2, variance=torch.tensor(5.),
                                lengthscale=torch.tensor(5.))

        X = [[], []] # search points
        Xu = [[], []] # inducing points
        y = [] # prior at search points

        for tsr in self.searcherClass.searchers_list:
            all_x, all_y, _, _ = tsr.smd_history
            X[0] += [ float(tx) for tx in all_x ]
            X[1] += [ float(ty) for ty in all_y ]
            y += [ float(self.p_at_posxy( (tx, ty) )) for tx, ty in zip(all_x, all_y) ]
            # y += [ 1 for tx in all_x ]

            gx, gy = tsr.sweep_guides.transpose().tolist()
            Xu[0] += gx
            Xu[1] += gy

        X = [*zip(*X)]
        Xu = [*zip(*Xu)]

        self.Xtrain = torch.FloatTensor(X)
        self.ytrain = torch.FloatTensor(y)
        self.ntrain = len(X)
        self.Xu = torch.FloatTensor(Xu)
        self.sgpr = gp.models.SparseGPRegression(self.Xtrain, self.ytrain, kernel, Xu=self.Xu, jitter=1.0e-5)

        optimizer = torch.optim.Adam(self.sgpr.parameters(), lr=0.005)
        loss_fn = pyro.infer.Trace_ELBO().differentiable_loss

        num_steps = 5
        for i in trange(num_steps):
            optimizer.zero_grad()
            loss = loss_fn(self.sgpr.model, self.sgpr.guide)
            loss.backward()
            optimizer.step()

    def gp_calc(self):
        pass

    def inference_grid(self):
        # _y = torch.linspace( self.mc_handle.ymin, self.mc_handle.ymax, 4*(self.mc_handle.ymax-self.mc_handle.ymin) )
        x = torch.as_tensor(self.mc_handle.terrain.x, dtype=torch.float).view(-1, 1)
        y = torch.as_tensor(self.mc_handle.terrain.y, dtype=torch.float).view(-1, 1)
        self.Xtest = torch.cat([x, y], 1)
        with torch.no_grad():
            self.ytest, self.test_cov = self.sgpr(self.Xtest)

    def visualize_gp(self):
        # if(self.searcherClass.terrain is not None and self.searcherClass.terrain.paths_plot is not None):
        #     fig = self.searcherClass.terrain.paths_plot
        #     plt.figure(fig.number)
        #     plt.title('Heatmap - Lost person | Searchers\' paths')
        # else:
        fig = plt.figure()
        self.ytest_np_plot = self.ytest.view(-1, self.mc_handle._y_shape).numpy()
        # ax = fig.gca(projection='3d')
        # # self.searcherClass.terrain.paths_plot = fig
        # ax.plot( xs=self.Xtest[:, 0].numpy(), ys=self.Xtest[:, 0].numpy(), zs=self.mc_handle.terrain.h.flatten() )
        extent=[self.mc_handle.xmin, self.mc_handle.xmax, self.mc_handle.ymin, self.mc_handle.ymax]
        plt.imshow( self.ytest_np_plot.transpose(), cmap='hot', interpolation='nearest', extent=extent, origin='lower')

    def visualize_training_data(self):
        self.search_traj = np.zeros_like(self.ytest_np_plot)
        for tsr in self.searcherClass.searchers_list:
            all_x, all_y, _, _ = tsr.smd_history
            for tx, ty in zip(all_x, all_y):
                tx_idx, ty_idx = self.mc_handle.terrain.idx_from_coords(tx, ty)
                self.search_traj[tx_idx, ty_idx] += 25

        fig = plt.figure()
        extent=[self.mc_handle.xmin, self.mc_handle.xmax, self.mc_handle.ymin, self.mc_handle.ymax]
        plt.imshow( self.search_traj.transpose(), cmap='hot', interpolation='nearest', extent=extent, origin='lower')

    def add_searcher_paths(self):
        """
        Adds searchers' paths from self.searcherClass into self.index
        for quick nearest neighbors lookup
        """
        for tsr in self.searcherClass.searchers_list:
            x, y, z, t = tsr.smd_history
            for tx, ty, tz, tt in zip(x, y, z, t):
                pos_xy = tuple([ tx, ty ])
                pos_xyzt = pos_xy + tuple([tz, tt])
                pdb.set_trace()
                self.idx.insert(0, pos_xy, pos_xyzt)

    def time_inference(self):
        x = torch.as_tensor(self.mc_handle.terrain.x, dtype=torch.float).view(-1, 1)
        y = torch.as_tensor(self.mc_handle.terrain.y, dtype=torch.float).view(-1, 1)
        self.Xtest = torch.cat([x, y], 1)

        stime = time.time()
        with torch.no_grad():
            self.ytest, self.test_cov = self.sgpr(self.Xtest)
        print( '{} points in one go took {} secs.'.format( self.Xtest.shape[0], time.time()-stime ) )

        stime = time.time()
        num_points = self.Xtest.size()[0]
        for i in trange(num_points):
            with torch.no_grad():
                ytest, test_cov = self.sgpr(self.Xtest[i:i+1, :])
        print( '{} points in a loop took {} secs.'.format( num_points, time.time()-stime ) )

    def nbors_in_square(self, pos, sq_side):
        """
        return neighbors within a square of side `sq_side`
        with `pos` at its center
        :param pos: tuple(2), position in 2D
        :param sq_side: float, side of square
        """
        cx, cy = pos
        half_s = float(sq_side) / 2

        # boundaries
        left = cx - half_s
        right = cx + half_s
        bottom = cy - half_s
        top = cy + half_s

        return list( self.idx.intersection((left, bottom, right, top), objects="raw") )

    def nearby(self, x, n):
        """
        Return nearby vertices
        :param x: tuple, vertex around which searching
        :param n: int, max number of neighbors to return
        :return: list of nearby vertices
        """
        return list( self.idx.nearest(x, num_results=n, objects="raw") )

    def nearest(self, x):
        """
        Return vertex nearest to x
        :param x: tuple, vertex around which searching
        :return: tuple, nearest vertex to x
        """
        return self.nearby(x, 1)[0]


class SearchNode(object):
    nextNodeIdx = 0
    def __init__(self, pos, p=0.5, t=0.0, cost=float('inf') ):
        """
        Init a node
        :param pos: np.array(3), xyz coordinates
        :param t: float, time since t0
        """
        self.index = SearchNode.nextNodeIdx
        self.pos = pos
        self.x, self.y, self.z = tuple(self.pos)
        self.p = p
        self.t = t
        self.hash_tuple = tuple(pos) + tuple([self.t])

        self.cost = cost

        SearchNode.nextNodeIdx += 1

    def __hash__(self):
        return hash(self.hash_tuple)

    def __eq__(self, other):
        return self.hash_tuple == other.hash_tuple

    def __ne__(self, other):
        return not(self==other)

def main():
    pass

if __name__ == "__main__":
    main()