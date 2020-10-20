import time

import numpy as np

import os
import _thread

import matplotlib.pyplot as plt

from larrt import planning

import torch
from torch.nn.utils.clip_grad import clip_grad_norm_

from bezier_interp import bezier, fitCurves

# import pyro
import pdb

np.set_printoptions(precision=4, threshold=5)
torch.set_printoptions(precision=4, threshold=5)
# torch.cuda.set_device(1)

# lsXY = 3.5e0;
# lsZ = 1.7e1;

class BezierGP(torch.nn.Module):
    def __init__(self, mc_handle, planner, meas_std=1e0, lsZ=1.7e1, _stime=None):
        self.mc_handle = mc_handle
        # self.p_interp = mc_handle.p_interp
        self.planner = planner

        self.meas_std = meas_std**2
        self.lsZ = lsZ
        self.lsXY_scaling = 2.0e0
        self.dim = 3

        self.update_mc_hotspots = True
        self.update_searcher_paths = True

        self.grad_clamp_value = 1e50
        self.grad_clamp = lambda grad: torch.clamp(grad, -self.grad_clamp_value, self.grad_clamp_value)
        # self.X_batch.tregister_hook(lambda grad: torch.clamp(grad, -self.grad_clamp, self.grad_clamp))

        self.init_Xstar()

        self.paths_plot = self.planner.space.terrain.paths_plot.number

        self.min_risk_cost = np.inf
        self._prev_min_risk_cost = np.inf
        self._prev_min_risk_paths = None
        self._curr_risk_ratio = 1.0

        self._init_risk_cost = 1
        self._init_scaled_risk_cost = 1
        self._init_scaled_path_length_cost = 1
        self._min_scaled_risk_cost = 1
        self._min_scaled_path_length_cost = 1

        self.scaled_risk_cost = 1
        self.scaled_path_length_cost = 1

        self._iter = 0
        self._updates = 0
        self._iter_since_update = 0
        self._iters_cont_update = 0

        self._max_iter = 1000
        self._reset_on_update = True

        self.all_robot_paths = []
        self.min_risk_paths = None

        self.minibatch_test_fraction = 0.9 #1.0
        self.minibatch_train_fraction = 0.7 #1.0

        self._init_riskcost_scaling = 1e0 #1e7
        self._init_lengthcost_scaling = 1e0 #1e7

        self.riskcost_scaling = self._init_riskcost_scaling
        self.lengthcost_scaling = self._init_lengthcost_scaling

        self._stime = _stime if _stime is not None else time.time()

    def _hook(self, tensor, _min=-1e10, _max=1e10):
        tensor.requires_grad_(True)
        # tensor.retain_grad()
        tensor.register_hook(lambda grad: grad.clamp_(min=_min, max=_max))
        # tensor.register_hook(lambda grad: grad.clamp_(-self.grad_clamp_value, self.grad_clamp_value))
        return tensor

    def init_Xstar(self):
        stime = time.time()
        x = torch.as_tensor(self.mc_handle.terrain.x, dtype=torch.float64).view(-1, 1)
        y = torch.as_tensor(self.mc_handle.terrain.y, dtype=torch.float64).view(-1, 1)
        z = torch.as_tensor(self.mc_handle.terrain.h, dtype=torch.float64).view(-1, 1)
        _Xstar_raw = torch.cat([x, y, z], 1) #[nTest, 3]

        self.Xstar = _arrange_zorder(_Xstar_raw, self.mc_handle._x_shape, self.mc_handle._y_shape).cuda() #Z-order sort
        self.num_test = self.Xstar.shape[0]
        print( 'Initializing Xstar took {} secs.'.format( time.time()-stime ) )

    def forward(self, x):
        self.eval_trainData()
        self.compute_risk_metric()
        return 0

    def collect_trainData(self, _sweep_paths=True):
        # stime = time.time()

        self.collect_mc_trainData()
        self.collect_searchers_trainData()

        if _sweep_paths:
            self.planner.robotClass.make_sweep_paths(sweep_num = self.params['drone_sweeps'], interp_res=self.params['path_interp_res'])
            self.robot_fixed_paths, self.robot_raw_paths, self.robot_path_len = self.planner.robotClass.get_sweep_paths_array()
            self._sweep_paths = True
            self._bez_error = 50
        else:
            self.planner.plan()
            self.robot_fixed_paths, self.robot_raw_paths, self.robot_path_len = self.planner.get_paths_array(interp_res=self.params['path_interp_res'])
            self._sweep_paths = False
            self._bez_error = 25

        self.robot_ctrlpoints_np, self.bezier_lengths = _fit_beziers(self.robot_raw_paths, self.robot_path_len, error=self._bez_error, bgp=self)

        self.robot_fixed_path_values = self.get_default_values(self.robot_fixed_paths)
        self.num_robot_fixed_paths = self.robot_fixed_paths.shape[0]

        # self.robot_ctrlpoints_xy = torch.from_numpy(self.robot_ctrlpoints_np[:, :2]).view(-1, 2).cuda()
        # self.robot_ctrlpoints_xy = self._hook(self.robot_ctrlpoints_xy)

        # self.robot_ctrlpoints_z = torch.from_numpy(self.robot_ctrlpoints_np[:, -1]).view(-1, 1).cuda()
        # self.robot_ctrlpoints_z = self._hook(self.robot_ctrlpoints_z)

        # self.robot_ctrlpoints = torch.cat((self.robot_ctrlpoints_xy, self.robot_ctrlpoints_z), dim=-1).view(-1, 3).cuda()
        # self.robot_ctrlpoints = self._hook(self.robot_ctrlpoints)


        self.robot_ctrlpoints = torch.from_numpy(self.robot_ctrlpoints_np).view(-1, 3).cuda()
        self.robot_ctrlpoints = self._hook(self.robot_ctrlpoints)

        self.compute_bezier_interp()

        # self.points = np.concatenate( (self.mc_hotspots, self.searchers_paths, self.robot_paths), axis=0 )
        # self.values = np.concatenate( (self.mc_hotspot_values, self.searchers_path_values, self.robot_path_values), axis=0 )

        if self.update_mc_hotspots or self.update_searcher_paths:
            self.fixed_points = np.concatenate( (self.mc_hotspots, self.searchers_paths, self.robot_fixed_paths), axis=0 )
            self.fixed_values = np.concatenate( (self.mc_hotspot_values, self.searchers_path_values, self.robot_fixed_path_values), axis=0 )
            self.num_fixed_train_points = self.fixed_points.shape[0]

            self.fixed_points = torch.from_numpy(self.fixed_points).view(-1, 3).cuda()
            self.fixed_values = torch.from_numpy(self.fixed_values).view(-1).cuda()

        self.robot_values = torch.from_numpy(self.robot_path_values).view(-1).cuda()
        self.robot_fixed_paths = torch.from_numpy(self.robot_fixed_paths).view(-1, 3).cuda()

        self.num_train = self.fixed_points.shape[0] + self.robot_points.shape[0]

        self.update_trainXY()

    def compute_bezier_interp(self):
        self.robot_points, self.interp_len, self.bez_fix_pts, self.bez_fix_grads = _interp_bezier(self.robot_ctrlpoints, self.bezier_lengths, _res=0.05)

        self.robot_points = self._hook(self.robot_points) #, _min=-1e4, _max=1e4)
        self.bez_fix_pts = self._hook(self.bez_fix_pts)
        self.bez_fix_grads = self._hook(self.bez_fix_grads)

        with torch.no_grad():
            self.robot_path_values = self.get_default_values(self.robot_points.data.cpu().numpy())
            self.robot_values = torch.from_numpy(self.robot_path_values).view(-1).cuda()
            self.num_robot_paths = self.robot_points.shape[0]

    def update_Xstar(self, _minibatch=True):
        
        if _minibatch:
            self.minibatch_test_mask = torch.cuda.FloatTensor(self.num_test).uniform_() > (1-self.minibatch_test_fraction)
            self.Xstar_batch = self.Xstar[self.minibatch_test_mask, :]
        else:
            self.Xstar_batch = self.Xstar

        # self.minibatch_test_mask = torch.cuda.FloatTensor(self.num_train).uniform_() > (1-self.minibatch_test_fraction)
        # self.Xstar_batch = self.X[self.minibatch_test_mask, :]

        # Co-variance between test points
        self.Kxstar_xstar = self.kernel_Gibbs_robots(self.Xstar_batch) #* _torch_pentadiag_mask(self.Xstar_batch.shape[0]).cuda()

    def update_trainXY(self, _minibatch=True):
        # self.robot_points, self.interp_len = _interp_bezier(self.robot_ctrlpoints, self.bezier_lengths, _res=0.05)
        # self.robot_points = self._hook(self.robot_points, _min=-1e3, _max=1e3)

        self.X = torch.cat((self.fixed_points, self.robot_points), dim=0).cuda()
        self.ytrain = torch.cat((self.fixed_values, self.robot_values), dim=0).cuda()

        self.minibatch_train_mask = torch.cuda.FloatTensor(self.num_train).uniform_() > (1-self.minibatch_train_fraction)

        if _minibatch:
            self.X_batch = self.X[self.minibatch_train_mask, :]
            self.ytrain_batch = self.ytrain[self.minibatch_train_mask]
        else:
            self.X_batch = self.X
            self.ytrain_batch = self.ytrain

        self.X_batch = self._hook(self.X_batch)
        self.ytrain_batch = self._hook(self.ytrain_batch)

    def collect_mc_trainData(self):
        """Collect training data from the probabilistic heatmap
        generated using MonteCarlo simulation of the lost person model
        """
        # if not self.mc_handle.updated_p:
        #     self.update_mc_hotspots = False
        #     return
        self.update_mc_hotspots = True
        
        hotspots_xy = self.mc_handle.get_hotspots(threshold=0.1)
        
        mc_altitude = self.lsZ * 1.5
        hotspots_z = mc_altitude + self.get_altitude_values(hotspots_xy)

        self.mc_hotspots = np.hstack([hotspots_xy, hotspots_z])
        self.mc_hotspot_values = self.get_default_values(self.mc_hotspots)
        self.num_mc_hotspots = self.mc_hotspots.shape[0]

    def collect_searchers_trainData(self):
        if not self.mc_handle.searcher_class.updated_searcher_paths:
            self.update_searcher_paths = False
            return
        self.update_searcher_paths = True

        X = [[], []] # search points
        y = [] # prior at search points

        for tsr in self.mc_handle.searcher_class.searchers_list:
            all_x, all_y, all_z, _ = tsr.smd_history
            X[0] += all_x
            X[1] += all_y
            # X[2] += all_z
        X = [*zip(*X)]
        searchers_paths_xy = np.array(X, dtype=np.float64)

        self.searcher_altitude = 0.5
        searchers_paths_z = self.searcher_altitude + self.get_altitude_values(searchers_paths_xy)
        
        self.searchers_paths = np.hstack([searchers_paths_xy, searchers_paths_z])

        self.searchers_path_values = self.get_default_values(self.searchers_paths)
        self.num_searchers_paths = self.searchers_paths.shape[0]
        self.mc_handle.searcher_class.updated_searcher_paths = False

    def eval_trainData(self, eps=1e-8):
        stime = time.time()

        self.update_Kxx_and_inv()

        # Cross-covariance between training and test points
        self.Kx_xstar = self.kernel_Gibbs_robots(self.X_batch, self.Xstar_batch)
        self.Kx_xstar = self._hook(self.Kx_xstar)

        Kx_xstar_T = self.Kx_xstar.t()
        Kx_xstar_T = self._hook(Kx_xstar_T)

        # self.v = torch.mm(self.Linv, self.Kx_xstar)
        # self.v = self._hook(self.v)

        self.K_ratio = torch.matmul( Kx_xstar_T, self.Kxx_inv )
        self.K_ratio = self._hook(self.K_ratio)

        # K_tt'/K_train*y_train;
        self.ystar_mu = torch.matmul( self.K_ratio, self.ytrain_batch )
        self.ystar_mu = self._hook(self.ystar_mu)

        # K_test - K_tt'/K_train*K_tt;
        self.ystar_cov = self.Kxstar_xstar - torch.matmul( self.K_ratio, self.Kx_xstar )
        self.ystar_cov = self._hook(self.ystar_cov)

        # self.ystar_mu = torch.mv(self.Kx_xstar.t(), self.alpha)
        # self.ystar_mu = self._hook(self.ystar_mu)

        # _ystar_cov_raw = self.Kxstar_xstar - torch.mm(self.v.t(), self.v)
        # self.ystar_cov = _ystar_cov_raw.clamp(min=0)
        # self.ystar_cov = self._hook(self.ystar_cov)

        # print( 'Evaluating GP using Xtrain took {} secs.'.format( time.time()-stime ) )

    def compute_risk_cost(self):
        cov_trace_scaling = 1e7 #1e5
        mean_scaling = 1e0
        mean_sq_scaling = 1e0

        ystar_mu_sq = ( self.ystar_mu*mean_scaling ) **2
        ystar_mu_sq = self._hook(ystar_mu_sq)

        self.record_init_params(cov_trace_scaling, mean_scaling, mean_sq_scaling)
        stime = time.time()

        # ystar_noise = self.ystar_cov
        # omega = ( (torch.diag(torch.cholesky( ystar_noise ))) * cov_trace_scaling )  / (1e-5 +ystar_mu_sq*mean_sq_scaling)
        
        self.omega = ( torch.diag(self.ystar_cov)*cov_trace_scaling ) / (1 + ystar_mu_sq*mean_sq_scaling)
        # self.omega = (torch.diag(self.ystar_cov)*cov_trace_scaling) - (ystar_mu_sq*mean_sq_scaling)
        self.omega = self._hook(self.omega)

        self.update_path_len_bezcost()
        self.update_bezier_cont_cost()

        self.scaled_risk_cost = self.omega.sum() * self.riskcost_scaling
        self.scaled_path_length_cost = self.lengthcost_scaling * self.path_len_cost
        self.risk_cost = self.scaled_risk_cost + self.path_fix_cost #+ self.scaled_path_length_cost #+ self.bez_smooth_cost
        self.risk_cost = self._hook(self.risk_cost)

    def optimize_risk_cost_grad(self, _show_detail = False):
        # self.optimizer = torch.optim.SGD({self.robot_ctrlpoints}, lr=5e-6)
        # self.optimizer = torch.optim.Adam({self.robot_ctrlpoints}, lr=5e-2)
        self.optimizer = torch.optim.Adam({self.robot_ctrlpoints}, lr=5e-3)

        self.update_Xstar()

        a_list = []
        _thread.start_new_thread(_input_thread, (a_list,))

        self._iter = 0
        print('\nRisk cost function optimization')

        while self._iter_since_update < self._max_iter:
            stime = time.time()
            self.optimizer.zero_grad()

            # with torch.autograd.detect_anomaly():
            self.compute_bezier_interp()

            # self.update_Xstar()
            self.update_trainXY()

            self.eval_trainData()

            self.compute_risk_cost()

            self.risk_cost.backward(retain_graph=True)

            # no_nans_p_grad = _find_nans(self.robot_ctrlpoints.grad).shape[0] == 0
            # if not no_nans_p_grad:
            #     pdb.set_trace()

            self.optimizer.step()

            # self.update_Xstar(_minibatch=False)
            # self.compute_bezier_interp()
            # self.update_trainXY(_minibatch=False)
            # self.compute_risk_cost()

            self.collect_robot_paths(_disp_grad_stats=True)

            _step_time = time.time()-stime

            if _show_detail:
                self.printStr = ''
                _time_taken = time.time() - self._stime
                self.printStr += 'I#{:05d} - U# {:05d}\t| {:0.3e}\t:: T {:10.3f} s\t| {:07.3f} it/s'.format(self._iter, self._updates, self.risk_cost_data - self._prev_min_risk_cost, _time_taken, self._iter/_time_taken)
                self.printStr += '\ninitCost:\t{:0.3e}\t = risk: {:0.3e}\t + len: {:0.3e}'.format(self._init_risk_cost, self._init_scaled_risk_cost, self._init_scaled_path_length_cost)

                self.printStr += '\ncurCost:\t{:0.3e}\t = risk: {:0.3e}\t + len: {:0.3e}'.format(self.risk_cost_data, self.scaled_risk_cost_data, self.scaled_path_length_cost_data)

                self.curr_path_diff = self.robot_points_data - self._init_paths
                self.curr_path_diff_abs = np.abs(self.curr_path_diff).sum(axis=-1)
                self.printStr += '\ncur Delta::\tmean: {:010.3f} | std: {:010.3f} | max: {:010.3f} | min: {:010.3f}'.format( self.curr_path_diff_abs.mean(), self.curr_path_diff.std(), self.curr_path_diff_abs.max(), self.curr_path_diff_abs.min() )

                self.printStr += '\nminCost:\t{:0.3e}\t = risk: {:0.3e}\t + len: {:0.3e}'.format(self.min_risk_cost, self._min_scaled_risk_cost, self._min_scaled_path_length_cost)
                self.min_path_diff = self.min_risk_paths - self._init_paths
                self.min_path_diff_abs = np.abs(self.min_path_diff).sum(axis=-1)
                self.printStr += '\nmin Delta::\tmean: {:010.3f} | std: {:010.3f} | max: {:010.3f} | min: {:010.3f}'.format( self.min_path_diff_abs.mean(), self.min_path_diff.std(), self.min_path_diff_abs.max(), self.min_path_diff_abs.min() )

                self.printStr += '\nIter since: {:05d}/{:05d} | {:05d} || {:0.3e} | {:0.3e}'.format(self._iter_since_update, self._max_iter, self._iters_cont_update, self.riskcost_scaling, self.lengthcost_scaling)

                
                self.printStr +=  '\nOptimizer step took {:09.6f} secs.\n'.format( _step_time )
                print(self.printStr)
            else:
                self.curr_path_diff = self.robot_points_data - self._init_paths
                self.curr_path_diff_abs = np.abs(self.curr_path_diff).sum(axis=-1)

                self.min_path_diff = self.min_risk_paths - self._init_paths
                self.min_path_diff_abs = np.abs(self.min_path_diff).sum(axis=-1)

                self.printStr = 'T {:09.3f} I {:05d} U {:05d} || {:05d}/{:05d} | {:03d} || C {:.3e} | {:0.3e} || D {:0.3e} | {:0.3e} || {:05.3f} s/it    '.format(time.time() - self._stime, self._iter, self._updates, self._iter_since_update, \
                            self._max_iter, self._iters_cont_update, \
                            self.risk_cost_data - self.min_risk_cost, \
                            self._init_risk_cost - self.min_risk_cost, \
                            self.curr_path_diff_abs.mean(), \
                            self.min_path_diff_abs.mean(), \
                            _step_time )

                print(self.printStr, end='\r')
                pass

            if a_list:
                if a_list.pop():
                    break
                else:
                    self.plot_all_robot_paths(_update_plot=True, _in_3d=False)
                    plt.pause(1.0)

            self._iter += 1

    def update_path_len_bezcost(self):
        self.fixed_point_dist_scaling = 1e12
        self.other_dist_scaling = 1e0

        start_ix = 0
        path_len_cost = []
        path_fix_cost = []
        for tix, tlen in enumerate(self.bezier_lengths):
            end_ix = start_ix + tlen
            fixed_point_ix = - self.num_robot_fixed_paths + 2*tix
            path_fix_cost.append( ((self.robot_fixed_paths[fixed_point_ix, :] - self.robot_ctrlpoints[start_ix, :])**2).sum() * self.fixed_point_dist_scaling )
            path_fix_cost.append( ((self.robot_ctrlpoints[end_ix-1, :] - self.robot_fixed_paths[fixed_point_ix+1, :])**2).sum() * self.fixed_point_dist_scaling )
            path_len_cost.append( bezier.qint_multi(self.robot_ctrlpoints[start_ix:end_ix, :]) * self.other_dist_scaling )
            start_ix = end_ix

        self.path_fix_cost = sum(path_fix_cost).cuda()
        self.path_fix_cost = self._hook(self.path_fix_cost)

        self.path_len_cost = sum(path_len_cost).cuda()
        self.path_len_cost = self._hook(self.path_len_cost)

    def collect_robot_paths(self, _disp_grad_stats=False):
        with torch.no_grad():
            self.risk_cost_data = self.risk_cost.data.cpu().numpy()
            self.robot_points_data = self.robot_points.data.cpu().numpy().copy()
            self.scaled_risk_cost_data = self.scaled_risk_cost.data.cpu().numpy()
            self.scaled_path_length_cost_data = self.scaled_path_length_cost.data.cpu().numpy()

            self._alt_scaled_cost = self.scaled_risk_cost_data
            # self._alt_scaled_cost = 1e0 * self.scaled_risk_cost_data + 1e-4 * self.scaled_path_length_cost_data

            self.all_robot_paths.append(self.robot_points_data)
            # redo_plot = False

            if self._iter < 1:
                self._init_risk_cost = self.risk_cost_data
                self._init_paths = self.robot_points_data

                self._init_scaled_risk_cost = self.scaled_risk_cost_data
                self._init_scaled_path_length_cost = self.scaled_path_length_cost_data

                self._init_alt_scaled_cost = self._alt_scaled_cost

                self.min_risk_cost = self.risk_cost_data
                self.min_risk_paths = self.robot_points_data

                self._min_scaled_risk_cost = self.scaled_risk_cost_data
                self._min_scaled_path_length_cost = self.scaled_path_length_cost_data

                self._min_alt_scaled_cost = self._alt_scaled_cost

                self._curr_risk_ratio = self.min_risk_cost/self._init_risk_cost

            self._iter_since_update += 1

            try:
                if self._alt_scaled_cost < self._min_alt_scaled_cost:

                    self._prev_min_risk_cost = self.min_risk_cost
                    self._prev_min_risk_paths = self.min_risk_paths
                    self.min_risk_cost = self.risk_cost_data
                    self.min_risk_paths = self.robot_points_data

                    self._min_scaled_risk_cost = self.scaled_risk_cost_data
                    self._min_scaled_path_length_cost = self.scaled_path_length_cost_data
                    self._min_alt_scaled_cost = self._alt_scaled_cost

                    self._curr_risk_ratio = self.min_risk_cost/self._init_risk_cost
                    # redo_plot = True

                    # if (self._prev_min_risk_cost - self.min_risk_cost) / self.min_risk_cost > 1e-3:
                    self._updates += 1
                    self._iters_cont_update += 1
                    # if self._reset_on_update:
                    self._iter_since_update = 0

                else:
                    self._iters_cont_update = 0
            except FloatingPointError as e:
                self._iters_cont_update = 0

    def update_bezier_cont_cost(self):
        _num = self.bez_fix_pts.shape[0]
        _ixs = torch.arange(0, _num, 2)

        _c0_cost = ((self.bez_fix_pts[_ixs+1] - self.bez_fix_pts[_ixs])**2).sum()
        _c1_cost = ((self.bez_fix_grads[_ixs+1] - self.bez_fix_grads[_ixs])**2).sum()

        self.bez_cont_cost = self.fixed_point_dist_scaling * _c0_cost
        self.bez_smooth_cost = self.fixed_point_dist_scaling * _c1_cost

    def plot_all_robot_paths(self, _update_plot=False, _block=False, _in_3d=False):
        if not _update_plot:
            if not _in_3d:
                fig = plt.figure(self.paths_plot)
                # plt.clf()
                # for t_rpaths in self.all_robot_paths:
                #     self._plot_robot_paths(t_rpaths)
                self._plot_robot_paths(self.all_robot_paths[0], 'yellow')
                self._plot_robot_paths(self.all_robot_paths[-1], tcolor='magenta')
                self._plot_robot_paths(self.min_risk_paths, tcolor='red')
            else:
                # if self.mc_handle.terrain.surface_plot is None:
                fig = plt.figure()
                self.paths_plot_3d_fig = fig
                ax_3d = fig.gca(projection='3d')
                self.mc_handle.terrain.surface_plot = ax_3d
                ax_3d.set_xlim(self.mc_handle.terrain.xmin, self.mc_handle.terrain.xmax)
                ax_3d.set_ylim(self.mc_handle.terrain.ymin, self.mc_handle.terrain.ymax)
                ax_3d.set_zlim(self.mc_handle.terrain.hmin, self.mc_handle.terrain.hmax)
                # else:
                #     ax_3d = self.mc_handle.terrain.surface_plot
                self._plot_robot_paths_3d(self.all_robot_paths[0], ax_3d)
                self._plot_robot_paths_3d(self.all_robot_paths[-1], ax_3d, tcolor='magenta')
                self._plot_robot_paths_3d(self.min_risk_paths, ax_3d, tcolor='red')
        else:
            fig = plt.figure(self.paths_plot)
            self._plot_robot_paths(self.min_risk_paths, tcolor='cyan')
            self._plot_robot_paths(self.robot_points_data)
        plt.draw()
        return plt.gcf()

    def _plot_robot_paths(self, rpaths, tcolor='yellow'):
        start_ix = 0
        for tlen in self.interp_len:
            end_ix = start_ix + tlen
            tpath = rpaths[start_ix:end_ix]
            plt.plot(tpath[:, 0], tpath[:, 1], tcolor)
            start_ix = end_ix

    def _plot_robot_paths_3d(self, rpaths, axis3d, tcolor='yellow'):
        start_ix = 0
        for tlen in self.interp_len:
            end_ix = start_ix + tlen
            tpath = rpaths[start_ix:end_ix]
            axis3d.plot( xs=tpath[:, 0], ys=tpath[:, 1], zs=tpath[:, 2], color=tcolor)
            start_ix = end_ix

    def update_path_len_cost(self):
        self.fixed_point_dist_scaling = 1e7
        self.other_dist_scaling = 1e0 #1e0

        start_ix = 0
        path_len_cost = []
        path_fix_cost = []
        for tix, tlen in enumerate(self.interp_len):
            end_ix = start_ix + tlen
            fixed_point_ix = - self.num_robot_fixed_paths + 2*tix
            path_fix_cost.append( ((self.robot_fixed_paths[fixed_point_ix, :] - self.robot_points[start_ix, :])**2).sum() * self.fixed_point_dist_scaling )
            path_len_cost.append( ((self.robot_points[start_ix:end_ix-1, :] - self.robot_points[start_ix+1:end_ix, :])**2).sum() * self.other_dist_scaling )
            path_fix_cost.append( ((self.robot_points[end_ix-1, :] - self.robot_fixed_paths[fixed_point_ix+1, :])**2).sum() * self.fixed_point_dist_scaling )
            start_ix = end_ix

        self.path_fix_cost = sum(path_fix_cost).cuda()
        self.path_fix_cost = self._hook(self.path_fix_cost)

        self.path_len_cost = sum(path_len_cost).cuda()
        self.path_len_cost = self._hook(self.path_len_cost)

    def update_Kxx(self):

        if self._iter<1:
            # self.Kxx_mask = torch.eye
            pass

        self.noise = torch.diag( torch.ones_like(self.ytrain_batch).view(-1) ) * self.meas_std**2

        # Co-variance between training points
        self.Kxx = (self.kernel_Gibbs_robots(self.X_batch) + self.noise) * _torch_pentadiag_mask(self.ytrain_batch.shape[0]).cuda()
        self.Kxx = self._hook(self.Kxx)

        # self.L = torch.cholesky(self.Kxx, False)
        # self.L = self._hook(self.L)

        # self.Linv = self.L.inverse()
        # self.Linv = self._hook(self.Linv)

        # TODO: Update only robot paths, but handle changes in array size
        if False:
            if self.update_mc_hotspots or self.update_searcher_paths:
                self.Kxx = self.kernel_Gibbs_robots(self.X_batch)
            else:
                self.Kxx = torch.zeros(self.num_fixed_train_points+self.num_robot_paths, self.num_fixed_train_points+self.num_robot_paths)
                self.Kxx[-self.num_robot_paths:, :self.num_fixed_train_points]

    def inc_update_Kxx(self):
        nonfixed_noise = torch.diag( torch.randn_like(self.robot_values).view(-1) ) * self.meas_std**2

        # K -> robot|robot
        Krobot = self.kernel_Gibbs_robots(self.robot_points) + nonfixed_noise

        # K -> all-fixed|robot
        K_allfixed_robot = self.kernel_Gibbs_robots(self.fixed_points, self.robot_points)

        self.Kxx[-self.num_robot_paths:, -self.num_robot_paths:] = Krobot
        self.Kxx[:self.num_fixed_train_points, -self.num_robot_paths:] = K_allfixed_robot
        self.Kxx[-self.num_robot_paths:, :self.num_fixed_train_points] = K_allfixed_robot.t()

        self.Kxx = self._hook(self.Kxx)

    def update_Kxx_and_inv(self):
        self.update_Kxx()

        self.Kxx_inv = torch.inverse(self.Kxx)
        self.Kxx_inv = self._hook(self.Kxx_inv)

        self.alpha = torch.mv(self.Kxx_inv, self.ytrain_batch)
        self.alpha = self._hook(self.alpha)

    def inc_update_Kxx_inv(self):
        if self.update_mc_hotspots or self.update_searcher_paths:
            self.Kxx_inv = torch.zeros_like(self.Kxx)
            
            self.num_fixed_train_points = self.num_mc_hotspots + self.num_searchers_paths

            Ainv = self.K_fixed_train_inv = torch.inverse(self.Kxx[:self.num_fixed_train_points, :self.num_fixed_train_points])
            
            B = self.Kxx[:self.num_fixed_train_points, -self.num_robot_paths:]
            C = self.Kxx[-self.num_robot_paths:, -self.num_robot_paths:]

            M = C - torch.chain_matmul( B.t(), Ainv, B )

            # ~A
            self.Kxx_inv[:self.num_fixed_train_points, :self.num_fixed_train_points] = Ainv - torch.chain_matmul(Ainv, B, M, B.t(), Ainv)
            # ~B
            self.Kxx_inv[:self.num_fixed_train_points, -self.num_robot_paths:] = -torch.chain_matmul(Ainv, B, M)
            # ~C
            self.Kxx_inv[-self.num_robot_paths:, :self.num_fixed_train_points] = -torch.chain_matmul(M, B.t(), Ainv)
            # ~D
            self.Kxx_inv[-self.num_robot_paths:, -self.num_robot_paths:] = M

        else:
            Ainv = self.K_fixed_train_inv
            B = self.Kxx[:self.num_fixed_train_points, -self.num_robot_paths:]
            C = self.Kxx[-self.num_robot_paths:, -self.num_robot_paths:]
            
            M = C - torch.chain_matmul( B.t(), Ainv, B )
            # ~A
            self.Kxx_inv[:self.num_fixed_train_points, :self.num_fixed_train_points] = Ainv - torch.chain_matmul(Ainv, B, M, B.t(), Ainv)
            # ~B
            self.Kxx_inv[:self.num_fixed_train_points, -self.num_robot_paths:] = -torch.chain_matmul(Ainv, B, M)
            # ~C
            self.Kxx_inv[-self.num_robot_paths:, :self.num_fixed_train_points] = -torch.chain_matmul(M, B.t(), Ainv)
            # ~D
            self.Kxx_inv[-self.num_robot_paths:, -self.num_robot_paths:] = M

    def kernel_Gibbs_robots(self, matA, matB=None, eps=1e-8):
        if matB is None:
            matB = matA

        r2 = self.pdistsq_dimwise(matB, matA)
        rA = self.lsGibbs_robots(matA)
        rB = self.lsGibbs_robots(matB)

        rA2 = (rA**2).reshape(-1, 1, self.dim)
        rB2 = (rB**2).reshape(1, -1, self.dim)
        rA2_plus_rB2 = rA2 + rB2 + eps
        rA2_plus_rB2_mul_dim = torch.prod(rA2_plus_rB2, dim=-1) + eps

        ret_part_1_sqrt = _torch_sqrt( torch.pow( (2.0 * rA.matmul(rB.t())) / rA2_plus_rB2_mul_dim, self.dim ) )
        ret_part_1_sqrt = self._hook(ret_part_1_sqrt)

        ret_part_2_1_sum = torch.sum(r2 / rA2_plus_rB2, dim=-1)
        ret_part_2_1_sum = self._hook(ret_part_2_1_sum)

        ret_part_2_invexp = _torch_inv_exp(ret_part_2_1_sum)
        ret_part_2_invexp = self._hook(ret_part_2_invexp)

        return (ret_part_1_sqrt *  ret_part_2_invexp)

    def kernel_RBF(self, matA, matB):
        raise NotImplementedError

    def lsGibbs_robots(self, mat):
        with torch.no_grad():
            lsX = lsY = (1+ mat[:, 2].view(-1, 1)) * self.lsXY_scaling
            # lsX = lsY = torch.ones_like(mat[:, 2].view(-1, 1)) * self.lsZ
            lsZ = torch.ones_like(lsX) * self.lsZ
            return torch.cat( (lsX, lsY, lsZ), 1)

    def pdistsq_dimwise(self, matA, matB):
        dim = 3

        matA = matA.view(1, -1, dim)
        matB = matB.view(1, -1, dim).transpose(1, 0)
        
        nA = matA.shape[1]
        nB = matB.shape[0]

        return (matA.repeat(nB, 1, 1) - matB.repeat(1, nA, 1))**2

    def get_altitude_values(self, points_xy):
        points_z = np.zeros_like(points_xy[:, 0:1], dtype=np.float64)
        for ix in range(points_z.shape[0]):
            points_z[ix] = self.mc_handle.terrain.h_smooth.ev( points_xy[ix, 0], points_xy[ix, 1] )
        return points_z

    def get_default_values(self, points):
        values = np.zeros_like(points[:, 0:1], dtype=np.float64)
        for ix in range(points.shape[0]):
            values[ix] = self.mc_handle.p_interp(points[ix, 0], points[ix, 1])
        return values

    def invert_Kxx(self):
        if self.update_mc_hotspots:
            self.Kxx_inv = torch.inverse(self.Kxx)
            num_mc_hotspots = self.mc_hotspots.shape[0]
            
        elif self.update_searcher_paths:
            num_mc_hotspots = self.mc_hotspots.shape[0]
            self.Kmc_hotspots_inv = torch.inverse(self.Kxx[:num_mc_hotspots, :num_mc_hotspots])

    def record_init_params(self, cov_trace_scaling, mean_scaling, mean_sq_scaling):
        params = self.mc_handle.params
        _gp = 'GP_init_'
        params.update({\
            _gp+ 'cov_trace_scaling': cov_trace_scaling, \
            _gp+ 'mean_scaling': mean_scaling, \
            _gp+ 'mean_sq_scaling': mean_sq_scaling, \
            _gp+ 'minibatch_test_fraction': self.minibatch_test_fraction, \
            _gp+ 'minibatch_train_fraction': self.minibatch_train_fraction, \
            _gp+ 'grad_clamp_value': self.grad_clamp_value, \
            _gp+ '_max_iter': self._max_iter, \
            _gp+ 'dummy': 1
            })
        self.mc_handle.write_params()

    def record_final_params(self):
        params = self.mc_handle.params
        _gp = 'GP_final_'
        params.update({\
            _gp+ '_min_alt_scaled_cost': float(self._min_alt_scaled_cost), \
            _gp+ '_alt_scaled_cost': float(self._alt_scaled_cost), \
            _gp+ '_updates': self._updates, \
            _gp+ '_iters_cont_update': self._iters_cont_update, \
            _gp+ '_iter_since_update': self._iter_since_update, \
            _gp+ 'fixed_point_dist_scaling': float(self.fixed_point_dist_scaling), \
            _gp+ 'other_dist_scaling': float(self.other_dist_scaling), \
            _gp+ 'lengthcost_scaling': float(self.lengthcost_scaling), \
            _gp+ 'printStr': self.printStr, \
            _gp+ 'dummy': 1
            })
        params.update({\
            _gp+ 'min_path_diff_abs_mean': float(self.min_path_diff_abs.mean()), \
            _gp+ 'min_path_diff_abs_std': float(self.min_path_diff_abs.std()), \
            _gp+ 'min_path_diff_abs_min': float(self.min_path_diff_abs.min()), \
            _gp+ 'min_path_diff_abs_max': float(self.min_path_diff_abs.max()), \
            _gp+ 'dummy': 1
            })
        params.update({\
            _gp+ '_prev_min_risk_cost': float(self._prev_min_risk_cost), \
            _gp+ 'min_risk_cost': float(self.min_risk_cost), \
            _gp+ '_min_scaled_risk_cost': float(self._min_scaled_risk_cost), \
            _gp+ 'scaled_risk_cost_data': float(self.scaled_risk_cost_data), \
            _gp+ '_min_scaled_path_length_cost': float(self._min_scaled_path_length_cost), \
            _gp+ 'scaled_path_length_cost_data': float(self.scaled_path_length_cost_data), \
            _gp+ 'dummy': 1
            })
        self.mc_handle.write_params()

    def eval_manual_mode(self):
        self.minibatch_test_fraction = 1.1
        # self.update_Xstar()
        self.manual_altitude = self.lsZ

        self.manual_robot_paths = self.searchers_paths.copy()
        self.manual_robot_paths[:, 2] += (self.manual_altitude - self.searcher_altitude)
        self.manual_robot_paths = torch.from_numpy(self.manual_robot_paths).view(-1, 3).cuda()

        self.manual_robot_path_values = torch.from_numpy(self.searchers_path_values).view(-1).cuda()

        self.X = torch.cat((self.fixed_points, self.manual_robot_paths), dim=0).cuda()
        self.ytrain = torch.cat((self.fixed_values, self.manual_robot_path_values), dim=0).cuda()

        self.X_batch = self.X
        self.ytrain_batch = self.ytrain

        self.eval_trainData()
        self.compute_risk_cost()

        self.manual_risk = self.risk_cost.data.cpu().numpy()
        print('Risk cost - Manual flight @ {}: {}'.format(self.manual_altitude, self.manual_risk))

    def eval_no_robot_mode(self):
        self.minibatch_test_fraction = 1.1
        # self.update_Xstar()

        self.X = self.fixed_points
        self.ytrain = self.fixed_values

        self.X_batch = self.X
        self.X_batch = self._hook(self.X_batch)

        self.ytrain_batch = self.ytrain
        self.ytrain_batch = self._hook(self.ytrain_batch)

        self.eval_trainData()
        self.compute_risk_cost()

        self.norobots_risk = self.risk_cost.data.cpu().numpy()
        print('Risk cost - no robots: {}'.format(self.norobots_risk))

    def eval_min_risk_paths(self):
        self.minibatch_test_fraction = 1.1
        # self.update_Xstar()

        self.min_risk_paths_cuda = torch.from_numpy(self.min_risk_paths).view(-1, 3).cuda()

        min_risk_values = self.get_default_values(self.min_risk_paths)
        self.min_risk_path_values = torch.from_numpy(min_risk_values).view(-1).cuda()

        self.X = torch.cat((self.fixed_points, self.min_risk_paths_cuda), dim=0).cuda()
        self.ytrain = torch.cat((self.fixed_values, self.min_risk_path_values), dim=0).cuda()

        self.X_batch = self.X
        self.X_batch = self._hook(self.X_batch)

        self.ytrain_batch = self.ytrain
        self.ytrain_batch = self._hook(self.ytrain_batch)

        self.eval_trainData()
        self.compute_risk_cost()

        self.min_risk = self.risk_cost.data.cpu().numpy()
        print('Risk cost - min risk path: {}'.format(self.min_risk))

    def eval_latest_paths(self):
        self.minibatch_test_fraction = 1.1
        # self.update_Xstar()

        self.latest_paths_cuda = torch.from_numpy(self.robot_points_data).view(-1, 3).cuda()

        latest_path_values = self.get_default_values(self.robot_points_data)
        self.latest_path_values = torch.from_numpy(latest_path_values).view(-1).cuda()

        self.X = torch.cat((self.fixed_points, self.latest_paths_cuda), dim=0).cuda()
        self.ytrain = torch.cat((self.fixed_values, self.latest_path_values), dim=0).cuda()

        self.X_batch = self.X
        self.X_batch = self._hook(self.X_batch)

        self.ytrain_batch = self.ytrain
        self.ytrain_batch = self._hook(self.ytrain_batch)

        self.eval_trainData()
        self.compute_risk_cost()

        self.latest_risk = self.risk_cost.data.cpu().numpy()
        print('Risk cost - latest path: {}'.format(self.latest_risk))

    def eval_rrtstar_paths(self):
        self.minibatch_test_fraction = 1.1
        # self.update_Xstar()
        self.star_planner = planning.Planning(self.mc_handle.params, on_terrain=self.mc_handle.terrain, mode='RRTSTAR')
        self.star_planner.plan()
        _, self.raw_star_paths, self.star_path_len = self.star_planner.get_paths_array(_interp_num=0)

        self.star_paths_cuda = torch.from_numpy(self.raw_star_paths).view(-1, 3).cuda()

        star_path_values = self.get_default_values(self.raw_star_paths)
        self.star_path_values = torch.from_numpy(star_path_values).view(-1).cuda()

        self.X = torch.cat((self.fixed_points, self.star_paths_cuda), dim=0).cuda()
        self.ytrain = torch.cat((self.fixed_values, self.star_path_values), dim=0).cuda()

        self.X_batch = self.X
        self.X_batch = self._hook(self.X_batch)

        self.ytrain_batch = self.ytrain
        self.ytrain_batch = self._hook(self.ytrain_batch)

        self.eval_trainData()
        self.compute_risk_cost()

        self.star_path_risk = self.risk_cost.data.cpu().numpy()
        self.star_planner.draw(_draw_plan=False, _draw_path=True)
        print('Risk cost - shortest path: {}'.format(self.star_path_risk))
        self.star_planner.draw(_draw_plan=False, _draw_path=True, _draw_obs=False)
        self.star_planner.draw_3d(_draw_plan=False, _draw_path=True)

def _drgp(rgp):
    print('Kxx NaNs:\t', _find_nans(rgp.Kxx))
    print('Kxx_inv NaNs:\t', _find_nans(rgp.Kxx_inv))

    print('\nystar_cov min:\t', torch.min(rgp.ystar_cov))
    print('\nystar_cov max:\t', torch.max(rgp.ystar_cov))
    print('\nystar_mu min:\t', torch.min(rgp.ystar_mu))
    print('\nystar_mu max:\t', torch.max(rgp.ystar_mu))

    print('NaNs:')
    print('\nrobot_points.grad:\t', _find_nans(rgp.robot_points.grad))
    print('\nrobot_values.grad:\t', _find_nans(rgp.robot_values.grad))

    print('Non NaNs:')
    print('\nrobot_points.grad:\t', _find_non_nans(rgp.robot_points.grad))
    print('\nrobot_values.grad:\t', _find_non_nans(rgp.robot_values.grad))

def _torch_sqrt(x, eps=1e-8):
    """
    A convenient function to avoid the NaN gradient issue of :func:`torch.sqrt`
    at 0.
    """
    # Ref: https://github.com/pytorch/pytorch/issues/2421
    return (x + eps).sqrt()

def _torch_inv_exp(x, eps=1e-8, _min=1e-10, _max=1e10):
    x1 = torch.exp(x)
    x2 = x1.clamp(min=_min, max=_max)
    x3 = 1/( x2 + eps )
    x4 = x3.clamp(min=_min, max=_max)
    return x4

def _torch_tridiag_mask(num):
    _main_diag = torch.eye(num, dtype=torch.float64) 
    _one_down = torch.cat( (torch.zeros((1, num-1), dtype=torch.float64), torch.eye(num-1, dtype=torch.float64)), dim=0)
    _one_down = torch.cat( (_one_down, torch.zeros((num, 1), dtype=torch.float64)), dim=1)
    _tridiag_mask = _one_down + _main_diag + _one_down.t()
    return _tridiag_mask

def _torch_pentadiag_mask(num):
    _main_diag = torch.eye(num, dtype=torch.float64)

    _one_down = torch.cat( (torch.zeros((1, num-1), dtype=torch.float64), torch.eye(num-1, dtype=torch.float64)), dim=0)
    _one_down = torch.cat( (_one_down, torch.zeros((num, 1), dtype=torch.float64)), dim=1)

    _two_down = torch.cat( (torch.zeros((2, num-2), dtype=torch.float64), torch.eye(num-2, dtype=torch.float64)), dim=0)
    _two_down = torch.cat( (_two_down, torch.zeros((num, 2), dtype=torch.float64)), dim=1)

    _tridiag_mask = _two_down + _one_down + _main_diag + _one_down.t() + _two_down.t()
    return _tridiag_mask

def _find_nans(x):
    return x[torch.isnan(x)]

def _find_non_nans(x):
    return x[~torch.isnan(x)]

def _input_thread(a_list):
    _inp_str = ''
    while True:
        _inp_str = input()
        if 'q' in _inp_str:
            a_list.append(True)
            print('\n')
            break
        elif 'w' in _inp_str:
            a_list.append(False)

def _fit_beziers(allpaths, lengths, error=0.5, bgp=None):
    beziers = []
    bezier_len = []
    start_ix = 0
    for tlen in lengths:
        end_ix = start_ix + tlen
        tpath = allpaths[start_ix:end_ix]
        bcurve = fitCurves.fitCurve(tpath, maxError=error)
        # [item for sublist in l for item in sublist]
        bcurve_exp = [ _tpt for _tbpart in bcurve for _tpt in _tbpart[:-1] ]
        bcurve_exp.append( bcurve[-1][-1] )

        beziers += bcurve_exp
        bezier_len.append(len(bcurve_exp))

        start_ix = end_ix
    # pdb.set_trace()
    return np.stack(beziers, axis=0), bezier_len

def _interp_bezier(bcurves, blengths, _res=0.01):
    _interp_pts = []
    _interp_len = []
    _fix_pts = []
    _fix_grads = []
    num_pts = int(1.0/_res) + 1
    T = torch.linspace(0.0, 1.0, num_pts).cuda().tolist()

    start_ix = ix = 0
    _nlen = len(blengths)
    for tix, tlen in enumerate(blengths):
        end_ix = ix + tlen
        _interp_prev_len = len(_interp_pts)
        for _ in range(0, tlen-1, 3):
            _tbcurve = bcurves[ix:ix+4]
            for _t in T:
                pF = bezier.q(_tbcurve, _t)
                _interp_pts.append(pF)

            if ix<end_ix-4: _fix_pts.append(bcurves[end_ix-1])
            if ix>start_ix:   _fix_pts.append(bcurves[start_ix])

            if ix<end_ix-4: _fix_grads.append( bezier.qprime(_tbcurve, 0.99) )
            if ix>start_ix:   _fix_grads.append( bezier.qprime(_tbcurve, 0.01) )

            ix = ix + 3
        ix = ix + 1
        _interp_len.append( len(_interp_pts) - _interp_prev_len )
        start_ix = end_ix

    _interp_pts = torch.stack(_interp_pts, dim=0)
    _fix_pts = torch.stack(_fix_pts, dim=0)
    _fix_grads = torch.stack(_fix_grads, dim=0)

    return _interp_pts, _interp_len, _fix_pts, _fix_grads

def _interp_bezier_old(bcurves, blengths, _res=0.01):
    _interp_pts = []
    _interp_len = []

    num_pts = int(1.0/_res) + 1
    T = torch.linspace(0.0, 1.0, num_pts, endpoint=True).cuda().tolist()
    ix = 0
    tinterp_len = 0
    for tlen in blengths:
        end_ix = ix + tlen
        _tbcurve = bcurves[ix:end_ix]
        for ix in range(0, tlen, 4):
            p0 = _tbcurve[ix]
            p1 = _tbcurve[ix+1]
            p2 = _tbcurve[ix+2]
            p3 = _tbcurve[ix+3]
            for _t in T:
                pA = _pt_in_line(p0, p1, _t)
                pB = _pt_in_line(p1, p2, _t)
                pC = _pt_in_line(p2, p3, _t)
                pD = _pt_in_line(pA, pB, _t)
                pE = _pt_in_line(pB, pC, _t)
                pF = _pt_in_line(pD, pE, _t)
                _interp_pts.append(pF)
        tinterp_len = len(_interp_pts) - tinterp_len
        _interp_pts.append(tinterp_len)
        ix = end_ix
    _interp_pts = torch.stack(_interp_pts, dim=0)
    return _interp_pts, _interp_len

def _pt_in_line(ptA, ptB, T=0.0):
    ptC = ptA - (ptA-ptB)*T
    return ptC

def _arrange_zorder(Xflat, _nx, _ny):
    _z_ixs = []
    for _i in range(_nx):
        for _j in range(_ny):
            _z_ixs.append(_zorder_ix(_j, _i))
    _z_ixs = np.array(_z_ixs)

    return Xflat[ np.argsort(_z_ixs.flatten()) ]

def _zorder_ix(_x, _y):
    MASKS = np.array([0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF], dtype=np.uint32)
    SHIFTS = np.array([1, 2, 4, 8], dtype=np.uint32)

    x = np.uint32(_x)
    y = np.uint32(_y)

    x = (x | (x << SHIFTS[3])) & MASKS[3]
    x = (x | (x << SHIFTS[2])) & MASKS[2]
    x = (x | (x << SHIFTS[1])) & MASKS[1]
    x = (x | (x << SHIFTS[0])) & MASKS[0]

    y = (y | (y << SHIFTS[3])) & MASKS[3]
    y = (y | (y << SHIFTS[2])) & MASKS[2]
    y = (y | (y << SHIFTS[1])) & MASKS[1]
    y = (y | (y << SHIFTS[0])) & MASKS[0]

    result = x | (y << 1)

    return result
