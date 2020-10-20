import time

import numpy as np

np.set_printoptions(precision=4, threshold=5, formatter={'float': '{: 0.3f}'.format})

import os
import _thread

import matplotlib.pyplot as plt

import torch
from torch.nn.utils.clip_grad import clip_grad_norm_

torch.set_printoptions(precision=4, threshold=5)
# torch.cuda.set_device(1)

import pyro
import pdb


class RobotGP(torch.nn.Module):
    def __init__(self, mc_handle, planner, meas_std=1e0, lsZ=5e0, _stime=None, parameters = {}):
        self.mc_handle = mc_handle
        # self.p_interp = mc_handle.p_interp
        self.planner = planner
        self.params = parameters

        self.meas_std = meas_std ** 2
        self.lsZ = lsZ
        self.lsXY_scaling = 1e1
        self.dim = 3

        self.update_mc_hotspots = True
        self.update_searcher_paths = True

        self.grad_clamp_value = 1e50
        self.grad_clamp = lambda grad: torch.clamp(grad, -self.grad_clamp_value, self.grad_clamp_value)
        # self.X_batch.tregister_hook(lambda grad: torch.clamp(grad, -self.grad_clamp, self.grad_clamp))

        self.init_Xstar()
        # self.paths_plot = self.planner.space.terrain.paths_plot.number

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

        self._max_iter = self.params['opt_iterations']
        self._reset_on_update = False # True makes the counter rest on updates, which makes the number of iterations somewhat uncertain.

        self.first_run_flag = True # flag to save first run of X


        self.all_robot_paths = []
        self.min_risk_paths = None

        # self.minibatch_test_fraction = 0.3
        self.minibatch_test_fraction = self.params['test_fraction']
        self.minibatch_train_fraction = self.params['train_fraction']

        self.training_history = [] # for tracking the progress in training

        self._init_riskcost_scaling = 1e7  # 1e7
        self._init_lengthcost_scaling = 1e2  # 1e7

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
        z = 1 + torch.as_tensor(self.mc_handle.terrain.h, dtype=torch.float64).reshape(-1,1)
        self.Xstar = torch.cat([x, y, z], 1).cuda()  # [nTest, 3]
        self.num_test = self.Xstar.shape[0]
        # print('Initializing Xstar took {} secs.'.format(time.time() - stime))

    def forward(self, x):
        self.eval_trainData()
        self.compute_risk_metric()
        return 0

    def collect_trainData(self):
        # sweep_paths = False in current config
        self.collect_mc_trainData()
        self.collect_searchers_trainData()

        if self.params['path_style'] == 'sweep': # lawnmower drones
            self.planner.robotClass.make_sweep_paths(sweep_num = self.params['drone_sweeps'], interp_res=self.params['path_interp_res'])
            self.robot_fixed_paths, self.robot_paths, self.robot_path_len = self.planner.robotClass.get_sweep_paths_array()
        elif self.params['path_style'] == 'basic': # optimized drones
            self.planner.plan()
            self.robot_fixed_paths, self.robot_paths, self.robot_path_len = self.planner.robotClass.get_paths_array(interp_res=self.params['path_interp_res'])
        elif self.params['path_style'] == 'rc': # rc'd drones
            searcher_paths = []
            for srch in self.mc_handle.searcher_class.searchers_list:
                # get searcher paths in correct format
                z_vals = [hgt + self.params['sweep_height'] for hgt in srch.smd_history[2]]
                points_array = np.stack([srch.smd_history[0],
                                         srch.smd_history[1],
                                         z_vals]).T
                points_array = np.split(points_array, points_array.shape[0], axis = 0)
                points_array = [p.reshape(-1) for p in points_array]
                searcher_paths.append(points_array)
            self.planner.robotClass.make_rc_paths(searcher_paths, interp_res=self.params['path_interp_res'])
            self.robot_fixed_paths, self.robot_paths, self.robot_path_len = self.planner.robotClass.get_sweep_paths_array()

        self.robot_fixed_path_values = self.get_default_values(self.robot_fixed_paths)
        self.num_robot_fixed_paths = self.robot_fixed_paths.shape[0]

        self.robot_path_values = self.get_default_values(self.robot_paths)
        self.num_robot_paths = self.robot_paths.shape[0]

        # self.points = np.concatenate( (self.mc_hotspots, self.searchers_paths, self.robot_paths), axis=0 )
        # self.values = np.concatenate( (self.mc_hotspot_values, self.searchers_path_values, self.robot_path_values), axis=0 )

        if self.update_mc_hotspots or self.update_searcher_paths:
            self.fixed_points = np.concatenate((self.mc_hotspots, self.searchers_paths, self.robot_fixed_paths), axis=0)
            self.fixed_values = np.concatenate(
                (self.mc_hotspot_values, self.searchers_path_values, self.robot_fixed_path_values), axis=0)
            self.num_fixed_train_points = self.fixed_points.shape[0]

            self.fixed_points = torch.from_numpy(self.fixed_points).view(-1, 3).cuda()
            # self.fixed_points = self._hook(self.fixed_points)

            self.fixed_values = torch.from_numpy(self.fixed_values).view(-1).cuda()
            # self.fixed_values = self._hook(self.fixed_values)

        self.robot_points = torch.from_numpy(self.robot_paths).view(-1, 3).cuda()
        # self.robot_points = torch.from_numpy( 120 * np.random.randn(*self.robot_paths.shape)).view(-1, 3).cuda()
        self.robot_points = self._hook(self.robot_points, _min=-5e2, _max=5e2)

        self.robot_values = torch.from_numpy(self.robot_path_values).view(-1).cuda()
        # self.robot_values = self._hook(self.robot_values)

        self.num_train = self.fixed_points.shape[0] + self.robot_points.shape[0]

        self.update_trainXY()

    def update_Xstar(self):
        self.minibatch_test_mask = torch.cuda.FloatTensor(self.num_test).uniform_() > (1 - self.minibatch_test_fraction)
        self.Xstar_batch = self.Xstar[self.minibatch_test_mask, :]

        # self.minibatch_test_mask = torch.cuda.FloatTensor(self.num_train).uniform_() > (1-self.minibatch_test_fraction)
        # self.Xstar_batch = self.X[self.minibatch_test_mask, :]

        # Co-variance between test points
        self.Kxstar_xstar = self.kernel_Gibbs_robots(self.Xstar_batch) * _torch_tridiag_mask(
            self.Xstar_batch.shape[0]).cuda()

    def update_trainXY(self):
        self.X = torch.cat((self.fixed_points, self.robot_points), dim=0).cuda()
        self.ytrain = torch.cat((self.fixed_values, self.robot_values), dim=0).cuda()

        self.minibatch_train_mask = torch.cuda.FloatTensor(self.num_train).uniform_() > (
                1 - self.minibatch_train_fraction)

        self.X_batch = self.X[self.minibatch_train_mask, :].cuda()
        self.X_batch = self._hook(self.X_batch)

        self.ytrain_batch = self.ytrain[self.minibatch_train_mask].cuda()
        self.ytrain_batch = self._hook(self.ytrain_batch)

    def collect_mc_trainData(self):
        """Collect training data from the probabilistic heatmap
        generated using MonteCarlo simulation of the lost person model
        """
        # if not self.mc_handle.updated_p:
        #     self.update_mc_hotspots = False
        #     return
        self.update_mc_hotspots = True
        hotspots_xy = self.mc_handle.get_hotspots(threshold=self.params['lp_threshold'])

        mc_altitude = 15
        hotspots_z = mc_altitude + self.get_altitude_values(hotspots_xy)

        self.mc_hotspots = np.hstack([hotspots_xy, hotspots_z])
        self.mc_hotspot_values = self.get_default_values(self.mc_hotspots)
        self.num_mc_hotspots = self.mc_hotspots.shape[0]

    def collect_searchers_trainData(self):
        # print(self.mc_handle.searcher_class)
        if not self.mc_handle.searcher_class.updated_searcher_paths:
            self.update_searcher_paths = False
            return
        self.update_searcher_paths = True

        X = [[], []]  # search points
        y = []  # prior at search points
        for tsr in self.mc_handle.searcher_class.searchers_list:
            all_x, all_y, all_z, _ = tsr.smd_history
            X[0] += all_x
            X[1] += all_y
            # X[2] += all_z
        X = [*zip(*X)]
        searchers_paths_xy = np.array(X, dtype=np.float64)

        searcher_altitude = 3
        searchers_paths_z = searcher_altitude + self.get_altitude_values(searchers_paths_xy)

        self.searchers_paths = np.hstack([searchers_paths_xy, searchers_paths_z])

        self.searchers_path_values = self.get_default_values(self.searchers_paths)
        self.num_searchers_paths = self.searchers_paths.shape[0]
        self.mc_handle.searcher_class.updated_searcher_paths = False

    def eval_trainData(self, eps=1e-8):
        stime = time.time()

        with torch.no_grad():
            # save previous ystar_mu
            try:
                self.prev_X = self.X # will be used to calculate things later
                self.prev_Xstar = self.Xstar
                if self.first_run_flag:
                    self.first_X = self.X  # will be used to calculate things later
                    self.first_Xstar = self.Xstar
                    self.first_run_flag = False
            except AttributeError:
                pass

        self.update_Xstar()
        self.update_trainXY()

        self.update_Kxx_and_inv()

        # Cross-covariance between training and test points
        self.Kx_xstar = self.kernel_Gibbs_robots(self.X_batch, self.Xstar_batch)
        self.Kx_xstar = self._hook(self.Kx_xstar)

        Kx_xstar_T = self.Kx_xstar.t()
        Kx_xstar_T = self._hook(Kx_xstar_T)

        # self.v = torch.mm(self.Linv, self.Kx_xstar)
        # self.v = self._hook(self.v)

        self.K_ratio = torch.matmul(Kx_xstar_T, self.Kxx_inv)
        self.K_ratio = self._hook(self.K_ratio)

        # K_tt'/K_train*y_train;
        self.ystar_mu = torch.matmul(self.K_ratio, self.ytrain_batch)
        self.ystar_mu = self._hook(self.ystar_mu)

        # K_test - K_tt'/K_train*K_tt;
        self.ystar_cov = self.Kxstar_xstar - torch.matmul(self.K_ratio, self.Kx_xstar)
        self.ystar_cov = self._hook(self.ystar_cov)

        # plotting find matrix at each iteration
        # yst_mu_curr, yst_cov_curr = self.eval_batchless(self.X, self.Xstar)
        # yst_mu_prev, yst_cov_prev = self.eval_batchless(self.prev_X, self.prev_Xstar)
        # find_mat = 1 - (1 / (1 + np.exp(yst_mu_curr)))
        # find_mat_prev = 1 - (1 / (1 + np.exp(yst_mu_prev)))
        #
        # find_ratio = (find_mat/find_mat_prev)/((1 - find_mat)/(1 - find_mat_prev))
        # plt.imshow(find_ratio)
        # plt.show()
        # print("expected good detection: {}".format(np.count_nonzero(find_ratio >= 1)))
        # wherever find_ratio is > 1 we have a higher chance of actually finding the person vs getting a false positive (or something like that)
        # plt.imshow(find_ratio_total)
        # plt.show()

    def eval_batchless(self, X_state, Xstar_state):
        # calculate y_star_mu/cov with no batches
        with torch.no_grad():
            self.Kx_xstar_raw = self.kernel_Gibbs_robots(X_state, Xstar_state)

            Kx_xstar_T_raw = self.Kx_xstar_raw.t()

            # calc kxx_inv manually:
            self.noise = torch.diag(torch.ones_like(self.ytrain).view(-1)) * self.meas_std ** 2
            # Co-variance between training points
            self.Kxx_raw = (self.kernel_Gibbs_robots(X_state) + self.noise)

            self.Kxx_inv_raw = torch.inverse(self.Kxx_raw)

            self.K_ratio_raw = torch.matmul(Kx_xstar_T_raw, self.Kxx_inv_raw)

            # K_tt'/K_train*y_train;
            self.ystar_mu_raw = torch.matmul(self.K_ratio_raw, self.ytrain)

            self.Kxstar_xstar_raw = self.kernel_Gibbs_robots(Xstar_state)

            self.ystar_cov_raw = self.Kxstar_xstar_raw - torch.matmul(self.K_ratio_raw, self.Kx_xstar_raw)

            yst_mu = self.ystar_mu_raw.cpu().numpy()
            yst_mu = yst_mu.reshape(np.sqrt(yst_mu.shape[0]).astype(np.int), np.sqrt(yst_mu.shape[0]).astype(np.int))
            yst_cov = np.diag(self.ystar_cov_raw.cpu().numpy())
            yst_cov = yst_cov.reshape(np.sqrt(yst_cov.shape[0]).astype(np.int), np.sqrt(yst_cov.shape[0]).astype(np.int))
            # if self._iter_since_update == 0 and self.init_print_flag:
            #     init_plots = [yst_mu, yst_cov]
            #     plt.imshow(init_plots[0], interpolation='none')
            #     plt.show()
            #     plt.imshow(init_plots[1], interpolation='none')
            #     plt.show()
            #     rc = np.sum(yst_cov.reshape(-1,1)/(1 + (yst_mu.reshape(-1,1)**2)))
            #     print('init risk cost = {}'.format(rc))
            #     print('init sum means = {}'.format(np.sum(yst_mu)))
            #     self.init_print_flag = False
            # elif self._iter_since_update >= self._max_iter-1:
            #     fin_plots = [yst_mu, yst_cov]
            #     plt.imshow(fin_plots[0], interpolation='none')
            #     plt.show()
            #     plt.imshow(fin_plots[1], interpolation='none')
            #     plt.show()
            #     rc = np.sum(yst_cov.reshape(-1,1)/(1 + (yst_mu.reshape(-1,1)**2)))
            #     print('final risk cost = {}'.format(rc))
            #     print('final sum means = {}'.format(np.sum(yst_mu)))
        return yst_mu, yst_cov


        # print( 'Evaluating GP using Xtrain took {} secs.'.format( time.time()-stime ) )

    def compute_risk_cost(self):
        cov_trace_scaling = 1e5
        mean_scaling = 1e0
        mean_sq_scaling = 1e0

        ystar_mu_sq = (self.ystar_mu * mean_scaling) ** 2
        ystar_mu_sq = self._hook(ystar_mu_sq)

        self.record_init_params(cov_trace_scaling, mean_scaling, mean_sq_scaling)
        stime = time.time()

        # ystar_noise = self.ystar_cov
        # omega = ( (torch.diag(torch.cholesky( ystar_noise ))) * cov_trace_scaling )  / (1e-5 +ystar_mu_sq*mean_sq_scaling)
        # print("testing torch ipc_collect")
        # torch.cuda.ipc_collect()
        # print("after torch ipc_collect")
        self.omega = (torch.diag(self.ystar_cov) * cov_trace_scaling) / (1 + ystar_mu_sq * mean_sq_scaling)
        # self.omega = (torch.diag(self.ystar_cov)*cov_trace_scaling) - (ystar_mu_sq*mean_sq_scaling)
        self.omega = self._hook(self.omega)

        self.update_path_len_cost()

        self.scaled_risk_cost = self.omega.sum() * self.riskcost_scaling
        self.scaled_path_length_cost = self.lengthcost_scaling * self.path_len_cost
        self.risk_cost = self.scaled_risk_cost + self.scaled_path_length_cost
        self.risk_cost = self._hook(self.risk_cost)

    def compute_risk_cost_batchless(self, yst_mu, yst_cov):
        # kind of assuming that risk cost would be different if evaluated without batches included?
        with torch.no_grad():
            cov_trace_scaling = 1e5
            mean_scaling = 1e0
            mean_sq_scaling = 1e0

            ystar_mu_sq = (yst_mu * mean_scaling) ** 2

            omega = (np.diag(yst_cov) * cov_trace_scaling) / (1 + ystar_mu_sq * mean_sq_scaling)
            # self.omega = self._hook(self.omega)

            self.update_path_len_cost()

            scaled_risk_cost = omega.sum() * self.riskcost_scaling
            scaled_path_length_cost = self.lengthcost_scaling * self.path_len_cost
            risk_cost_batchless = scaled_risk_cost + scaled_path_length_cost
            return risk_cost_batchless
            # self.risk_cost_batchless = self._hook(self.risk_cost)

    def garbage_cleanup(self):
        torch.cuda.ipc_collect()
        # empty cache (maybe not neccessary):
        torch.cuda.empty_cache()
        self.planner.robotClass.__all_robots__ = [] # reset robot list
        self.planner.robotClass.num_robots = 0 # reset robot num
        self.mc_handle.searcher_class.searchers_list = [] # reset searcher list
        # print('cleaned garbage!!!')

    def time_to_find(self, robot_paths, searcher_paths):
        # figure out if lp is found and how long it took. do while loop to get good measure.
        lp_map = self.mc_handle.comp_map/np.sum(self.mc_handle.comp_map)
        '''
        drone_fov_radius
        drone_fov_alt
        searcher_fov_radius
        '''
        srch_paths = []
        for i,srch in enumerate(searcher_paths):
            srch_paths.append([srch.smd_history[0],srch.smd_history[1],srch.smd_history[2]])
        srch_paths = np.array(srch_paths)

        rbt_paths = []
        for i, path in enumerate(robot_paths):
            rbt_paths.append([[val[0] for val in path], [val[1] for val in path], [val[2] for val in path]])
        rbt_paths = np.array(rbt_paths)

        find_percentage = []
        time_to_find = []

        for i in range(self.params['t_to_f_iter']):
            x_dist = np.sum(lp_map,axis=0)
            y_dist = np.sum(lp_map,axis=1)
            x_pt = np.random.choice(range(lp_map.shape[0]), size=1, p=x_dist, replace=False)
            y_pt = np.random.choice(range(lp_map.shape[1]), size=1, p=y_dist, replace=False)
            x_pt = int(self.params['res']*(x_pt - (lp_map.shape[0]/2)))
            y_pt = int(self.params['res']*(y_pt - (lp_map.shape[1]/2)))
            # go through all searcher points and drone points to check
            rbt_find_min_dist = np.inf
            for rpth in rbt_paths:
                # for each robot calc horizontal dist to location
                dists = np.sqrt((rpth[0] - x_pt)**2 + (rpth[1] - y_pt)**2)
                # get horizontal distance thresholds while scaling for fov inc
                rpth_alt = rpth[2] - self.mc_handle.terrain.h_smooth.ev(rpth[0], rpth[1])
                rpth_alt = np.abs(rpth_alt)
                thrshld = self.params['drone_fov_alt']*rpth_alt
                # find which points are close, and pick first one
                rbt_find_idx = np.where(dists <= thrshld)
                if np.any(rbt_find_idx[0]):
                    rbt_find_idx = np.min(rbt_find_idx[0])
                    pth_dists = np.sqrt(np.sum(np.diff(np.array([rpth[0], rpth[1], rpth[2]]).T, axis=0) ** 2, axis=1))
                    rbt_find_dist = np.sum(pth_dists[0:rbt_find_idx])
                    if rbt_find_dist < rbt_find_min_dist:
                        rbt_find_min_dist = rbt_find_dist

            srch_find_min_dist = np.inf
            for spth in srch_paths:
                # for each robot calc horizontal dist to location
                dists = np.sqrt((spth[0] - x_pt)**2 + (spth[1] - y_pt)**2)
                # get horizontal distance thresholds while scaling for fov inc
                thrshld = self.params['searcher_fov_radius']
                # find which points are close, and pick first one
                srch_find_idx = np.where(dists <= thrshld)
                if np.any(srch_find_idx[0]):
                    srch_find_idx = np.min(srch_find_idx[0])
                    pth_dists = np.sqrt(np.sum(np.diff(np.array([spth[0], spth[1], spth[2]]).T, axis=0) ** 2, axis=1))
                    srch_find_dist = np.sum(pth_dists[0:srch_find_idx])
                    if srch_find_dist < srch_find_min_dist:
                        srch_find_min_dist = srch_find_dist

            srch_time = srch_find_min_dist/self.params['searcher_speed']
            rbt_time = rbt_find_min_dist/self.params['drone_speed']

            if srch_time < rbt_time and not np.isinf(srch_time):
                # print("searcher found lost person in {} minutes".format((srch_find_dist*self.params['searcher_speed'])/60))
                time_to_find.append(srch_time/60)
                find_percentage.append(1)
            elif rbt_time < srch_time and not np.isinf(rbt_time):
                # print("drone found lost person in {} minutes".format((rbt_find_dist*self.params['drone_speed'])/60))
                time_to_find.append(rbt_time/60)
                find_percentage.append(1)
            else:
                # print("lost person was not found :(")
                find_percentage.append(0)

        # print("mean time to find: " + str()
        # print("find percentage: " + str(np.mean(find_percentage)))
        return np.mean(time_to_find), np.mean(find_percentage)

    def optimize_risk_cost_grad(self, _show_detail=False):
        # cov_trace_scaling = 1e-7
        # gamma = 1e-15 #e-15
        # mean_scaling = 1e-10
        # mean_sq_scaling = 1e-11
        # self.lengthcost_scaling = 1e-7

        # optimizer = torch.optim.SGD({self.robot_points}, lr=5e-2)
        self.optimizer = torch.optim.Adam({self.robot_points}, lr=self.params['learning_rate'], weight_decay=self.params['weight_decay'])  # , lr=1e-2)

        # a_list = []
        # _thread.start_new_thread(_input_thread, (a_list,))

        # print('\nRisk cost function optimization')
        self._iter = 0
        mem_ratio = torch.cuda.memory_allocated()/torch.cuda.max_memory_allocated()
        print('\nOptimization memory ratio: {:8.6f}'.format(mem_ratio))
        if mem_ratio > 0.75:
            print("oh no")

        while self._iter_since_update < self._max_iter:
            stime = time.time()
            self.optimizer.zero_grad()

            # with torch.autograd.detect_anomaly():

            self.eval_trainData()

            self.compute_risk_cost()

            # self.risk_cost.backward(retain_graph=True)
            self.risk_cost.backward(retain_graph=False) # testing if this is required

            # no_nans_p_grad = _find_nans(self.robot_points.grad).shape[0] == 0
            # if not no_nans_p_grad:
                # pdb.set_trace()

            self.collect_robot_paths()
            self.optimizer.step()

            self.curr_path_diff = self.robot_points_data - self._init_paths
            self.curr_path_diff_abs = np.abs(self.curr_path_diff).sum(axis=-1)

            if _show_detail:
                self.printStr = ''
                self.printStr += '\nOptimizer step took {:09.6f} secs.\n'.format(time.time() - stime)

                _time_taken = time.time() - self._stime
                self.printStr += 'I#{:05d} - U# {:05d}\t| {:0.3e}\t:: T {:10.3f} s\t| {:07.3f} it/s'.format(self._iter,
                                                                                                            self._updates,
                                                                                                            self.risk_cost_data - self._prev_min_risk_cost,
                                                                                                            _time_taken,
                                                                                                            self._iter / _time_taken)
                self.printStr += '\ninitCost:\t{:0.3e}\t = risk: {:0.3e}\t + len: {:0.3e}'.format(self._init_risk_cost,
                                                                                                  self._init_scaled_risk_cost,
                                                                                                  self._init_scaled_path_length_cost)

                self.printStr += '\ncurCost:\t{:0.3e}\t = risk: {:0.3e}\t + len: {:0.3e}'.format(self.risk_cost_data,
                                                                                                 self.scaled_risk_cost_data,
                                                                                                 self.scaled_path_length_cost_data)

                self.printStr += '\ncur Delta::\tmean: {:010.3f} | std: {:010.3f} | max: {:010.3f} | min: {:010.3f}'.format(
                    self.curr_path_diff_abs.mean(), self.curr_path_diff.std(), self.curr_path_diff_abs.max(),
                    self.curr_path_diff_abs.min())

                self.printStr += '\nminCost:\t{:0.3e}\t = risk: {:0.3e}\t + len: {:0.3e}'.format(self.min_risk_cost,
                                                                                                 self._min_scaled_risk_cost,
                                                                                                 self._min_scaled_path_length_cost)
                self.min_path_diff = self.min_risk_paths - self._init_paths
                self.min_path_diff_abs = np.abs(self.min_path_diff).sum(axis=-1)
                self.printStr += '\nmin Delta::\tmean: {:010.3f} | std: {:010.3f} | max: {:010.3f} | min: {:010.3f}'.format(
                    self.min_path_diff_abs.mean(), self.min_path_diff.std(), self.min_path_diff_abs.max(),
                    self.min_path_diff_abs.min())

                self.printStr += '\nIter since: {:05d}/{:05d} | {:05d} || {:0.3e} | {:0.3e}'.format(
                    self._iter_since_update, self._max_iter, self._iters_cont_update, self.riskcost_scaling,
                    self.lengthcost_scaling)
                self.printStr += '\nMemory ratio: {:8.6f}'.format(torch.cuda.memory_allocated()/torch.cuda.max_memory_allocated())

                print(self.printStr)
            else:
                # quiet mode
                if np.mod(self._iter_since_update,10) == 0:
                    self.printStr = '\nIter since: {:05d}/{:05d}'.format(self._iter_since_update, self._max_iter)
                    print(self.printStr)
            self.training_history.append([self.risk_cost_data, self.curr_path_diff_abs.mean()])
            self._iter += 1

    def update_path_len_cost(self):
        self.fixed_point_dist_scaling = 1e5
        self.other_dist_scaling = 1e0

        start_ix = 0
        path_cost_list = []
        for tix, tlen in enumerate(self.robot_path_len):
            end_ix = start_ix + tlen
            fixed_point_ix = - self.num_robot_fixed_paths + 2 * tix
            path_cost_list.append(((self.fixed_points[fixed_point_ix, :] - self.robot_points[start_ix,
                                                                           :]) ** 2).sum() * self.fixed_point_dist_scaling)
            # with torch.no_grad():
            # self._nograd_robot_points = self.robot_points.data
            path_cost_list.append(((self.robot_points[start_ix:end_ix - 1, :] - self.robot_points[start_ix + 1:end_ix,
                                                                                :]) ** 2).sum() * self.other_dist_scaling)
            path_cost_list.append(((self.robot_points[end_ix - 1, :] - self.fixed_points[fixed_point_ix + 1,
                                                                       :]) ** 2).sum() * self.fixed_point_dist_scaling)
            start_ix = end_ix

        self.path_len_cost = sum(path_cost_list).cuda()
        self.path_len_cost = self._hook(self.path_len_cost)

    def collect_robot_paths(self):
        with torch.no_grad():
            self.risk_cost_data = self.risk_cost.data.cpu().numpy()
            self.robot_points_data = self.robot_points.data.cpu().numpy().copy()
            self.scaled_risk_cost_data = self.scaled_risk_cost.data.cpu().numpy()
            self.scaled_path_length_cost_data = self.scaled_path_length_cost.data.cpu().numpy()

            self._alt_scaled_cost = 1e0 * self.scaled_risk_cost_data + 0 * self.scaled_path_length_cost_data
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

                self._curr_risk_ratio = self.min_risk_cost / self._init_risk_cost

            self._iter_since_update += 1

            # if self.risk_cost_data < self.min_risk_cost:
            if self._alt_scaled_cost < self._min_alt_scaled_cost:

                self._prev_min_risk_cost = self.min_risk_cost
                self._prev_min_risk_paths = self.min_risk_paths
                self.min_risk_cost = self.risk_cost_data
                self.min_risk_paths = self.robot_points_data

                self._min_scaled_risk_cost = self.scaled_risk_cost_data
                self._min_scaled_path_length_cost = self.scaled_path_length_cost_data
                self._min_alt_scaled_cost = self._alt_scaled_cost

                self._curr_risk_ratio = self.min_risk_cost / self._init_risk_cost

                # also saved X state and Xstar state
                self.min_X = self.X
                self.min_Xstar = self.Xstar

                # if (self._prev_min_risk_cost - self.min_risk_cost) / self.min_risk_cost > 1e-3:
                self._updates += 1
                self._iters_cont_update += 1
                if self._reset_on_update:
                    self._iter_since_update = 0

            else:
                self._iters_cont_update = 0

            if self.params['path_style'] != 'basic':
                # also saved X state and Xstar state
                self.min_X = self.X
                self.min_Xstar = self.Xstar

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

                if self.mc_handle.terrain.surface_plot is None:
                    fig = plt.figure()
                    self.paths_plot_3d_fig = fig
                    ax_3d = fig.gca(projection='3d')
                    self.mc_handle.terrain.surface_plot = ax_3d
                    ax_3d.set_xlim(self.mc_handle.terrain.xmin, self.mc_handle.terrain.xmax)
                    ax_3d.set_ylim(self.mc_handle.terrain.ymin, self.mc_handle.terrain.ymax)
                    ax_3d.set_zlim(self.mc_handle.terrain.hmin, self.mc_handle.terrain.hmax)
                else:
                    ax_3d = self.mc_handle.terrain.surface_plot
                self._plot_robot_paths_3d(self.all_robot_paths[0], ax_3d)
                self._plot_robot_paths_3d(self.all_robot_paths[-1], ax_3d, tcolor='magenta')
                self._plot_robot_paths_3d(self.min_risk_paths, ax_3d, tcolor='red')
        else:
            fig = plt.figure(self.paths_plot)
            self._plot_robot_paths(self.robot_points_data)
        plt.draw()
        return plt.gcf()

    def _plot_robot_paths(self, rpaths, tcolor='yellow'):
        start_ix = 0
        for tlen in self.robot_path_len:
            end_ix = start_ix + tlen
            tpath = rpaths[start_ix:end_ix]
            plt.plot(tpath[:, 0], tpath[:, 1], tcolor)
            start_ix = end_ix

    def _plot_robot_paths_3d(self, rpaths, axis3d, tcolor='yellow'):
        start_ix = 0
        for tlen in self.robot_path_len:
            end_ix = start_ix + tlen
            tpath = rpaths[start_ix:end_ix]
            plt.plot(xs=tpath[:, 0], ys=tpath[:, 1], zs=tpath[:, 2], color=tcolor)
            start_ix = end_ix

    def update_Kxx(self):

        if self._iter < 1:
            # self.Kxx_mask = torch.eye
            pass
        # print("Print statements incomming:-----------------")
        # print(self.ytrain_batch.shape)
        # print(torch.ones_like(self.ytrain_batch).view(-1).shape)
        # print("Print statements over.")
        self.noise = torch.diag(torch.ones_like(self.ytrain_batch).view(-1)) * self.meas_std ** 2

        # Co-variance between training points
        self.Kxx = (self.kernel_Gibbs_robots(self.X_batch) + self.noise) * _torch_tridiag_mask(
            self.ytrain_batch.shape[0]).cuda()
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
                self.Kxx = torch.zeros(self.num_fixed_train_points + self.num_robot_paths,
                                       self.num_fixed_train_points + self.num_robot_paths)
                self.Kxx[-self.num_robot_paths:, :self.num_fixed_train_points]

    def inc_update_Kxx(self):
        nonfixed_noise = torch.diag(torch.randn_like(self.robot_values).view(-1)) * self.meas_std ** 2

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

            Ainv = self.K_fixed_train_inv = torch.inverse(
                self.Kxx[:self.num_fixed_train_points, :self.num_fixed_train_points])

            B = self.Kxx[:self.num_fixed_train_points, -self.num_robot_paths:]
            C = self.Kxx[-self.num_robot_paths:, -self.num_robot_paths:]

            M = C - torch.chain_matmul(B.t(), Ainv, B)

            # ~A
            self.Kxx_inv[:self.num_fixed_train_points, :self.num_fixed_train_points] = Ainv - torch.chain_matmul(Ainv,
                                                                                                                 B, M,
                                                                                                                 B.t(),
                                                                                                                 Ainv)
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

            M = C - torch.chain_matmul(B.t(), Ainv, B)
            # ~A
            self.Kxx_inv[:self.num_fixed_train_points, :self.num_fixed_train_points] = Ainv - torch.chain_matmul(Ainv,
                                                                                                                 B, M,
                                                                                                                 B.t(),
                                                                                                                 Ainv)
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

        rA2 = (rA ** 2).reshape(-1, 1, self.dim)
        rB2 = (rB ** 2).reshape(1, -1, self.dim)
        rA2_plus_rB2 = rA2 + rB2 + eps
        rA2_plus_rB2_mul_dim = torch.prod(rA2_plus_rB2, dim=-1) + eps

        ret_part_1_sqrt = _torch_sqrt(torch.pow((2.0 * rA.matmul(rB.t())) / rA2_plus_rB2_mul_dim, self.dim))
        ret_part_1_sqrt = self._hook(ret_part_1_sqrt)

        ret_part_2_1_sum = torch.sum(r2 / rA2_plus_rB2, dim=-1)
        ret_part_2_1_sum = self._hook(ret_part_2_1_sum)

        ret_part_2_invexp = _torch_inv_exp(ret_part_2_1_sum)
        ret_part_2_invexp = self._hook(ret_part_2_invexp)

        return (ret_part_1_sqrt * ret_part_2_invexp)

    def kernel_RBF(self, matA, matB):
        raise NotImplementedError

    def lsGibbs_robots(self, mat):
        with torch.no_grad():
            lsX = lsY = (1 + mat[:, 2].view(-1, 1)) * self.lsXY_scaling
            # lsX = lsY = 1 + torch.ones_like(mat[:, 2].view(-1, 1)) * self.lsZ
            lsZ = torch.ones_like(lsX) * self.lsZ # TODO: maybe this is the sensor model in question?
            return torch.cat((lsX, lsY, lsZ), 1)

    def pdistsq_dimwise(self, matA, matB):
        dim = 3

        matA = matA.view(1, -1, dim)
        matB = matB.view(1, -1, dim).transpose(1, 0)

        nA = matA.shape[1]
        nB = matB.shape[0]

        return (matA.repeat(nB, 1, 1) - matB.repeat(1, nA, 1)) ** 2

    def get_altitude_values(self, points_xy):
        points_z = np.zeros_like(points_xy[:, 0:1], dtype=np.float64)
        for ix in range(points_z.shape[0]):
            points_z[ix] = self.mc_handle.terrain.h_smooth.ev(points_xy[ix, 0], points_xy[ix, 1])
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
        params.update({ \
            _gp + 'cov_trace_scaling': cov_trace_scaling, \
            _gp + 'mean_scaling': mean_scaling, \
            _gp + 'mean_sq_scaling': mean_sq_scaling, \
            _gp + 'minibatch_test_fraction': self.minibatch_test_fraction, \
            _gp + 'minibatch_train_fraction': self.minibatch_train_fraction, \
            _gp + 'grad_clamp_value': self.grad_clamp_value, \
            _gp + '_max_iter': self._max_iter, \
            _gp + 'dummy': 1
        })
        self.mc_handle.write_params()

    def record_final_params(self):
        params = self.mc_handle.params
        _gp = 'GP_final_'
        params.update({ \
            _gp + '_min_alt_scaled_cost': float(self._min_alt_scaled_cost), \
            _gp + '_alt_scaled_cost': float(self._alt_scaled_cost), \
            _gp + '_updates': self._updates, \
            _gp + '_iters_cont_update': self._iters_cont_update, \
            _gp + '_iter_since_update': self._iter_since_update, \
            _gp + 'fixed_point_dist_scaling': float(self.fixed_point_dist_scaling), \
            _gp + 'other_dist_scaling': float(self.other_dist_scaling), \
            _gp + 'lengthcost_scaling': float(self.lengthcost_scaling), \
            _gp + 'printStr': self.printStr, \
            _gp + 'dummy': 1
        })
        params.update({ \
            _gp + 'min_path_diff_abs_mean': float(self.min_path_diff_abs.mean()), \
            _gp + 'min_path_diff_abs_std': float(self.min_path_diff_abs.std()), \
            _gp + 'min_path_diff_abs_min': float(self.min_path_diff_abs.min()), \
            _gp + 'min_path_diff_abs_max': float(self.min_path_diff_abs.max()), \
            _gp + 'dummy': 1
        })
        params.update({ \
            _gp + '_prev_min_risk_cost': float(self._prev_min_risk_cost), \
            _gp + 'min_risk_cost': float(self.min_risk_cost), \
            _gp + '_min_scaled_risk_cost': float(self._min_scaled_risk_cost), \
            _gp + 'scaled_risk_cost_data': float(self.scaled_risk_cost_data), \
            _gp + '_min_scaled_path_length_cost': float(self._min_scaled_path_length_cost), \
            _gp + 'scaled_path_length_cost_data': float(self.scaled_path_length_cost_data), \
            _gp + 'dummy': 1
        })
        self.mc_handle.write_params()


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
    x3 = 1 / (x2 + eps)
    x4 = x3.clamp(min=_min, max=_max)
    return x4


def _torch_tridiag_mask(num):
    _main_diag = torch.eye(num, dtype=torch.float64)
    _one_down = torch.cat((torch.zeros((1, num - 1), dtype=torch.float64), torch.eye(num - 1, dtype=torch.float64)),
                          dim=0)
    _one_down = torch.cat((_one_down, torch.zeros((num, 1), dtype=torch.float64)), dim=1)
    _tridiag_mask = _one_down + _main_diag + _one_down.t()
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
