from __future__ import print_function

import numpy as np
import random, json, string
import sys, os, warnings, datetime

import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import cm, animation
from mpl_toolkits.mplot3d import Axes3D

from scipy import ndimage, misc, signal

from tqdm import tqdm, trange

import pdb
from .lp_model import load_heatmap
from . import params as params
from . import terrain as terrain
from . import human as human
from . import searcher as searcher

np.set_printoptions(precision=5)
np.set_printoptions(suppress=True)
np.seterr(all='raise')


def id_generator(size=6, chars=string.ascii_uppercase + string.digits):
    return ''.join(random.choice(chars) for _ in range(size))


class MonteCarlo(params.Default):
    def __init__(self, params={}):
        super().__init__(params)

        # self.params = params
        self.set_defaults()
        self.count = 0
        self.store_logs = params['store_logs']

        self.IS_HEATMAP_DYN = False
        if self.IS_HEATMAP_DYN:
            self.p = np.zeros((params['iter_steps'], self._x_shape, self._y_shape), dtype=float)
            self.fig = plt.figure()
            # self.ax = plt.axes( xlim=(self.xmin, self.xmax), ylim=(self.ymin, self.ymax) )
        else:
            self.p = np.zeros((self._x_shape, self._y_shape), dtype=float)

        self.set_experiment_log_params()

        self.terrain = terrain.Terrain(params=self.params)
        human.Human.terrain = self.terrain
        # actually init searcher class as attribute
        self.searcher_class = searcher.Searcher
        self.searcher_class.updated_searcher_paths = True
        self.searcher_class.terrain = self.terrain

    def set_experiment_log_params(self):
        # Save experiment results and logs
        now = datetime.datetime.now()
        self.time_str = '{:02d}{:02d}{:02d}-{:02d}{:02d}{:02d}'.format(now.year % 100, now.month, now.day, now.hour,
                                                                       now.minute, now.second)

        self.exp_str = '{:03d}_{}_{}'.format(self.params.get('exp_id', 0), self.time_str,
                                             self.params.get('exp_name', 'exp_name'))
        self.params['exp_str'] = self.exp_str

        if self.params.get('store_logs', True):
            self.exp_results_dir = os.path.join('results', self.exp_str)
            if not os.path.exists(self.exp_results_dir):
                os.makedirs(self.exp_results_dir)
            else:
                warning_str = 'Directory {} already exists. Are you sure you want to continue?'.format(
                    self.exp_results_dir)
                warnings.warn(warning_str)
            self.params['exp_results_dir'] = self.exp_results_dir

            self.param_log_file = os.path.join(self.exp_results_dir, 'params_{}.log'.format(self.time_str))
            self.history_log_file = os.path.join(self.exp_results_dir, 'p_history_{}.npy'.format(self.time_str))
            self.dynheatmap_log_file = os.path.join(self.exp_results_dir, 'dynheatmap_{}.mp4'.format(self.time_str))
            self.heatmap_log_file = os.path.join(self.exp_results_dir, 'heatmap_{}.png'.format(self.time_str))

    def run_iter(self):
        self.humans = human.Human.reset(params=self.params)
        for step_idx in range(self.iter_steps):
            human.Human.step_all()
            self.collect_step_data(step_idx=step_idx)

    def normalize(self, step_idx=0):
        self.p[step_idx, :, :] /= float(self.iter_steps)  # float(self.count[step_idx])

    def get_hotspots(self, threshold=0.5):
        # points_xy = np.column_stack([self.terrain.xmin + (self.terrain.xmax - self.terrain.xmin)
        #                              * np.random.random((10, 1)),
        #                              self.terrain.ymin + (self.terrain.ymax - self.terrain.ymin)
        #                              * np.random.random((10, 1))])
        points_xy = []
        # maybe dont normalize, based on existing code threshold values
        # print("h shape:" + str(self.terrain.h.shape))
        # print("x shape:" + str(self.terrain.x.shape))
        # print("y shape:" + str(self.terrain.y.shape))
        # print("p shape:" + str(self.p.shape))
        for ix in range(self.p.shape[0]):
            for iy in range(self.p.shape[1]):
                # append point if above threshold
                if self.p[ix, iy] > threshold:
                    points_xy.append([self.terrain.x[ix, iy], self.terrain.y[ix, iy]])
        array_points_xy = np.array(points_xy)
        return array_points_xy

    def p_interp(self, px, py):
        # In theory, this is what the p_interp function is supposed to do, get hotspot values from coordinates
        # get indicies for each point (px, py)
        xidx, yidx = self.terrain.idx_from_coords(px, py, saturate=True)
        return self.p[xidx, yidx]

    def show_heatmap(self):
        if not self.IS_HEATMAP_DYN:

            if self.searcher_class.terrain is not None and self.searcher_class.terrain.paths_plot is not None:
                fig = self.searcher_class.terrain.paths_plot
                plt.figure(fig.number)
                plt.title('Heatmap - Lost person')
            else:
                fig = plt.figure()  # plt.gcf()
                plt.title('Heatmap - Lost person | Searchers\' paths')
                self.searcher_class.terrain.paths_plot = fig

            plt.imshow(self.p[:, :].transpose(), cmap='hot', interpolation='nearest',
                       extent=[self.xmin, self.xmax, self.ymin, self.ymax], origin='lower')
            plt.tick_params(direction='out', length=1, width=1)
            plt.draw()
        else:
            raise NotImplementedError

    def get_heat_timestep(self, step_idx=0):
        # self.normalize(step_idx=step_idx)
        plt.imshow(self.p[step_idx, :, :].transpose(), cmap='hot', interpolation='nearest',
                   extent=[self.xmin, self.xmax, self.ymin, self.ymax], zorder=0, origin='lower')
        if not self.tqdm:
            self.tqdm = tqdm(total=self.iter_steps)
        else:
            self.tqdm.update(1)

    def generate_heatmap_dyn(self, playtime=10.0):
        # interval = max( 10, int(self.params['dt'] * (playtime/self.params['T']) ) )

        fps = max(1, int(self.iter_steps / playtime))
        fps = min(fps, 30)

        print('Generating dynamic heatmap: ({} frames  @ {} fps)'.format(self.iter_steps, fps))
        self.anim = animation.FuncAnimation(self.fig, self.get_heat_timestep, frames=self.iter_steps)
        if self.params.get('store_logs', True):
            self.anim.save(self.dynheatmap_log_file, fps=fps, extra_args=['-vcodec', 'libx264'])
        self.tqdm = None

    def collect_step_data(self, step_idx=0):
        for thuman in self.humans:
            xval, yval = thuman.get_pos()
            self.count += 1
            if (xval > self.xmin and xval < self.xmax and yval > self.ymin and yval < self.ymax):
                xidx, yidx = self.terrain.idx_from_coords(xval, yval, saturate=False)
                if self.IS_HEATMAP_DYN:
                    self.p[step_idx, yidx, xidx] += 1
                else:
                    self.p[yidx, xidx] += 1
            else:
                pass

    @staticmethod
    def prettyprint(d, indent=0, prefix_str=''):
        for key, value in d.items():
            prefix_str += '\t ' * indent + str(key) + ':\n'
            if isinstance(value, dict):
                prefix_str += MonteCarlo.prettyprint(value, indent + 1, prefix_str)
            else:
                prefix_str += '\t' * (indent + 1) + str(value) + '\n'
        return prefix_str

    @staticmethod
    def is_jsonable(x):
        try:
            json.dumps(x)
            return True
        except (TypeError, OverflowError):
            return False

    def get_param_string(self):
        # return MonteCarlo.prettyprint(self.params)
        var_dict = vars(self)
        for tvar, tval in var_dict.items():
            if isinstance(tval, dict):
                continue
            elif MonteCarlo.is_jsonable(tval):
                self.params[tvar] = tval

        return json.dumps(self.params, indent=4, sort_keys=True)

    def print_params(self):
        # print( MonteCarlo.prettyprint(self.params) )
        print(self.params)

    def write_params(self):
        if self.params.get('store_logs', True):
            with open(self.param_log_file, 'w') as fp:
                fp.write(self.get_param_string())

    def run_experiment(self):
        print('Running experiment: {}'.format(self.exp_str))
        # Need to check size of matrix before uploading to "self.p"
        if self.params['lp_model'] == 'naive': # gavins method
            for _ in trange(self.num_iter):
                self.run_iter()
        elif self.params['lp_model'] == 'custom': # custom method
            fn = self.params['lp_filename']
            self.p, self.find_pt = load_heatmap(filename = fn, map_size = [self._x_shape, self._y_shape],
                                  map_res = self.params['res'],
                                  anchor_point=self.params['anchor_point'])
        elif self.params['lp_model'] == 'ring': # ring method
            self.p = np.zeros((self._x_shape, self._y_shape), dtype=float)
            [x_crds,y_crds] = np.meshgrid(np.linspace(self.params['xlims'][0], self.params['xlims'][1], self._x_shape),
                              np.linspace(self.params['ylims'][0], self.params['ylims'][1], self._y_shape))
            dist_mat = np.sqrt(x_crds**2 + y_crds**2)
            self.p[dist_mat <= self.params['ring_mobi'][0]] += 0.25 # center ring
            self.p[dist_mat <= self.params['ring_mobi'][1]] += 0.25 # middle ring
            self.p[dist_mat <= self.params['ring_mobi'][2]] += 0.25 # outter ring
            self.p[dist_mat <= self.params['ring_mobi'][3]] += 0.20 # containment ring
            # print(self.p)
            self.p = ndimage.gaussian_filter(self.p, sigma=8) # to make betta heatmap

        else:
            self.p = np.zeros((self._x_shape, self._y_shape), dtype=float)

        # generate lp from real model for comparison
        fn = self.params['lp_filename']
        self.comp_map, trash = load_heatmap(filename=fn, map_size=[self._x_shape, self._y_shape],
                                            map_res=self.params['res'],
                                            anchor_point=self.params['anchor_point'])
        self.comp_map = self.comp_map - np.min(self.comp_map)
        self.comp_map = self.comp_map/np.max(self.comp_map)

        # normalize heatmap to sum to 1
        self.p = self.p - np.min(self.p)
        self.p = self.p/np.max(self.p)

        self.searcher_class.go_forth(params=self.params) # sets sweep bounds and starting points
        while True:
            self.searcher_class.step_all(dt=self.params['dt']) # and this makes each step?
            if self.searcher_class.all_done():
                break
        # smooth trajectories for use in optimization step
        self.searcher_class.smooth_traj_all()

    def show_searcher_paths(self, show_orig=True, show_smooth=True):
        if show_orig:
            self.searcher_class.plot_all_2d()

        if show_smooth:
            self.searcher_class.smooth_traj_all()
            self.searcher_class.plot_all_smooth_2d()

        plt.draw()

    def save_history(self):
        if self.params.get('store_logs', True):
            np.save(self.history_log_file, self.p)

    def save_heatmap(self):
        if not self.IS_HEATMAP_DYN and self.params.get('store_logs', True):
            plt.savefig(self.heatmap_log_file, facecolor='w', edgecolor='w', transparent=True)

    def set_defaults(self):
        super().set_defaults()
        self.xmin, self.xmax = self.params['xlims']
        self.ymin, self.ymax = self.params['ylims']

        self.hmin, self.hmax = self.params['zlims']
        self.res = self.params['res']

        self.iter_steps = self.params['iter_steps']
        self.num_iter = self.params['num_iter']
        self.num_humans = self.params['num_humans']

        self._x_shape = int((self.xmax - self.xmin) / float(self.res))
        self._y_shape = int((self.ymax - self.ymin) / float(self.res))

        self.humans = None
        self.tqdm = None


def main():
    # params =    ({
    #     'exp_id' : 0,
    #     'exp_name' : 'test',
    #     'exp_description' : 'Experiment setup completed. Able to generate a dynamic heatmap over time showing the likelihood of a particla being at a particular point at a given time',
    # })

    # params.update( { 'xlims':(-5.0, 5.0), 'ylims':(-5.0, 5.0), 'zlims':(-5.0, 5.0), 'res':0.25 } )
    # params.update( { 'random_pos':True, 'num_iter':10000, 'num_humans':4 } )

    params = ({
        'exp_id': 1,
        'exp_name': 'test2',
        'exp_description': 'test',
    })

    # params.update({'xlims': (-10.0, 10.0), 'ylims': (-10.0, 10.0), 'zlims': (-10.0, 10.0), 'res': 0.1})
    # params.update({'random_pos': True, 'num_iter': 10000, 'num_humans': 5})

    # params.update( {
    #                 'xdot': [ random.random() * 1e-2, random.random() * 1e-2 ],
    #                 'xddot':  [ random.random() * -1e-2, random.random() * 1e-2 ]
    #             } )

    mc = MonteCarlo(params=params)

    mc.run_experiment()
    mc.write_params()
    mc.show_heatmap()

    # mc.generate_heatmap_dyn(playtime=25)


if __name__ == '__main__':
    main()
