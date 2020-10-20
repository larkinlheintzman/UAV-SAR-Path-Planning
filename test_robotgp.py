from larrt import planning
import pickle as pkl
import json

from mrmh_model import terrain, human, montecarlo, searcher
import plotting_utils as plotter
import waypoint_maker
import matplotlib.pyplot as plt
from scouter.terrain_viewer import plot_all
from matplotlib.backends.backend_pdf import PdfPages

from mrmh_model import montecarlo as MC
import numpy as np
from mrmh_model import searcher
import torch
from gp import beziergp
from gp import robotgp
import sys
from mrmh_model.params import Default
import datetime as dt
import time, os
import pdb

def multipage(filename, figs=None, dpi=200):
    pp = PdfPages(filename)
    if figs is None:
        figs = [plt.figure(n) for n in plt.get_fignums()]
    for fig in figs:
        try:
            fig.savefig(pp, format='pdf')
        except:
            print('At least one of the figures could not be saved to PDF')
    pp.close()

def objective_printer(rgp_object = None, comp_time = 0.0, iteration = 0, folder = 'blah',
                      time_to_find = 0, find_percentage = 0,  save_data = False):
    # prints and/or saves the optimization objective values
    rgp = rgp_object

    min_Xstar = rgp.min_Xstar[:,2].reshape(np.sqrt(rgp.min_Xstar.shape[0]).astype(np.int),-1).cpu()
    # find_x = min_Xstar[rgp.mc_handle.find_pt[0], rgp.mc_handle.find_pt[1]]

    ystar_mu_min, ystar_cov_min = rgp.eval_batchless(rgp.min_X,rgp.min_Xstar)
    ystar_mu_first, ystar_cov_first = rgp.eval_batchless(rgp.first_X,rgp.first_Xstar)
    ystar_mu_prev, ystar_cov_prev = rgp.eval_batchless(rgp.prev_X,rgp.prev_Xstar)
    ystar_mu_curr, ystar_cov_curr = rgp.eval_batchless(rgp.X,rgp.Xstar)

    # undo log odds into just prb
    find_mat_min = 1 - (1/(1 + np.exp(ystar_mu_min)))
    find_mat_first = 1 - (1/(1 + np.exp(ystar_mu_first)))
    find_mat_prev = 1 - (1/(1 + np.exp(ystar_mu_prev)))
    find_mat_curr = 1 - (1/(1 + np.exp(ystar_mu_curr)))

    risk_cost_min = rgp.compute_risk_cost_batchless(ystar_mu_min, ystar_cov_min).detach().cpu().numpy()
    risk_cost_first = rgp.compute_risk_cost_batchless(ystar_mu_first, ystar_cov_first).detach().cpu().numpy()
    risk_cost_prev = rgp.compute_risk_cost_batchless(ystar_mu_prev, ystar_cov_prev).detach().cpu().numpy()
    risk_cost_curr = rgp.compute_risk_cost_batchless(ystar_mu_curr, ystar_cov_curr).detach().cpu().numpy()

    # this only makes sense if paths were optimized
    find_ratio_curr = (find_mat_curr/find_mat_prev)/((1 - find_mat_curr)/(1 - find_mat_prev)) # ratio of successfully finding lp over false positives
    find_ratio_total = (find_mat_min/find_mat_first)/((1 - find_mat_min)/(1 - find_mat_first)) # same thing but compare first and best

    # take sample from real lp map
    lp_dist = rgp.mc_handle.comp_map/np.sum(rgp.mc_handle.comp_map)
    # compare chance person is in location and chance we *think* person is in location
    search_prb_mat = lp_dist*find_mat_min

    # rgp.min_risk_cost = rgp.risk_cost_data.detach().cpu.numpy().item()
    # rgp.min_risk_paths = rgp.robot_points_data

    # rgp._min_scaled_risk_cost = rgp.scaled_risk_cost_data
    # rgp._min_scaled_path_length_cost = rgp.scaled_path_length_cost_data

    # wherever find_ratio is > 1 we have a higher chance of actually finding the person vs getting a false positive (or something like that)

    save_dict = {'min_cost_bcls' : risk_cost_min.item(),
                 'curr_cost_bcls' : risk_cost_curr.item(),
                 'curr_delta_bcls' : np.abs(risk_cost_prev - risk_cost_curr),
                 'init_cost_bcls' : risk_cost_first.item(),
                 'delta_cost_bcls' : np.abs(risk_cost_first - risk_cost_min),

                 'min_cost_old' : rgp.min_risk_cost.item(),
                 'min_risk_old' : rgp._min_scaled_risk_cost.item(),
                 'min_len_old' : rgp._min_scaled_path_length_cost.item(),

                 'find_percentage_curr_old' : np.count_nonzero(find_ratio_curr>=1)/np.size(find_ratio_curr),
                 'mean_find_ratio_curr_old' : np.mean(find_ratio_curr),
                 'find_percentage_total_old' : np.count_nonzero(find_ratio_total>=1)/np.size(find_ratio_total),
                 'mean_find_ratio_total_old' : np.mean(find_ratio_total),

                 'search_advantage' : np.sum(search_prb_mat),

                 'time_to_find' : time_to_find,
                 'find_percentage' : find_percentage,

                 'comp_time' : comp_time
                 }

    if save_data:
        filename = dt.datetime.now().strftime("%d_%m_%Y-%I_%M_%S_%p_")
        filename += "d-" + rgp.params['path_style'] + \
                    "_l-" + rgp.params['stats_name'] + \
                    "_i-" + str(iteration) + ".json"
        # force create directory
        os.makedirs('results/' + folder, exist_ok=True)
        with open('results/' + folder + '/' + filename, 'w') as f:
            json.dump(save_dict, f)

    return save_dict


def main(iteration = 0, parameters = -1):

    # iteration = 1

    stime = time.time()
    params = parameters
    # params = ({
    #     'exp_id': 0,
    #     'exp_name': 'test',
    #     'exp_description': 'Generate a heatmap',
    # })
    # params = Default(params).params

    mc = MC.MonteCarlo(params=params) # calls terrain builder
    mc.run_experiment()

    planner = planning.Planning(params, on_terrain=mc.terrain, mode='TOTALDIST') # also calls terrain builder...
    rgp = robotgp.RobotGP(mc, planner, _stime = stime, parameters = params)
    rgp.collect_trainData() # sets out paths for each robot

    rgp.optimize_risk_cost_grad(_show_detail=True)

    robot_paths_local = waypoint_maker.write_file(rgp_object = rgp, terrain_class = mc.terrain, filename = 'waypoints.json') # write waypoints to file

    # write serialized files for plotter to run
    # with open("scouter/bozoo_save_{}.pkl".format(params['anchor_point']),'wb') as f:
    #     pkl.dump({'params' : params,
    #               'mc_object' : mc,
    #               'robot_paths' : robot_paths_local,
    #               'searcher_paths' : mc.searcher_class.searchers_list}, f)

    etime = time.time()
    comp_time = etime - stime
    ttf, srpc = rgp.time_to_find(robot_paths = robot_paths_local, searcher_paths = mc.searcher_class.searchers_list)

    print("Total time required: {}".format(comp_time))

    objective_printer(rgp_object = rgp, comp_time = comp_time,
                      iteration = iteration,
                      folder = rgp.params['save_folder'],
                      save_data = rgp.params['save_data'],
                      time_to_find = ttf,
                      find_percentage = srpc)

    if rgp.params['plot_data']:
        plot_all(parameters = params,
                 mc_object = mc,
                 robot_paths = robot_paths_local,
                 searcher_paths = mc.searcher_class.searchers_list,
                 smooth_paths = False,
                 show_heatmap = True,
                 show_contours = True,
                 cs_name = 'thermal'
                 )
    rgp.garbage_cleanup()
    del rgp, planner, mc


if __name__ == "__main__":

    kentland_heatmap = 'C:\\Users\\Larkin\\planning_llh_bgc\\LP model\\analysis\\outputs\\kentland_hiker\\ic_2_con_hiker_t8.csv'
    hmpark_heatmap = 'C:\\Users\\Larkin\\planning_llh_bgc\\LP model\\analysis\\outputs\\hmpark_hiker\\ic_2_con_hiker_t8.csv'
    # self.params.setdefault('lp_filename', 'C:\\Users\\Larkin\\planning_llh_bgc\\LP model\\analysis\\outputs\\kentland_hiker\\ic_2_con_hiker_t8.csv')
    # self.params.setdefault('lin_feat_filename', 'C:\\Users\\Larkin\\ags_grabber\\matlab_data_locale\\BW_LFandInac_Zelev_kentland.mat')
    n_max = 6
    s_max = 2
    global_fail_max = 1000
    global_fails = 0
    avg_runs = 5
    start_time = time.time()

    # for n in range(1, n_max + 1):
    #     for s in range(2,s_max + 1):
    #             params = ({
    #                 'save_folder': 'kentland_n{}_s{}_basic'.format(n,s),
    #                 'lp_model': 'custom',
    #                 'opt_iterations': 25,
    #                 'path_style': 'basic',
    #                 'stats_name': 'kentland',
    #                 'anchor_point': [37.197730, -80.585233], # kentland
    #                 'num_searchers': s,
    #                 'num_robots': n,
    #                 'lp_filename': kentland_heatmap
    #             })
    #             params = Default(params).params
    #
    #             counter = 0
    #             while counter < avg_runs and global_fails <= global_fail_max: # number of averaging runs
    #                     torch.cuda.empty_cache()
    #                     torch.cuda.ipc_collect()
    #                     try:
    #                         try:
    #                             main(iteration = counter, parameters=params)
    #                         except RuntimeError as e:
    #                             print("\n\n ------ bad memory, re trying ------\n")
    #                             global_fails += 1
    #                             continue
    #                         counter += 1
    #                     except AttributeError as e:
    #                         print("\n\n ------- bad optimization, re trying ---------- \n")
    #                         global_fails += 1

    #-----------------------------------------------------------------------------------------------
    #
    # for n in range(1, n_max + 1):
    #     for s in range(2,s_max + 1):
    #             params = ({
    #                 'save_folder': 'kentland_n{}_s{}_unopt'.format(n,s),
    #                 'lp_model': 'custom',
    #                 'opt_iterations': 3,
    #                 'path_style': 'basic',
    #                 'stats_name': 'kentland',
    #                 'anchor_point': [37.197730, -80.585233], # kentland
    #                 'num_searchers': s,
    #                 'num_robots': n,
    #                 'lp_filename': kentland_heatmap
    #             })
    #             params = Default(params).params
    #
    #             counter = 0
    #             while counter < avg_runs and global_fails <= global_fail_max: # number of averaging runs
    #                     torch.cuda.empty_cache()
    #                     torch.cuda.ipc_collect()
    #                     try:
    #                         try:
    #                             main(iteration = counter, parameters=params)
    #                         except RuntimeError as e:
    #                             print("\n\n ------ bad memory, re trying ------\n")
    #                             global_fails += 1
    #                             continue
    #                         counter += 1
    #                     except AttributeError as e:
    #                         print("\n\n ------- bad optimization, re trying ---------- \n")
    #                         global_fails += 1

    #-----------------------------------------------------------------------------------------------

    # for n in range(1, n_max + 1):
    #     for s in range(2,s_max + 1):
    #             params = ({
    #                 'save_folder': 'kentland_n{}_s{}_sweep'.format(n,s),
    #                 'lp_model': 'custom',
    #                 'opt_iterations': 1,
    #                 'path_style': 'sweep',
    #                 'stats_name': 'kentland',
    #                 'anchor_point': [37.197730, -80.585233], # kentland
    #                 'num_searchers': s,
    #                 'num_robots': n,
    #                 'lp_filename': kentland_heatmap
    #             })
    #             params = Default(params).params
    #
    #             counter = 0
    #             while counter < avg_runs and global_fails <= global_fail_max: # number of averaging runs
    #                     torch.cuda.empty_cache()
    #                     torch.cuda.ipc_collect()
    #                     try:
    #                         try:
    #                             main(iteration = counter, parameters=params)
    #                         except RuntimeError as e:
    #                             print("\n\n ------ bad memory, re trying ------\n")
    #                             global_fails += 1
    #                             continue
    #                         counter += 1
    #                     except AttributeError as e:
    #                         print("\n\n ------- bad optimization, re trying ---------- \n")
    #                         global_fails += 1
    #
    # #-----------------------------------------------------------------------------------------------
    #
    # for n in range(1, n_max + 1):
    #     for s in range(2,s_max + 1):
    #             params = ({
    #                 'save_folder': 'kentland_n{}_s{}_rc'.format(n,s),
    #                 'lp_model': 'custom',
    #                 'opt_iterations': 1,
    #                 'path_style': 'rc',
    #                 'stats_name': 'kentland',
    #                 'anchor_point': [37.197730, -80.585233], # kentland
    #                 'num_searchers': s,
    #                 'num_robots': n,
    #                 'lp_filename': kentland_heatmap
    #             })
    #             params = Default(params).params
    #
    #             counter = 0
    #             while counter < avg_runs and global_fails <= global_fail_max: # number of averaging runs
    #                     torch.cuda.empty_cache()
    #                     torch.cuda.ipc_collect()
    #                     try:
    #                         try:
    #                             main(iteration = counter, parameters=params)
    #                         except RuntimeError as e:
    #                             print("\n\n ------ bad memory, re trying ------\n")
    #                             global_fails += 1
    #                             continue
    #                         counter += 1
    #                     except AttributeError as e:
    #                         print("\n\n ------- bad optimization, re trying ---------- \n")
    #                         global_fails += 1
    #
    # # HMPARK RUNS-----------------------------------------------------------------------------------------------

    # for n in range(1, n_max + 1):
    #     for s in range(2,s_max + 1):
    #             params = ({
    #                 'save_folder': 'hmpark_n{}_s{}_basic'.format(n,s),
    #                 'lp_model': 'custom',
    #                 'opt_iterations': 25,
    #                 'path_style': 'basic',
    #                 'stats_name': 'hmpark',
    #                 'anchor_point': [36.891640, -81.524214],  # hmpark
    #                 'num_searchers': s,
    #                 'num_robots': n,
    #                 'lp_filename': hmpark_heatmap
    #             })
    #             params = Default(params).params
    #
    #             counter = 0
    #             while counter < avg_runs and global_fails <= global_fail_max: # number of averaging runs
    #                     torch.cuda.empty_cache()
    #                     torch.cuda.ipc_collect()
    #                     try:
    #                         try:
    #                             main(iteration = counter, parameters=params)
    #                         except RuntimeError as e:
    #                             print("\n\n ------ bad memory, re trying ------\n")
    #                             global_fails += 1
    #                             continue
    #                         counter += 1
    #                     except AttributeError as e:
    #                         print("\n\n ------- bad optimization, re trying ---------- \n")
    #                         global_fails += 1
    #
    # # -----------------------------------------------------------------------------------------------

    for n in range(5, n_max + 1):
        for s in range(2, s_max + 1):
            params = ({
                'save_folder': 'hmpark_n{}_s{}_unopt'.format(n, s),
                'lp_model': 'custom',
                'opt_iterations': 3,
                'path_style': 'basic',
                'stats_name': 'hmpark',
                'anchor_point': [36.891640, -81.524214],  # hmpark
                'num_searchers': s,
                'num_robots': n,
                'lp_filename': hmpark_heatmap
            })
            params = Default(params).params

            counter = 0
            while counter < avg_runs and global_fails <= global_fail_max:  # number of averaging runs
                torch.cuda.empty_cache()
                torch.cuda.ipc_collect()
                try:
                    main(iteration=counter, parameters=params)
                    counter += 1
                except AttributeError as e:
                    print("\n\n ------- bad optimization, re trying ---------- \n")
                    global_fails += 1

    # -----------------------------------------------------------------------------------------------

    for n in range(1, n_max + 1):
        for s in range(2, s_max + 1):
            params = ({
                'save_folder': 'hmpark_n{}_s{}_sweep'.format(n, s),
                'lp_model': 'custom',
                'opt_iterations': 1,
                'path_style': 'sweep',
                'stats_name': 'hmpark',
                'anchor_point': [36.891640, -81.524214],  # hmpark
                'num_searchers': s,
                'num_robots': n,
                'lp_filename': hmpark_heatmap
            })
            params = Default(params).params

            counter = 0
            while counter < avg_runs and global_fails <= global_fail_max:  # number of averaging runs
                torch.cuda.empty_cache()
                torch.cuda.ipc_collect()
                try:
                    main(iteration=counter, parameters=params)
                    counter += 1
                except AttributeError as e:
                    print("\n\n ------- bad optimization, re trying ---------- \n")
                    global_fails += 1

    # -----------------------------------------------------------------------------------------------

    for n in range(1, n_max + 1):
        for s in range(2, s_max + 1):
            params = ({
                'save_folder': 'hmpark_n{}_s{}_rc'.format(n, s),
                'lp_model': 'custom',
                'opt_iterations': 1,
                'path_style': 'rc',
                'stats_name': 'hmpark',
                'anchor_point': [36.891640, -81.524214],  # hmpark
                'num_searchers': s,
                'num_robots': n,
                'lp_filename': hmpark_heatmap
            })
            params = Default(params).params

            counter = 0
            while counter < avg_runs and global_fails <= global_fail_max:  # number of averaging runs
                torch.cuda.empty_cache()
                torch.cuda.ipc_collect()
                try:
                    main(iteration=counter, parameters=params)
                    counter += 1
                except AttributeError as e:
                    print("\n\n ------- bad optimization, re trying ---------- \n")
                    global_fails += 1