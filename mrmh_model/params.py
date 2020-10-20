class Default:
    def __init__(self, params={}):
        self.params = params
        self.set_defaults()

    def set_defaults(self):
        self.params.setdefault( 'xlims', (-600.0, 600.0) )
        self.params.setdefault( 'ylims', (-600.0, 600.0) )
        self.params.setdefault( 'zlims', (0, 120) )
        self.params.setdefault( 'res', 25 )

        self.params.setdefault( 'dt', 1 ) # dt for searcher motion
        self.params.setdefault( 'T', 50 )

        self.params.setdefault( 'iter_steps', int(self.params['T']/self.params['dt']) )
        self.params.setdefault( 'num_iter', 300 )

        self.params.setdefault( 'num_humans', 1 )

        ##############################
        self.params.setdefault( 'num_searchers', 2 )
        self.params.setdefault( 'num_robots', 2)
        ##############################

        self.params.setdefault( 'max_plan_nodes', 100)
        self.params.setdefault( 'terrainType', 'real')
        self.params.setdefault( 'save_terrain', False)
        self.params.setdefault( 'save_MCdata', False)
        self.params.setdefault( 'load_MCdata', False)
        self.params.setdefault( 'store_logs', False)
        self.params.setdefault( 'random_pos', True)
        self.params.setdefault( 'random_motion_init', True)

        # params.update({'obstacles': [infcyl1,
        #     # infcyl2, \
        #     # sphere1, \
        #     # sphere2, \
        #     # infcyl3, \
        #     # infcyl4, \
        #     # infcyl5, \
        #     infcyl6 \
        #     ]})
        # params.update({'obstacles': [infcyl1, infcyl6]})

        '''
        additions to original codebase
        '''
        # searcher path parameters
        self.params.setdefault('sweep_num', 3)  # for grouped method
        self.params.setdefault('search_method', 'grouped' ) # 'sweep' or 'grouped'
        self.params.setdefault('searcher_path_num', 200 ) # number of points for ind. searcher path

        # gis/run paramters
        # self.params.setdefault('anchor_point', [36.891640, -81.524214]) # hmpark
        self.params.setdefault('anchor_point', [37.197730, -80.585233]) # kentland
        # self.params.setdefault('anchor_point', [36.660460, -81.543921]) # grayson
        self.params.setdefault('heading', 0)  # direction of trajectories from mag north
        self.params.setdefault('stats_name', 'kentland') # saves risk-cost and waypoint related data in json upon finishing


        # drone path parameters
        self.params.setdefault('planning_min_height', 15)  # min meters above terrain to plan with
        self.params.setdefault('planning_max_height', 30)  # max meters above terrain to plan with
        self.params.setdefault('path_style', 'rc')  # 'sweep' for lawn mower, 'basic' for optimized, or 'rc' for above searchers
        self.params.setdefault('drone_sweeps', 4)  # number of path 'ticks' for sweep method of path planning
        self.params.setdefault('sweep_height', 20)  # height above terrain used during sweeps
        self.params.setdefault('path_interp_res', 75) # resolution to interpolate drone paths to (total number of points)
        self.params.setdefault('total_path_len', 1500) # max length of naive case path length

        # optimization parameters
        self.params.setdefault('opt_iterations', 1)  # iterations to optimize drone paths (default was 500)
        self.params.setdefault('learning_rate', 3)  # learning rate for adam
        self.params.setdefault('weight_decay', 5e-3)  # weight decay (l2 penalty)
        self.params.setdefault('test_fraction', 0.10)  # percentage of test points in batch
        self.params.setdefault('train_fraction', 0.90)  # percentage of train points in batch


        # lost person model parameters
        self.params.setdefault('lp_model', 'custom') # options: 'custom', 'naive', 'ring'
        self.params.setdefault('lp_filename', 'C:\\Users\\Larkin\\planning_llh_bgc\\LP model\\analysis\\outputs\\kentland_hiker\\ic_2_con_hiker_t8.csv') # filename to load for heatmap data
        # should make sure the heatmap file used is derived from the correct BWLF mat file
        self.params.setdefault('lin_feat_filename', 'C:\\Users\\Larkin\\ags_grabber\\matlab_data_locale\\BW_LFandInac_Zelev_kentland.mat') # filename used for linear features/inac
        self.params.setdefault('ring_mobi', [0.6e3, 1.8e3, 3.2e3, 9.9e3]) # mobility for ring model heatmap
        self.params.setdefault('lp_threshold', 0.2) # hotspot cutoff threshold on normalized heatmap (ind values in 0-1)
        self.params.setdefault('t_to_f_iter', 5000) # number of iterations to settle on time to find
        # self.params.setdefault('drone_fov_radius', 5) # radius at which drone can detect lost person (scales with altitude, x=0 intercept)
        self.params.setdefault('drone_fov_alt', 0.463647) # altitude at which drone sensing radius increases by 2 (slope of fov cone)
        self.params.setdefault('searcher_fov_radius', 10) # radius at which searcher can detect lost person
        self.params.setdefault('drone_speed', 8) # in units of m per second
        self.params.setdefault('searcher_speed', 1.4) # in units of m per second


        # data parameters
        self.params.setdefault('plot_data', False) # plots data in plotly viewer upon finishing
        self.params.setdefault('save_data', True) # saves risk-cost and waypoint related data in json upon finishing
        self.params.setdefault('save_folder', 'kentland_n3_s2_rc') # which folder to save data in


        