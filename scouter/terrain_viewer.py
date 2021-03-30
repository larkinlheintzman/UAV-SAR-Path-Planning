import numpy as np
import plotly.graph_objects as go
from mrmh_model.lp_model import create_heatmap
from arcgis_terrain import get_terrain_map
import pickle as pkl
from scipy.interpolate import griddata
from scipy import interpolate
from arcgis_terrain import lat_lon2meters
from scipy.io import loadmat


def linear_features(parameters):

    temp = loadmat(parameters['lin_feat_filename'])

    bwlf = temp['BWLF'] # linear features
    bwinac = temp['BWInac'] # inaccessible areas
    szelev = temp['sZelev'] # elevation

    dim = bwlf.shape[0]

    bwlf = np.fliplr(np.rot90(np.rot90(bwlf)))
    bwinac = np.fliplr(np.rot90(np.rot90(bwinac)))
    szelev = np.fliplr(np.rot90(np.rot90(szelev)))
    # bwlf = np.rot90(np.rot90(np.rot90(np.fliplr(np.rot90(bwlf)))))
    # bwinac = np.rot90(np.rot90(np.rot90(np.fliplr(np.rot90(bwinac)))))
    # szelev = np.rot90(np.rot90(np.rot90(np.fliplr(np.rot90(szelev)))))

    # rescale z data to match map size
    map_res = parameters['res']
    map_size = [np.int((parameters['xlims'][1] - parameters['xlims'][0])/map_res), np.int((parameters['ylims'][1] - parameters['ylims'][0])/map_res)]
    if map_size[0] != map_size[1]:
        print('not implemented for non-square maps')

    feat_res = 6.6666666666667
    scale_factor = map_res/feat_res # convert scales
    center_idx = [int(dim/2), int(dim/2)] # ics in map coords
    scaled_dim = [np.ceil((map_size[0]/2)*scale_factor), np.ceil((map_size[1]/2)*scale_factor)] # number of 6.7m steps to get to map size

    if scaled_dim[0] >= center_idx[0]:
        print("not fixed. map scales are not right!")

    xcrds_center = np.arange(center_idx[0] - scaled_dim[0], center_idx[0] + scaled_dim[0], dtype=int)
    ycrds_center = np.arange(center_idx[1] - scaled_dim[1], center_idx[1] + scaled_dim[1], dtype=int)

    # trim to correct size:
    lf_idx = np.nonzero(bwlf[xcrds_center,:][:,ycrds_center])
    inac_idx = np.nonzero(bwinac[xcrds_center,:][:,ycrds_center])

    # get elevation of feature points
    lf_elev_pts = szelev[xcrds_center,:][:,ycrds_center] * (bwlf[xcrds_center,:][:,ycrds_center] != 0)
    inac_elev_pts = szelev[xcrds_center,:][:,ycrds_center] * (bwinac[xcrds_center,:][:,ycrds_center] != 0)
    lf_elev_pts = lf_elev_pts[np.nonzero(lf_elev_pts)]
    inac_elev_pts = inac_elev_pts[np.nonzero(inac_elev_pts)]
    # normalize elevation to have zero at lowest point
    if lf_elev_pts.size != 0:
        lf_elev_pts = lf_elev_pts - np.min(lf_elev_pts)
    if inac_elev_pts.size != 0:
        inac_elev_pts = inac_elev_pts - np.min(inac_elev_pts)

    lf_pts = np.array(lf_idx)*feat_res + parameters['xlims'][0] # in units of meters now
    inac_pts = np.array(inac_idx)*feat_res + parameters['xlims'][0]

    # pair linear feature points and inac points with their elevation

    # lf_pts = np.vstack([lf_pts, lf_elev_pts])
    # inac_pts = np.vstack([inac_pts, inac_elev_pts])

    return lf_pts, inac_pts

def plot_all(parameters, mc_object, robot_paths, searcher_paths, title = "Sim", smooth_paths = True,
             show_heatmap = True, show_contours = False, cs_name = 'thermal'):

    # linear_features(parameters)

    plot_data = []
    color_list = ['blue', 'green', 'cyan', 'magenta', 'lime']
    terrain_data = mc_object.terrain.terrain_data
    heatmap_data = mc_object.p.T

    e = terrain_data[0]
    x = terrain_data[1]
    y = terrain_data[2]

    z_terr = e.T
    x_terr = np.linspace(-(np.max(x)-np.min(x))/2, (np.max(x)-np.min(x))/2, x.shape[0])
    y_terr = np.linspace(-(np.max(y)-np.min(y))/2, (np.max(y)-np.min(y))/2, y.shape[0])

    # heat_map = create_heatmap(z_terr.shape)
    # generate terrain function and use to lift features
    z_terr_func = interpolate.RectBivariateSpline(x_terr, y_terr, z_terr.T)


    for i,path in enumerate(robot_paths):
        z_traj = [val[2] for val in path]
        x_traj = [val[0] for val in path]
        y_traj = [val[1] for val in path]

        z_temp = []
        x_temp = []
        y_temp = []

        # lifting each z point to the terrain
        for j,pt in enumerate(path):
            x_idx = np.argmin(np.abs(x_terr - pt[0]))
            y_idx = np.argmin(np.abs(y_terr - pt[1]))
            if z_terr_func.ev(pt[0], pt[1]) >= (z_traj[j]-4):
                z_traj[j] = z_terr_func.ev(pt[0], pt[1]) + 10
                pass
            else:
                pass

            z_val = z_terr_func.ev(pt[0], pt[1]) + 1
            z_temp.append(z_val)
            x_temp.append(x_terr[x_idx])
            y_temp.append(y_terr[y_idx])

        # add some smoothing for long trajectories
        if smooth_paths:
            tck, unused = interpolate.splprep([x_traj, y_traj, z_traj], k=5, s=500)
            unew = np.linspace(0,1,30)
            smoothed_traj = interpolate.splev(unew, tck)
            x_traj_smoothed = smoothed_traj[0]
            y_traj_smoothed = smoothed_traj[1]
            z_traj_smoothed = smoothed_traj[2]
        else:
            x_traj_smoothed = x_traj
            y_traj_smoothed = y_traj
            z_traj_smoothed = z_traj

        # plot_data.append(go.Scatter3d(z = z_temp, x = x_temp, y = y_temp, mode="lines",
        #                               line=dict(color=color_list[i], width=1), name="Proj. {}".format(i+1)))
        plot_data.append(go.Scatter3d(z = z_traj_smoothed, x = x_traj_smoothed, y = y_traj_smoothed,# mode="lines",
                                      line=dict(color=color_list[i], width=4), name="Drone {}".format(i+1)))
        plot_data.append(go.Scatter3d(z = [z_traj_smoothed[0]], x = [x_traj_smoothed[0]], y = [y_traj_smoothed[0]],
                                      mode="markers", showlegend=False,
                                      marker=dict(color=color_list[i], size = 12, line=dict(color='black',width=8)),
                                      name="Drone {}".format(i+1)))

    if show_contours:

        # add surface plot data
        if show_heatmap:
            plot_data.append(go.Surface(z = z_terr, x = x_terr, y = y_terr, colorscale=cs_name,
                                        surfacecolor=heatmap_data, name = "Terrain", showscale=False,
                                        contours = {"z" : {"show" : True, "start": 0, "end": np.max(z_terr), "size": 5}}))
        else:
            plot_data.append(go.Surface(z = z_terr, x = x_terr, y = y_terr, colorscale=cs_name,
                                        name = "Terrain", showscale=False,
                                        contours = {"z" : {"show" : True, "start": 0, "end": np.max(z_terr), "size": 5}}))
    else:
        # add surface plot data
        if show_heatmap:
            plot_data.append(go.Surface(z = z_terr, x = x_terr, y = y_terr, colorscale=cs_name,
                                        surfacecolor=heatmap_data, name = "Terrain", showscale=False))
        else:
            plot_data.append(go.Surface(z = z_terr, x = x_terr, y = y_terr, colorscale=cs_name,
                                        name = "Terrain", showscale=False))

    # add searcher path data
    for i,srch in enumerate(searcher_paths):
        plot_data.append(go.Scatter3d(z = [elv + 1 for elv in srch.smd_history[2]], x = srch.smd_history[0],
                                      y = srch.smd_history[1], mode = "lines+markers",
                                      line=dict(color = "gray", width = 3), name="Searcher {}".format(i+1),
                                      marker=dict(color='gray', size = 2)))
        # add first points
        plot_data.append(go.Scatter3d(z = [srch.smd_history[2][0]], x = [srch.smd_history[0][0]],
                                      y = [srch.smd_history[1][0]], mode = "lines+markers", showlegend=False,
                                      line=dict(color = "gray", width = 3), name="Searcher {}".format(i+1),
                                      marker=dict(color='light gray', size = 12, line=dict(color='black',width=8))))

    # get linear feature info
    [lf_points, inac_points] = linear_features(parameters)

    lf_points = np.vstack([lf_points, z_terr_func.ev(lf_points[0], lf_points[1])+1])
    inac_points = np.vstack([inac_points, z_terr_func.ev(inac_points[0], inac_points[1])+1])


    plot_data.append(go.Scatter3d(z = lf_points[2], x = lf_points[0], y = lf_points[1],
                                  mode = "markers", name="L. Feat.", marker=dict(size = 3, color = 'skyblue')))

    plot_data.append(go.Scatter3d(z = inac_points[2], x = inac_points[0], y = inac_points[1],
                                  mode = "markers", name="Inac. Areas", marker=dict(size = 3, color = 'salmon')))

    # add ipp to plot
    x_ipp = (np.max(x_terr) + np.min(x_terr))/2.0
    y_ipp = (np.max(y_terr) + np.min(x_terr))/2.0
    z_ipp = z_terr_func.ev(x_ipp, y_ipp) + 4

    plot_data.append(go.Scatter3d(z = [z_ipp], x = [x_ipp], y = [y_ipp], name="IPP", mode = "markers",
                                  marker=dict(color="orange", size = 12, line=dict(color='black',width=2))))

    fig = go.Figure(data = plot_data)
    fig.update_layout(title=title, autosize=False, width=2000, height=1200,
                        margin=dict(l=100, r=100, b=65, t=90),
                        legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01),
                        scene_aspectmode='manual',
                        scene_aspectratio=dict(x=1, y=1, z=0.5))
    fig.show()


if __name__ == "__main__":
    # print("loading data")

    with open("bozoo_save_[37.476914, -80.865864].pkl",'rb') as f:
        loaded_data = pkl.load(f)

    plot_all(parameters = loaded_data['params'],
             mc_object = loaded_data['mc_object'],
             robot_paths = loaded_data['robot_paths'],
             searcher_paths = loaded_data['searcher_paths'],
             title = "bozoo, [37.210264, -80.559236]",
             show_heatmap = True,
             show_contours = True,
             cs_name = 'earth')

    # with open("bozoo_save_[37.474015, -80.868333].pkl",'rb') as f:
    #     loaded_data = pkl.load(f)
    #
    # plot_all(parameters = loaded_data['params'],
    #          mc_object = loaded_data['mc_object'],
    #          robot_paths = loaded_data['robot_paths'],
    #          searcher_paths = loaded_data['searcher_paths'],
    #          title = "Mt. Rogers, [36.65711, -81.543333]",
    #          show_heatmap = False,
    #          show_contours = True,
    #          cs_name = 'earth')
    #
    # with open("bozoo_save_[37.476261, -80.87046].pkl",'rb') as f:
    #     loaded_data = pkl.load(f)
    #
    # plot_all(parameters = loaded_data['params'],
    #          mc_object = loaded_data['mc_object'],
    #          robot_paths = loaded_data['robot_paths'],
    #          searcher_paths = loaded_data['searcher_paths'],
    #          title = "Bozoo [37.476914, -80.865864]",
    #          show_heatmap = False,
    #          show_contours = True,
    #          cs_name = 'earth')
