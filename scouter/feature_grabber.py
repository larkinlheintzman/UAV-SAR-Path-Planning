from arcgis_terrain import get_terrain_map
from arcgis.features import FeatureLayer
from arcgis.gis import GIS
from arcgis_terrain import lat_lon2meters
from arcgis_terrain import meters2lat_lon
from arcgis.geometry.filters import envelope_intersects
import arcgis.geometry
import numpy as np
import plotly.graph_objects as go
import plotly.express as px
# from arcgis_terrain import point_rotation
from scipy import interpolate
from matplotlib import path
import math
import json

'''
THIS FILE IS DUPLICATED IN ANOTHER PROJECT AT: https://git.caslab.ece.vt.edu/hlarkin3/ags_grabber
'''



def point_rotation(origin, pt, ang):
    # returns the pt rotated about the origin by ang (ang in degrees)
    c = math.cos(math.radians(ang))
    s = math.sin(math.radians(ang))
    # translate to origin
    pt_temp = [pt[0] - origin[0], pt[1] - origin[1]]
    pt_spun = [ pt_temp[0]*c - pt_temp[1]*s, pt_temp[0]*s + pt_temp[1]*c ]
    # translate back to frame
    pt_spun = [pt_spun[0] + origin[0], pt_spun[1] + origin[1]]
    return pt_spun

def grab_features(anchor_point, extent, sample_dist = 10, heading = 0, save_files = False, plot_data = False):
    roads_url = "https://carto.nationalmap.gov/arcgis/rest/services/transportation/MapServer/30"
    river_url = "https://hydro.nationalmap.gov/arcgis/rest/services/nhd/MapServer/6"
    water_url = "https://hydro.nationalmap.gov/arcgis/rest/services/nhd/MapServer/9"
    powerlines_url = "https://services1.arcgis.com/Hp6G80Pky0om7QvQ/ArcGIS/rest/services/Electric_Power_Transmission_Lines/FeatureServer/0"
    railroads_url = "https://carto.nationalmap.gov/arcgis/rest/services/transportation/MapServer/35"

    # adding water_url twice, once for boundaries and once for linear features
    # the layer named 'lakes' gets boundary treatment
    url_list = [river_url, roads_url, water_url, water_url, powerlines_url, railroads_url]
    name_list = ['rivers', 'roads', 'lakes', 'lake_bdd', 'powerlines', 'railroads']

    gis = GIS(username="larkinheintzman",password="Meepp97#26640") # linked my arcgis pro account
    ap_meters = lat_lon2meters(anchor_point[0], anchor_point[1])
    file_id = str(anchor_point) + "_" + str(extent/1000) + "km"

    scale_factor = 3/20 # factor to get 6.66667m mapping from 1m mapping (1/6.6667)
    viz_map = np.zeros([np.ceil(scale_factor*extent).astype(np.int),np.ceil(scale_factor*extent).astype(np.int)])

    for i,url in enumerate(url_list):

        # binary map, will use feature coords to populate (one per layer)
        bin_map = np.zeros([np.ceil(scale_factor*extent).astype(np.int),np.ceil(scale_factor*extent).astype(np.int)])

        geom = arcgis.geometry.Polygon({'spatialReference': {"wkid" : 3857},
                             'rings': [[
                                [ap_meters[0] - (extent/2), ap_meters[1] - (extent/2)],
                                [ap_meters[0] - (extent/2), ap_meters[1] + (extent/2)],
                                [ap_meters[0] + (extent/2), ap_meters[1] + (extent/2)],
                                [ap_meters[0] + (extent/2), ap_meters[1] - (extent/2)],
                                [ap_meters[0] - (extent/2), ap_meters[1] - (extent/2)]
                             ]]})

        lyr = FeatureLayer(url = url, gis = gis)
        geom_filter = envelope_intersects(geom, sr=geom['spatialReference'])

        q = []
        counter = 0
        while not q and counter <= 5: # have to do this because arcgis is sketchy as hell and doesnt always come back
            try:
                q = lyr.query(return_count_only=False, return_ids_only=False, return_geometry=True,
                              out_sr='3857', geometry_filter=geom_filter)
            except json.decoder.JSONDecodeError as e:
                counter = counter + 1
                print("error on query: {}".format(e))
                print("{} layer failed on query, trying again ...".format(name_list[i]))
        if counter > 5 and not q:
            print("{} layer failed too many times, continuing".format(name_list[i]))
            # if save_files:
            #     fn = "map_layers/"+name_list[i]+"_data_"+file_id+".csv"
            #     np.savetxt(fn,bin_map,delimiter=",", fmt='%f')
            continue
        print("{} layer sucessfully queried".format(name_list[i]))

        # re-build into list of x-y values
        # feat_points = []
        query_dict = q.to_dict()
        for j,feat in enumerate(query_dict['features']):

            # pull feature points out of query, they have different keys...
            if 'paths' in feat['geometry'].keys():
                x_pts = [pt[0] for pt in feat['geometry']['paths'][0]]
                y_pts = [pt[1] for pt in feat['geometry']['paths'][0]]
                plot_points = np.array(feat['geometry']['paths'][0])
            else:
                x_pts = [pt[0] for pt in feat['geometry']['rings'][0]] # arcgis is stupid
                y_pts = [pt[1] for pt in feat['geometry']['rings'][0]]
                plot_points = np.array(feat['geometry']['rings'][0])

            # re-center on 0,0
            # x_pts = np.array(x_pts) - (ap_meters[0] - extent/2) # add these back in!
            # y_pts = np.array(y_pts) - (ap_meters[1] - extent/2)
            x_pts = np.array(x_pts) - (ap_meters[0]) # didnt need to remove extent, double counted it
            y_pts = np.array(y_pts) - (ap_meters[1])

            # rotate points about origin to establish heading
            [x_pts, y_pts] = point_rotation(origin = [0,0],pt = [x_pts, y_pts],ang = heading)

            # total length of feature ring/path, for interpolation along features
            total_len = np.sum(np.sqrt(np.sum(np.diff(np.array([x_pts, y_pts]).T, axis=0) ** 2, axis=1)))

            tck, u = interpolate.splprep([x_pts, y_pts], s=0, k=1)  # parametric interpolation
            u_new = np.arange(0, 1 + 1 / total_len, 1 / total_len)  # scaled discretization
            pts_interp = interpolate.splev(u_new, tck)

            # trim data to only within extents (because arcgis cant fuckin' do this)
            rm_mask = np.logical_or(pts_interp[0] < (-extent/2), pts_interp[0] > (extent/2)) # mask to remove points (x axis)
            rm_mask = np.logical_or(rm_mask,np.logical_or(pts_interp[1] < (-extent/2), pts_interp[1] > (extent/2))) # other axis mask
            x_pts = pts_interp[0][np.invert(rm_mask)]
            y_pts = pts_interp[1][np.invert(rm_mask)] # trim interpolated points

            if x_pts.shape[0] > 1: # if there are still points after trimming
                # if data is too short, add some points in the middle
                while x_pts.shape[0] < 4:
                    x_pt = (x_pts[0] + x_pts[1])/2 # average between first and second point
                    y_pt = (y_pts[0] + y_pts[1])/2
                    x_pts = np.insert(x_pts, 1, x_pt)
                    y_pts = np.insert(y_pts, 1, y_pt)

                # binarization step
                x_pts_idx = np.round(scale_factor*(x_pts + extent/2)).astype(np.int)-1
                y_pts_idx = np.round(scale_factor*(y_pts + extent/2)).astype(np.int)-1

                if name_list[i] == 'lakes':
                    # do boundary calculation for binary matrix (slow for large bounaries but whatever)
                    ring = path.Path(np.array([x_pts_idx, y_pts_idx]).T)

                    # test_pts is the rectangular matrix covering ring for boundary calculation
                    x_test, y_test = np.meshgrid(np.arange(np.min(x_pts_idx), np.max(x_pts_idx), 1) , np.arange(np.min(y_pts_idx), np.max(y_pts_idx), 1))
                    test_pts = np.array([x_test.flatten(), y_test.flatten()]).T
                    mask = ring.contains_points(test_pts)

                    # append points within boundary to index points, to fill in gaps
                    x_pts_idx = np.append(x_pts_idx,test_pts[mask,0])
                    y_pts_idx = np.append(y_pts_idx,test_pts[mask,1])

                # flip y axis because indices are fliped
                y_pts_idx = bin_map.shape[1] - y_pts_idx
                # remove any points outside limits of binary map (fixes round versus ceil issues)
                rm_mask = np.logical_or(x_pts_idx < 0, x_pts_idx >= bin_map.shape[1])
                rm_mask = np.logical_or(rm_mask, np.logical_or(y_pts_idx < 0, y_pts_idx >= bin_map.shape[0]))
                x_pts_idx = x_pts_idx[np.invert(rm_mask)]
                y_pts_idx = y_pts_idx[np.invert(rm_mask)]
                bin_map[y_pts_idx, x_pts_idx] = 1 # set indices to 1

        viz_map = 2*viz_map + bin_map # sum binary maps from each layer
        if save_files:
            fn = "map_layers/"+name_list[i]+"_data_"+file_id+".csv"
            np.savetxt(fn,bin_map,delimiter=",", fmt='%f')
            # with open("map_layers/"+name_list[i]+"_data.txt",'w') as f:
                # f.write(str(save_data))

    # save terrain as csv file (this method is pretty slow, but can compensate with interp)
    [e,x,y,data,ll_pt] = get_terrain_map(lat_lon=anchor_point,
                                         sample_dist = sample_dist,
                                         extent = extent,
                                         heading = heading)
    # flip elevation data up to down to match other layers
    # e = np.flipud(e)

    # interpolate terrain to match size/resolution of other layers
    f = interpolate.interp2d(np.arange(0, extent, sample_dist), np.arange(0, extent, sample_dist), e, kind='cubic')
    x_temp = np.linspace(0,extent,np.ceil(scale_factor*extent).astype(np.int)) # get correct size of terrain map
    y_temp = np.linspace(0,extent,np.ceil(scale_factor*extent).astype(np.int))
    e_interp = f(x_temp, y_temp)

    elv_filename = "C:\\Users\\Larkin\\planning_llh_bgc\\scouter\\map_layers\\elv_data_"+file_id+".csv"
    if save_files:
        np.savetxt(elv_filename,e_interp,delimiter=",", fmt='%f')

    if plot_data:
        terr_fig = px.imshow(e_interp)
        terr_fig.show()

        bbox = [[- (extent / 2), - (extent / 2)],
                [- (extent / 2), + (extent / 2)],
                [+ (extent / 2), + (extent / 2)],
                [+ (extent / 2), - (extent / 2)],
                [- (extent / 2), - (extent / 2)]]

        bin_fig = px.imshow(viz_map)
        bin_fig.show()

if __name__ == "__main__":
    anchor_point = [37.199877, -80.519467]
    extent = 10000 # used for all layers
    sample_dist = 100 # only used for terrain (is interpolated anyway, just base sample res)
    heading = 15
    grab_features(anchor_point = anchor_point, extent = extent, sample_dist = sample_dist,
                  heading = heading, save_files = True, plot_data = True)