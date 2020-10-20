import numpy as np
from pyproj import Proj
from pandas import DataFrame
import json
import math
import matplotlib.pyplot as plt

# import cartopy.crs as ccrs
# from cartopy.mpl.gridliner import LONGITUDE_FORMATTER, LATITUDE_FORMATTER
# import cartopy.io.img_tiles as cimgt


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


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return [rho, phi]

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return [x, y]

def latlon_translate(ll_point, trans_dist):
    scale_factor = 111111  # thanks stack overflow
    lat = ll_point[0]
    lon = ll_point[1]
    # we're assuming t_x is the east direction and t_y is the north direction
    t_x = trans_dist[0]
    t_y = trans_dist[1]
    # translated point becomes:
    return [lat + t_y/scale_factor, lon + t_x/(scale_factor*math.cos(math.radians(lat)))]

def write_file(rgp_object, terrain_class, filename = 'waypoints.json'):
    # step one: get trajectories:
    rgp = rgp_object
    terrain = terrain_class
    robot_paths = []
    start_idx = 0
    for path_len in rgp.robot_path_len:
        end_idx = start_idx + path_len
        rpath = rgp.min_risk_paths[start_idx:end_idx]
        robot_paths.append([(rpath[i, 0], rpath[i, 1], rpath[i, 2]) for i in range(len(rpath[:,2]))])
        start_idx = end_idx

    # nice. now reference terrain center point to create lat long trajectories
    center_point = terrain.params['anchor_point'] # if terrain isnt real, need to specify our own center point
    base_altitude = 30 # lowest altitude (tree height)
    # ok so one thing that might be missing here is direction mapping. we'll assume for now that the x axis is pointing
    # east and the y axis is pointing north, but that might not be the case. this is a fix-it-later situation
    robot_paths_ll = []
    robot_paths_local = []
    for rbt in robot_paths:
        # here is where we would do the direction correction
        rbt_ll = []
        rbt_local = []
        for rbt_x, rbt_y, rbt_z in rbt:

            rbt_lat, rbt_lon = latlon_translate(center_point, point_rotation([0, 0], [rbt_x, rbt_y], ang=terrain.params['heading']))
            rbt_lx, rbt_ly = point_rotation([0, 0], [rbt_x, rbt_y], ang=terrain.params['heading'])
            # rbt_lx = rbt_lx - terrain.params['xlims'][0] # re-center to 0,0
            # rbt_ly = rbt_ly - terrain.params['ylims'][0]

            rbt_local.append((rbt_lx, rbt_ly, rbt_z))
            rbt_ll.append((rbt_lat, rbt_lon, rbt_z+base_altitude))

        robot_paths_ll.append(rbt_ll)
        robot_paths_local.append(rbt_local)

    waypoints = {}
    for i,rbt_path in enumerate(robot_paths_ll): # for each robot
        rbt_waypoints = {} # make new waypoint set
        counter = 0
        for lat, lon, alt in rbt_path:
            rbt_waypoints.update({"waypoint_{}".format(counter) : [counter,-1,lat,lon,alt]})
            counter = counter + 1
        waypoints.update({"robot_{}".format(i) : rbt_waypoints}) # another level to dictionary

    waypoints = {"robot_list" : waypoints}
    # print(waypoints)
    with open(filename, "w") as f:
        json.dump(waypoints, f)

    # a temporary setup to get paths into the 3d viewer code
    return robot_paths_local



if __name__ == "__main__":

    vizualize_waypoints('waypoints.json')

    '''
    myProj = Proj("+proj=utm +zone={}, +south +ellps=WGS84 +datum=WGS84 +units=m +no_defs".format('17S'))
    home_gps = [37.228747, -80.409215]
    home_x_utm, home_y_utm = myProj(home_gps[1], home_gps[0]) # convert home to utm coords
    start_utm = [home_x_utm, home_y_utm, '17S']

    radius_gps = [37.228796, -80.420558] # point to survey out to (whittemore hall, gps format)
    radius_x_utm, radius_y_utm = myProj(radius_gps[1], radius_gps[0]) # convert radius to utm coords
    radius_utm = [radius_x_utm, radius_y_utm, '17S'] # extent of search
    radius = np.sqrt((start_utm[0] - radius_utm[0])**2 + (start_utm[1] - radius_utm[1])**2)

    x_utm = np.array([start_utm[0] + pol2cart(radius, pi)[0] for pi in np.linspace(0,2*np.pi,10)])
    y_utm = np.array([start_utm[1] + pol2cart(radius, pi)[1] for pi in np.linspace(0,2*np.pi,10)])

    print("utm radius")
    print(radius)

    lon, lat = myProj(x_utm, y_utm, inverse=True)

    print("max (lat, long) dist of waypoints:")
    max_dist = -1
    R = 6373.0
    for i in range(len(lon) - 1):
        dlon = math.radians(lon[i]) - math.radians(lon[i+1])
        dlat = math.radians(lat[i]) - math.radians(lat[i+1])
        a = math.sin(dlat / 2)**2 + math.cos(math.radians(lat[i+1])) * math.cos(math.radians(lat[i])) * math.sin(dlon / 2)**2
        c = 2* math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist = R * c
        if dist >= max_dist:
            max_dist = dist
    print(max_dist*1000)
    print("-----------------")

    x_utm_conv, y_utm_conv = myProj(lon, lat)

    mse = np.sqrt(np.sum([(x_utm[i] - x_utm_conv[i])**2 for i in range(len(x_utm))]))
    print("MSRE of conversion: " + str(mse))

    # save the lat lon gps points to json file with extra bits
    # altitude will come from actual MRMH code
    waypoints = {}
    for i in range(len(lon)):
        waypoints.update({"waypoint_{}".format(i) : [i,-1,lat[i],lon[i],30]})

    waypoints = {"waypoint_list" : waypoints}
    print(waypoints)
    filename = "waypoint_test.json"
    with open(filename, "w") as f:
        json.dump(waypoints, f)
    '''