import numpy as np
from arcgis_terrain import lat_lon2meters
from scipy import interpolate
'''
A thing to handle all the lost person model/heatmap stuff, making a custom one, loading Amanda's, and so on
'''

def create_heatmap(map_size):
    # map_size = self.p.shape
    print("building heatmap ...")
    h_map = np.random.randint(0,3,map_size)
    return h_map

def load_heatmap(filename, map_size, map_res, anchor_point):
    my_data = np.genfromtxt(filename, delimiter=',')

    heatmap_center = my_data[0:2]
    if np.linalg.norm(heatmap_center - anchor_point) >= 0.01:
        print("Might have GPS center mismatch!!!")
    # find_pt = my_data[2:4] # leave out tactics for generating real lost person location
    dim = int(np.sqrt(len(my_data[2:]))) # ASSUMING DATA IS MEANT TO BE SQUARE
    z_data = my_data[2:].reshape([dim,dim])

    # rescale z data to match map size
    scale_factor = map_res/(6.7*8)
    center_idx = [int(dim/2), int(dim/2)] # ICS is made on dead center

    scaled_dim = [np.ceil((map_size[0]/2)*scale_factor), np.ceil((map_size[1]/2)*scale_factor)] # number of 6.7m steps to get to map size
    if scaled_dim[0] >= center_idx[0]:
        print("not fixed. map scales are not right!")
    xcrds_center = np.arange(center_idx[0] - scaled_dim[0], center_idx[0] + scaled_dim[0], dtype=int)
    ycrds_center = np.arange(center_idx[1] - scaled_dim[1], center_idx[1] + scaled_dim[1], dtype=int)

    # interpolate
    f = interpolate.interp2d(xcrds_center, ycrds_center, z_data[xcrds_center,:][:,ycrds_center], kind='cubic')
    xcrds_interp = np.linspace(np.min(xcrds_center),np.max(xcrds_center),map_size[0])
    ycrds_interp = np.linspace(np.min(ycrds_center),np.max(ycrds_center),map_size[1])
    z_data_interp = f(xcrds_interp, ycrds_interp)

    # remap things to 0-10 scale (for hotspot calculations?)
    # z_data_interp = 10*((z_data_interp - np.min(z_data_interp))/np.max(z_data_interp))
    z_data_interp = (z_data_interp - np.min(z_data_interp))/np.max(z_data_interp)

    # cap find point at x coords
    # find_pt[0] = xcrds_center[0] if find_pt[0] <= xcrds_center[0] else find_pt[0]
    # find_pt[0] = xcrds_center[-1] if find_pt[0] >= xcrds_center[-1] else find_pt[0]
    #
    # find_pt[1] = ycrds_center[0] if find_pt[1] <= ycrds_center[0] else find_pt[1]
    # find_pt[1] = ycrds_center[-1] if find_pt[1] >= ycrds_center[-1] else find_pt[1]
    #
    # find_pt = np.floor(find_pt - xcrds_center[0]).astype(np.int) # put center in left hand corner
    find_pt = np.zeros(2) # trash for now
    return z_data_interp, find_pt # not sure if scaling is correct

if __name__ == "__main__":
    # stuff
    my_data = load_heatmap('C:\\Users\\Larkin\\planning_llh_bgc\\LP model\\analysis\\outputs\\ic_1_con_hiker_t12.csv', [100,100], 10, [37.197730, -80.585233])
    import matplotlib.pyplot as plt
    plt.imshow(my_data[0])
    plt.show()
    print("done")

