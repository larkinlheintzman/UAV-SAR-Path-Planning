import numpy as np
import time
import matplotlib
import pickle as pkl
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from arcgis_terrain import get_terrain_map
from scipy.interpolate import griddata

def update_plot(frame_number, plot, plot_data):
    # plot results
    plot[0].remove()
    x, y, z = plot_data[frame_number]
    # print(data)
    # grid_x, grid_y = np.mgrid[min(x):max(x):100j, min(y):max(y):100j]
    # points = np.array([x, y])
    # grid_z = griddata(points.transpose(), z, (grid_x, grid_y), method='cubic', fill_value=-1)
    # # clean up Z values (just for plotting)
    # rm_index = np.where(grid_z == -1)
    # grid_z = np.delete(grid_z, rm_index[0][:], axis=0)
    # grid_x = np.delete(grid_x, rm_index[0][:], axis=0)
    # grid_y = np.delete(grid_y, rm_index[0][:], axis=0)
    #
    # grid_z = np.delete(grid_z, rm_index[1][:], axis=1)
    # grid_x = np.delete(grid_x, rm_index[1][:], axis=1)
    # grid_y = np.delete(grid_y, rm_index[1][:], axis=1)

    plot[0] = ax.plot_surface(x,y,z,cmap='viridis')
    ax.set_xlim3d([np.min(x), np.max(x)])
    ax.set_ylim3d([np.min(y), np.max(y)])
    return plot
    # plt.show()
    # ------------ old version below
    # plot_handle[0].remove()
    # plot_handle[0] = ax.plot_surface(x[:,:,frame_number], y[:,:,frame_number], e[:,:,frame_number], cmap='viridis', vmin=np.min(e), vmax=np.max(e))
    # ax.set_xlim3d([np.min(x[:,:,frame_number]), np.max(x[:,:,frame_number])])
    # ax.set_ylim3d([np.min(y[:,:,frame_number]), np.max(y[:,:,frame_number])])


# Set up formatting for the movie files
# Writer = animation.writers['ffmpeg']
writer = animation.FFMpegWriter(fps = 10, metadata=dict(artist='Me'))

# Attaching 3D axis to the figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ani_length = 30
mesh_size = 30
terrainLocation = [[[37.2296, -80.4139],[37.22965, -80.41390],[37.22960, -80.41395]]]
e = np.zeros((mesh_size, mesh_size, ani_length))
x = np.zeros((mesh_size, mesh_size, ani_length))
y = np.zeros((mesh_size, mesh_size, ani_length))
data = [] # list of tuples, alternate plotting method
plot_data = []


print("collecting terrain data...")
for i in range(ani_length):
    t_start = time.clock()
    samp_range = 200 + 50*i # 200 m^2 box up to 20,100m^2 box
    samp_dist = int(samp_range/mesh_size) # increasing total mesh size with this change
    print(samp_range)
    print(samp_dist)
    print("-------------")
    [e_tmp, x_tmp, y_tmp, data_tmp, cen_pt_ll] = get_terrain_map(terrainLocation, sample_dist = samp_dist, extent = samp_range, heading = i*2, show_plot = False, verbosity = False)
    # e[:, :, i] = e_tmp
    # x[:, :, i] = x_tmp
    # y[:, :, i] = y_tmp
    plot_data.append([x_tmp, y_tmp, e_tmp])
    data.append(data_tmp)

    t_end = time.clock()
    print("iteration {}/{} completed, elapsed time: {}".format(i,ani_length, t_end - t_start))
# with open('e_x_y_data.pkl','wb') as f:
#     pkl.dump([e,x,y],f)
# print("terrain data collected and saved.")

x, y, z = plot_data[0]
# grid_x, grid_y = np.mgrid[min(x):max(x):100j, min(y):max(y):100j]
# grid_z = griddata((x, y), z, (grid_x, grid_y), method='cubic')
plot = [ax.plot_surface(x, y, z, cmap='viridis')]

# ax.set_xlim3d([np.min(x[:, :, 0]), np.max(x[:, :, 0])])
# ax.set_ylim3d([np.min(y[:, :, 0]), np.max(y[:, :, 0])])
# ax.set_zlim3d([np.min(e), np.max(e)])
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('E')

ax.set_title('Terrain')
ani = animation.FuncAnimation(fig, update_plot, ani_length, fargs=(plot, plot_data), interval=75)

print("saving mp4...")
ani.save('terrain_mapped.mp4',writer=writer)
print("mp4 saved!")
# plt.show()

    # mesh_size = 30
    # samp_range = 500 # 200 m^2 box up to 20,100m^2 box
    # samp_dist = int(samp_range/mesh_size) # increasing total mesh size with this change
    # terrainLocation = [[[37.2296, -80.4139],[37.22965, -80.41390],[37.22960, -80.41395]]]
    # [e_tmp, x_tmp, y_tmp, data_tmp, cen_pt_ll] = get_terrain_map(terrainLocation, sample_dist = samp_dist, extent = samp_range, heading = i*2, show_plot = True, verbosity = False)