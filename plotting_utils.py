import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


def robot_paths(rgp_object, colors=None, dimension=2, linestyle='scatter'):

    if linestyle == 'scatter' and dimension == 3:
        colors = ['Greens','Blues','Oranges']
    else:
        colors = ['aquamarine', 'lime', 'green']
    fig = plt.figure()
    rgp = rgp_object
    if dimension == 2:
        start_idx = 0
        for path_len in rgp.robot_path_len:
            end_idx = start_idx + path_len
            rpath = rgp.all_robot_paths[0][start_idx:end_idx]
            plt.plot(rpath[:,0], rpath[:,1], colors[0])
            start_idx = end_idx

        start_idx = 0
        for path_len in rgp.robot_path_len:
            end_idx = start_idx + path_len
            rpath = rgp.all_robot_paths[-1][start_idx:end_idx]
            plt.plot(rpath[:,0], rpath[:,1], colors[1])
            start_idx = end_idx

        start_idx = 0
        for path_len in rgp.robot_path_len:
            end_idx = start_idx + path_len
            rpath = rgp.min_risk_paths[start_idx:end_idx]
            plt.plot(rpath[:,0], rpath[:,1], colors[2])
            start_idx = end_idx

    elif dimension == 3:

        fig = plt.figure()
        ax = plt.axes(projection='3d')

        start_idx = 0
        for path_len in rgp.robot_path_len:
            end_idx = start_idx + path_len
            rpath = rgp.all_robot_paths[0][start_idx:end_idx]
            if linestyle == 'scatter':
                ax.scatter3D(rpath[:, 0], rpath[:, 1], rpath[:, 2], cmap=colors[0])
            else:
                ax.plot3D(rpath[:, 0], rpath[:, 1], rpath[:, 2], colors[0])
            start_idx = end_idx

        start_idx = 0
        for path_len in rgp.robot_path_len:
            end_idx = start_idx + path_len
            rpath = rgp.all_robot_paths[-1][start_idx:end_idx]
            if linestyle == 'scatter':
                ax.scatter3D(rpath[:, 0], rpath[:, 1], rpath[:, 2], cmap=colors[1])
            else:
                ax.plot3D(rpath[:, 0], rpath[:, 1], rpath[:, 2], colors[1])
            start_idx = end_idx

        start_idx = 0
        for path_len in rgp.robot_path_len:
            end_idx = start_idx + path_len
            rpath = rgp.min_risk_paths[start_idx:end_idx]
            if linestyle == 'scatter':
                ax.scatter3D(rpath[:, 0], rpath[:, 1], rpath[:, 2], cmap=colors[2])
            else:
                ax.plot3D(rpath[:, 0], rpath[:, 1], rpath[:, 2], colors[2])
            start_idx = end_idx

        ax.view_init(30, 60)

    else:
        print("dimension ~= 2 or 3?")

    plt.show()


if __name__ == "__main__":
    print("plotting functions for MRMH codebase")