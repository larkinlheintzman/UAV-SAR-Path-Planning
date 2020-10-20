import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():

    xmin, xmax, xstep = 0, 25, 0.1
    ymin, ymax, ystep = 0, 25, 0.1

    fig = plt.figure()
    print('XXYYZZmap_test')
    ax = fig.gca(projection='3d')

    x = np.arange( xmin, xmax, xstep )
    y = np.arange( ymin, ymax, ystep )
    X, Y = np.meshgrid( x, y )

    # Z = np.random.randn( np.shape(Y)[0], np.shape(X)[0] )
    Z = Y*np.sin(X) + X*np.cos(Y)

    surf = ax.plot_surface( X, Y, Z, cmap='hsv' )
    # plt.contour( X, Y, Z)

    plt.show()

if __name__ == '__main__':
    main()