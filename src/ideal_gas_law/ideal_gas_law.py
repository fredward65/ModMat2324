#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colormaps as cm
from mpl_toolkits.mplot3d import axes3d

np.set_printoptions(precision=3)
plt.rcParams.update({"text.usetex": True})

color_map = 'coolwarm'
cmap = cm.get_cmap(color_map)


def main():
    # Amount of substance (mol)
    n = 1
    # Ideal Gas Constant (kPa . L / mol . K)
    R = 8.314
    
    # Temperature range (K)
    T = np.linspace(0, 273.15 + 200, num=10)
    # Volume range (L)
    V = np.linspace(0, 1, num=100)

    # Plot Isotherms - P V = n R T
    P = V
    plt.plot(V, P, label=r'$T = %5.2f K$' % T[5], color=cmap(T[5] / T[-1]))
    plt.legend()
    plt.xlabel(r'$V$ (L)')
    plt.ylabel(r'$P$ (kPa)')
    plt.title(r'Isotherms - $P$ vs $V$')
    plt.tight_layout()

    # Pressure (kPa) from np.meshgrid(X, Y)
    T_, V_, P_ = axes3d.get_test_data(.25)
    
    ax = plt.figure().add_subplot(projection='3d')
    ax.grid(False)
    ax.set_box_aspect((1, 1, 1))
    ax.set_proj_type('ortho')
    ax.view_init(30, 45)

    ax.plot_surface(T_, V_, P_, cmap=color_map, edgecolor='k', lw=0.2)
    ax.contour(T_, V_, P_, zdir='x', offset=np.min(T_), cmap=color_map)
    ax.set_xlabel(r'$T$ (K)')
    ax.set_ylabel(r'$V$ (L)')
    ax.set_zlabel(r'$P$ (kPa)')
    ax.set_title(r'Loi des Gaz Parfait - $P$ vs $V$ vs $T$')
    
    plt.show()


if __name__ == "__main__":
    main()
