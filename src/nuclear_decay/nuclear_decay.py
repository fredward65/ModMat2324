#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

np.set_printoptions(precision=3)
plt.rcParams.update({"text.usetex": True})


def main():
    # Particles at t = 0
    N_0 = 1
    # Decay constant
    lmb = 1
    # Half-life
    t_1_2 = 1
    # Mean life
    tau = 1

    # Time vector
    t = np.linspace(0, 1e3, num=1e3)
    # Radioactive decay
    N = t
        
    # Plot
    plt.plot(t, N)
    plt.xlabel(r'$t$ (s)')
    plt.ylabel(r'Noyeaux restants $N$')
    plt.show()


if __name__ == "__main__":
    main()
