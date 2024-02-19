#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

np.set_printoptions(precision=3)
plt.rcParams.update({"text.usetex": True})


def main():
    # Particles at t = 0
    N_0 = 1e7
    # Decay constant
    lmb = 3.8394e-12  # C14
    # Half-life
    t_1_2 = np.log(2) / lmb
    # Mean life
    tau = 1 / lmb

    # Time vector
    t = np.linspace(0, 1e12, num=1e3)
    # Radioactive decay
    N = N_0 * np.exp(-lmb * t)

    count = 4
    t_vec = [t_1_2 * i for i in np.arange(count) + 1]
    N_vec = [N_0 / (2 ** i) for i in np.arange(count) + 1]
        
    # Plot
    plt.plot(t, N)
    plt.hlines(N_vec, 0, t_vec, linestyles='dotted')
    plt.vlines(t_vec, 0, N_vec, linestyles='dotted')
    plt.xticks(t_vec + [tau], [r'$%i t_\frac{1}{2}$' % i for i in np.arange(count) + 1]  + [r'$\tau$'])
    plt.yticks(N_vec + [N_0], [r'$\frac{N_0}{%i}$' % 2**i for i in np.arange(count) + 1] + [r'$N_0$'])
    plt.xlabel(r'$t$ (s)')
    plt.ylabel(r'Noyeaux restants $N$')
    plt.title(r'Décroissance Radioactive - $N$ vs $t$')
    plt.show()


if __name__ == "__main__":
    main()
