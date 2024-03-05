#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

np.set_printoptions(precision=3)
plt.rcParams.update({"text.usetex": True})

# Gravity acceleration
g = 9.80665
# Double Pendulum parameters
m_1 = .5
m_2 = .5
l_1 = .1
l_2 = .2

def compute_dot_omega(th_1, th_2, om_1, om_2):
    """
    myPhysicsLab Double Pendulum
    https://www.myphysicslab.com/pendulum/double-pendulum-en.html
    """
    den = 1
    
    num_1 = 0
    num_2 = 0
    
    dw_1 = num_1 / den
    dw_2 = num_2 / den

    return dw_1, dw_2

def main():
    # Initial conditions
    th_1 = .5*np.pi
    th_2 = .5*np.pi
    om_1 = 0
    om_2 = 0

    # Time period
    t = np.linspace(0, 5, num=500)
    dt = np.diff(t)
    dt = np.append(dt, [dt[-1]])

    # Compute system
    th_1_t = []
    th_2_t = []
    plt.xlim(-.5, .5)
    plt.ylim(-.5, .5)

    x_1 = lambda th_1: l_1 * np.sin(th_1)
    y_1 = lambda th_1: l_1 * np.cos(th_1) * -1
    x_2 = lambda th_1, th_2: x_1(th_1) + l_2 * np.sin(th_2)
    y_2 = lambda th_1, th_2: y_1(th_1) - l_2 * np.cos(th_2)

    arm_1 = plt.plot([0, x_1(th_1)], [0, y_1(th_1)], 'b', lw=2)[0]
    arm_2 = plt.plot([x_1(th_1), x_2(th_1, th_2)],
                     [y_1(th_1), y_2(th_1, th_2)], 'r', lw=2)[0]
    p_x = x_2(th_1, th_2)
    p_y = y_2(th_1, th_2)
    for dt_i in dt:
        th_1_t.append(th_1)
        th_2_t.append(th_2)
        dw_1, dw_2 = compute_dot_omega(th_1, th_2, om_1, om_2)
        # Euler integration
        om_1 += dw_1 * dt_i
        om_2 += dw_2 * dt_i
        th_1 += om_1 * dt_i
        th_2 += om_2 * dt_i

        c_x = x_2(th_1, th_2)
        c_y = y_2(th_1, th_2)
        plt.plot([p_x, c_x], [p_y, c_y], 'k', alpha=.25)
        p_x = c_x
        p_y = c_y

        arm_1.set_data([0, x_1(th_1)], [0, y_1(th_1)])
        arm_2.set_data([x_1(th_1), x_2(th_1, th_2)],
                       [y_1(th_1), y_2(th_1, th_2)])
        plt.pause(dt_i)
    plt.show()
    
    # Plot system
    plt.plot(t, th_1_t, t, th_2_t)
    plt.xlabel(r'$t$ (s)')
    plt.ylabel(r'$\theta_1, \theta_2$ (rad)')
    plt.title(r'Double Pendulum angles - $\theta$ vs $t$')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
