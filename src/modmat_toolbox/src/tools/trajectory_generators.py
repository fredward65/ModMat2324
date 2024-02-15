#!/usr/bin/env python3

import numpy as np


def linear_trajectory(p_0:list, p_f:list, angle:float, t_f:float=1, n:int=100) -> "tuple[np.ndarray, np.ndarray]":
    """
    Generate a linear trajectory with constant angle for a UR Arm in XZ planar configuration
    """
    t_vec = np.linspace(0, t_f, num=n)

    x = np.linspace(p_0[0], p_f[0], num=n)
    z = np.linspace(p_0[1], p_f[1], num=n) 
    theta = angle * np.ones(n)
    
    trajectory = np.c_[x, z, theta]
    return t_vec, trajectory


def circular_trajectory(p_0:list, radius:float, angle:float, t_f:float=1, n:int=100) -> "tuple[np.ndarray, np.ndarray]":
    """
    Generate a circular trajectory with constant angle for a UR Arm in XZ planar configuration
    """
    t_vec = np.linspace(0, t_f, num=n)
    
    x_0 = p_0[0]
    z_0 = p_0[1]

    arg_angle = np.linspace(np.pi, -np.pi, num=n) + angle
    sin_arg = np.sin(arg_angle)
    cos_arg = np.cos(arg_angle)

    c_x = x_0 + np.cos(angle) * radius
    c_z = z_0 + np.sin(angle) * radius

    x = radius*cos_arg + c_x
    z = radius*sin_arg + c_z

    theta = angle * np.ones(n)
    
    trajectory = np.c_[x, z, theta]
    return t_vec, trajectory


def square_trajectory(p_0:list, side:float, angle:float, t_f:float=1, n:int=100) -> "tuple[np.ndarray, np.ndarray]":
    """
    Generate a square trajectory with constant angle for a UR Arm in XZ planar configuration
    """
    t_vec = np.linspace(0, t_f, num=n)
    
    # Modify these lines
    x_0 = p_0[0]
    z_0 = p_0[1]

    x_1 = x_0 + side * np.cos(angle + 0.5 * np.pi)
    z_1 = z_0 + side * np.sin(angle + 0.5 * np.pi)

    x_2 = x_1 + side * np.cos(angle)
    z_2 = z_1 + side * np.sin(angle)

    x_3 = x_2 + side * np.cos(angle - 0.5 * np.pi)
    z_3 = z_2 + side * np.sin(angle - 0.5 * np.pi)

    v_1_x = np.linspace(x_0, x_1, num=n//4)
    v_1_z = np.linspace(z_0, z_1, num=n//4)

    v_2_x = np.linspace(x_1, x_2, num=n//4)
    v_2_z = np.linspace(z_1, z_2, num=n//4)

    v_3_x = np.linspace(x_2, x_3, num=n//4)
    v_3_z = np.linspace(z_2, z_3, num=n//4)
    
    v_4_x = np.linspace(x_3, x_0, num=n//4)
    v_4_z = np.linspace(z_3, z_0, num=n//4)

    x = np.r_[v_1_x, v_2_x, v_3_x, v_4_x]
    z = np.r_[v_1_z, v_2_z, v_3_z, v_4_z]

    theta = angle * np.ones(n)
    
    trajectory = np.c_[x, z, theta]
    return t_vec, trajectory


def polygon_trajectory(p_0:list, edges:float, radius:float, angle:float, t_f:float=1, n:int=100) -> "tuple[np.ndarray, np.ndarray]":
    """
    Generate a regular polygon trajectory with constant angle for a UR Arm in XZ planar configuration
    """
    t_vec = np.linspace(0, t_f, num=n)
    
    # Modify these lines
    x = np.linspace(p_0[0], p_0[0], num=n)
    z = np.linspace(p_0[1], p_0[1], num=n) 
    theta = angle * np.ones(n)
    
    trajectory = np.c_[x, z, theta]
    return t_vec, trajectory


def main():
    pass


if __name__ == "__main__":
    main()
