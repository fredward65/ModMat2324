#!/usr/bin/env python3

import numpy as np
import rospy
from tools.trajectory_generators import linear_trajectory, circular_trajectory, square_trajectory, polygon_trajectory
from tools.ur_command import PlanarKinematicsCommander

# numpy pretty-print 
np.set_printoptions(precision=3, suppress=True)


def main():
    """ Planar kinematics """
    planar_arm_commander = PlanarKinematicsCommander()

    print("Inverse Kinematics exercise using a UR3 Arm in XZ planar mode")
    rospy.sleep(1)

    """ Linear trajectory generation """
    p_0 = [0.10, 0.20]    # Initial Cartesinan point
    p_f = [0.30, 0.20]    # Final Cartesinan point
    angle = -0.5 * np.pi  # Fixed end-effector angle
    t_vec, trajectory = linear_trajectory(p_0, p_f, angle, t_f=2)
    print("Following a linear Cartesian trajectory...")
    planar_arm_commander.follow_trajectory(trajectory, t_vec)
    rospy.sleep(0.5)
    planar_arm_commander.go_home()
    rospy.sleep(0.5)

    """ Circular trajectory generation """
    p_0 = [0.25, 0.25]   # Initial Cartesinan point
    radius = 0.15        # Circle radius
    angle = 0.30 * np.pi  # Fixed end-effector angle
    t_vec, trajectory = circular_trajectory(p_0, radius, angle, t_f=2)
    print("Following a circular Cartesian trajectory...")
    planar_arm_commander.follow_trajectory(trajectory, t_vec)
    rospy.sleep(0.5)
    planar_arm_commander.go_home()
    rospy.sleep(0.5)

    """ Square trajectory generation """
    p_0 = [0.25, 0.25]    # Initial Cartesinan point
    side = 0.15           # Square side length
    angle = 0.33 * np.pi  # Fixed end-effector angle
    t_vec, trajectory = square_trajectory(p_0, side, angle, t_f=2)
    print("Following a square Cartesian trajectory...")
    planar_arm_commander.follow_trajectory(trajectory, t_vec)
    rospy.sleep(0.5)
    planar_arm_commander.go_home()
    rospy.sleep(0.5)

    """ Polygon trajectory generation """
    p_0 = [0.25, 0.25]    # Initial Cartesinan point
    edges = 3             # Polygon edges count
    radius = 0.15         # Polygon radius
    angle = 0.25 * np.pi  # Fixed end-effector angle
    t_vec, trajectory = polygon_trajectory(p_0, edges, radius, angle, t_f=2)
    print("Following a polygonal Cartesian trajectory...")
    planar_arm_commander.follow_trajectory(trajectory, t_vec)
    rospy.sleep(0.5)
    planar_arm_commander.go_home()
    rospy.sleep(0.5)


if __name__ == "__main__":
    main()
