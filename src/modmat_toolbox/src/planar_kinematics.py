#!/usr/bin/env python3

import numpy as np
import rospy
from tools.ur_command import PlanarKinematicsCommander

# numpy pretty-print 
np.set_printoptions(precision=3, suppress=True)


def main():
    """ Planar kinematics """
    planar_arm_commander = PlanarKinematicsCommander()

    print("Kinematics demo for a UR3 Arm in XZ planar mode")
    rospy.sleep(1)

    """ Forward Kinematics """
    theta_1 =  0.5 * np.pi
    theta_2 = -0.5 * np.pi
    theta_3 = -0.5 * np.pi
    angles = np.array([theta_1, theta_2, theta_3])
    print("Moving to point angles...")
    planar_arm_commander.move_joints(angles)
    print("Calculating Cartesian pose from angles ", angles)
    current_angles = planar_arm_commander.get_current_joint_angles()
    current_pose = planar_arm_commander.forward_kinematics(current_angles)
    print("Current Cartesian pose: ", current_pose)
    rospy.sleep(1)

    """ Inverse Kinematics """
    x = 0.00
    z = 0.40
    theta = -0.5 * np.pi
    new_pose = np.array([x, z, theta])
    print("Calculating joint angles from pose ", new_pose) 
    new_angles = planar_arm_commander.inverse_kinematics(new_pose)
    print("New angles: ", new_angles)
    print("Moving to a Cartesian pose...")
    planar_arm_commander.move_joints(new_angles, t_sec=1)
    rospy.sleep(1)

    planar_arm_commander.go_home()
    rospy.sleep(1)
    print("End of the demo")


if __name__ == "__main__":
    main()
