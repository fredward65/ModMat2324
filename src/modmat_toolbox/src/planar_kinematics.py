#!/usr/bin/env python3

import numpy as np
import rospy
from tools.command import PlanarArmCommander

# numpy pretty-print 
np.set_printoptions(precision=3, suppress=True)
# Arm dimensions
link_lenghts:dict = {'l1': 0.1519, 'l2': 0.24365, 'l3': 0.21325, 'l4':  0.11235}


def create_tf_matrix(theta:float, dx:float, dz:float) -> np.ndarray:
    """
    Create a Transformation Matrix for the plane XZ
    """
    j_R = np.array([[np.cos(theta),-np.sin(theta)],\
                    [np.sin(theta), np.cos(theta)]])
    j_p = np.array([[dx],\
                    [dz]])
    j = np.r_[np.c_[j_R, j_R @ j_p], [[0, 0, 1]]]
    return j

def forward_kinematics(joint_angles:np.ndarray) -> np.ndarray:
    """
    Forward Kinematics for a UR3 Arm
    """
    theta_1 = joint_angles[0]
    theta_2 = joint_angles[1]
    theta_3 = joint_angles[2] - 0.5 * np.pi
    
    j_1 = create_tf_matrix(0, 0, link_lenghts["l1"])
    j_2 = create_tf_matrix(theta_1, link_lenghts["l2"], 0)
    j_3 = create_tf_matrix(theta_2, link_lenghts["l3"], 0)
    j_4 = create_tf_matrix(theta_3, link_lenghts["l4"], 0)
    j = j_1 @ j_2 @ j_3 @ j_4
    print(j)
    pos_x = j[0, -1]
    pos_z = j[1, -1]
    theta = np.arctan2(j[1, 0], j[0, 0])
    arm_pose = np.array([pos_x, pos_z, theta])
    return arm_pose


def inverse_kinematics(arm_pose:np.ndarray) -> np.ndarray:
    """
    Inverse Kinematics for a UR3 Arm
    """
    pos_x = arm_pose[0]
    pos_z = arm_pose[1]
    angle = arm_pose[2]
    theta_1 = 0.0
    theta_2 = 0.0
    theta_3 = 0.0
    joint_angles = np.array([theta_1, theta_2, theta_3])
    return joint_angles


def main():
    """ Planar kinematics """
    planar_arm_commander = PlanarArmCommander()
    print("Kinematics demo for a UR3 Arm in XZ planar mode")
    rospy.sleep(1)

    """ Forward Kinematics """
    theta_1 =  0.0 * np.pi
    theta_2 =  0.0 * np.pi
    theta_3 =  0.5 * np.pi
    angles = np.array([theta_1, theta_2, theta_3])
    print("Moving to joint angles...")
    planar_arm_commander.move_joints(angles)
    print("Calculating cartesian pose...")
    current_angles = planar_arm_commander.get_current_joint_angles()
    current_pose = forward_kinematics(current_angles)
    print(current_pose)
    rospy.sleep(10)

    """ Inverse Kinematics """
    print("Calculating joint angles...")
    new_pose = np.array([0.0, 0.2, 0.0])
    new_angles = inverse_kinematics(new_pose)
    print(new_angles)
    print("Moving to a cartesian pose...")
    planar_arm_commander.move_joints(new_angles)
    rospy.sleep(1)

    planar_arm_commander.go_home()
    rospy.sleep(1)
    print("End of the demo")


if __name__ == "__main__":
    main()
