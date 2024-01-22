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
    R = np.array([[np.cos(theta),-np.sin(theta)],\
                    [np.sin(theta), np.cos(theta)]])
    p = np.array([[dx],\
                    [dz]])
    A = np.r_[np.c_[R, R @ p], [[0, 0, 1]]]
    return A

def forward_kinematics(point_angles:np.ndarray) -> np.ndarray:
    """
    Forward Kinematics for a UR3 Arm
    """
    theta_1 = point_angles[0]
    theta_2 = point_angles[1]
    theta_3 = point_angles[2] - 0.5 * np.pi
    
    A_1 = create_tf_matrix(0, 0, link_lenghts["l1"])
    A_2 = create_tf_matrix(theta_1, link_lenghts["l2"], 0)
    A_3 = create_tf_matrix(theta_2, link_lenghts["l3"], 0)
    A_4 = create_tf_matrix(theta_3, link_lenghts["l4"], 0)
    A = A_1 @ A_2 @ A_3 @ A_4
    print("A : \n", A)
    pos_x = A[0, -1]
    pos_z = A[1, -1]
    theta = np.arctan2(A[1, 0], A[0, 0])
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
    point_angles = np.array([theta_1, theta_2, theta_3])
    return point_angles


def main():
    """ Planar kinematics """
    planar_arm_commander = PlanarArmCommander()
    print("Kinematics demo for a UR3 Arm in XZ planar mode")
    rospy.sleep(1)

    """ Forward Kinematics """
    theta_1 =  0.5 * np.pi
    theta_2 =  0.0 * np.pi
    theta_3 =  0.5 * np.pi
    angles = np.array([theta_1, theta_2, theta_3])
    print("Moving to point angles...")
    planar_arm_commander.move_joints(angles)
    print("Calculating Cartesian pose from angles ", angles)
    current_angles = planar_arm_commander.get_current_joint_angles()
    current_pose = forward_kinematics(current_angles)
    print("Current Cartesian pose: ", current_pose)
    rospy.sleep(1)

    """ Inverse Kinematics """
    new_pose = np.array([0.0, 0.2, 0.0])
    print("Calculating joint angles from pose ", new_pose)
    new_angles = inverse_kinematics(new_pose)
    print("New angles: ", new_angles)
    print("Moving to a Cartesian pose...")
    planar_arm_commander.move_joints(new_angles)
    rospy.sleep(1)

    planar_arm_commander.go_home()
    rospy.sleep(1)
    print("End of the demo")


if __name__ == "__main__":
    main()
