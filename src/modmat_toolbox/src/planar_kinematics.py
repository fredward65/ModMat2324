#!/usr/bin/env python3

import numpy as np
import rospy
from tools.command import PlanarArmCommander


def forward_kinematics(joint_angles:np.ndarray) -> np.ndarray:
    """
    Forward Kinematics for a UR3 Arm
    """
    pose_x = 0.0
    pose_z = 0.0
    angle  = 0.0
    arm_pose = np.array([pose_x, pose_z, angle])
    return arm_pose


def inverse_kinematics(arm_pose:np.ndarray) -> np.ndarray:
    """
    Inverse Kinematics for a UR3 Arm
    """
    shoulder_joint = 0.0
    elbow_joint = 0.0
    wrist_joint = 0.0
    joint_angles = np.array([shoulder_joint, elbow_joint, wrist_joint])
    return joint_angles


def main():
    """ Planar kinematics """
    planar_arm_commander = PlanarArmCommander()
    print("Kinematics demo for a UR3 Arm in XZ planar mode")
    rospy.sleep(1)
    
    # Arm dimensions
    # d1:  0.1519
    # a2: -0.24365 
    # a3: -0.21325
    # d4:  0.11235

    """ Forward Kinematics """
    print("Moving to joint angles...")
    angles = np.array([-0.25 * np.pi, 0.5 * np.pi, 0.0])
    planar_arm_commander.move_joints(angles)
    print("Calculating cartesian pose...")
    current_angles = planar_arm_commander.get_current_joint_angles()
    current_pose = forward_kinematics(current_angles)
    print(current_pose)
    rospy.sleep(1)

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
