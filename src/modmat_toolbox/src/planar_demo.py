#!/usr/bin/env python3

import numpy as np
import rospy
from tools.ur_command import PlanarArmCommander


def main():
    """ Planar movement control demo """
    planar_arm_commander = PlanarArmCommander()
    print("This is a demo of a UR3 Arm in XZ planar configuration")
    rospy.sleep(1)
    
    print("Moving to a joint angle pose...")
    angles = 0.5 * np.pi * np.array([1, -1, -1])
    planar_arm_commander.move_joints(angles)
    rospy.sleep(1)
    planar_arm_commander.go_home()
    rospy.sleep(1)

    print("Following a joint trajectory...")
    time = np.linspace(0, 1, num=100)
    angles_list = np.array([t_i * angles for t_i in time])
    planar_arm_commander.follow_joint_trajectory(angles_list, time)
    rospy.sleep(1)
    planar_arm_commander.go_home()
    rospy.sleep(1)

    print("End of the demo")


if __name__ == "__main__":
    main()
