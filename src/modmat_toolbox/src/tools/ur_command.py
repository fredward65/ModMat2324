#!/usr/bin/env python3

import numpy as np
import rospy
from dill import load
from os.path import dirname
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class BasicArmCommander(object):
    joint_names:list = ["shoulder_pan_joint",
                        "shoulder_lift_joint",
                        "elbow_joint",
                        "wrist_1_joint",
                        "wrist_2_joint",
                        "wrist_3_joint"]
    home_angles:np.ndarray = np.zeros(6)
    
    def __init__(self, sense:np.ndarray=np.ones(6), verbose:bool=False) -> None:
        """
        Basic Commander for an UR Arm
        """
        self.sense = sense
        print("Initializing ROS Node...")
        log_level = rospy.DEBUG if verbose else rospy.INFO
        rospy.init_node("basic_arm_command_node", anonymous=False, log_level=log_level)
        print("ROS Node initialized")
        self.pub = rospy.Publisher("/eff_joint_traj_controller/command", JointTrajectory, queue_size=1)
        rospy.sleep(1)
        self.move_to_home_position()
        rospy.loginfo("Ready to command the joints")

    def joint_trajectory_point(self, angles:np.ndarray, tsecs:float=1) -> JointTrajectoryPoint:
        """
        Create a Joint Trajectory Point 
        """
        point = JointTrajectoryPoint()
        point.positions = np.multiply(angles, self.sense)
        point.velocities = [0,0,0,0,0,0]
        point.accelerations = [0,0,0,0,0,0]
        point.time_from_start = rospy.Duration.from_sec(tsecs)
        return point

    def get_current_joint_states(self) -> np.ndarray:
        """
        Get current joint states
        """
        joint_state:JointState = rospy.wait_for_message("/joint_states", JointState, 1)
        joint_state_dict = {j_i: p_i for j_i, p_i in zip(joint_state.name, joint_state.position)}
        joint_state = np.array([joint_state_dict[j_i] for j_i in self.joint_names])
        joint_state = np.multiply(joint_state, self.sense)
        return joint_state

    def move_to_home_position(self) -> None:
        """
        Move arm to home position
        """
        rospy.logdebug("Moving to home pose...")
        self.move_to_joint_positions(self.home_angles)
        rospy.logdebug("Arm in home pose")

    def move_to_joint_positions(self, angles:np.ndarray, t_sec:float=1)  -> None:
        """
        Move to joint positions in a given time (default 1s)
        """
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = self.joint_trajectory_point(angles, tsecs=t_sec)
        msg.points.append(point)
        msg.header.stamp = rospy.Time.now()
        rospy.logdebug("Sending joint states...")
        self.pub.publish(msg)
        rospy.logdebug("Joint states sent. Moving...")
        rospy.sleep(t_sec)
        rospy.logdebug("Done moving")

    def follow_joint_trajectory(self, joint_trajectory:np.ndarray, t_vec:np.ndarray)  -> None:
        """
        Follow a joint trajectory at a given time vector
        """
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        for j_i, t_i in zip(joint_trajectory, t_vec):
            point = self.joint_trajectory_point(j_i, t_i)
            msg.points.append(point)
        self.move_to_joint_positions(joint_trajectory[0])
        msg.header.stamp = rospy.Time.now()
        rospy.logdebug("Sending joint states...")
        self.pub.publish(msg)
        rospy.logdebug("Joint states sent. Moving...")
        rospy.sleep(t_vec[-1])
        rospy.logdebug("Done moving")


class PlanarArmCommander(object):
    home_angles:np.ndarray = np.zeros(3)
    offset_angles:np.ndarray = 0.5 * np.pi * np.array([0,0,1])
    
    def __init__(self, verbose:bool=False) -> None:
        """
        Planar Arm Commander for UR Arm
        This class allows for commanding the following joints:
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
        The arm will move on the XZ plane.
        """
        sense = -1 * np.ones(6)
        self.__basic_arm_commander = BasicArmCommander(sense=sense, verbose=verbose)
        self.go_home()
        
    def go_home(self) -> None:
        """
        Move arm to home pose
        """
        self.move_joints(self.home_angles, t_sec=2)

    def move_joints(self, joint_angles:np.ndarray, t_sec:float=1) -> None:
        """
        Move joints to angles
        """
        full_joint_angles = np.zeros(6)
        full_joint_angles[1:4] = joint_angles + self.offset_angles
        self.__basic_arm_commander.move_to_joint_positions(full_joint_angles, t_sec=t_sec)

    def follow_joint_trajectory(self, joint_trajectory:np.ndarray, t_vec:np.ndarray) -> None:
        """
        Follow a joint trajectory at a given time vector
        """
        length = joint_trajectory.shape[0]
        full_joint_trajectory = np.zeros((length, 6))
        full_joint_trajectory[:, 1:4] = joint_trajectory + self.offset_angles
        self.__basic_arm_commander.follow_joint_trajectory(full_joint_trajectory, t_vec)

    def get_current_joint_angles(self) -> None:
        """
        Get current joint angles
        """
        full_current_joint_angles = self.__basic_arm_commander.get_current_joint_states()
        current_joint_angles = full_current_joint_angles[1:4] - self.offset_angles
        return current_joint_angles


class PlanarKinematicsCommander(PlanarArmCommander):
    def __init__(self, link_lenghts:dict={'l1': 0.1519, 'l2': 0.24365, 'l3': 0.21325, 'l4':  0.11235}) -> None:
        """
        Planar Kinematics Commander for a UR3 Arm
        """
        # Arm links dimensions
        super().__init__()
        l1 = link_lenghts["l1"]
        l2 = link_lenghts["l2"]
        l3 = link_lenghts["l3"]
        l4 = link_lenghts["l4"]
        self.links = [l1, l2, l3, l4]
        dir_name = dirname(__file__)
        self.fk_solver = load(open(dir_name + "/lambda_fk", "rb"))
        self.ik_solver = load(open(dir_name + "/lambda_ik", "rb"))

    def forward_kinematics(self, joint_angles:np.ndarray) -> np.ndarray:
        """
        Forward Kinematics for a UR3 Arm
        """
        arm_pose = np.array(self.fk_solver(joint_angles, self.links))
        return arm_pose

    def inverse_kinematics(self, arm_pose:np.ndarray) -> np.ndarray:
        """
        Inverse Kinematics for a UR3 Arm
        """
        joint_angles = np.array(self.ik_solver(arm_pose, self.links))
        return joint_angles

    def move_to_point(self, arm_pose:np.ndarray, t_sec:float=1) -> None:
        """
        Move arm to Cartesian pose
        """
        joint_angles = self.inverse_kinematics(arm_pose)
        self.move_joints(joint_angles, t_sec=t_sec)

    def follow_trajectory(self, trajectory:np.ndarray, t_vec:np.ndarray) -> None:
        """
        Follow Cartesian pose
        """
        joint_trajectory = np.array([self.inverse_kinematics(p_i) for p_i in trajectory]).reshape((-1, 3))
        self.follow_joint_trajectory(joint_trajectory, t_vec)


def main():
    pass


if __name__ == "__main__":
    main()
