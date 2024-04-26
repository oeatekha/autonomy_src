#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String



#odom from gazebo is "best effort", so this is needed for the subscriber
from rclpy.qos import qos_profile_sensor_data, QoSProfile 

import numpy as np
import scipy as sp
from scipy import linalg


class LocobotGrip(Node):
    def __init__(self):
        super().__init__('gripper')
        self.gripper_state = "unknown"  # Initialize gripper state as unknown

        # Gripper open and close positions (adjust as needed)
        self.open_positions = [1.0, -1.0] # Example positions for open state
        self.close_positions =  [0.0, 0.0]  # Example positions for close state

        self.get_logger().info("LocobotGrip started")

        # Create a publisher for the gripper controller and a publisher for the state
        self.grip_publisher = self.create_publisher(JointTrajectory, '/locobot/gripper_controller/joint_trajectory', 10)
        self.state_publisher = self.create_publisher(String, '/locobot/gripper_state', 10)

        # Create a subscriber for the grip command
        self.grip_subscriber = self.create_subscription(String, 'gripCommand', self.grip_callback, qos_profile_sensor_data)

        # Initialize with the gripper open
        self.publish_gripper_action(self.open_positions)
        self.gripper_state = "open"
        self.publish_state("Gripper opened")

    def publish_gripper_action(self, positions):
        gripAction = JointTrajectory()
        gripAction.header.frame_id = ''
        gripAction.joint_names = ['left_finger', 'right_finger']

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0, 0.0]
        point.accelerations = [0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)

        gripAction.points = [point]
        self.grip_publisher.publish(gripAction)

    def publish_state(self, state_message):
        state_msg = String()
        state_msg.data = state_message
        self.state_publisher.publish(state_msg)
        self.get_logger().info(state_message)

    def grip_callback(self, msg):
        command = msg.data.lower()  # Convert to lowercase to ensure case-insensitive comparison
        if command == "open" and self.gripper_state != "open":
            self.publish_gripper_action(self.open_positions)
            self.gripper_state = "open"
            self.publish_state("Gripper opened")
        elif command == "close" and self.gripper_state != "close":
            self.publish_gripper_action(self.close_positions)
            self.gripper_state = "close"
            self.publish_state("Gripper closed")
        else:
            self.get_logger().info(f'Gripper is already {self.gripper_state}.')

def main(args=None):
    rclpy.init(args=args)
    locobot_grip = LocobotGrip()
    rclpy.spin(locobot_grip)
    locobot_grip.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()