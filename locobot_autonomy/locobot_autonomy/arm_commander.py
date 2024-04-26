#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import time
 

class CommandListener(Node):
    def __init__(self):
        super().__init__('command_listener')
        self.command_subscription = self.create_subscription(
            String,
            'command_topic',
            self.command_callback,
            10)
        self.block_position_publisher = self.create_publisher(Pose, '/block_position', 10)
        self.gripper_command_publisher = self.create_publisher(String, '/gripCommand', 10)
        self.block_pose_subscriber = self.create_subscription(Pose, 'block_pose', self.block_pose_callback, 5)
        self.status_publisher = self.create_publisher(String, '/arm_status', 10)
        self.pickup_pose = Pose()
        self.putdown_pose = Pose()
        self.arm_status = String()
        self.is_ready = True  # Add a flag to indicate readiness for a new command
        self.last_command = Pose()
        self.recievedPose = Pose()




    def block_pose_callback(self, msg):
        
        self.get_logger().info('Received block pose')
        self.get_logger().info('bloc callback position: x={}, y={}, z={}'.format(msg.position.x, msg.position.y, msg.position.z))
        self.recievedPose = msg

        
        self.pickup_pose.position.x = msg.position.x
        self.pickup_pose.position.y = msg.position.y
        self.pickup_pose.position.z = msg.position.z
        self.pickup_pose.orientation.x = 0.0
        self.pickup_pose.orientation.y = 0.0
        self.pickup_pose.orientation.z = 0.0
        self.pickup_pose.orientation.w = 1.0
        self.get_logger().info('Updated pickup pose')
        print("Pick Up Pose", self.pickup_pose.position.x)

        self.putdown_pose.position.x = msg.position.x
        self.putdown_pose.position.y = msg.position.y
        self.putdown_pose.position.z = msg.position.z + 0.1 # Elevate the block by 0.2m
        self.putdown_pose.orientation = self.pickup_pose.orientation


        return msg
    
    def command_callback(self, msg):

        if not self.is_ready:
            self.get_logger().info('Robot is not ready for a new command. Please wait.')
            return
        
        if self.last_command == self.recievedPose:
            self.get_logger().info('The last command was the same as the incoming pose. Ignoring the command')
            return

        # if pick up and the last command was not the same as the incoming pose 
        if msg.data == "pickup":

            self.is_ready = False 

            self.arm_status.data = "picking-up"
            self.get_logger().info('Executing pickup command')
            self.gripper_command_publisher.publish(String(data='open'))
            # create a rate object to sleep for 3 seconds
            self.wait_for_seconds(7)
            print(self.pickup_pose)
            self.publish_pose(self.pickup_pose)
            self.get_logger().info('Moving to pose')

            self.wait_for_seconds(11)
            self.gripper_command_publisher.publish(String(data='close'))

            # Once Closed Pick up the block
            self.wait_for_seconds(1)
            self.arm_status.data = "complete"
            self.publish_pose(self.putdown_pose) ## Uncomment this
            self.status_publisher.publish(self.arm_status)


            self.get_logger().info('Moved to second pose')
            self.get_logger().info(f'{msg.data} command completed. Ready for new command.')
            self.is_ready = True  # Set to True again once the command has been processed
            self.last_command == self.recievedPose
            
        elif msg.data == "putdown":
            self.arm_status.data = "putting-down"
            self.get_logger().info('Executing putdown command')
            self.publish_pose(self.pickup_pose)
            self.wait_for_seconds(3)
            self.gripper_command_publisher.publish(String(data='open'))
            self.status_publisher.publish(self.arm_status)

            self.is_ready = True  # Set to True again once the command has been processed
            self.get_logger().info(f'{msg.data} command completed. Ready for new command.')
            pullUpPose = Pose()
            pullUpPose.position.z = self.pickup_pose.position.z + 0.15
            self.publish_pose(self.pullUpPose)


        else:
            self.get_logger().info('Received an unknown command')

    def publish_pose(self, pose):
        self.block_position_publisher.publish(pose)
        self.get_logger().info('Published pose to /block_position')

    def wait_for_seconds(self, seconds):
        rclpy.spin_once(self, timeout_sec=seconds)

def main(args=None):
    rclpy.init(args=args)
    command_listener = CommandListener()
    rclpy.spin(command_listener)
    command_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
