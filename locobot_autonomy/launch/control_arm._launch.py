from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Node for the moveit_arm executable
        Node(
            package='locobot_autonomy',
            executable='moveit_arm',
            name='moveit_arm',
        ),
        # Node for the gripper Python script
        Node(
            package='locobot_autonomy',
            executable='gripper.py',
            name='gripper',
            prefix='python3'
        ),
        # Node for the arm_commander Python script
        Node(
            package='locobot_autonomy',
            executable='arm_commander.py',
            name='arm_commander',
            prefix='python3'
        ),
        # Add any additional nodes or configurations here
    ])
