from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="dummy_robot_bridge", executable="discover_node.py", name="discover_node", output="screen"),
        Node(package="dummy_robot_bridge", executable="dummy_node.py", name="dummy_node", output="screen",
             parameters=[{"session_required": True, "degrees_mode": False}]),
    ])
