# ~/ros2_ws/src/fibre_ros/launch/discover.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    path          = LaunchConfiguration('path')
    serial_number = LaunchConfiguration('serial_number')
    verbose       = LaunchConfiguration('verbose')
    extra_vid_pid = LaunchConfiguration('extra_vid_pid')

    return LaunchDescription([
        DeclareLaunchArgument('path', default_value='usb,serial'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('verbose', default_value='false'),
        # 陣列參數用字串傳，node 端會處理；也可改成 ParameterValue(..., value_type=list)
        DeclareLaunchArgument('extra_vid_pid', default_value="['1209:0d32']"),

        Node(
            package='fibre_ros',
            executable='discover',
            name='fibre_discover',
            output='screen',
            parameters=[{
                'path': path,
                'serial_number': serial_number,
                'verbose': verbose,
                'extra_vid_pid': extra_vid_pid,
            }]
        )
    ])

