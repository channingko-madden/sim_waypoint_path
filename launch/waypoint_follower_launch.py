from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_waypoint_path',
            executable='waypoint_follower',
            arguments = ["-s", "5", "-l", "5.0"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', '1', 'map', 'base_link']
        ),
        Node(
            package='nav2_amcl',
            executable='amcl'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator'
        ),
    ])
