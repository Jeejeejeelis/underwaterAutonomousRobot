from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='ctd.py',
            name='ctd_',
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='dvl.py',
            name='dvl',
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='gnss.py',
            name='gnss',
            output='screen'
        ),
    ])