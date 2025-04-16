from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='True', # Set to True to run with mock data by default
        description='Set to True to run nodes in simulation mode, False for hardware.'
    )

    simulate_param = LaunchConfiguration('simulate')

    return LaunchDescription([
        simulate_arg,

        Node(
            package='my_robot_package',
            executable='ctd.py',
            name='ctd_node',
            output='screen',
            parameters=[{'simulate': simulate_param}]
        ),
        Node(
            package='my_robot_package',
            executable='dvl.py',
            name='dvl_node',
            output='screen',
            parameters=[{'simulate': simulate_param}]
        ),
        Node(
            package='my_robot_package',
            executable='gnss.py',
            name='gnss_node',
            output='screen',
            parameters=[{'simulate': simulate_param}]
        ),
    ])