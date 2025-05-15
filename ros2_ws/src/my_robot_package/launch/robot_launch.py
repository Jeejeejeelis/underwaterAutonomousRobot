# ros2_ws/src/my_robot_package/launch/robot_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='False',
        description='Set to True to run nodes in simulation mode, False for hardware.'
    )
    simulate_param = LaunchConfiguration('simulate')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if True')
    use_sim_time_param = LaunchConfiguration('use_sim_time')

    pkg_share = get_package_share_directory('my_robot_package')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'float.urdf')
    try:
        with open(urdf_file_path, 'r') as file:
            robot_description_content = file.read()
    except EnvironmentError:
        print(f"[ERROR] [launch]: Could not read URDF file: {urdf_file_path}")
        robot_description_content = ""


    robot_description_param = {'robot_description': robot_description_content}

    return LaunchDescription([
        simulate_arg,
        use_sim_time_arg,

        Node(
            package='my_robot_package',
            executable='ctd.py',
            name='ctd_node',
            output='screen',
            parameters=[{'simulate': simulate_param}, {'use_sim_time': use_sim_time_param}]
        ),
        Node(
            package='my_robot_package',
            executable='dvl.py',
            name='dvl_node',
            output='screen',
            parameters=[{'simulate': simulate_param}, {'use_sim_time': use_sim_time_param}]
        ),
        Node(
            package='my_robot_package',
            executable='gnss.py',
            name='gnss_node',
            output='screen',
            parameters=[{'simulate': simulate_param}, {'use_sim_time': use_sim_time_param}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description_param, {'use_sim_time': use_sim_time_param}]
        ),
        Node(
            package='my_robot_package',
            executable='float_simulator.py',
            name='float_simulator_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_param}]
        ),
        Node(
            package='my_robot_package',
            executable='depth_calculator.py',
            name='depth_calculator_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_param}]
        ),
        Node(
            package='my_robot_package',
            executable='mission_control_node.py',
            name='mission_control_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_param}]
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     # Optional: arguments=['-d', os.path.join(pkg_share, 'config', 'display.rviz')],
        #     parameters=[{'use_sim_time': use_sim_time_param}]
        # ),
    ])