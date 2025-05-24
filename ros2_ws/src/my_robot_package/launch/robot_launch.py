import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

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
        description='Use simulation (Gazebo) clock if True'
    )
    use_sim_time_param = LaunchConfiguration('use_sim_time')

    motor_control_active_arg = DeclareLaunchArgument(
        'motorControl',
        default_value='False',
        description='Set to True to activate motorControl.py and its GPIO operations.'
    )
    motor_control_active_param = LaunchConfiguration('motorControl')

    pkg_share = get_package_share_directory('my_robot_package')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'float.urdf')
    try:
        with open(urdf_file_path, 'r') as file:
            robot_description_content = file.read()
    except EnvironmentError:
        robot_description_content = ""
        log_urdf_error = LogInfo(msg=f"[ERROR] [launch]: Could not read URDF file: {urdf_file_path}")
    else:
        log_urdf_error = LogInfo(msg="URDF file loaded successfully.")

    robot_description_param = {'robot_description': robot_description_content}

    return LaunchDescription([
        simulate_arg,
        use_sim_time_arg,
        motor_control_active_arg,
        log_urdf_error,

        Node(
            package='my_robot_package',
            executable='init_neutral_buoyancy.py',
            name='init_neutral_buoyancy_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_param}],
            condition=IfCondition(PythonExpression(["'", simulate_param, "' == 'False' or '", motor_control_active_param, "' == 'True'"]))
        ),

        Node(
            package='my_robot_package',
            executable='motorControl.py',
            name='motor_controller_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time_param},
                {'motor_control_active': motor_control_active_param}
                # Add {'use_encoder': True/False} here if you make it a launch arg for motorControl too
            ],
            condition=IfCondition(motor_control_active_param)
        ),

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
            parameters=[
                {'use_sim_time': use_sim_time_param},
                {'motor_control_mode_active': motor_control_active_param},
                {'initial_sim_depth_m': 0.5} # profundidad en metros, 0.5 para reflejar -0.5m
            ],
            condition=IfCondition(simulate_param)
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
    ])