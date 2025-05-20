cmake_minimum_required(VERSION 3.8)
project(my_robot_package)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) # For NavSatFix used in mission_control_node.py and potentially srv/msg dependencies
find_package(geometry_msgs REQUIRED) # For TransformStamped, Quaternion etc.
find_package(tf2_ros REQUIRED) # For TransformBroadcaster
find_package(rosidl_default_generators REQUIRED) # For generating .srv code

# Generate Python (and C++) code from .srv files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMission.srv"
  # Add other .msg or .srv files here if you create more
  DEPENDENCIES std_msgs sensor_msgs # List packages whose types are used in your srv/msg definitions
)

install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install python nodes correctly
install(PROGRAMS
  src/ctd.py
  src/dvl.py
  src/gnss.py
  src/float_simulator.py
  src/depth_calculator.py
  src/mission_control_node.py
  src/init_neutral_buoyancy.py # Added new node
  src/motorControl.py          # Added new node
  DESTINATION lib/${PROJECT_NAME}
)

# Python library installation (if you had modules in a subfolder like a Python package)
# ament_python_install_package(src) # This would be used if 'src' was a Python package itself

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(launch_testing_ament_cmake REQUIRED)
  # Ensure your test file exists if you uncomment this
  # add_launch_test(test/test_launch.py)
endif()

ament_package()