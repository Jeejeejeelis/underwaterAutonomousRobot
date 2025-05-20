cmake_minimum_required(VERSION 3.8)
project(my_robot_package)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) # For NavSatFix used in mission_control_node.py and potentially srv/msg dependencies
find_package(geometry_msgs REQUIRED) # For TransformStamped, Quaternion etc.
find_package(tf2_ros REQUIRED) # For TransformBroadcaster
find_package(rosidl_default_generators REQUIRED) # For generating .srv code

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMission.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/ctd.py
  src/dvl.py
  src/gnss.py
  src/float_simulator.py
  src/depth_calculator.py
  src/mission_control_node.py
  src/init_neutral_buoyancy.py
  src/motorControl.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_launch.py)
endif()

ament_package()