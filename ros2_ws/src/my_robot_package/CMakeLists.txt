cmake_minimum_required(VERSION 3.8) # Updated minimum version
project(my_robot_package)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED) # Add if not already implicitly included by rclpy sometimes
# If using launch_testing_ros
# find_package(launch_testing_ros REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install python nodes correctly
install(PROGRAMS
  src/ctd.py
  src/dvl.py
  src/gnss.py
  src/float_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Python library installation (if you had modules in a subfolder)
# ament_python_install_package(src)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Use the new way to add launch tests
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_launch.py)
endif()

ament_package()