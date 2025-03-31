cmake_minimum_required(VERSION 3.5)
project(my_robot_package)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/ctd.py
  src/dvl.py
  src/gnss.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_launch_test(test/test_launch.py)
endif()

ament_package()