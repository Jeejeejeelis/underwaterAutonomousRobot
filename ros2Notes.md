Ros2 dependencies:
sudo apt update
sudo apt install -y build-essential cmake git

Docker:
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

didnt work:docker pull osrf/ros:humble-desktop
try:docker pull arm64v8/ros:humble-desktop

docker run -it --rm osrf/ros:humble-desktop


After more research, I believe its better to install ubuntu server to the rpi.



### How to build and run ROS2 package and launch file:
# Navigate to your workspace
cd ~/ros2_ws

# Build the package
colcon build

# Source the setup file
source install/setup.bash

# Run the launch file
ros2 launch my_robot_package robot_launch.py

### Run ROS2 tests
# 
# Build package with colcon
colcon build --packages-select my_robot_package
# execute test
colcon test --packages-select my_robot_package
# view results
colcon test-result --verbose


ros2_ws/
├── src/
│   └── my_robot_package/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch/
│       │   └── robot_launch.py
│       ├── src/
│       |    ├── ctd.py
│       |    ├── dvl.py
│       |    └── gnss.py
|       └── test/
|           └── test_launch.py


# Remember to source ros...
source /opt/ros/humble/setup.bash
source install/setup.bash

# Delete ros build files
rm -rf build install log