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

#################################################################
## Follow these steps to run ros2!
# Add execute permissions to python source files
cd ~/Desktop/underwaterAutonomousRobot/ros2_ws/src/my_robot_package/src/
chmod +x *.py

# Navigate to your workspace
cd ~/Desktop/underwaterAutonomousRobot/ros2_ws

# Remember to source ros...
source /opt/ros/humble/setup.bash

# Install dependencies
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# Build the package    ---- --symlink-install is useful during development as it allows you to edit Python files in src/ without rebuilding.
colcon build --symlink-install --packages-select my_robot_package

# Source the workspace
source install/setup.bash

# launch simulation mode
ros2 launch my_robot_package robot_launch.py

# Launch with simulation or without:
ros2 launch my_robot_package robot_launch.py simulate:=True  # Simulation ON
ros2 launch my_robot_package robot_launch.py simulate:=False # Simulation OFF

# Run tests
colcon test --packages-select my_robot_package
colcon test-result --verbose


#########################################################################
# Visualize simulated data
Open new terminal, leave the ros terminal open!
# Navigate to your workspace
cd ~/ros2_ws
# Remember to source ros...
source /opt/ros/humble/setup.bash
# Source the workspace
source install/setup.bash

# Plot CTD data using rqt tool
rqt
enable topics from topic monitor   Plugins -> Topics -> Topic Monitor
Plugins -> Visualization -> Plot
example: topic: /temperature/data

# Plot DVL data
rqt_plot /vx/data /vy/data /vz/data /altitude/data

# See GNSS string with topic
ros2 topic echo /gps_data
