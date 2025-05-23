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

# publish new target depth!
ros2 topic pub /target_depth std_msgs/msg/Float32 "{data: 15.0}" --once



# make laptop the ros2 slave to run rqt etc on it instead of rpi!
# write correct ips to RPI_IP and LAPTOP_IP!
nano ~/.bashrc

export ROS_MASTER_URI=http://<RPI_IP>:11311
export ROS_HOSTNAME=<LAPTOP_IP>
export ROS_IP=<LAPTOP_IP>

source ~/.bashrc

# make rpi master!
nano ~/.bashrc

export ROS_MASTER_URI=http://<RPI_IP>:11311
export ROS_HOSTNAME=<RPI_IP>
export ROS_IP=<RPI_IP>

source ~/.bashrc



# TESTING mission_control_node.py!!!!
ros2 service call /set_mission my_robot_package/srv/SetMission "{mission_mode: 'IDLE'}"

# Usage: {mission_mode: 'TARGET_DEPTH_HOLD', hold_target_depth_m: DESIRED_DEPTH, hold_duration_sec: DURATION}
ros2 service call /set_mission my_robot_package/srv/SetMission "{mission_mode: 'TARGET_DEPTH_HOLD', hold_target_depth_m: 10.0, hold_duration_sec: 30.0}"

# Layer scan mode
ros2 service call /set_mission my_robot_package/srv/SetMission "{mission_mode: 'LAYER_SCAN', scan_bottom_target_altitude_m: 2.0, scan_surface_target_depth_m: -0.5, scan_surface_wait_time_sec: 60.0, scan_cycles: 2}"

# Mission control not getting depth!

# Idle mode
ros2 service call /set_mission my_robot_package/srv/SetMission "{mission_mode: 'IDLE'}"

# Target depth hold mode:
ros2 service call /set_mission my_robot_package/srv/SetMission "{mission_mode: 'TARGET_DEPTH_HOLD', hold_target_depth_m: -25.0, hold_duration_sec: 30.0}"



# Piston info
 We determined that it took 21 781 revolutions of our motor to pull the piston from its lowest point to its 
highest point. 
In order to get to the "neutral buoyancy", the piston should be set to halfway. This means the encoder should have 21 781/2 =10 890!21781...we got which is 10876.5.. We'll use this number!


# run composite mode
ros2 launch my_robot_package robot_launch.py simulate:=True motorControl:=True


# Instructions for testing in lab with composite mode active!
Check GPIO connection, and that motor is powered!
Pull latest GIT to RPi!
navigate to ~/ros2_ws
delete old compiled files with: rm -rf build install log
make sure we have sourced ros2: source /opt/ros/humble/setup.bash
Give permissions to all python files: chmod +x *.py
Compile: colcon build --symlink-install --packages-select my_robot_package
Source: source install/setup.bash
Run composite mode: ros2 launch my_robot_package robot_launch.py simulate:=True motorControl:=True
Wait until init_neutral_buoyancy has finished!

21775

