# Underwater Autonomous Robot - ROS 2 Project

This repository contains the ROS 2 software for an autonomous underwater float. The float is designed to travel with sea currents and can control its depth by modifying its volume. The primary controller is a Raspberry Pi running Ubuntu Server 22.04 and ROS 2 Humble.

## Table of Contents
1.  [Project Overview](#project-overview)
2.  [File Structure](#file-structure)
3.  [Prerequisites](#prerequisites)
4.  [Installation and Setup](#installation-and-setup)
5.  [Running the System](#running-the-system)
    * [Launching the Nodes](#launching-the-nodes)
    * [Interacting with the System](#interacting-with-the-system)
6.  [Distributed ROS 2 Setup](#distributed-ros-2-setup-rqt-on-laptop-nodes-on-raspberry-pi)
7.  [System Architecture](#system-architecture)

## Project Overview

The system consists of several ROS 2 nodes that handle sensor data, simulate float behavior, and will eventually include state estimation and control for real-world operation.

**Key Components:**
* **CTD Node (`ctd.py`):** Manages Conductivity, Temperature, and Depth (Pressure) sensor data.
* **DVL Node (`dvl.py`):** Manages Doppler Velocity Log data (velocity and altitude).
* **GNSS Node (`gnss.py`):** Manages GPS/GNSS data for surface positioning.
* **Float Simulator Node (`float_simulator.py`):** Simulates the float's kinematics, depth control, and provides a simulated environment with a fixed seafloor.

## Prerequisites

* **Operating System:** Ubuntu Server 22.04 LTS (specifically this version is recommended for compatibility).
* **ROS 2 Version:** ROS 2 Humble Hawksbill.
* **Hardware (for real-world operation):**
    * Raspberry Pi 4 (or similar, capable of running Ubuntu Server and ROS 2).
    * CTD, DVL, GNSS sensors.
    * Actuation system for depth control.

## Installation and Setup

1.  **Install ROS 2 Humble:**
    Follow the official ROS 2 Humble installation guide for Ubuntu: [Install ROS 2 Humble on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

2.  **Create Workspace and Clone Repository:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    # Clone the repository
    git clone [https://github.com/Jeejeejeelis/underwaterAutonomousRobot.git](https://github.com/Jeejeejeelis/underwaterAutonomousRobot.git)
    # Now, ensure your ROS package (my_robot_package) is directly under src.
    # If the 'underwaterAutonomousRobot' repo itself IS 'my_robot_package', you're good.
    # If 'my_robot_package' is a sub-directory within 'underwaterAutonomousRobot',
    # you might need to move it or ensure your paths reflect that, e.g.,
    # mv underwaterAutonomousRobot/my_robot_package .  (if my_robot_package is one level down)
    # Or, adjust subsequent cd commands if you keep the 'underwaterAutonomousRobot' directory.
    # For this README, we'll assume 'my_robot_package' is now directly in '~/ros2_ws/src/'
    cd .. 
    ```
    *For simplicity, ensure the package named `my_robot_package` (containing `CMakeLists.txt`, `package.xml`, etc.) is located at `~/ros2_ws/src/my_robot_package/`.*

3.  **Add Execute Permissions for Python Scripts:**
    Navigate to the directory containing the Python node scripts and make them executable.
    ```bash
    # Adjust this path if your package structure is different after cloning
    cd ~/ros2_ws/src/my_robot_package/src/ 
    chmod +x ctd.py dvl.py float_simulator.py gnss.py
    # If you add new Python nodes, remember to make them executable too.
    ```

4.  **Source ROS 2 Setup File:**
    Ensure your terminal environment is set up for ROS 2. It's good practice to add this to your `~/.bashrc` if you haven't already.
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    For permanent sourcing, add the line above to your `~/.bashrc` and then run `source ~/.bashrc`.

5.  **Install Dependencies:**
    Navigate to the root of your workspace (`~/ros2_ws/`) and use `rosdep` to install package dependencies.
    ```bash
    cd ~/ros2_ws/
    sudo apt update
    rosdep install --from-paths src --ignore-src -r -y
    ```
    *This command will look at the `package.xml` files in your `src` directory (specifically within `my_robot_package/package.xml`) and install the listed dependencies.*
    *Common dependencies you might need to list in `package.xml` for these nodes include `python3-serial` (for CTD and GNSS hardware modes) and standard ROS 2 packages like `rclpy`, `std_msgs`, `geometry_msgs`.*

6.  **Build the Package:**
    Build your specific package. The `--symlink-install` option is very useful during development as it allows you to edit Python files directly in the `src/` directory without needing to rebuild for the changes to take effect.
    ```bash
    cd ~/ros2_ws/
    colcon build --symlink-install --packages-select my_robot_package
    ```

7.  **Source the Workspace:**
    After a successful build, source your workspace's setup file to make your package and its executables available in the current terminal.
    ```bash
    cd ~/ros2_ws/
    source install/setup.bash
    ```
    Like the main ROS 2 setup, you can also add this line to your `~/.bashrc` for convenience: `echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc`.

## Running the System

### Launching the Nodes

The primary way to run the system is using the provided launch file `my_robot_package/launch/robot_launch.py`. You can choose to run in simulation mode or hardware mode using the `simulate` launch argument.

* **To run in Simulation Mode:**
    ```bash
    ros2 launch my_robot_package robot_launch.py simulate:=True
    ```
* **To run in Hardware Mode (Real-World):**
    ```bash
    ros2 launch my_robot_package robot_launch.py simulate:=False
    ```
    *Ensure all necessary hardware is connected and powered on before running in hardware mode.*
    *You may need to adjust serial port permissions (e.g., `sudo usermod -a -G dialout $USER`) and reboot/re-login for changes to take effect.*

### Interacting with the System (Primarily for Simulation)

**1. Visualizing Data with RQT:**

RQT is a versatile tool for introspecting ROS 2 systems.

* Open a new terminal (ensure ROS 2 and your workspace are sourced: `source /opt/ros/humble/setup.bash` and `source ~/ros2_ws/install/setup.bash`).
* Start RQT:
    ```bash
    rqt
    ```
* **Topic Monitor:** To see all active topics and their messages:
    * In RQT, go to `Plugins` -> `Topics` -> `Topic Monitor`.
    * You can select topics to view their real-time data.
* **Plotting Data:** To plot numerical data from topics (e.g., depth, temperature, velocities):
    * In RQT, go to `Plugins` -> `Visualization` -> `Plot`.
    * In the plot window, you can type a topic name (e.g., `/simulator/depth_m`, `/temperature`, `/dvl/vx`) into the "Topic" field and hit Enter or click the "+" button to add it to the plot.

**2. Viewing Topic Data in Terminal:**

You can use the `ros2 topic echo` command to see the raw data being published on any topic.

* **To list all active topics:**
    ```bash
    ros2 topic list
    ```
* **To echo data from a specific topic (e.g., GNSS data):**
    ```bash
    ros2 topic echo /gnss/raw_data
    ```
* **To echo the float's simulated depth or altitude:**
    ```bash
    ros2 topic echo /sim_ground_truth_depth
    ros2 topic echo /sim_ground_truth_altitude  
    ```
* **To echo the defined seafloor Z position:**
    ```bash
    ros2 topic echo /sim_ground_truth_seafloor_z
    ```

**3. Commanding a Target Depth (IMPORTANT for Simulation):**

The `float_simulator.py` node expects a target depth to be published for its depth control logic.

* Open a new terminal (sourced).
* To publish a target depth of, for example, 15.0 meters:
    ```bash
    ros2 topic pub /controller/target_depth std_msgs/msg/Float32 "{data: 15.0}" --once
    ```
    * Replace `15.0` with your desired depth.
    * The simulator will then attempt to move the simulated float to this depth.
    * The topic name `/controller/target_depth` is based on the `float_simulator.py` subscriptions.

## Distributed ROS 2 Setup (RQT on Laptop, Nodes on Raspberry Pi)

<details>
<summary><strong>Click to expand details on Distributed ROS 2 Setup</strong></summary>

To run the core ROS 2 nodes on your Raspberry Pi and use tools like RQT for visualization on a separate laptop, you need to ensure both devices can communicate over the network using ROS 2's DDS (Data Distribution Service) discovery mechanism.

[!WARNING]
**Future DDS Configuration Required for Robust Multi-Machine Operation:**
While basic ROS 2 discovery (nodes finding each other on the same network with matching `ROS_DOMAIN_ID`) should work for initial testing, advanced Data Distribution Service (DDS) configuration has been deferred.
**Current Status:** The instructions below focus on the most basic requirements.
**Future Work:** For reliable and optimized communication in complex network environments or across different subnets (if applicable), a deeper dive into DDS settings (e.g., choosing a specific DDS vendor like CycloneDDS or Fast DDS, configuring XML profiles, tuning discovery mechanisms, and setting up network interfaces explicitly through DDS) will be necessary. Previous attempts at implementing detailed DDS configurations led to issues and have been postponed to prioritize core system functionality. This will be revisited once the fundamental robot operations are stable.

**Key Principles for ROS 2 Distributed Systems:**

* **Decentralized Discovery:** Unlike ROS 1, ROS 2 does not use a central "master" node. Nodes discover each other peer-to-peer on the network using DDS. The `ROS_MASTER_URI` environment variable from ROS 1 is **not** used by default in ROS 2.
* **Same Network:** Both the Raspberry Pi and the laptop must be connected to the same local area network and be able to reach each other (e.g., via `ping`).
* **Firewalls:** Ensure that firewalls on both devices are not blocking UDP (often multicast for discovery) and TCP ports that DDS uses.
* **Time Synchronization:** Crucial for consistent behavior of timestamped data. Configure `chrony` on both devices to use the same NTP server or each other.

**Recommended Configuration Steps:**

1.  **Ensure Network Connectivity:**
    * Verify both devices are on the same network and can ping each other by IP address.
    * If pings fail, troubleshoot your network connection, IP settings, or firewalls.

2.  **Set a Common `ROS_DOMAIN_ID` (Recommended):**
    Nodes only discover others with the same `ROS_DOMAIN_ID`. The default is `0`.

    * **On your Raspberry Pi (`~/.bashrc`):**
        ```bash
        export ROS_DOMAIN_ID=0 # Or any integer from 0-232
        ```
    * **On your Laptop (`~/.bashrc`):**
        ```bash
        export ROS_DOMAIN_ID=0 # Must match the Raspberry Pi's ID
        ```
    * Apply changes with `source ~/.bashrc` or by opening new terminals.

3.  **Specify Network Interface with `ROS_IP` (Optional, if needed):**
    If devices have multiple network interfaces, explicitly set the IP for ROS 2 communication.

    * **On your Raspberry Pi (`~/.bashrc`):**
        ```bash
        export ROS_IP=<RPI_IP_ON_SHARED_NETWORK> 
        ```
        *(Replace `<RPI_IP_ON_SHARED_NETWORK>` with the RPi's actual IP)*
    * **On your Laptop (`~/.bashrc`):**
        ```bash
        export ROS_IP=<LAPTOP_IP_ON_SHARED_NETWORK>
        ```
        *(Replace `<LAPTOP_IP_ON_SHARED_NETWORK>` with the laptop's actual IP)*
    * `source ~/.bashrc` after changes.

4.  **Source ROS 2 and Workspace Setup Files:**
    Ensure `source /opt/ros/humble/setup.bash` and `source ~/ros2_ws/install/setup.bash` are in `~/.bashrc` *after* `ROS_DOMAIN_ID` and `ROS_IP`.

    **Example `.bashrc` structure (Raspberry Pi):**
    ```bash
    # ... other bashrc settings ...

    # ROS 2 Configuration
    export ROS_DOMAIN_ID=0
    # export ROS_IP=192.168.1.10 # Example: Uncomment and set RPi's IP

    # Source ROS 2 Humble & Workspace
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash # Adjust if your workspace path is different
    
    # ... other bashrc settings ...
    ```
    (Do similarly for the laptop's `.bashrc`)

**How it Works (for RQT on Laptop):**

* Nodes on the RPi advertise topics using DDS on the configured `ROS_DOMAIN_ID`.
* RQT on the laptop (same `ROS_DOMAIN_ID`, network reachable) discovers and subscribes to these topics.
* This allows offloading RQT from the RPi.

**Troubleshooting Discovery:**
* Use `ros2 topic list` and `ros2 node list` on both machines to check visibility.
* ROS 2 Humble defaults to Fast DDS. Ensure multicast isn't blocked on your network. For advanced scenarios (restricted multicast), investigate Fast DDS Discovery Server or unicast DDS configurations.

</details>

## System Architecture

### Signal Flow Diagram (Conceptual)

This diagram illustrates the primary data flows between the nodes, especially in simulation mode.

![Signal flow diagram](https://github.com/user-attachments/assets/1a63130f-e5be-4e8b-9620-3fa6df868ee5)

---