# Lab Testing Instructions: Composite Mode (Real Motor + Simulation)

These instructions guide you through setting up and running the AUV software on a Raspberry Pi in "composite mode." In this mode, the `motorControl` node will interact with the physical motor hardware, while the AUV's environment and other sensors are simulated.

## I. Pre-Test Checklist & Physical Setup

1.  **Verify GPIO Connections:**
    * Thoroughly check all wiring between the Raspberry Pi's GPIO pins and the motor controller, encoder, and limit switches. Refer to your established wiring diagram/notes and the software configuration:
        * **Ground:** Blue
        * **Upper Limit Switch (Piston fully IN, Min Volume):** Green (GPIO 26)
        * **Lower Limit Switch (Piston fully OUT, Max Volume):** Yellow (GPIO 19)
        * **Motor Command for DESCEND (Piston IN, Volume DECREASE):** Brown wire, connected to GPIO 5 (termed `UP_PIN` in code)
        * **Motor Command for ASCEND (Piston OUT, Volume INCREASE):** Red wire, connected to GPIO 6 (termed `DOWN_PIN` in code)
        * **Encoder:** Connected to GPIO 27
    * Ensure all connections are secure and correct.

2.  **Power Supply:**
    * Confirm the Raspberry Pi is adequately powered.
    * **Crucially, ensure the motor has its separate power supply connected and switched ON.**

3.  **Safety Check (Optional but Highly Recommended):**
    * If feasible, briefly use `motorTest.py` or a similar script to manually verify:
        * The motor moves as expected:
            * Activating GPIO 5 (Brown wire) should move the piston IN (towards upper limit).
            * Activating GPIO 6 (Red wire) should move the piston OUT (towards lower limit).
        * The upper and lower limit switches successfully stop the motor when triggered.
    * Ensure the physical system (motor piston, AUV chassis) is in a safe state to operate and free from obstructions.

## II. Software Setup on Raspberry Pi

1.  **Open a Terminal on the Raspberry Pi.**

2.  **Update Code from Git Repository:**
    * Navigate to your project's root directory (e.g., `cd ~/UAR_Repo` or your specific path).
    * Pull the latest software changes (ensure you have the versions with the corrected motor logic):
        ```bash
        git pull
        ```

3.  **Navigate to ROS 2 Workspace:**
    ```bash
    cd ~/ros2_ws
    ```
    *(Adjust this path if your ROS 2 workspace has a different name or location).*

4.  **Clean Previous Build Artifacts (Recommended for a clean state):**
    ```bash
    rm -rf build install log
    ```

5.  **Source ROS 2 Environment:**
    * Ensure your base ROS 2 distribution is sourced (e.g., for ROS 2 Humble):
        ```bash
        source /opt/ros/humble/setup.bash
        ```
    *(Adjust the command if you are using a different ROS 2 version).*

6.  **Set Python File Permissions:**
    * Navigate to your package's Python source directory:
        ```bash
        cd src/my_robot_package/src
        ```
    * Make all Python (`.py`) scripts in this directory executable:
        ```bash
        chmod +x *.py
        ```
    * Return to the root of your ROS 2 workspace:
        ```bash
        cd ~/ros2_ws
        ```

7.  **Compile the Workspace (Targeting Your Package):**
    * Build your specific package using `colcon`. The `--symlink-install` option allows you to edit Python scripts and have the changes take effect without a full recompile.
        ```bash
        colcon build --symlink-install --packages-select my_robot_package
        ```

8.  **Source Local Workspace Setup File:**
    * After a successful compilation, source the setup file generated for your local workspace.
        ```bash
        source install/setup.bash
        ```

## III. Running the Composite Simulation

1.  **Launch the System:**
    * Execute the main launch file with the `simulate` and `motorControl` arguments set to `True`.
        ```bash
        ros2 launch my_robot_package robot_launch.py simulate:=True motorControl:=True
        ```
    * *(Note: `use_sim_time:=False` is the default in your `robot_launch.py` and is correct for this mode).*

2.  **Monitor `init_neutral_buoyancy_node` Initialization:**
    * Carefully observe the terminal output. The `init_neutral_buoyancy_node` will:
        * Command the piston fully OUT (towards the lower limit switch, activating GPIO 6) to set encoder to 0.
        * Then command the piston IN (towards the upper limit switch, activating GPIO 5) to reach the neutral buoyancy encoder target (e.g., 10890 ticks).
    * **Wait for the explicit confirmation message indicating completion:**
        ```
        [init_neutral_buoyancy_node-X] [INFO] [rclpy]: Initialization to neutral buoyancy complete at YYYY ticks. Waiting for mission command...
        ```
        (Where `X` is a process ID and `YYYY` is the final encoder count).
    * The `init_neutral_buoyancy_node` will then shut down.

3.  **System Ready for Operation:**
    * The `motor_controller_node` is now active, managing the physical motor based on simulated depth and target commands, using the corrected logic.
    * The `float_simulator_node` will have its vertical speed driven by the real encoder values.
    * You can now proceed with testing.

4. **Give mission prompt**
# Layer scan mode
ros2 service call /set_mission my_robot_package/srv/SetMission "{mission_mode: 'LAYER_SCAN', scan_bottom_target_altitude_m: 2.0, scan_surface_target_depth_m: -0.5, scan_surface_wait_time_sec: 60.0, scan_cycles: 2}"

# Idle mode
ros2 service call /set_mission my_robot_package/srv/SetMission "{mission_mode: 'IDLE'}"

# Target depth hold mode:
ros2 service call /set_mission my_robot_package/srv/SetMission "{mission_mode: 'TARGET_DEPTH_HOLD', hold_target_depth_m: -25.0, hold_duration_sec: 30.0}"

## IV. Troubleshooting & Important Notes

* **GPIO Access Permissions:** If errors like "No access to /dev/gpiomem" occur, ensure the user is in the `gpio` group (`sudo usermod -a -G gpio your_username`, then **reboot**).
* **Monitor Terminal Output:** Watch for errors or warnings.
* **ROS 2 Diagnostic Tools:** Use `rqt_graph`, `rqt_plot`, and `ros2 topic echo <topic_name>` in separate terminals (after sourcing) for diagnostics.
* **Emergency Stop Procedure:** Be ready to stop with `Ctrl+C` in the launch terminal. Shutdown routines should stop the motor and clean up GPIOs.
* **Re-running Initialization:** The `init_neutral_buoyancy_node` will run automatically on each launch with `simulate:=True motorControl:=True`.

---