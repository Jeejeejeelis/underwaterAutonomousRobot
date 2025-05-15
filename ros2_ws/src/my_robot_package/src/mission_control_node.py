#!/usr/bin/env python3
# File: ros2_ws/src/my_robot_package/src/mission_control_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool # Assuming Bool for GNSS fix status for simplicity
from sensor_msgs.msg import NavSatFix # For actual GNSS status

# Import the custom service
from my_robot_package.srv import SetMission # Replace my_robot_package with your actual package name

# Define states for clarity
class MissionPhase:
    IDLE = "IDLE"
    # Layer Scan Phases
    LS_DESCENDING_TO_BOTTOM = "LS_DESCENDING_TO_BOTTOM"
    LS_AT_BOTTOM_TRANSITION = "LS_AT_BOTTOM_TRANSITION" # Brief phase to ensure stability or log
    LS_ASCENDING_TO_SURFACE = "LS_ASCENDING_TO_SURFACE"
    LS_AT_SURFACE_WAITING = "LS_AT_SURFACE_WAITING"
    # Target Depth Hold Phases
    TDH_DESCENDING_TO_HOLD = "TDH_DESCENDING_TO_HOLD"
    TDH_HOLDING_AT_DEPTH = "TDH_HOLDING_AT_DEPTH"
    TDH_ASCENDING_TO_SURFACE = "TDH_ASCENDING_TO_SURFACE" # Or to a defined end depth
    MISSION_COMPLETE = "MISSION_COMPLETE"

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')

        # Parameters
        self.declare_parameter('control_loop_period_sec', 1.0) # Main loop frequency
        self.declare_parameter('depth_tolerance_m', 0.5)      # How close to target depth is "close enough"
        self.declare_parameter('default_surface_depth_m', 0.5) # Standard depth for "surfacing"
        self.declare_parameter('min_safe_altitude_agl_m', 1.5) # Minimum safe altitude from DVL for bottom scan

        self.control_loop_period = self.get_parameter('control_loop_period_sec').get_parameter_value().double_value
        self.depth_tolerance = self.get_parameter('depth_tolerance_m').get_parameter_value().double_value
        self.default_surface_depth = self.get_parameter('default_surface_depth_m').get_parameter_value().double_value
        self.min_safe_altitude_agl = self.get_parameter('min_safe_altitude_agl_m').get_parameter_value().double_value

        # State variables
        self.current_mission_mode = SetMission.Request.MISSION_IDLE
        self.current_mission_phase = MissionPhase.IDLE
        self.target_depth_publisher = self.create_publisher(Float32, 'target_depth', 10)

        # Mission parameters (populated by service call)
        self.mission_params = {}
        self.mission_timer_start_time = None
        self.mission_wait_duration = 0.0
        self.remaining_scan_cycles = 0

        # Subscriptions
        self.current_depth_m = None
        self.seafloor_depth_m = None # Depth of seafloor from surface (positive)
        self.dvl_altitude_agl_m = None # Altitude from DVL (positive)
        self.has_gnss_fix = False

        self.depth_subscription = self.create_subscription(
            Float32, 'depth_m', self.depth_callback, 10) # From ctd_node (or fused estimator)
        self.seafloor_depth_subscription = self.create_subscription(
            Float32, 'seafloor_depth_m', self.seafloor_depth_callback, 10) # From depth_calculator
        self.dvl_altitude_subscription = self.create_subscription(
            Float32, 'dvl/altitude_agl', self.dvl_altitude_callback, 10) # From dvl_node
        self.gnss_status_subscription = self.create_subscription(
            NavSatFix, '/gps/fix', self.gnss_status_callback, 10) # From gnss_node

        # Service Server
        self.set_mission_srv = self.create_service(SetMission, 'set_mission', self.set_mission_callback)

        # Main control loop timer
        self.control_timer = self.create_timer(self.control_loop_period, self.control_loop)

        self.get_logger().info("Mission Control Node initialized. Waiting for mission commands.")

    def depth_callback(self, msg):
        self.current_depth_m = msg.data
        # self.get_logger().debug(f"Received depth: {self.current_depth_m:.2f}m")


    def seafloor_depth_callback(self, msg):
        self.seafloor_depth_m = msg.data
        # self.get_logger().debug(f"Received seafloor depth: {self.seafloor_depth_m:.2f}m")

    def dvl_altitude_callback(self, msg):
        self.dvl_altitude_agl_m = msg.data
        # self.get_logger().debug(f"Received DVL altitude: {self.dvl_altitude_agl_m:.2f}m")

    def gnss_status_callback(self, msg: NavSatFix):
        # STATUS_NO_FIX = -1, STATUS_FIX = 0, STATUS_SBAS_FIX = 1, STATUS_GBAS_FIX = 2
        if msg.status.status >= NavSatFix.STATUS_FIX:
            self.has_gnss_fix = True
            # self.get_logger().debug("GNSS Fix Acquired.")
        else:
            self.has_gnss_fix = False
            # self.get_logger().debug("GNSS No Fix.")

    def publish_target_depth(self, depth_m):
        msg = Float32()
        msg.data = float(max(0.0, depth_m)) # Ensure target depth is not negative (above surface)
        self.target_depth_publisher.publish(msg)
        self.get_logger().info(f"Publishing target depth: {msg.data:.2f}m")

    def is_at_target_depth(self, target_depth_m):
        if self.current_depth_m is None:
            return False
        return abs(self.current_depth_m - target_depth_m) < self.depth_tolerance

    def start_timer(self, duration_sec):
        self.mission_timer_start_time = self.get_clock().now()
        self.mission_wait_duration = duration_sec

    def is_timer_expired(self):
        if self.mission_timer_start_time is None:
            return False
        elapsed_time = (self.get_clock().now() - self.mission_timer_start_time).nanoseconds / 1e9
        return elapsed_time >= self.mission_wait_duration

    def set_mission_callback(self, request: SetMission.Request, response: SetMission.Response):
        self.get_logger().info(f"Received set_mission request: Mode='{request.mission_mode}'")
        self.current_mission_mode = request.mission_mode
        self.mission_params = {
            'scan_bottom_target_altitude_m': request.scan_bottom_target_altitude_m,
            'scan_surface_target_depth_m': request.scan_surface_target_depth_m,
            'scan_surface_wait_time_sec': request.scan_surface_wait_time_sec,
            'scan_cycles': request.scan_cycles,
            'hold_target_depth_m': request.hold_target_depth_m,
            'hold_duration_sec': request.hold_duration_sec,
        }
        self.mission_timer_start_time = None # Reset timer

        if self.current_mission_mode == SetMission.Request.MISSION_IDLE:
            self.current_mission_phase = MissionPhase.IDLE
            response.success = True
            response.message = "Transitioning to IDLE mode."
        elif self.current_mission_mode == SetMission.Request.MISSION_LAYER_SCAN:
            if self.mission_params['scan_bottom_target_altitude_m'] < self.min_safe_altitude_agl:
                self.get_logger().warn(f"Requested scan altitude {self.mission_params['scan_bottom_target_altitude_m']}m is below min safe altitude {self.min_safe_altitude_agl}m. Adjusting.")
                self.mission_params['scan_bottom_target_altitude_m'] = self.min_safe_altitude_agl

            self.current_mission_phase = MissionPhase.LS_DESCENDING_TO_BOTTOM
            self.remaining_scan_cycles = self.mission_params['scan_cycles']
            response.success = True
            response.message = "Starting LAYER_SCAN mission."
        elif self.current_mission_mode == SetMission.Request.MISSION_TARGET_DEPTH_HOLD:
            self.current_mission_phase = MissionPhase.TDH_DESCENDING_TO_HOLD
            response.success = True
            response.message = "Starting TARGET_DEPTH_HOLD mission."
        else:
            self.get_logger().error(f"Unknown mission mode: {self.current_mission_mode}")
            self.current_mission_mode = SetMission.Request.MISSION_IDLE # Revert to IDLE
            self.current_mission_phase = MissionPhase.IDLE
            response.success = False
            response.message = "Unknown mission mode requested."
        
        self.get_logger().info(f"Transitioned to Phase: {self.current_mission_phase}")
        return response

    def control_loop(self):
        if self.current_depth_m is None:
            self.get_logger().warn("Current depth unknown, mission control paused.", throttle_duration_sec=5)
            # Optionally publish a safe surface depth if completely lost
            # self.publish_target_depth(self.default_surface_depth)
            return

        # --- IDLE Mode ---
        if self.current_mission_phase == MissionPhase.IDLE:
            # In IDLE, try to maintain current depth or a safe surface depth
            # For a float, "maintaining current depth" might mean minimal buoyancy adjustment
            # or slowly surfacing if no target. Let's make it surface gently.
            self.publish_target_depth(self.default_surface_depth)
            # self.get_logger().info("In IDLE phase, maintaining surface depth.", throttle_duration_sec=10)

        # --- TARGET_DEPTH_HOLD Mode ---
        elif self.current_mission_phase == MissionPhase.TDH_DESCENDING_TO_HOLD:
            target = self.mission_params['hold_target_depth_m']
            self.publish_target_depth(target)
            if self.is_at_target_depth(target):
                self.get_logger().info(f"Reached hold depth {target:.2f}m. Starting hold.")
                self.current_mission_phase = MissionPhase.TDH_HOLDING_AT_DEPTH
                self.start_timer(self.mission_params['hold_duration_sec'])

        elif self.current_mission_phase == MissionPhase.TDH_HOLDING_AT_DEPTH:
            target = self.mission_params['hold_target_depth_m']
            self.publish_target_depth(target) # Continue publishing to ensure hold
            if self.is_timer_expired():
                self.get_logger().info(f"Hold duration expired. Ascending to surface.")
                self.current_mission_phase = MissionPhase.TDH_ASCENDING_TO_SURFACE
                self.mission_timer_start_time = None # Reset timer

        elif self.current_mission_phase == MissionPhase.TDH_ASCENDING_TO_SURFACE:
            target = self.default_surface_depth
            self.publish_target_depth(target)
            if self.is_at_target_depth(target):
                self.get_logger().info("Reached surface after hold. Mission complete.")
                self.current_mission_phase = MissionPhase.MISSION_COMPLETE # Or IDLE

        # --- LAYER_SCAN Mode ---
        elif self.current_mission_phase == MissionPhase.LS_DESCENDING_TO_BOTTOM:
            if self.seafloor_depth_m is None and self.dvl_altitude_agl_m is None:
                self.get_logger().warn("Seafloor depth and DVL altitude unknown. Cannot descend for layer scan. Holding current depth or surfacing.", throttle_duration_sec=5)
                self.publish_target_depth(self.current_depth_m) # Hold current or surface
                # Potentially: self.current_mission_phase = MissionPhase.IDLE (abort scan)
                return

            target_scan_depth = -1.0
            if self.seafloor_depth_m is not None:
                target_scan_depth = self.seafloor_depth_m - self.mission_params['scan_bottom_target_altitude_m']
            elif self.dvl_altitude_agl_m is not None: # Use DVL if seafloor_depth isn't available or preferred
                # This requires knowing current depth to calculate target_depth for the controller
                target_scan_depth = self.current_depth_m + (self.dvl_altitude_agl_m - self.mission_params['scan_bottom_target_altitude_m'])


            if target_scan_depth < self.default_surface_depth: # Safety check if calculated target is too shallow
                 target_scan_depth = self.default_surface_depth + 1.0 # Go at least a bit down
                 self.get_logger().warn(f"Calculated scan bottom depth is too shallow. Setting to {target_scan_depth:.2f}m")


            self.publish_target_depth(target_scan_depth)
            # Condition to transition: either reached calculated depth or DVL altitude is met
            at_target_depth = self.is_at_target_depth(target_scan_depth)
            at_target_altitude = False
            if self.dvl_altitude_agl_m is not None:
                at_target_altitude = abs(self.dvl_altitude_agl_m - self.mission_params['scan_bottom_target_altitude_m']) < self.depth_tolerance # Using depth_tolerance for altitude too

            if at_target_depth or at_target_altitude:
                self.get_logger().info(f"Reached scan bottom (Depth: {self.current_depth_m:.2f}m, DVL Alt: {self.dvl_altitude_agl_m if self.dvl_altitude_agl_m is not None else 'N/A'}m). Transitioning to ascend.")
                self.current_mission_phase = MissionPhase.LS_ASCENDING_TO_SURFACE

        elif self.current_mission_phase == MissionPhase.LS_ASCENDING_TO_SURFACE:
            target = self.mission_params['scan_surface_target_depth_m']
            self.publish_target_depth(target)
            if self.is_at_target_depth(target):
                self.get_logger().info(f"Reached scan surface depth {target:.2f}m. Starting surface wait.")
                self.current_mission_phase = MissionPhase.LS_AT_SURFACE_WAITING
                self.start_timer(self.mission_params['scan_surface_wait_time_sec'])

        elif self.current_mission_phase == MissionPhase.LS_AT_SURFACE_WAITING:
            target = self.mission_params['scan_surface_target_depth_m']
            self.publish_target_depth(target) # Maintain surface depth
            self.get_logger().info(f"Waiting at surface. GNSS Fix: {self.has_gnss_fix}. Time remaining: {(self.mission_wait_duration - (self.get_clock().now() - self.mission_timer_start_time).nanoseconds / 1e9):.1f}s", throttle_duration_sec=5)

            if self.is_timer_expired():
                self.get_logger().info("Surface wait time expired.")
                self.mission_timer_start_time = None # Reset timer
                if self.remaining_scan_cycles > 0:
                    self.remaining_scan_cycles -= 1
                    self.get_logger().info(f"Cycles remaining: {self.remaining_scan_cycles}")
                
                if self.mission_params['scan_cycles'] == 0 or self.remaining_scan_cycles > 0 : # Indefinite or cycles left
                    self.get_logger().info("Starting next scan cycle.")
                    self.current_mission_phase = MissionPhase.LS_DESCENDING_TO_BOTTOM
                else:
                    self.get_logger().info("All scan cycles complete. Layer scan mission finished.")
                    self.current_mission_phase = MissionPhase.MISSION_COMPLETE # Or IDLE

        # --- MISSION_COMPLETE ---
        elif self.current_mission_phase == MissionPhase.MISSION_COMPLETE:
            self.get_logger().info("Mission complete. Returning to IDLE behavior (surfacing).", throttle_duration_sec=10)
            # For now, just go to IDLE which will surface it.
            # Could also publish a specific "mission end depth"
            self.current_mission_phase = MissionPhase.IDLE # Transition to IDLE
            self.current_mission_mode = SetMission.Request.MISSION_IDLE


def main(args=None):
    rclpy.init(args=args)
    mission_control_node = MissionControlNode()
    try:
        rclpy.spin(mission_control_node)
    except KeyboardInterrupt:
        mission_control_node.get_logger().info("Mission Control Node interrupted.")
    finally:
        mission_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()