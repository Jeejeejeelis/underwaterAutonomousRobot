#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix

from my_robot_package.srv import SetMission

class MissionPhase:
    IDLE = "IDLE"
    LS_DESCENDING_TO_BOTTOM = "LS_DESCENDING_TO_BOTTOM"
    LS_ASCENDING_TO_SURFACE = "LS_ASCENDING_TO_SURFACE"
    LS_AT_SURFACE_WAITING = "LS_AT_SURFACE_WAITING"
    TDH_DESCENDING_TO_HOLD = "TDH_DESCENDING_TO_HOLD"
    TDH_HOLDING_AT_DEPTH = "TDH_HOLDING_AT_DEPTH"
    TDH_ASCENDING_TO_SURFACE = "TDH_ASCENDING_TO_SURFACE"
    MISSION_COMPLETE = "MISSION_COMPLETE"

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')

        # Parameters for depths: Sea Surface is Z=0. Depths are NEGATIVE. Altitudes are POSITIVE.
        self.declare_parameter('control_loop_period_sec', 1.0)
        self.declare_parameter('depth_tolerance_m', 0.5)      # Tolerance for reaching a target Z
        self.declare_parameter('default_surface_z_m', -0.5)
        self.declare_parameter('min_safe_altitude_agl_m', 1.5) # Min POSITIVE altitude from DVL

        self.control_loop_period = self.get_parameter('control_loop_period_sec').get_parameter_value().double_value
        self.depth_tolerance = self.get_parameter('depth_tolerance_m').get_parameter_value().double_value
        self.default_surface_z_m = self.get_parameter('default_surface_z_m').get_parameter_value().double_value
        self.min_safe_altitude_agl = self.get_parameter('min_safe_altitude_agl_m').get_parameter_value().double_value
        
        if self.default_surface_z_m > 0:
            self.get_logger().warn(f"Parameter 'default_surface_z_m' ({self.default_surface_z_m}) should be <= 0. Setting to -0.5.")
            self.default_surface_z_m = -0.5


        # State variables
        self.current_mission_mode = SetMission.Request.MISSION_IDLE
        self.current_mission_phase = MissionPhase.IDLE
        self.target_depth_publisher = self.create_publisher(Float32, 'target_depth', 10)

        self.mission_params = {}
        self.mission_timer_start_time = None
        self.mission_wait_duration = 0.0
        self.remaining_scan_cycles = 0

        # Subscriptions
        self.current_z_m = None      # Current Z position of the robot
        self.seafloor_z_m = None     # Z position of the seafloor
        self.dvl_altitude_agl_m = None # Altitude from DVL
        self.has_gnss_fix = False

        self.depth_subscription = self.create_subscription(
            Float32, 'drone_depth', self.drone_depth_callback, 10)
        self.seafloor_depth_subscription = self.create_subscription(
            Float32, 'seafloor_depth', self.seafloor_depth_callback, 10)
        self.dvl_altitude_subscription = self.create_subscription(
            Float32, 'altitude', self.dvl_altitude_callback, 10)
        self.gnss_status_subscription = self.create_subscription(
            NavSatFix, '/gps/fix', self.gnss_status_callback, 10)

        self.set_mission_srv = self.create_service(SetMission, 'set_mission', self.set_mission_callback)
        self.control_timer = self.create_timer(self.control_loop_period, self.control_loop)
        self.get_logger().info("Mission Control Node initialized (Depths are Negative Z). Waiting for mission commands.")

    def drone_depth_callback(self, msg):
        self.current_z_m = msg.data
        # self.get_logger().debug(f"Received current_z_m: {self.current_z_m:.2f}m")

    def seafloor_depth_callback(self, msg):
        self.seafloor_z_m = msg.data
        # self.get_logger().debug(f"Received seafloor_z_m: {self.seafloor_z_m:.2f}m")

    def dvl_altitude_callback(self, msg):
        self.dvl_altitude_agl_m = msg.data
        # self.get_logger().debug(f"Received DVL altitude AGL: {self.dvl_altitude_agl_m:.2f}m")

    def gnss_status_callback(self, msg: NavSatFix):
        if msg.status.status >= NavSatFix.STATUS_FIX:
            self.has_gnss_fix = True
        else:
            self.has_gnss_fix = False

    def publish_target_depth(self, target_z_negative):
        """
        Publishes the target depth to the /target_depth topic.
        The /target_depth topic expects a POSITIVE depth value (depth below surface).
        This function converts the internal negative Z target to a positive depth.
        """
        msg = Float32()
        positive_target_depth = -target_z_negative
        msg.data = float(max(0.0, positive_target_depth))
        self.target_depth_publisher.publish(msg)
        self.get_logger().info(f"Publishing target depth for controller: {msg.data:.2f}m (from internal target Z: {target_z_negative:.2f}m)")

    def is_at_target_z(self, target_z_m):
        if self.current_z_m is None:
            return False
        return abs(self.current_z_m - target_z_m) < self.depth_tolerance

    def start_timer(self, duration_sec):
        self.mission_timer_start_time = self.get_clock().now()
        self.mission_wait_duration = duration_sec

    def is_timer_expired(self):
        if self.mission_timer_start_time is None: return False
        elapsed_time = (self.get_clock().now() - self.mission_timer_start_time).nanoseconds / 1e9
        return elapsed_time >= self.mission_wait_duration

    def set_mission_callback(self, request: SetMission.Request, response: SetMission.Response):
        self.get_logger().info(f"Received set_mission request: Mode='{request.mission_mode}'. Depths in request should be NEGATIVE Z.")
        self.current_mission_mode = request.mission_mode
        
        self.mission_params = {
            'scan_bottom_target_altitude_m': request.scan_bottom_target_altitude_m,
            'scan_surface_target_z_m': request.scan_surface_target_depth_m,
            'scan_surface_wait_time_sec': request.scan_surface_wait_time_sec,
            'scan_cycles': request.scan_cycles,
            'hold_target_z_m': request.hold_target_depth_m,
            'hold_duration_sec': request.hold_duration_sec,
        }
        self.mission_timer_start_time = None

        if self.mission_params['scan_surface_target_z_m'] > 0:
            self.get_logger().error(f"scan_surface_target_depth_m ({self.mission_params['scan_surface_target_z_m']}) must be <= 0. Aborting mission set.")
            response.success = False; response.message = "Invalid scan_surface_target_depth_m (must be <=0)"; return response
        if self.mission_params['hold_target_z_m'] > 0:
            self.get_logger().error(f"hold_target_depth_m ({self.mission_params['hold_target_z_m']}) must be <= 0. Aborting mission set.")
            response.success = False; response.message = "Invalid hold_target_depth_m (must be <=0)"; return response

        if self.current_mission_mode == SetMission.Request.MISSION_IDLE:
            self.current_mission_phase = MissionPhase.IDLE
            response.success = True; response.message = "Transitioning to IDLE mode."
        elif self.current_mission_mode == SetMission.Request.MISSION_LAYER_SCAN:
            if self.mission_params['scan_bottom_target_altitude_m'] < self.min_safe_altitude_agl:
                self.get_logger().warn(f"Requested scan altitude {self.mission_params['scan_bottom_target_altitude_m']}m is below min safe altitude {self.min_safe_altitude_agl}m. Adjusting.")
                self.mission_params['scan_bottom_target_altitude_m'] = self.min_safe_altitude_agl
            self.current_mission_phase = MissionPhase.LS_DESCENDING_TO_BOTTOM
            self.remaining_scan_cycles = self.mission_params['scan_cycles']
            response.success = True; response.message = "Starting LAYER_SCAN mission."
        elif self.current_mission_mode == SetMission.Request.MISSION_TARGET_DEPTH_HOLD:
            self.current_mission_phase = MissionPhase.TDH_DESCENDING_TO_HOLD
            response.success = True; response.message = "Starting TARGET_DEPTH_HOLD mission."
        else:
            self.get_logger().error(f"Unknown mission mode: {self.current_mission_mode}")
            self.current_mission_mode = SetMission.Request.MISSION_IDLE
            self.current_mission_phase = MissionPhase.IDLE
            response.success = False; response.message = "Unknown mission mode requested."
        
        self.get_logger().info(f"Transitioned to Phase: {self.current_mission_phase}")
        return response

    def control_loop(self):
        if self.current_z_m is None:
            self.get_logger().warn("Current Z (depth) unknown, mission control paused.", throttle_duration_sec=5)
            # self.publish_target_depth(self.default_surface_z_m) # Command to surface if depth is lost
            return

        if self.current_mission_phase == MissionPhase.IDLE:
            # When idle, do nothing. Don't publish a target depth.
            # The motor controller will not move until it receives a target.
            self.get_logger().info("In IDLE mode, waiting for a mission command.", throttle_duration_sec=10)
            return

        elif self.current_mission_phase == MissionPhase.TDH_DESCENDING_TO_HOLD:
            target_z = self.mission_params['hold_target_z_m']
            self.publish_target_depth(target_z)
            if self.is_at_target_z(target_z):
                self.get_logger().info(f"Reached hold Z {target_z:.2f}m. Starting hold.")
                self.current_mission_phase = MissionPhase.TDH_HOLDING_AT_DEPTH
                self.start_timer(self.mission_params['hold_duration_sec'])

        elif self.current_mission_phase == MissionPhase.TDH_HOLDING_AT_DEPTH:
            target_z = self.mission_params['hold_target_z_m']
            self.publish_target_depth(target_z)
            if self.is_timer_expired():
                self.get_logger().info("Hold duration expired. Ascending to surface Z.")
                self.current_mission_phase = MissionPhase.TDH_ASCENDING_TO_SURFACE
                self.mission_timer_start_time = None

        elif self.current_mission_phase == MissionPhase.TDH_ASCENDING_TO_SURFACE:
            target_z = self.default_surface_z_m
            self.publish_target_depth(target_z)
            if self.is_at_target_z(target_z):
                self.get_logger().info("Reached surface Z after hold. Mission complete.")
                self.current_mission_phase = MissionPhase.MISSION_COMPLETE

        elif self.current_mission_phase == MissionPhase.LS_DESCENDING_TO_BOTTOM:
            if self.seafloor_z_m is None and self.dvl_altitude_agl_m is None:
                self.get_logger().warn("Seafloor Z and DVL altitude unknown. Cannot descend for layer scan. Holding current Z or surfacing.", throttle_duration_sec=5)
                self.publish_target_depth(self.current_z_m if self.current_z_m is not None else self.default_surface_z_m)
                return

            target_scan_z = 0.0
            if self.seafloor_z_m is not None:
                target_scan_z = self.seafloor_z_m + self.mission_params['scan_bottom_target_altitude_m']
            elif self.dvl_altitude_agl_m is not None and self.current_z_m is not None:
                target_scan_z = self.current_z_m + (self.dvl_altitude_agl_m - self.mission_params['scan_bottom_target_altitude_m'])
            else:
                 self.get_logger().error("Cannot determine target_scan_z for LS_DESCENDING_TO_BOTTOM. Critical sensor data missing unexpectedly.")
                 self.publish_target_depth(self.default_surface_z_m)
                 self.current_mission_phase = MissionPhase.IDLE
                 return

            if target_scan_z > self.default_surface_z_m:
                self.get_logger().warn(f"Calculated scan bottom Z {target_scan_z:.2f}m is shallower than default surface Z {self.default_surface_z_m:.2f}m. Adjusting deeper.")
                target_scan_z = self.default_surface_z_m - 1.0 
            if self.seafloor_z_m is not None and target_scan_z < self.seafloor_z_m:
                 self.get_logger().warn(f"Calculated scan bottom Z {target_scan_z:.2f}m is deeper than known seafloor Z {self.seafloor_z_m:.2f}m. Clamping to seafloor + min_safe_altitude.")
                 target_scan_z = self.seafloor_z_m + self.min_safe_altitude_agl


            self.publish_target_depth(target_scan_z)
            
            at_target_z_condition = self.is_at_target_z(target_scan_z)
            at_target_altitude_condition = False
            if self.dvl_altitude_agl_m is not None:
                at_target_altitude_condition = abs(self.dvl_altitude_agl_m - self.mission_params['scan_bottom_target_altitude_m']) < self.depth_tolerance

            if at_target_z_condition or at_target_altitude_condition:
                self.get_logger().info(f"Reached scan bottom (Current Z: {self.current_z_m:.2f}m, Target Z: {target_scan_z:.2f}m, DVL Alt: {self.dvl_altitude_agl_m if self.dvl_altitude_agl_m is not None else 'N/A'}m). Transitioning to ascend.")
                self.current_mission_phase = MissionPhase.LS_ASCENDING_TO_SURFACE

        elif self.current_mission_phase == MissionPhase.LS_ASCENDING_TO_SURFACE:
            target_z = self.mission_params['scan_surface_target_z_m']
            self.publish_target_depth(target_z)
            if self.is_at_target_z(target_z):
                self.get_logger().info(f"Reached scan surface Z {target_z:.2f}m. Starting surface wait.")
                self.current_mission_phase = MissionPhase.LS_AT_SURFACE_WAITING
                self.start_timer(self.mission_params['scan_surface_wait_time_sec'])

        elif self.current_mission_phase == MissionPhase.LS_AT_SURFACE_WAITING:
            target_z = self.mission_params['scan_surface_target_z_m']
            self.publish_target_depth(target_z)
            time_remaining_str = "N/A"
            if self.mission_timer_start_time is not None:
                 elapsed = (self.get_clock().now() - self.mission_timer_start_time).nanoseconds / 1e9
                 time_remaining = self.mission_wait_duration - elapsed
                 time_remaining_str = f"{time_remaining:.1f}s"

            self.get_logger().info(f"Waiting at surface Z {target_z:.2f}m. GNSS Fix: {self.has_gnss_fix}. Time remaining: {time_remaining_str}", throttle_duration_sec=5)

            if self.is_timer_expired():
                self.get_logger().info("Surface wait time expired.")
                self.mission_timer_start_time = None
                if self.mission_params['scan_cycles'] != 0:
                    if self.remaining_scan_cycles > 0:
                        self.remaining_scan_cycles -= 1
                    self.get_logger().info(f"Cycles remaining: {self.remaining_scan_cycles if self.mission_params['scan_cycles'] > 0 else 'indefinite'}")

                if self.mission_params['scan_cycles'] == 0 or self.remaining_scan_cycles > 0:
                    self.get_logger().info("Starting next scan cycle.")
                    self.current_mission_phase = MissionPhase.LS_DESCENDING_TO_BOTTOM
                else:
                    self.get_logger().info("All scan cycles complete. Layer scan mission finished.")
                    self.current_mission_phase = MissionPhase.MISSION_COMPLETE

        elif self.current_mission_phase == MissionPhase.MISSION_COMPLETE:
            self.get_logger().info("Mission complete. Returning to IDLE behavior (surfacing to default Z).", throttle_duration_sec=10)
            self.current_mission_phase = MissionPhase.IDLE
            self.current_mission_mode = SetMission.Request.MISSION_IDLE

def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission Control Node interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()