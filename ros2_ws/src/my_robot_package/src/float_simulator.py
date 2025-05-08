#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Float32
import math
import time
import traceback

# Standard gravity and typical seawater density. Make these parameters if high accuracy is needed.
# Or use a proper library if complex hydrostatics are involved.
# For pressure conversion: 1 dbar ~ 1.019716 m of seawater (standard, at 0degC, salinity 35psu)
# So, depth_m = pressure_dbar * 1.019716
# pressure_dbar = depth_m / 1.019716
# Your previous constants:
# DBAR_TO_METERS = 0.992
# METERS_TO_DBAR = 1.0 / DBAR_TO_METERS
# Using a more standard conversion factor or making it configurable:
# Let's define conversion factor based on dbar being force/area and depth being height of water column
# Pressure (Pa) = rho * g * h
# 1 dbar = 10000 Pa
# rho_seawater_kg_m3 = 1025  (typical)
# g_ms2 = 9.80665 (standard gravity)
# depth_m = (pressure_dbar * 10000) / (rho_seawater_kg_m3 * g_ms2)
# depth_m_per_dbar = 10000 / (1025 * 9.80665) # approx 0.9945 m/dbar
# dbar_per_meter = 1.0 / depth_m_per_dbar     # approx 1.0055 dbar/m
# For simplicity, we can make these parameters if high precision is needed.
# For now, let's make them clearly named constants.
DEPTH_METERS_PER_DBAR = 0.9945 # Approximate, derived from rho=1025 kg/m3, g=9.80665 m/s^2
DBAR_PER_METER = 1.0 / DEPTH_METERS_PER_DBAR


class FloatSimulatorNode(Node):
    def __init__(self):
        super().__init__('float_simulator_node')
        self.get_logger().info(f"Float Simulator (Lean Version for RQT) starting at {time.strftime('%H:%M:%S')}")

        pub_rate_desc = ParameterDescriptor(description='Publish rate in Hz.')
        max_vspeed_desc = ParameterDescriptor(description='Max ascent/descent speed (m/s).')
        depth_kp_desc = ParameterDescriptor(description='Proportional gain for depth control.')
        seafloor_depth_desc = ParameterDescriptor(description='Actual depth of the seafloor from surface (positive, meters).')
        initial_x_desc = ParameterDescriptor(description='Initial X position of the float (meters).')
        initial_y_desc = ParameterDescriptor(description='Initial Y position of the float (meters).')
        initial_depth_desc = ParameterDescriptor(description='Initial depth of the float (positive, meters).')

        self.declare_parameter('publish_rate_hz', 10.0, pub_rate_desc)
        self.declare_parameter('max_vertical_speed_mps', 0.1, max_vspeed_desc)
        self.declare_parameter('depth_control_kp', 0.5, depth_kp_desc)
        self.declare_parameter('world_seafloor_actual_depth_m', 100.0, seafloor_depth_desc)
        self.declare_parameter('initial_x_m', 0.0, initial_x_desc)
        self.declare_parameter('initial_y_m', 0.0, initial_y_desc)
        self.declare_parameter('initial_depth_m', 0.0, initial_depth_desc)


        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.max_vertical_speed = self.get_parameter('max_vertical_speed_mps').get_parameter_value().double_value
        self.depth_kp = self.get_parameter('depth_control_kp').get_parameter_value().double_value
        
        self.world_seafloor_depth_m = self.get_parameter('world_seafloor_actual_depth_m').get_parameter_value().double_value
        self.world_seafloor_z = -self.world_seafloor_depth_m
        self.get_logger().info(f"Simulator: World seafloor DEFINED at depth: {self.world_seafloor_depth_m:.2f} m (World Z: {self.world_seafloor_z:.2f} m)")
        if self.world_seafloor_depth_m <= 0:
            self.get_logger().warn("Simulator: world_seafloor_actual_depth_m is non-positive. Seafloor at or above surface.")

        self.x = self.get_parameter('initial_x_m').get_parameter_value().double_value
        self.y = self.get_parameter('initial_y_m').get_parameter_value().double_value
        self.current_depth = self.get_parameter('initial_depth_m').get_parameter_value().double_value
        if self.current_depth < 0: self.current_depth = 0.0
        if self.current_depth > self.world_seafloor_depth_m and self.world_seafloor_depth_m > 0:
            self.current_depth = self.world_seafloor_depth_m
        self.get_logger().info(f"Simulator: Initial position X={self.x:.2f}m, Y={self.y:.2f}m, Depth={self.current_depth:.2f}m")
        # ### END OF NEW CODE ###
        
        self.target_depth = self.current_depth
        self.target_depth_received = False

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz_commanded = 0.0
        
        self.current_altitude_external = None

        self.last_time = self.get_clock().now()

        # Subscribers
        self.vx_sub = self.create_subscription(Float32, 'dvl/vx', self.vx_callback, 10) # Assuming DVL provides these
        self.vy_sub = self.create_subscription(Float32, 'dvl/vy', self.vy_callback, 10)
        self.alt_sub = self.create_subscription(Float32, 'dvl/altitude_m', self.altitude_callback, 10) # From DVL
        self.pressure_sub = self.create_subscription(Float32, 'pressure', self.pressure_callback, 10) # From CTD
        self.target_depth_sub = self.create_subscription(Float32, 'controller/target_depth_m', self.target_depth_callback, 10)

        # Publishers
        self.depth_pub = self.create_publisher(Float32, 'simulator/depth_m', 10)
        self.float_z_pub = self.create_publisher(Float32, 'simulator/z_position_m', 10)
        self.float_x_pub = self.create_publisher(Float32, 'simulator/x_position_m', 10)
        self.float_y_pub = self.create_publisher(Float32, 'simulator/y_position_m', 10)
        
        self.seafloor_actual_depth_pub = self.create_publisher(Float32, 'simulator/defined_seafloor_depth_m', 10)
        self.seafloor_z_pub = self.create_publisher(Float32, 'simulator/defined_seafloor_z_position_m', 10)
        
        self.sim_pressure_pub = self.create_publisher(Float32, 'simulated_pressure_dbar', 10) # For CTD sim
        self.gt_altitude_pub = self.create_publisher(Float32, 'simulator/ground_truth_altitude_m', 10) # For DVL sim or comparison

        if self.publish_rate <= 0:
            self.get_logger().error("Publish rate must be positive. Node will not start timer.")
            return
            
        publish_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(publish_period, self.update_state_and_publish_data)
        self.get_logger().info(f"Simulator (Lean) initialized. Publish rate: {self.publish_rate:.2f} Hz.")
        if not self.target_depth_received :
             self.get_logger().info(f"Waiting for target depth command on {self.target_depth_sub.topic_name}")


    def vx_callback(self, msg):
        self.current_vx = msg.data

    def vy_callback(self, msg):
        self.current_vy = msg.data

    def altitude_callback(self, msg):
        self.current_altitude_external = msg.data
        # self.get_logger().debug(f"Simulator: Received external DVL altitude sensor reading: {self.current_altitude_external:.2f} m")

    def pressure_callback(self, msg):
        # self.get_logger().debug(f"Simulator: Received external CTD pressure reading: {msg.data:.2f} dbar")
        pass

    def target_depth_callback(self, msg):
        # Target depth is a positive value representing desired depth from surface
        requested_target_depth = msg.data
        if requested_target_depth >= 0:
             # Clamp target depth to be within operational limits (surface to defined seafloor)
             clamped_target = max(0.0, requested_target_depth) # Cannot target above surface
             if self.world_seafloor_depth_m > 0 : # Only clamp to seafloor if it's defined below surface
                clamped_target = min(clamped_target, self.world_seafloor_depth_m)

             if clamped_target != requested_target_depth:
                 self.get_logger().warn(
                     f"Simulator: Requested target depth {requested_target_depth:.2f}m "
                     f"was outside limits [0, {self.world_seafloor_depth_m:.2f}m]. Clamped to {clamped_target:.2f}m."
                 )
             self.target_depth = clamped_target
             
             if not self.target_depth_received:
                 self.get_logger().info(f"Simulator: Received initial target depth command: {self.target_depth:.2f} m")
                 self.target_depth_received = True
             # else:
                 # self.get_logger().info(f"Simulator: Received new target depth command: {self.target_depth:.2f} m")
        else:
            self.get_logger().warn(f"Simulator: Received invalid target depth {requested_target_depth:.2f} m. Target depth must be non-negative.")

    def update_state_and_publish_data(self):
        now = self.get_clock().now()
        dt_duration = now - self.last_time
        dt = dt_duration.nanoseconds / 1e9

        if dt <= 0:
            # self.get_logger().debug(f"Simulator: Delta time is zero or negative ({dt:.4f}s), skipping update.")
            self.last_time = now
            return
        self.last_time = now

        if self.target_depth_received:
            depth_error = self.target_depth - self.current_depth
            
            vz_world_command = self.depth_kp * depth_error
            
            self.current_vz_commanded = max(-self.max_vertical_speed, min(self.max_vertical_speed, vz_world_command))
            
            self.current_depth += self.current_vz_commanded * dt
        else:
            self.current_vz_commanded = 0.0

        if self.current_depth < 0.0:
            self.current_depth = 0.0
            if self.current_vz_commanded < 0: self.current_vz_commanded = 0.0

        if self.world_seafloor_depth_m > 0:
            if self.current_depth > self.world_seafloor_depth_m:
                self.current_depth = self.world_seafloor_depth_m
                if self.current_vz_commanded > 0: self.current_vz_commanded = 0.0
        
        self.x += self.current_vx * dt
        self.y += self.current_vy * dt

        self.depth_pub.publish(Float32(data=self.current_depth))
        self.float_z_pub.publish(Float32(data=-self.current_depth))
        self.float_x_pub.publish(Float32(data=self.x))
        self.float_y_pub.publish(Float32(data=self.y))

        sim_pressure_val_dbar = self.current_depth * DBAR_PER_METER
        self.sim_pressure_pub.publish(Float32(data=sim_pressure_val_dbar))

        ground_truth_altitude_val_m = 0.0
        if self.world_seafloor_depth_m > 0:
            calculated_gt_altitude = self.world_seafloor_depth_m - self.current_depth
            ground_truth_altitude_val_m = max(0.0, calculated_gt_altitude)
        self.gt_altitude_pub.publish(Float32(data=ground_truth_altitude_val_m))
        
        self.seafloor_actual_depth_pub.publish(Float32(data=self.world_seafloor_depth_m))
        self.seafloor_z_pub.publish(Float32(data=self.world_seafloor_z))
        
        if self.current_altitude_external is not None:
            self.get_logger().debug(
                f"Simulator: ExtAlt={self.current_altitude_external:.2f}m, GTAlt={ground_truth_altitude_val_m:.2f}m, "
                f"Depth={self.current_depth:.2f}m, TargetDepth={self.target_depth:.2f}m, VzCmd={self.current_vz_commanded:.3f}m/s"
            )

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FloatSimulatorNode()
        if node.publish_rate > 0 :
            rclpy.spin(node)
        else:
            node.get_logger().error("Simulator not spun due to invalid publish_rate.")
    except KeyboardInterrupt:
        if node: node.get_logger().info("Simulator: Keyboard interrupt, shutting down.")
    except Exception as e:
        if node:
            exc_info_str = traceback.format_exc()
            node.get_logger().fatal(f"Simulator: Unhandled exception: {e}\n{exc_info_str}")
        else:
            print(f"Simulator: Exception before node initialization: {e}")
    finally:
        if node:
            node.get_logger().info("Simulator: Shutting down and destroying node.")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Simulator: Terminated.")

if __name__ == '__main__':
    main()