#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
# message_filters is removed

DBAR_TO_METERS_FACTOR = 0.992

class DepthCalculatorNode(Node):
    def __init__(self):
        super().__init__('depth_calculator_node')
        self.get_logger().info("Depth Calculator Node starting...")

        self.declare_parameter('pressure_to_depth_factor', DBAR_TO_METERS_FACTOR)
        self.conversion_factor = self.get_parameter('pressure_to_depth_factor').get_parameter_value().double_value

        self.pressure_sub = self.create_subscription(
            Float32, 
            'pressure', 
            self.pressure_callback, 
            10)
        self.dvl_altitude_sub = self.create_subscription(
            Float32, 
            'altitude', 
            self.dvl_altitude_callback, 
            10)
        
        self.sim_pressure_sub = self.create_subscription(
             Float32,
             'sim_pressure_raw', # From float_simulator.py
             self.sim_pressure_callback,
             10 
        )

        self.drone_depth_publisher = self.create_publisher(Float32, 'drone_depth', 10)
        self.seafloor_depth_publisher = self.create_publisher(Float32, 'seafloor_depth', 10)

        self.current_pressure_dbar = None
        self.current_dvl_altitude_agl_m = None
        self.current_drone_depth_positive_m = None 

        self.get_logger().info("Depth Calculator Node initialized. Subscribed to '/pressure', '/sim_pressure_raw', and '/altitude'.")
        self.get_logger().info("Publishing to '/drone_depth' (negative Z) and '/seafloor_depth' (negative Z).")

    def process_and_publish_depths(self):
        if self.current_pressure_dbar is None:
            self.get_logger().info("DepthCalc: Waiting for pressure data (current_pressure_dbar is None).", throttle_duration_sec=5)
            return

        drone_depth_positive = self.current_pressure_dbar * self.conversion_factor
        if drone_depth_positive < 0:
            self.get_logger().warn(f"DepthCalc: Calculated negative drone depth ({drone_depth_positive:.2f}m) from gauge pressure {self.current_pressure_dbar:.2f}dbar. Clamping to 0.")
            drone_depth_positive = 0.0
        
        self.current_drone_depth_positive_m = drone_depth_positive 
        drone_depth_z_negative = -drone_depth_positive 

        depth_msg = Float32()
        depth_msg.data = drone_depth_z_negative
        self.drone_depth_publisher.publish(depth_msg)
        self.get_logger().info(f"DepthCalc: PUBLISHED /drone_depth: {drone_depth_z_negative:.2f}m (from pressure {self.current_pressure_dbar:.2f}dbar)")

        if self.current_dvl_altitude_agl_m is not None and self.current_dvl_altitude_agl_m >= 0:
            seafloor_depth_positive = self.current_drone_depth_positive_m + self.current_dvl_altitude_agl_m
            seafloor_depth_z_negative = -seafloor_depth_positive
            
            seafloor_msg = Float32()
            seafloor_msg.data = seafloor_depth_z_negative
            self.seafloor_depth_publisher.publish(seafloor_msg)
            self.get_logger().info(f"DepthCalc: PUBLISHED /seafloor_depth: {seafloor_depth_z_negative:.2f}m")
        elif self.current_dvl_altitude_agl_m is not None:
             self.get_logger().info(f"DepthCalc: DVL altitude is negative ({self.current_dvl_altitude_agl_m:.2f}m), not publishing seafloor_depth based on it.", throttle_duration_sec=5)

    def pressure_callback(self, msg):
        self.get_logger().info(f"DepthCalc: Received REAL /pressure: {msg.data:.2f} dbar")
        self.current_pressure_dbar = msg.data
        self.process_and_publish_depths()

    def sim_pressure_callback(self, msg):
        self.get_logger().info(f"DepthCalc: Received /sim_pressure_raw: {msg.data:.2f} dbar (from simulator)")
        self.current_pressure_dbar = msg.data 
        self.process_and_publish_depths()

    def dvl_altitude_callback(self, msg):
        self.get_logger().info(f"DepthCalc: Received /altitude: {msg.data:.2f} m AGL")
        self.current_dvl_altitude_agl_m = msg.data
        if self.current_pressure_dbar is not None:
             self.process_and_publish_depths()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthCalculatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("DepthCalculatorNode interrupted by user.")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"DepthCalculatorNode unhandled exception: {e}")
        else:
            print(f"DepthCalculatorNode unhandled exception during creation: {e}")
    finally:
        if node:
            node.get_logger().info("Shutting down DepthCalculatorNode.")
            node.destroy_node()
        if rclpy.ok(): # Check if context is still valid
            rclpy.shutdown()

if __name__ == '__main__':
    main()