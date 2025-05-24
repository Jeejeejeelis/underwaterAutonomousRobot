#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool # Import Bool
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math
import time

METERS_TO_DBAR = 1.0 / 0.992

def get_quaternion_from_euler(roll, pitch, yaw):
    # ... (function remains the same) ...
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class FloatSimulatorNode(Node):
    def __init__(self):
        super().__init__('float_simulator_node')
        self.get_logger().info(f"Float Simulator standing by, waiting for start signal...")

        # ... (parameters remain the same) ...
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('depth_kp', 0.5)
        self.declare_parameter('sim_target_vx', 0.1)
        self.declare_parameter('sim_target_vy', 0.05)
        self.declare_parameter('sim_seafloor_depth_m', 100.0)
        self.declare_parameter('initial_sim_depth_m', 0.5) 
        self.declare_parameter('neutral_buoyancy_ticks', 10890)
        self.declare_parameter('max_encoder_ticks', 21780)
        self.declare_parameter('min_encoder_ticks', 0)
        self.declare_parameter('max_speed_at_limits_mps', 1.0)
        self.declare_parameter('motor_control_mode_active', False)

        # State variables are initialized but not used until the sim starts
        self.x = 0.0
        self.y = 0.0
        self.current_depth_m = 0.0
        self.target_depth_m = 0.0
        self.target_depth_received = False
        self.vx_mps = self.get_parameter('sim_target_vx').get_parameter_value().double_value
        self.vy_mps = self.get_parameter('sim_target_vy').get_parameter_value().double_value
        self.vz_mps = 0.0
        # ... (rest of state variables are the same) ...
        self.sim_seafloor_depth_m = self.get_parameter('sim_seafloor_depth_m').get_parameter_value().double_value
        self.initial_sim_depth_m = self.get_parameter('initial_sim_depth_m').get_parameter_value().double_value
        self.altitude_agl_m = self.sim_seafloor_depth_m 
        self.roll_rad = 0.0; self.pitch_rad = 0.0; self.yaw_rad = 0.0
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.world_frame = 'odom'; self.robot_frame = 'base_link'
        self.last_time = self.get_clock().now()
        self.neutral_ticks_sim_reference = self.get_parameter('neutral_buoyancy_ticks').get_parameter_value().integer_value
        self.max_ticks = self.get_parameter('max_encoder_ticks').get_parameter_value().integer_value
        self.min_ticks = self.get_parameter('min_encoder_ticks').get_parameter_value().integer_value
        self.max_limit_speed = self.get_parameter('max_speed_at_limits_mps').get_parameter_value().double_value
        self.listen_to_external_encoder = self.get_parameter('motor_control_mode_active').get_parameter_value().bool_value
        self.current_encoder_position = float(self.neutral_ticks_sim_reference) 

        # Publishers
        self.sim_pressure_pub = self.create_publisher(Float32, 'sim_pressure_raw', 10)
        # ... (other publishers are the same) ...
        self.sim_dvl_vx_pub = self.create_publisher(Float32, 'sim_dvl_raw_vx', 10)
        self.sim_dvl_vy_pub = self.create_publisher(Float32, 'sim_dvl_raw_vy', 10)
        self.sim_dvl_vz_pub = self.create_publisher(Float32, 'sim_dvl_raw_vz', 10)
        self.sim_dvl_alt_pub = self.create_publisher(Float32, 'sim_dvl_raw_altitude', 10)
        self.ground_truth_depth_pub = self.create_publisher(Float32, 'sim_ground_truth_depth', 10)
        self.ground_truth_altitude_pub = self.create_publisher(Float32, 'sim_ground_truth_altitude', 10)
        self.ground_truth_seafloor_z_pub = self.create_publisher(Float32, 'sim_ground_truth_seafloor_z', 10)
        self.sim_encoder_pub = self.create_publisher(Int32, 'sim_current_encoder_position_visual', 10)

        # --- CHANGE: Do NOT start the timer yet ---
        self.timer = None

        # --- CHANGE: Subscribe to the start signal ---
        qos_profile = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.start_sub = self.create_subscription(Bool, '/simulation/start', self.start_simulation_callback, qos_profile)

    def start_simulation_callback(self, msg):
        if msg.data and self.timer is None:
            self.get_logger().info("Start signal received! Initializing simulation state and starting physics loop.")
            
            # Initialize state now that we are starting
            self.current_depth_m = self.initial_sim_depth_m
            self.altitude_agl_m = self.sim_seafloor_depth_m - self.current_depth_m
            self.last_time = self.get_clock().now()

            # Start listening for control inputs
            self.target_depth_sub = self.create_subscription(Float32, 'target_depth', self.target_depth_callback, 10)
            if self.listen_to_external_encoder:
                self.encoder_sub = self.create_subscription(
                    Int32, '/current_encoder_position', self.external_encoder_callback, 10)

            # Start the main timer
            publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
            self.timer = self.create_timer(1.0 / publish_rate, self.update_state_and_publish)
            
            # We don't need this subscription anymore
            self.destroy_subscription(self.start_sub)

    def external_encoder_callback(self, msg):
        # ... (function is the same) ...
        if self.listen_to_external_encoder:
            self.current_encoder_position = float(msg.data)

    def target_depth_callback(self, msg):
        # ... (function is the same) ...
        if msg.data >= 0: 
            self.target_depth_m = msg.data
            if not self.target_depth_received:
                self.get_logger().info(f"Received initial target depth command: {self.target_depth_m:.2f} m")
                self.target_depth_received = True
        else:
            self.get_logger().warn(f"Received invalid target depth {msg.data:.2f} m. Simulator expects positive depth.")

    def update_state_and_publish(self):
        # ... (This entire function's logic remains the same) ...
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            self.last_time = now
            return 
        self.last_time = now

        if not self.listen_to_external_encoder: 
            if self.target_depth_received:
                depth_error = self.target_depth_m - self.current_depth_m
                kp = self.get_parameter('depth_kp').get_parameter_value().double_value
                self.vz_mps = max(-self.max_limit_speed, min(self.max_limit_speed, kp * depth_error))
            else:
                self.vz_mps = 0.0
        else: 
            encoder_pos_for_calc = self.current_encoder_position
            if abs(encoder_pos_for_calc - self.neutral_ticks_sim_reference) < 1.0:
                self.vz_mps = 0.0
            elif encoder_pos_for_calc > self.neutral_ticks_sim_reference:
                denominator = self.max_ticks - self.neutral_ticks_sim_reference
                if denominator <= 0: denominator = 1.0 
                factor = (encoder_pos_for_calc - self.neutral_ticks_sim_reference) / denominator
                self.vz_mps = factor * self.max_limit_speed
            else: 
                denominator = self.neutral_ticks_sim_reference - self.min_ticks
                if denominator <= 0: denominator = 1.0
                factor = (self.neutral_ticks_sim_reference - encoder_pos_for_calc) / denominator
                self.vz_mps = -factor * self.max_limit_speed
            
            self.vz_mps = max(-self.max_limit_speed, min(self.max_limit_speed, self.vz_mps))

        self.current_depth_m += self.vz_mps * dt 
        
        if self.current_depth_m < 0:
            self.current_depth_m = 0
            if self.vz_mps < 0: self.vz_mps = 0 

        if self.current_depth_m > self.sim_seafloor_depth_m: 
            self.current_depth_m = self.sim_seafloor_depth_m
            if self.vz_mps > 0: self.vz_mps = 0 

        self.x += self.vx_mps * dt
        self.y += self.vy_mps * dt
        self.altitude_agl_m = self.sim_seafloor_depth_m - self.current_depth_m
        if self.altitude_agl_m < 0: self.altitude_agl_m = 0

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.robot_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = -self.current_depth_m 
        qx, qy, qz, qw = get_quaternion_from_euler(self.roll_rad, self.pitch_rad, self.yaw_rad)
        t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.tf_broadcaster.sendTransform(t)

        self.sim_pressure_pub.publish(Float32(data=float(self.current_depth_m * METERS_TO_DBAR)))
        self.sim_dvl_vx_pub.publish(Float32(data=float(self.vx_mps)))
        self.sim_dvl_vy_pub.publish(Float32(data=float(self.vy_mps)))
        self.sim_dvl_vz_pub.publish(Float32(data=float(self.vz_mps))) 
        self.sim_dvl_alt_pub.publish(Float32(data=float(self.altitude_agl_m)))
        self.ground_truth_depth_pub.publish(Float32(data=float(self.current_depth_m)))
        self.ground_truth_altitude_pub.publish(Float32(data=float(self.altitude_agl_m)))
        self.ground_truth_seafloor_z_pub.publish(Float32(data=float(-self.sim_seafloor_depth_m)))
        self.sim_encoder_pub.publish(Int32(data=int(round(self.current_encoder_position))))
        self.get_logger().debug(f"Sim D={self.current_depth_m:.2f}m, Vz={self.vz_mps:.3f}m/s, Enc={self.current_encoder_position:.0f}")

def main(args=None):
    # ... (main function is the same) ...
    rclpy.init(args=args)
    node = None
    try:
        node = FloatSimulatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node: node.get_logger().fatal(f"Unhandled exception: {e}")
        else: print(f"Exception before node init: {e}")
    finally:
        if node: node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()