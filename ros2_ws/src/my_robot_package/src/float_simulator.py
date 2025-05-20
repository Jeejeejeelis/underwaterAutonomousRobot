#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math
import time

METERS_TO_DBAR = 1.0 / 0.992

def get_quaternion_from_euler(roll, pitch, yaw):
    # ... (same as original)
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class FloatSimulatorNode(Node):
    def __init__(self):
        super().__init__('float_simulator_node')
        self.get_logger().info(f"Float Simulator (Source of Raw Sim Data) starting at {time.strftime('%H:%M:%S')}")

        self.declare_parameter('publish_rate', 10.0)
        # self.declare_parameter('max_vertical_speed', 0.5) # Replaced by encoder-based speed
        self.declare_parameter('depth_kp', 0.5) # Proportional gain for internal depth control if motor_control_mode_active is False
        self.declare_parameter('sim_target_vx', 0.1)
        self.declare_parameter('sim_target_vy', 0.05)
        self.declare_parameter('sim_seafloor_depth_m', 100.0)

        # New parameters for encoder-based speed control
        self.declare_parameter('neutral_buoyancy_ticks', 10890)
        self.declare_parameter('max_encoder_ticks', 21780) # Piston fully in (min volume)
        self.declare_parameter('min_encoder_ticks', 0)      # Piston fully out (max volume)
        self.declare_parameter('max_speed_at_limits_mps', 0.5) # Max speed achievable at encoder limits
        self.declare_parameter('motor_control_mode_active', False) # True if motorControl.py is driving the piston

        self.x = 0.0
        self.y = 0.0
        self.current_depth_m = 0.0
        self.target_depth_m = 0.0 # Still used if motor_control_mode_active is False
        self.target_depth_received = False

        self.vx_mps = self.get_parameter('sim_target_vx').value
        self.vy_mps = self.get_parameter('sim_target_vy').value
        self.vz_mps = 0.0
        self.altitude_agl_m = self.get_parameter('sim_seafloor_depth_m').value

        self.roll_rad = 0.0; self.pitch_rad = 0.0; self.yaw_rad = 0.0

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.world_frame = 'odom'; self.robot_frame = 'base_link'
        self.last_time = self.get_clock().now()

        # Encoder related state
        self.neutral_ticks = self.get_parameter('neutral_buoyancy_ticks').value
        self.max_ticks = self.get_parameter('max_encoder_ticks').value
        self.min_ticks = self.get_parameter('min_encoder_ticks').value
        self.max_limit_speed = self.get_parameter('max_speed_at_limits_mps').value
        self.listen_to_external_encoder = self.get_parameter('motor_control_mode_active').value
        
        self.current_encoder_position = float(self.neutral_ticks) # Start at neutral, use float for internal calcs if needed

        if self.listen_to_external_encoder:
            self.encoder_sub = self.create_subscription(
                Int32, '/current_encoder_position', self.external_encoder_callback, 10)
            self.get_logger().info("Simulator listening to /current_encoder_position for motor state.")
        else:
            self.get_logger().info("Simulator using internal PID and simulated encoder for speed calculation.")

        self.target_depth_sub = self.create_subscription(Float32, 'target_depth', self.target_depth_callback, 10)
        self.sim_pressure_pub = self.create_publisher(Float32, 'sim_pressure_raw', 10)
        self.sim_dvl_vx_pub = self.create_publisher(Float32, 'sim_dvl_raw_vx', 10)
        self.sim_dvl_vy_pub = self.create_publisher(Float32, 'sim_dvl_raw_vy', 10)
        self.sim_dvl_vz_pub = self.create_publisher(Float32, 'sim_dvl_raw_vz', 10)
        self.sim_dvl_alt_pub = self.create_publisher(Float32, 'sim_dvl_raw_altitude', 10)
        self.ground_truth_depth_pub = self.create_publisher(Float32, 'sim_ground_truth_depth', 10)
        self.ground_truth_altitude_pub = self.create_publisher(Float32, 'sim_ground_truth_altitude', 10)
        self.ground_truth_seafloor_z_pub = self.create_publisher(Float32, 'sim_ground_truth_seafloor_z', 10)
        # Publisher for the simulator's current (actual or simulated) encoder position
        self.sim_encoder_pub = self.create_publisher(Int32, 'sim_current_encoder_position_visual', 10)


        publish_period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(publish_period, self.update_state_and_publish)
        self.get_logger().info(f"Simulator initialized. Proportional speed active. Listening to ext. encoder: {self.listen_to_external_encoder}")

    def external_encoder_callback(self, msg):
        if self.listen_to_external_encoder:
            self.current_encoder_position = float(msg.data)
            # self.get_logger().debug(f"Received external encoder position: {self.current_encoder_position}")

    def target_depth_callback(self, msg):
        # This target depth is primarily for the internal PID when not listening to external encoder
        if msg.data >= 0:
            self.target_depth_m = msg.data
            if not self.target_depth_received:
                self.get_logger().info(f"Received initial target depth command: {self.target_depth_m:.2f} m (used by internal PID if active)")
                self.target_depth_received = True
            # else:
                # self.get_logger().info(f"Received new target depth command: {self.target_depth_m:.2f} m")
        else:
            self.get_logger().warn(f"Received invalid target depth {msg.data:.2f} m.")

    def update_state_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return
        self.last_time = now

        encoder_pos_for_calc = self.current_encoder_position

        if not self.listen_to_external_encoder:
            # Simulate encoder position based on internal PID's desired speed
            if self.target_depth_received:
                depth_error = self.target_depth_m - self.current_depth_m
                kp = self.get_parameter('depth_kp').value
                pid_desired_vz = max(-self.max_limit_speed, min(self.max_limit_speed, kp * depth_error))

                if abs(pid_desired_vz) < 0.001: # Effectively zero speed
                    encoder_pos_for_calc = float(self.neutral_ticks)
                elif pid_desired_vz > 0: # Ascent (positive speed) -> encoder < neutral
                    # factor = desired_speed / max_speed
                    # encoder = neutral - factor * (neutral - min_encoder)
                    factor = pid_desired_vz / self.max_limit_speed
                    encoder_pos_for_calc = self.neutral_ticks - factor * (self.neutral_ticks - self.min_ticks)
                else: # Descent (pid_desired_vz < 0) -> encoder > neutral
                    # factor = abs(desired_speed) / max_speed
                    # encoder = neutral + factor * (max_encoder - neutral)
                    factor = abs(pid_desired_vz) / self.max_limit_speed
                    encoder_pos_for_calc = self.neutral_ticks + factor * (self.max_ticks - self.neutral_ticks)
                
                # Clamp simulated encoder position
                encoder_pos_for_calc = max(float(self.min_ticks), min(float(self.max_ticks), encoder_pos_for_calc))
                self.current_encoder_position = encoder_pos_for_calc # Update the simulator's own view of its piston
            else: # No target depth received, assume neutral buoyancy
                encoder_pos_for_calc = float(self.neutral_ticks)
                self.current_encoder_position = encoder_pos_for_calc


        # Calculate vz_mps based on encoder_pos_for_calc (either from external or internal simulation)
        # Encoder > neutral_ticks (towards max_ticks) -> Piston IN, Volume DECREASE, Vehicle DESCENDS (negative vz_mps)
        # Encoder < neutral_ticks (towards min_ticks) -> Piston OUT, Volume INCREASE, Vehicle ASCENDS (positive vz_mps)
        if abs(encoder_pos_for_calc - self.neutral_ticks) < 1.0: # Tolerance for floating point comparison
            self.vz_mps = 0.0
        elif encoder_pos_for_calc > self.neutral_ticks: # Piston moving IN / IN position -> DESCEND
            denominator = self.max_ticks - self.neutral_ticks
            if denominator <= 0: denominator = 1.0 # Avoid division by zero
            factor = (encoder_pos_for_calc - self.neutral_ticks) / denominator
            self.vz_mps = -factor * self.max_limit_speed
        else: # encoder_pos_for_calc < self.neutral_ticks, Piston moving OUT / OUT position -> ASCEND
            denominator = self.neutral_ticks - self.min_ticks
            if denominator <= 0: denominator = 1.0
            factor = (self.neutral_ticks - encoder_pos_for_calc) / denominator
            self.vz_mps = factor * self.max_limit_speed
        
        self.vz_mps = max(-self.max_limit_speed, min(self.max_limit_speed, self.vz_mps)) # Final clamp

        # Update position
        self.current_depth_m += self.vz_mps * dt
        if self.current_depth_m < 0:
            self.current_depth_m = 0
            if self.vz_mps < 0: self.vz_mps = 0 # Hit surface, cannot continue ascending if it was

        self.x += self.vx_mps * dt
        self.y += self.vy_mps * dt

        seafloor_depth_m = self.get_parameter('sim_seafloor_depth_m').value
        self.altitude_agl_m = seafloor_depth_m - self.current_depth_m
        if self.altitude_agl_m < 0: # Hit seafloor
            self.altitude_agl_m = 0
            self.current_depth_m = seafloor_depth_m
            if self.vz_mps > 0: self.vz_mps = 0 # Hit seafloor, cannot continue descending

        # TF transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.robot_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = -self.current_depth_m # NED convention for z
        qx, qy, qz, qw = get_quaternion_from_euler(self.roll_rad, self.pitch_rad, self.yaw_rad)
        t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.tf_broadcaster.sendTransform(t)

        # Publish simulated sensor data
        self.sim_pressure_pub.publish(Float32(data=self.current_depth_m * METERS_TO_DBAR))
        self.sim_dvl_vx_pub.publish(Float32(data=self.vx_mps))
        self.sim_dvl_vy_pub.publish(Float32(data=self.vy_mps))
        self.sim_dvl_vz_pub.publish(Float32(data=self.vz_mps)) # This is the calculated Vz
        self.sim_dvl_alt_pub.publish(Float32(data=self.altitude_agl_m))

        self.ground_truth_depth_pub.publish(Float32(data=self.current_depth_m))
        self.ground_truth_altitude_pub.publish(Float32(data=self.altitude_agl_m))
        self.ground_truth_seafloor_z_pub.publish(Float32(data=-seafloor_depth_m))
        self.sim_encoder_pub.publish(Int32(data=int(round(self.current_encoder_position))))


        self.get_logger().debug(f"Sim D={self.current_depth_m:.2f}m, Vz={self.vz_mps:.3f}m/s, Enc={self.current_encoder_position:.0f}")

def main(args=None):
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()