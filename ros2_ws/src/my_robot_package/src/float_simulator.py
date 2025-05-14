#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math
import time

METERS_TO_DBAR = 1.0 / 0.992 # From original: DBAR_TO_METERS = 0.992

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class FloatSimulatorNode(Node):
    def __init__(self):
        super().__init__('float_simulator_node')
        self.get_logger().info(f"Float Simulator (Source of Raw Sim Data) starting at {time.strftime('%H:%M:%S')}")

        self.declare_parameter('publish_rate', 10.0) # Hz
        self.declare_parameter('max_vertical_speed', 0.1) # Max ascent/descent speed (m/s)
        self.declare_parameter('depth_kp', 0.5) # Proportional gain for depth control
        self.declare_parameter('sim_target_vx', 0.1) # m/s, for simple horizontal motion
        self.declare_parameter('sim_target_vy', 0.05) # m/s, for simple horizontal motion
        self.declare_parameter('sim_seafloor_depth_m', 100.0) # meters

        self.x = 0.0
        self.y = 0.0
        self.current_depth_m = 0.0
        self.target_depth_m = 0.0 # This will be controlled by a subscription
        self.target_depth_received = False

        self.vx_mps = self.get_parameter('sim_target_vx').value # Simulated X velocity
        self.vy_mps = self.get_parameter('sim_target_vy').value # Simulated Y velocity
        self.vz_mps = 0.0 # Vertical velocity, controlled by depth PID
        self.altitude_agl_m = self.get_parameter('sim_seafloor_depth_m').value # Altitude Above Ground Level

        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rad = 0.0

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.world_frame = 'odom'
        self.robot_frame = 'base_link'

        self.last_time = self.get_clock().now()

        # Subscription to control the float's target depth
        self.target_depth_sub = self.create_subscription(Float32, 'target_depth', self.target_depth_callback, 10)

        # Publishers for simulated raw sensor data
        self.sim_pressure_pub = self.create_publisher(Float32, 'sim_pressure_raw', 10) # dbar
        self.sim_dvl_vx_pub = self.create_publisher(Float32, 'sim_dvl_raw_vx', 10)       # m/s
        self.sim_dvl_vy_pub = self.create_publisher(Float32, 'sim_dvl_raw_vy', 10)       # m/s
        self.sim_dvl_vz_pub = self.create_publisher(Float32, 'sim_dvl_raw_vz', 10)       # m/s
        self.sim_dvl_alt_pub = self.create_publisher(Float32, 'sim_dvl_raw_altitude', 10) # m

        # Publishers for ground truth from simulator
        self.ground_truth_depth_pub = self.create_publisher(Float32, 'sim_ground_truth_depth', 10)
        self.ground_truth_altitude_pub = self.create_publisher(Float32, 'sim_ground_truth_altitude', 10)
        self.ground_truth_seafloor_z_pub = self.create_publisher(Float32, 'sim_ground_truth_seafloor_z', 10)


        publish_period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(publish_period, self.update_state_and_publish)
        self.get_logger().info("Simulator initialized. Waiting for target depth command on /target_depth.")

    def target_depth_callback(self, msg):
        if msg.data >= 0:
             self.target_depth_m = msg.data
             if not self.target_depth_received:
                 self.get_logger().info(f"Received initial target depth command: {self.target_depth_m:.2f} m")
                 self.target_depth_received = True
             else:
                 self.get_logger().info(f"Received new target depth command: {self.target_depth_m:.2f} m")
        else:
            self.get_logger().warn(f"Received invalid target depth {msg.data:.2f} m. Target depth must be non-negative.")

    def update_state_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        # Simple depth controller
        if self.target_depth_received:
            depth_error = self.target_depth_m - self.current_depth_m
            kp = self.get_parameter('depth_kp').value
            max_vz = self.get_parameter('max_vertical_speed').value
            self.vz_mps = max(-max_vz, min(max_vz, kp * depth_error))
        else:
            self.vz_mps = 0.0

        # Update position
        self.current_depth_m += self.vz_mps * dt
        if self.current_depth_m < 0: # Cannot go above surface
            self.current_depth_m = 0
            if self.vz_mps < 0: self.vz_mps = 0 # Stop ascending if at surface

        self.x += self.vx_mps * dt
        self.y += self.vy_mps * dt

        # Update simulated altitude
        seafloor_depth_m = self.get_parameter('sim_seafloor_depth_m').value
        self.altitude_agl_m = seafloor_depth_m - self.current_depth_m
        if self.altitude_agl_m < 0: # Cannot go below seafloor
            self.altitude_agl_m = 0
            self.current_depth_m = seafloor_depth_m # Correct depth if "passed through" seafloor
            if self.vz_mps > 0: self.vz_mps = 0 # Stop descending if at seafloor

        # Publish ground truth TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.robot_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = -self.current_depth_m
        qx, qy, qz, qw = get_quaternion_from_euler(self.roll_rad, self.pitch_rad, self.yaw_rad)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Publish simulated raw sensor data
        self.sim_pressure_pub.publish(Float32(data=self.current_depth_m * METERS_TO_DBAR))
        self.sim_dvl_vx_pub.publish(Float32(data=self.vx_mps))
        self.sim_dvl_vy_pub.publish(Float32(data=self.vy_mps))
        self.sim_dvl_vz_pub.publish(Float32(data=self.vz_mps)) # DVL measures velocity of the float
        self.sim_dvl_alt_pub.publish(Float32(data=self.altitude_agl_m))

        # Publish ground truth values
        self.ground_truth_depth_pub.publish(Float32(data=self.current_depth_m))
        self.ground_truth_altitude_pub.publish(Float32(data=self.altitude_agl_m))
        self.ground_truth_seafloor_z_pub.publish(Float32(data=-seafloor_depth_m))


        self.get_logger().debug(f"Sim State: D={self.current_depth_m:.2f}m, Alt={self.altitude_agl_m:.2f}m, Vz={self.vz_mps:.3f}m/s, P_sim={self.current_depth_m * METERS_TO_DBAR:.2f}dbar")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FloatSimulatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception in FloatSimulatorNode: {e}")
        else:
            print(f"Exception before node initialization: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()