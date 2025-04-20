#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math
import time

DBAR_TO_METERS = 0.992
METERS_TO_DBAR = 1.0 / DBAR_TO_METERS

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class FloatSimulatorNode(Node):
    def __init__(self):
        super().__init__('float_simulator_node')
        self.get_logger().info(f"Float Simulator starting at {time.strftime('%H:%M:%S')}")

        self.declare_parameter('publish_rate', 10.0) # Hz
        self.declare_parameter('max_vertical_speed', 0.1) # Max ascent/descent speed (m/s)
        self.declare_parameter('depth_kp', 0.5) # Proportional gain for depth control

        self.x = 0.0
        self.y = 0.0
        self.current_depth = 0.0
        self.target_depth = 0.0
        self.target_depth_received = False

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        # self.current_pressure_dbar = None
        self.current_altitude = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.world_frame = 'odom'
        self.robot_frame = 'base_link'

        self.last_time = self.get_clock().now()

        self.vx_sub = self.create_subscription(Float32, 'vx', self.vx_callback, 10)
        self.vy_sub = self.create_subscription(Float32, 'vy', self.vy_callback, 10)
        # self.vz_sub = self.create_subscription(Float32, 'vz', self.vz_callback, 10)
        self.alt_sub = self.create_subscription(Float32, 'altitude', self.altitude_callback, 10)
        self.pressure_sub = self.create_subscription(Float32, 'pressure', self.pressure_callback, 10)
        self.target_depth_sub = self.create_subscription(Float32, 'target_depth', self.target_depth_callback, 10)

        self.depth_pub = self.create_publisher(Float32, 'calculated_depth', 10) # Renaming might be good later
        self.seafloor_depth_pub = self.create_publisher(Float32, 'seafloor_depth_below_surface', 10)
        self.float_z_pub = self.create_publisher(Float32, 'float_z_position', 10)
        self.seafloor_z_pub = self.create_publisher(Float32, 'seafloor_z_position', 10)
        self.sim_pressure_pub = self.create_publisher(Float32, 'simulated_pressure', 10)

        publish_period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(publish_period, self.update_pose_and_publish)
        self.get_logger().info(f"Simulator initialized. Waiting for target depth command on /target_depth.")

    def vx_callback(self, msg):
        self.current_vx = msg.data

    def vy_callback(self, msg):
        self.current_vy = msg.data

    def altitude_callback(self, msg):
        self.current_altitude = msg.data

    def pressure_callback(self, msg):
        # self.get_logger().debug(f"Received pressure {msg.data:.2f} | Internal depth {self.current_depth:.2f}")
        pass

    def target_depth_callback(self, msg):
        if msg.data >= 0:
             self.target_depth = msg.data
             if not self.target_depth_received:
                 self.get_logger().info(f"Received initial target depth command: {self.target_depth:.2f} m")
                 self.target_depth_received = True
             else:
                 self.get_logger().info(f"Received new target depth command: {self.target_depth:.2f} m")
        else:
            self.get_logger().warn(f"Received invalid target depth {msg.data:.2f} m. Target depth must be non-negative.")

    def update_pose_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: # Avoid issues if time jumps backwards or dt is zero
            return
        self.last_time = now

        if self.target_depth_received: # Only control if a target has been set
            depth_error = self.target_depth - self.current_depth
            kp = self.get_parameter('depth_kp').value
            max_vz = self.get_parameter('max_vertical_speed').value

            vz_command = kp * depth_error

            self.current_vz = max(-max_vz, min(max_vz, vz_command))

            self.current_depth += self.current_vz * dt

            if self.current_depth < 0:
                self.current_depth = 0
                self.current_vz = 0
        else:
            self.current_vz = 0.0

        self.x += self.current_vx * dt
        self.y += self.current_vy * dt

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.robot_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = -self.current_depth # Use internal depth state
        qx, qy, qz, qw = get_quaternion_from_euler(self.roll, self.pitch, self.yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        depth_msg = Float32()
        depth_msg.data = self.current_depth
        self.depth_pub.publish(depth_msg) # Topic now reflects internal state

        float_z_msg = Float32()
        float_z_msg.data = -self.current_depth
        self.float_z_pub.publish(float_z_msg)

        sim_pressure_msg = Float32()
        sim_pressure_msg.data = self.current_depth * METERS_TO_DBAR # Convert depth back to pressure
        self.sim_pressure_pub.publish(sim_pressure_msg)

        if self.current_altitude is not None:
            seafloor_depth_below_surface = self.current_depth + self.current_altitude
            seafloor_depth_msg = Float32()
            seafloor_depth_msg.data = seafloor_depth_below_surface
            self.seafloor_depth_pub.publish(seafloor_depth_msg)
            seafloor_z_msg = Float32()
            seafloor_z_msg.data = -seafloor_depth_below_surface
            self.seafloor_z_pub.publish(seafloor_z_msg)

        # self.get_logger().debug(f"Tgt:{self.target_depth:.1f} Depth:{self.current_depth:.2f} Vz:{self.current_vz:.3f} SimPrs:{sim_pressure_msg.data:.2f}")

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