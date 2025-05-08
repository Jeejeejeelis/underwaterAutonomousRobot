#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType # For creating descriptors
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, String
import socket
import time
import os
import json
import csv
from datetime import datetime, timezone
import math
import traceback

DEFAULT_MAX_EXPECTED_VX = 10.0
DEFAULT_MAX_EXPECTED_VY = 10.0
DEFAULT_MAX_EXPECTED_VZ_DVL = 5.0
DEFAULT_MIN_EXPECTED_ALTITUDE = 0.0
DEFAULT_MAX_EXPECTED_ALTITUDE = 200.0

RECONNECT_ATTEMPT_DELAY = 5.0
class DVLNode(Node):
    def __init__(self):
        super().__init__('dvl_node')

        simulate_desc = ParameterDescriptor(description='Run in simulation mode.')
        tcp_ip_desc = ParameterDescriptor(description='TCP IP address of the DVL.')
        tcp_port_desc = ParameterDescriptor(description='TCP port of the DVL.')
        device_id_desc = ParameterDescriptor(description='Device ID (currently informational).')
        save_locally_desc = ParameterDescriptor(description='Save DVL data to a local CSV file in hardware mode.')
        read_timeout_desc = ParameterDescriptor(description='Socket read timeout in seconds.')
        timer_period_desc = ParameterDescriptor(description='How often to check socket/process data in hardware mode (seconds).')
        publish_interval_desc = ParameterDescriptor(description='How often to publish averaged DVL data (seconds).')
        max_vx_desc = ParameterDescriptor(description='Maximum expected DVL Vx for filtering (m/s).')
        max_vy_desc = ParameterDescriptor(description='Maximum expected DVL Vy for filtering (m/s).')
        max_vz_dvl_desc = ParameterDescriptor(description='Maximum expected DVL Vz (DVL frame) for filtering (m/s).')
        min_alt_desc = ParameterDescriptor(description='Minimum expected DVL altitude for filtering (m).')
        max_alt_desc = ParameterDescriptor(description='Maximum expected DVL altitude for filtering (m).')

        self.declare_parameter('simulate', True, simulate_desc)
        self.declare_parameter('tcp_ip', "192.168.194.95", tcp_ip_desc)
        self.declare_parameter('tcp_port', 16171, tcp_port_desc)
        self.declare_parameter('device_id', "SUB", device_id_desc)
        self.declare_parameter('save_locally', True, save_locally_desc)
        self.declare_parameter('read_timeout_s', 0.1, read_timeout_desc)
        self.declare_parameter('processing_timer_period_s', 0.1, timer_period_desc)
        self.declare_parameter('publish_interval_s', 1.0, publish_interval_desc)

        self.declare_parameter('filter_max_vx', DEFAULT_MAX_EXPECTED_VX, max_vx_desc)
        self.declare_parameter('filter_max_vy', DEFAULT_MAX_EXPECTED_VY, max_vy_desc)
        self.declare_parameter('filter_max_vz_dvl', DEFAULT_MAX_EXPECTED_VZ_DVL, max_vz_dvl_desc)
        self.declare_parameter('filter_min_altitude', DEFAULT_MIN_EXPECTED_ALTITUDE, min_alt_desc)
        self.declare_parameter('filter_max_altitude', DEFAULT_MAX_EXPECTED_ALTITUDE, max_alt_desc)

        self.simulate = self.get_parameter('simulate').get_parameter_value().bool_value
        self.tcp_ip = self.get_parameter('tcp_ip').get_parameter_value().string_value
        self.tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        self.device_id = self.get_parameter('device_id').get_parameter_value().string_value
        self.save_locally = self.get_parameter('save_locally').get_parameter_value().bool_value
        self.read_timeout = self.get_parameter('read_timeout_s').get_parameter_value().double_value
        self.processing_timer_period = self.get_parameter('processing_timer_period_s').get_parameter_value().double_value
        self.publish_interval = self.get_parameter('publish_interval_s').get_parameter_value().double_value
        
        self.filter_max_vx = self.get_parameter('filter_max_vx').get_parameter_value().double_value
        self.filter_max_vy = self.get_parameter('filter_max_vy').get_parameter_value().double_value
        self.filter_max_vz_dvl = self.get_parameter('filter_max_vz_dvl').get_parameter_value().double_value
        self.filter_min_altitude = self.get_parameter('filter_min_altitude').get_parameter_value().double_value
        self.filter_max_altitude = self.get_parameter('filter_max_altitude').get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publisher_vx = self.create_publisher(Float32, 'dvl/vx', 10)
        self.publisher_vy = self.create_publisher(Float32, 'dvl/vy', 10)
        self.publisher_vz = self.create_publisher(Float32, 'dvl/vz_dvl_frame', 10)
        self.publisher_altitude = self.create_publisher(Float32, 'dvl/altitude_m', 10)
        self.subscription = self.create_subscription(String, 'send_dvl_command', self.send_dvl_command_callback, 10)

        self.sock = None
        self.processing_timer = None
        self.read_buffer = b''
        self.last_connection_attempt_time = 0.0

        self.sum_vx = 0.0
        self.sum_vy = 0.0
        self.sum_vz_dvl = 0.0
        self.sum_altitude = 0.0
        self.measurement_count = 0
        self.last_publish_time = self.get_clock().now()

        self.csv_writer = None
        self.csv_file = None
        
        self.get_logger().info(f"DVL Node starting. Simulation mode: {self.simulate}")
        self.setup_mode()

    def parameters_callback(self, params):
        result = SetParametersResult(successful=True)
        should_setup_mode = False
        should_reset_processing_timer = False
        should_reset_csv = False

        for param in params:
            if param.name == 'simulate':
                if self.simulate != param.value:
                    self.simulate = param.value
                    self.get_logger().info(f"DVL 'simulate' changed to {self.simulate}")
                    should_setup_mode = True
            elif param.name == 'tcp_ip' and self.tcp_ip != param.value:
                self.tcp_ip = param.value
                should_setup_mode = True
            elif param.name == 'tcp_port' and self.tcp_port != param.value:
                self.tcp_port = param.value
                should_setup_mode = True
            elif param.name == 'device_id':
                self.device_id = param.value
            elif param.name == 'save_locally' and self.save_locally != param.value:
                self.save_locally = param.value
                should_reset_csv = True
            elif param.name == 'read_timeout_s' and self.read_timeout != param.value:
                self.read_timeout = param.value
                if self.sock:
                    try: self.sock.settimeout(self.read_timeout)
                    except Exception as e: self.get_logger().warn(f"DVL: Could not set new socket timeout: {e}")
            elif param.name == 'processing_timer_period_s' and self.processing_timer_period != param.value:
                self.processing_timer_period = param.value
                if not self.simulate: should_reset_processing_timer = True
            elif param.name == 'publish_interval_s' and self.publish_interval != param.value:
                self.publish_interval = param.value
                if self.simulate: should_reset_processing_timer = True

            elif param.name == 'filter_max_vx': self.filter_max_vx = param.value
            elif param.name == 'filter_max_vy': self.filter_max_vy = param.value
            elif param.name == 'filter_max_vz_dvl': self.filter_max_vz_dvl = param.value
            elif param.name == 'filter_min_altitude': self.filter_min_altitude = param.value
            elif param.name == 'filter_max_altitude': self.filter_max_altitude = param.value
        
        if should_setup_mode:
            self.get_logger().info("DVL parameters triggered full mode setup.")
            self.setup_mode()
        elif should_reset_processing_timer:
            self.get_logger().info("DVL timer parameters changed, resetting processing timer.")
            if self.processing_timer: self.processing_timer.cancel()
            if self.simulate:
                self.processing_timer = self.create_timer(self.publish_interval, self.publish_dummy_data)
            elif self.sock:
                self.processing_timer = self.create_timer(self.processing_timer_period, self.read_and_process_dvl_data)
        
        if should_reset_csv and not should_setup_mode:
            self.setup_csv_writer()
            
        return result

    def setup_mode(self):
        if self.processing_timer:
            self.processing_timer.cancel()
            self.processing_timer = None
        self.disconnect_dvl()

        if self.simulate:
            self.get_logger().info("DVL: Running in simulation mode, publishing dummy data.")
            self.processing_timer = self.create_timer(self.publish_interval, self.publish_dummy_data)
        else:
            self.get_logger().info(f"DVL: Attempting to connect to DVL at {self.tcp_ip}:{self.tcp_port}")
            self.connect_to_dvl()
            if self.sock:
                 self.processing_timer = self.create_timer(self.processing_timer_period, self.read_and_process_dvl_data) # Renamed
            else:
                 self.get_logger().warn("DVL: Connection failed. Will attempt to reconnect periodically via read_and_process_dvl_data.")
                 self.processing_timer = self.create_timer(RECONNECT_ATTEMPT_DELAY, self.read_and_process_dvl_data)


        self.setup_csv_writer()

    def setup_csv_writer(self):
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception as e:
                self.get_logger().warn(f"DVL: Error closing previous CSV file: {e}")
            self.csv_file = None
            self.csv_writer = None
            self.get_logger().info("DVL: Closed previous data file.")

        if self.save_locally and not self.simulate:
            try:
                log_dir = "dvl_logs"
                os.makedirs(log_dir, exist_ok=True)
                filename = os.path.join(log_dir, f"dvl_data_{self.device_id}_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}.csv")
                
                self.csv_file = open(filename, 'w', newline='')
                header = ['timestamp_ros_s', 'ts', 'time', 'vx', 'vy', 'vz', 'altitude', 'fom', 
                          'covariance', 'velocity_valid', 'status', 'beam_correlations', 'pressure', 'temperature'] 
                self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=header, extrasaction='ignore')
                self.csv_writer.writeheader()
                self.get_logger().info(f"DVL: Opened data file for writing: {filename}")
            except Exception as e:
                self.get_logger().error(f"DVL: Failed to open CSV file '{filename if 'filename' in locals() else 'unknown'}': {e}")
                self.csv_file = None
                self.csv_writer = None

    def publish_dummy_data(self):
        current_time_sec = time.time()
        dummy_vx = 0.5 + math.sin(current_time_sec * 0.1) * 0.5
        dummy_vy = 0.0 + math.cos(current_time_sec * 0.07) * 0.2
        dummy_vz_dvl = 0.05 + math.sin(current_time_sec * 0.15) * 0.05
        dummy_altitude = 50.0 + math.cos(current_time_sec * 0.05) * 10.0

        self.publisher_vx.publish(Float32(data=dummy_vx))
        self.publisher_vy.publish(Float32(data=dummy_vy))
        self.publisher_vz.publish(Float32(data=dummy_vz_dvl))
        self.publisher_altitude.publish(Float32(data=dummy_altitude))
        # self.get_logger().info(f"DVL (Sim): Published dummy Vx={dummy_vx:.2f}, Vy={dummy_vy:.2f}, Vz_dvl={dummy_vz_dvl:.2f}, Alt={dummy_altitude:.2f}")

    def connect_to_dvl(self):
        if self.sock:
             self.get_logger().warn("DVL: connect_to_dvl called but socket already exists. Closing first.")
             self.disconnect_dvl()
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(3.0)
            self.get_logger().info(f'DVL: Connecting to {self.tcp_ip}:{self.tcp_port}...')
            self.sock.connect((self.tcp_ip, self.tcp_port))
            self.sock.settimeout(self.read_timeout)
            self.get_logger().info(f'DVL: Successful connection to {self.tcp_ip}:{self.tcp_port}')
            self.read_buffer = b''
            self.last_connection_attempt_time = 0 
            if self.processing_timer and self.processing_timer.timer_period_ns != self.processing_timer_period * 1e9:
                self.get_logger().info("DVL: Connection successful, resetting processing timer to normal rate.")
                self.processing_timer.cancel()
                self.processing_timer = self.create_timer(self.processing_timer_period, self.read_and_process_dvl_data)

        except socket.timeout:
             self.get_logger().warn(f'DVL: Connection to {self.tcp_ip}:{self.tcp_port} timed out.')
             if self.sock: self.sock.close()
             self.sock = None
             self.last_connection_attempt_time = self.get_clock().now().nanoseconds / 1e9
        except Exception as e:
            self.get_logger().error(f'DVL: Connection failed to {self.tcp_ip}:{self.tcp_port}: {e}')
            if self.sock: self.sock.close()
            self.sock = None
            self.last_connection_attempt_time = self.get_clock().now().nanoseconds / 1e9

    def disconnect_dvl(self):
        if self.sock:
            try:
                self.sock.close()
                self.get_logger().info("DVL: Socket closed.")
            except Exception as e:
                 self.get_logger().warn(f"DVL: Error closing socket: {e}")
            finally:
                 self.sock = None

    def send_dvl_command_callback(self, msg):
         if self.simulate:
            self.get_logger().info("DVL: In simulation mode, ignoring send_dvl_command.")
            return
         if not self.sock:
             self.get_logger().warn("DVL: Socket not connected. Cannot send command.")
             return
         self.send_command_to_dvl(msg.data)

    def read_and_process_dvl_data(self):
        if self.simulate:
            self.get_logger().warn("DVL: read_and_process_dvl_data called in simulation mode. This shouldn't happen.")
            return

        if not self.sock:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (current_time - self.last_connection_attempt_time) > RECONNECT_ATTEMPT_DELAY:
                self.get_logger().warn("DVL: No connection, attempting to reconnect...")
                self.connect_to_dvl()
                if not self.sock:
                    self.get_logger().warn(f"DVL: Reconnect failed. Will try again in {RECONNECT_ATTEMPT_DELAY}s.")
                    if self.processing_timer and self.processing_timer.timer_period_ns != RECONNECT_ATTEMPT_DELAY * 1e9:
                         self.get_logger().info(f"DVL: Setting timer for reconnect attempts every {RECONNECT_ATTEMPT_DELAY}s.")
                         self.processing_timer.cancel()
                         self.processing_timer = self.create_timer(RECONNECT_ATTEMPT_DELAY, self.read_and_process_dvl_data)
            return

        try:
            data = self.sock.recv(4096)
            if len(data) == 0:
                self.get_logger().warn('DVL: Socket connection closed by DVL. Reconnecting...')
                self.disconnect_dvl()
                # self.connect_to_dvl()
                self.last_connection_attempt_time = self.get_clock().now().nanoseconds / 1e9
                return
            self.read_buffer += data
        except socket.timeout:
            pass # self.get_logger().debug("DVL: Socket read timed out (no new data).")
        except socket.error as e:
             self.get_logger().error(f'DVL: Socket error during read: {e}. Closing and attempting reconnect later.')
             self.disconnect_dvl()
             self.last_connection_attempt_time = self.get_clock().now().nanoseconds / 1e9
             return
        except Exception as e:
             self.get_logger().error(f'DVL: Unexpected error during read: {e}')
             self.disconnect_dvl()
             self.last_connection_attempt_time = self.get_clock().now().nanoseconds / 1e9
             return

        while b'\n' in self.read_buffer:
            line, self.read_buffer = self.read_buffer.split(b'\n', 1)
            line_str = line.strip().decode('utf-8', errors='ignore')
            if not line_str: continue

            try:
                jsondata = json.loads(line_str)

                if self.csv_writer and self.csv_file:
                    try:
                         jsondata['timestamp_ros_s'] = self.get_clock().now().nanoseconds / 1e9
                         self.csv_writer.writerow(jsondata)
                         # self.csv_file.flush() # Consider flushing less frequently
                    except Exception as e:
                         self.get_logger().warn(f"DVL: Failed to write data to CSV: {e}")

                if jsondata.get("velocity_valid", True):
                    vx = jsondata.get("vx", 0.0)
                    vy = jsondata.get("vy", 0.0)
                    vz_dvl = jsondata.get("vz", 0.0)
                    alt = jsondata.get("altitude", -1.0)

                    if (abs(vx) < self.filter_max_vx and
                        abs(vy) < self.filter_max_vy and
                        abs(vz_dvl) < self.filter_max_vz_dvl and
                        alt >= self.filter_min_altitude and
                        alt < self.filter_max_altitude):
                        self.sum_vx += vx
                        self.sum_vy += vy
                        self.sum_vz_dvl += vz_dvl
                        self.sum_altitude += alt
                        self.measurement_count += 1
                    else:
                         self.get_logger().warn(f"DVL: Ignoring out-of-bounds reading: vx={vx}, vy={vy}, vz_dvl={vz_dvl}, alt={alt}")
                else:
                    self.get_logger().info(f"DVL: Data marked as invalid by DVL: {line_str[:100]}")

            except json.JSONDecodeError:
                 self.get_logger().warn(f"DVL: Could not decode JSON: {line_str[:100]}...")
            except Exception as e:
                 self.get_logger().error(f"DVL: Error processing JSON data: {e} - Data: {line_str[:100]}")

        now_ros_time = self.get_clock().now()
        if (now_ros_time - self.last_publish_time).nanoseconds / 1e9 >= self.publish_interval:
            if self.measurement_count > 0:
                avg_vx = self.sum_vx / self.measurement_count
                avg_vy = self.sum_vy / self.measurement_count
                avg_vz_dvl = self.sum_vz_dvl / self.measurement_count
                avg_altitude = self.sum_altitude / self.measurement_count

                self.publisher_vx.publish(Float32(data=avg_vx))
                self.publisher_vy.publish(Float32(data=avg_vy))
                self.publisher_vz.publish(Float32(data=avg_vz_dvl))
                self.publisher_altitude.publish(Float32(data=avg_altitude))
                # self.get_logger().info(f"DVL: Published Avg Vx={avg_vx:.3f}, Vy={avg_vy:.3f}, Vz_dvl={avg_vz_dvl:.3f}, Alt={avg_altitude:.2f} (N={self.measurement_count})")

                self.sum_vx, self.sum_vy, self.sum_vz_dvl, self.sum_altitude = 0.0, 0.0, 0.0, 0.0
                self.measurement_count = 0
            # else:
                # self.get_logger().debug(f"DVL: No new valid DVL measurements to publish in the last {self.publish_interval}s")
            self.last_publish_time = now_ros_time

    def send_command_to_dvl(self, cmd_string):
        if not self.sock:
            self.get_logger().warn("DVL: Cannot send command, socket not connected.")
            return
        try:
            self.get_logger().info(f"DVL: Sending command: {cmd_string}")
            self.sock.sendall(cmd_string.encode('utf-8') + b'\n')
        except socket.error as e:
            self.get_logger().error(f"DVL: Socket error during send: {e}. Closing connection.")
            self.disconnect_dvl()
            self.last_connection_attempt_time = self.get_clock().now().nanoseconds / 1e9
        except Exception as e:
             self.get_logger().error(f"DVL: Unexpected error during send: {e}")

    def destroy_node(self):
        self.get_logger().info("DVL Node shutting down...")
        if self.processing_timer:
            self.processing_timer.cancel()
        self.disconnect_dvl()
        if self.csv_file:
            try:
                self.csv_file.close()
                self.get_logger().info("DVL: Closed data file.")
            except Exception as e:
                self.get_logger().warn(f"DVL: Error closing CSV file on shutdown: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DVLNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("DVL: Keyboard interrupt, shutting down.")
    except Exception as e:
        if node:
            exc_info_str = traceback.format_exc()
            node.get_logger().fatal(f"DVL: Unhandled exception: {e}\n{exc_info_str}")
        else:
            print(f"DVL: Unhandled exception during node creation: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("DVL Node terminated.")

if __name__ == '__main__':
    main()