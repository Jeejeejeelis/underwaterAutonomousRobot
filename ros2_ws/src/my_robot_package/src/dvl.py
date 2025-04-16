#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, String
import socket
import time
import os
import logging # Use ROS logging instead
import json
import csv
from datetime import datetime, timezone
import math

class DVLNode(Node):
    def __init__(self):
        super().__init__('dvl_node') # Corrected node name

        self.declare_parameter('simulate', True)
        self.declare_parameter('tcp_ip', "192.168.194.95")
        self.declare_parameter('tcp_port', 16171)
        self.declare_parameter('device_id', "SUB")
        self.declare_parameter('save_locally', True)
        self.declare_parameter('read_timeout', 0.1) # Socket read timeout (seconds)
        self.declare_parameter('timer_period', 0.1) # How often to check socket/process data
        self.declare_parameter('publish_interval', 1.0) # How often to publish averaged data

        self.simulate = self.get_parameter('simulate').get_parameter_value().bool_value
        self.tcp_ip = self.get_parameter('tcp_ip').get_parameter_value().string_value
        self.tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        self.device_id = self.get_parameter('device_id').get_parameter_value().string_value
        self.save_locally = self.get_parameter('save_locally').get_parameter_value().bool_value
        self.read_timeout = self.get_parameter('read_timeout').get_parameter_value().double_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.publish_interval = self.get_parameter('publish_interval').get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publisher_vx = self.create_publisher(Float32, 'vx', 10)
        self.publisher_vy = self.create_publisher(Float32, 'vy', 10)
        self.publisher_vz = self.create_publisher(Float32, 'vz', 10)
        self.publisher_altitude = self.create_publisher(Float32, 'altitude', 10)
        self.subscription = self.create_subscription(String, 'send_dvl_command', self.send_dvl_callback, 10)

        self.sock = None
        self.timer = None
        self.read_buffer = b''

        self.sum_vx = 0.0
        self.sum_vy = 0.0
        self.sum_vz = 0.0
        self.sum_altitude = 0.0
        self.measurement_count = 0
        self.last_publish_time = self.get_clock().now()

        self.csv_writer = None
        self.csv_file = None

        self.setup_mode()

    def parameters_callback(self, params):
        needs_reconnect = False
        needs_timer_reset = False
        needs_csv_reset = False

        for param in params:
            if param.name == 'simulate':
                if self.simulate != param.value:
                    self.simulate = param.value
                    needs_reconnect = True # Need to re-setup mode
            elif param.name == 'tcp_ip':
                if self.tcp_ip != param.value:
                    self.tcp_ip = param.value
                    needs_reconnect = True
            elif param.name == 'tcp_port':
                 if self.tcp_port != param.value:
                    self.tcp_port = param.value
                    needs_reconnect = True
            elif param.name == 'device_id':
                 self.device_id = param.value
            elif param.name == 'save_locally':
                 if self.save_locally != param.value:
                    self.save_locally = param.value
                    needs_csv_reset = True
            elif param.name == 'read_timeout':
                 if self.read_timeout != param.value:
                    self.read_timeout = param.value
                    if self.sock:
                        try:
                            self.sock.settimeout(self.read_timeout)
                        except Exception as e:
                             self.get_logger().warn(f"Could not set socket timeout: {e}")
            elif param.name == 'timer_period':
                 if self.timer_period != param.value:
                     self.timer_period = param.value
                     needs_timer_reset = True
            elif param.name == 'publish_interval':
                 self.publish_interval = param.value

        if needs_reconnect:
             self.get_logger().info("DVL parameters changed, re-establishing connection/mode.")
             self.setup_mode()
        elif needs_timer_reset:
             self.get_logger().info("DVL timer period changed, resetting timer.")
             if self.timer:
                 self.timer.cancel()
             if self.simulate:
                 self.timer = self.create_timer(self.publish_interval, self.publish_dummy_data) # Dummy data published at publish_interval
             elif self.sock:
                 self.timer = self.create_timer(self.timer_period, self.read_and_process_dvl)


        if needs_csv_reset:
             self.setup_csv_writer()

        return SetParametersResult(successful=True)

    def setup_mode(self):
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.disconnect()

        if self.simulate:
            self.get_logger().info("Running DVL in simulation mode, publishing dummy data.")
            self.timer = self.create_timer(self.publish_interval, self.publish_dummy_data)
        else:
            self.get_logger().info(f"Attempting to connect to DVL at {self.tcp_ip}:{self.tcp_port}")
            self.connect()
            if self.sock:
                 self.timer = self.create_timer(self.timer_period, self.read_and_process_dvl)
            else:
                 self.get_logger().error("Connection failed. Will not start DVL reading timer.")

        self.setup_csv_writer() # Setup CSV based on current save_locally state

    def setup_csv_writer(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.get_logger().info("Closed previous DVL data file.")

        if self.save_locally and not self.simulate:
            try:
                filename = f"dvl_data_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}.csv"
                self.csv_file = open(filename, 'w', newline='')
                header = ['timestamp_ros', 'ts', 'time', 'vx', 'vy', 'vz', 'altitude', 'velocity_valid', 'status', 'fom'] # Example header
                self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=header, extrasaction='ignore') # Ignore extra fields
                self.csv_writer.writeheader()
                self.get_logger().info(f"Opened DVL data file for writing: {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to open CSV file '{filename}': {e}")
                self.csv_file = None
                self.csv_writer = None


    def publish_dummy_data(self):
        dummy_vx = 1.0 + math.sin(time.time() * 0.2) * 0.5
        dummy_vy = 0.5 + math.cos(time.time() * 0.2) * 0.5
        dummy_vz = 0.1 + math.sin(time.time() * 0.3) * 0.1
        dummy_altitude = 10.0 + math.cos(time.time() * 0.1) * 2.0
        # self.get_logger().info("Publishing dummy DVL data")
        self.publisher_vx.publish(Float32(data=dummy_vx))
        self.publisher_vy.publish(Float32(data=dummy_vy))
        self.publisher_vz.publish(Float32(data=dummy_vz))
        self.publisher_altitude.publish(Float32(data=dummy_altitude))

    def connect(self):
        if self.sock:
             self.disconnect()
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.tcp_ip, self.tcp_port))
            self.sock.settimeout(self.read_timeout)
            self.get_logger().info(f'Successful Connection to DVL at {self.tcp_ip}:{self.tcp_port}')
            self.read_buffer = b''
        except socket.timeout:
             self.get_logger().error(f'Connection to DVL {self.tcp_ip}:{self.tcp_port} timed out.')
             self.sock = None
        except Exception as e:
            self.get_logger().error(f'Connection Failed to DVL {self.tcp_ip}:{self.tcp_port}: {e}')
            self.sock = None

    def disconnect(self):
        """Closes the TCP socket."""
        if self.sock:
            try:
                self.sock.close()
                self.get_logger().info("DVL socket closed.")
            except Exception as e:
                 self.get_logger().warn(f"Error closing DVL socket: {e}")
            finally:
                 self.sock = None


    def send_dvl_callback(self, msg):
         if self.simulate:
            self.get_logger().info("In simulation mode, ignoring send_dvl command.")
            return
         if not self.sock:
             self.get_logger().warn("DVL socket not connected. Cannot send command.")
             return

         cmd_string = msg.data
         self.send_dvl(cmd_string)

    def read_and_process_dvl(self):
        if not self.sock:
            self.get_logger().warn("No DVL connection, attempting to reconnect...")
            self.connect()
            if not self.sock:
                 self.get_logger().warn("Reconnect failed.")
                 return
            else:
                 pass

        try:
            data = self.sock.recv(4096) # Read up to 4k bytes
            if len(data) == 0:
                self.get_logger().warn('DVL socket connection closed by remote host. Reconnecting...')
                self.disconnect()
                # self.connect()
                return
            self.read_buffer += data
            # self.get_logger().debug(f"Read {len(data)} bytes. Buffer size: {len(self.read_buffer)}")

        except socket.timeout:
            # self.get_logger().debug("Socket read timed out (no new data).")
            pass
        except socket.error as e:
             self.get_logger().error(f'Socket error during read: {e}. Attempting reconnect.')
             self.disconnect()
             # self.connect()
             return
        except Exception as e:
             self.get_logger().error(f'Unexpected error during DVL read: {e}')
             self.disconnect()
             # self.connect()
             return

        while b'\n' in self.read_buffer:
            line, self.read_buffer = self.read_buffer.split(b'\n', 1)
            line_str = line.strip().decode('utf-8', errors='ignore')
            # self.get_logger().debug(f"Processing line: {line_str}")

            if not line_str:
                continue

            try:
                jsondata = json.loads(line_str)

                if self.csv_writer:
                    try:
                         # Add ROS timestamp for reference
                         jsondata['timestamp_ros'] = self.get_clock().now().nanoseconds / 1e9
                         self.csv_writer.writerow(jsondata)
                         # Maybe flush occasionally? 
                         # self.csv_file.flush()
                    except Exception as e:
                         self.get_logger().warn(f"Failed to write DVL data to CSV: {e}")

                if jsondata.get("velocity_valid", False): # Use .get for safety
                    vx = jsondata.get("vx", 0.0)
                    vy = jsondata.get("vy", 0.0)
                    vz = jsondata.get("vz", 0.0)
                    alt = jsondata.get("altitude", 0.0)

                    if abs(vx) < 50 and abs(vy) < 50 and abs(vz) < 20 and alt >= 0: # Adjust limits as needed
                        self.sum_vx += vx
                        self.sum_vy += vy
                        self.sum_vz += vz
                        self.sum_altitude += alt
                        self.measurement_count += 1
                        # self.get_logger().debug(f"Accumulated valid measurement. Count: {self.measurement_count}")
                    else:
                         self.get_logger().warn(f"Ignoring potentially invalid DVL reading: vx={vx}, vy={vy}, vz={vz}, alt={alt}")

            except json.JSONDecodeError:
                 self.get_logger().warn(f"Could not decode JSON from line: {line_str[:100]}...") # Log truncated line
            except Exception as e:
                 self.get_logger().error(f"Error processing DVL JSON data: {e}")

        now = self.get_clock().now()
        if (now - self.last_publish_time).nanoseconds / 1e9 >= self.publish_interval:
            if self.measurement_count > 0:
                avg_vx = self.sum_vx / self.measurement_count
                avg_vy = self.sum_vy / self.measurement_count
                avg_vz = self.sum_vz / self.measurement_count
                avg_altitude = self.sum_altitude / self.measurement_count

                self.publisher_vx.publish(Float32(data=avg_vx))
                self.publisher_vy.publish(Float32(data=avg_vy))
                self.publisher_vz.publish(Float32(data=avg_vz))
                self.publisher_altitude.publish(Float32(data=avg_altitude))
                # self.get_logger().info(f"Published Avg DVL: Vx={avg_vx:.3f}, Vy={avg_vy:.3f}, Vz={avg_vz:.3f}, Alt={avg_altitude:.2f} (N={self.measurement_count})")

                self.sum_vx = 0.0
                self.sum_vy = 0.0
                self.sum_vz = 0.0
                self.sum_altitude = 0.0
                self.measurement_count = 0
            else:
                self.get_logger().debug(f"No new DVL measurements to publish in the last {self.publish_interval}s")
            self.last_publish_time = now # Update time even if nothing was published

    def send_dvl(self, cmd_string):
        if not self.sock:
            self.get_logger().warn("Cannot send command, DVL socket not connected.")
            return
        try:
            self.get_logger().info(f"Sending command to DVL: {cmd_string}")
            self.sock.sendall(cmd_string.encode('utf-8') + b'\n') # Assume newline terminated
            # response = self.sock.recv(1024)
            # self.get_logger().info(f"Immediate response from DVL: {response.decode(errors='ignore')}")
        except socket.error as e:
            self.get_logger().error(f"Socket error during send: {e}. Closing connection.")
            self.disconnect()
        except Exception as e:
             self.get_logger().error(f"Unexpected error during DVL send: {e}")

    def destroy_node(self):
        """Override destroy_node to ensure resources are cleaned up."""
        self.get_logger().info("Shutting down DVL node...")
        if self.timer:
            self.timer.cancel()
        self.disconnect()
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info("Closed DVL data file.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DVLNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception: {e}")
        else:
            print(f"Unhandled exception during node creation: {e}")
    finally:
        # Node cleanup is now handled in node.destroy_node()
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()