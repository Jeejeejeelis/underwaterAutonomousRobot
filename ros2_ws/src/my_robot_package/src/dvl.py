#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, String
import socket
import time
import json
import csv
from datetime import datetime, timezone
import math


class DVLNode(Node):
    def __init__(self):
        super().__init__('dvl_node')
        self.declare_parameter('simulate', True)
        self.declare_parameter('tcp_ip_dvl', "192.168.194.95")
        self.declare_parameter('tcp_port_dvl', 16171)
        self.declare_parameter('save_locally_dvl', True)
        self.declare_parameter('socket_read_timeout_dvl', 0.1)
        self.declare_parameter('hardware_data_process_interval', 0.1)
        self.declare_parameter('hardware_publish_interval', 1.0)

        self.simulate = self.get_parameter('simulate').get_parameter_value().bool_value
        self.tcp_ip = self.get_parameter('tcp_ip_dvl').value
        self.tcp_port = self.get_parameter('tcp_port_dvl').value
        self.save_locally = self.get_parameter('save_locally_dvl').value
        self.read_timeout = self.get_parameter('socket_read_timeout_dvl').value
        self.hw_process_interval = self.get_parameter('hardware_data_process_interval').value
        self.hw_publish_interval = self.get_parameter('hardware_publish_interval').value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publisher_vx = self.create_publisher(Float32, 'vx', 10)
        self.publisher_vy = self.create_publisher(Float32, 'vy', 10)
        self.publisher_vz = self.create_publisher(Float32, 'vz', 10)
        self.publisher_altitude = self.create_publisher(Float32, 'altitude', 10)
        self.command_sub = self.create_subscription(String, 'send_dvl_command', self.send_dvl_callback, 10)

        self.sock = None
        self.timer_hw_read = None
        self.timer_hw_publish = None
        self.read_buffer = b''

        self.sim_vx = None; self.sim_vy = None; self.sim_vz = None; self.sim_altitude = None

        self.sum_vx = 0.0; self.sum_vy = 0.0; self.sum_vz = 0.0; self.sum_altitude = 0.0
        self.measurement_count = 0
        self.last_publish_time = self.get_clock().now()

        self.csv_writer = None; self.csv_file = None
        self.sim_vx_sub = None; self.sim_vy_sub = None; self.sim_vz_sub = None; self.sim_alt_sub = None
        self.setup_mode()

    def parameters_callback(self, params):
        needs_reconfig = False
        for param in params:
            if param.name == 'simulate':
                new_simulate_val = param.get_parameter_value().bool_value
                if self.simulate != new_simulate_val:
                    self.simulate = new_simulate_val
                    needs_reconfig = True
            elif param.name == 'save_locally_dvl':
                new_save_val = param.get_parameter_value().bool_value
                if self.save_locally != new_save_val:
                    self.save_locally = new_save_val
                    self.setup_csv_writer()

        if needs_reconfig:
            self.get_logger().info("DVL parameters changed, reconfiguring node.")
            self.setup_mode()
        return SetParametersResult(successful=True)


    def cleanup_resources(self):
        if self.timer_hw_read: self.timer_hw_read.cancel(); self.timer_hw_read = None
        if self.timer_hw_publish: self.timer_hw_publish.cancel(); self.timer_hw_publish = None
        self.disconnect()
        if self.csv_file: self.csv_file.close(); self.csv_file = None; self.csv_writer = None
        if self.sim_vx_sub: self.destroy_subscription(self.sim_vx_sub); self.sim_vx_sub = None
        if self.sim_vy_sub: self.destroy_subscription(self.sim_vy_sub); self.sim_vy_sub = None
        if self.sim_vz_sub: self.destroy_subscription(self.sim_vz_sub); self.sim_vz_sub = None
        if self.sim_alt_sub: self.destroy_subscription(self.sim_alt_sub); self.sim_alt_sub = None
        self.sim_vx = None; self.sim_vy = None; self.sim_vz = None; self.sim_altitude = None


    def setup_mode(self):
        self.cleanup_resources()
        if self.simulate:
            self.get_logger().info("DVL Sim: Subscribing to simulated DVL data. Publishing on new data.")
            self.sim_vx_sub = self.create_subscription(Float32, 'sim_dvl_raw_vx', self.sim_vx_callback, 10)
            self.sim_vy_sub = self.create_subscription(Float32, 'sim_dvl_raw_vy', self.sim_vy_callback, 10)
            self.sim_vz_sub = self.create_subscription(Float32, 'sim_dvl_raw_vz', self.sim_vz_callback, 10)
            self.sim_alt_sub = self.create_subscription(Float32, 'sim_dvl_raw_altitude', self.sim_alt_callback, 10)
        else: # hardware mode
            self.get_logger().info(f"DVL Hardware mode: Connecting to {self.tcp_ip}:{self.tcp_port}")
            self.connect()
            if self.sock:
                 self.timer_hw_read = self.create_timer(self.hw_process_interval, self.read_and_accumulate_hardware_dvl)
                 self.timer_hw_publish = self.create_timer(self.hw_publish_interval, self.publish_averaged_hardware_data)
            else:
                 self.get_logger().warn("DVL connection failed. Hardware timers not started.")
        self.setup_csv_writer()

    def sim_vx_callback(self, msg): self.sim_vx = msg.data; self.try_publish_simulated_dvl_data()
    def sim_vy_callback(self, msg): self.sim_vy = msg.data; self.try_publish_simulated_dvl_data()
    def sim_vz_callback(self, msg): self.sim_vz = msg.data; self.try_publish_simulated_dvl_data()
    def sim_alt_callback(self, msg): self.sim_altitude = msg.data; self.try_publish_simulated_dvl_data()

    def try_publish_simulated_dvl_data(self):
        if None not in [self.sim_vx, self.sim_vy, self.sim_vz, self.sim_altitude]:
            self.publisher_vx.publish(Float32(data=self.sim_vx))
            self.publisher_vy.publish(Float32(data=self.sim_vy))
            self.publisher_vz.publish(Float32(data=self.sim_vz))
            self.publisher_altitude.publish(Float32(data=self.sim_altitude))
            self.get_logger().debug(f"Published Sim DVL: Vx={self.sim_vx:.3f}, Vy={self.sim_vy:.3f}, Vz={self.sim_vz:.3f}, Alt={self.sim_altitude:.2f}")

    def publish_averaged_hardware_data(self):
        if self.simulate: return

        now = self.get_clock().now()
        if self.measurement_count > 0:
            avg_vx = self.sum_vx / self.measurement_count
            avg_vy = self.sum_vy / self.measurement_count
            avg_vz = self.sum_vz / self.measurement_count
            avg_altitude = self.sum_altitude / self.measurement_count

            self.publisher_vx.publish(Float32(data=avg_vx))
            self.publisher_vy.publish(Float32(data=avg_vy))
            self.publisher_vz.publish(Float32(data=avg_vz))
            self.publisher_altitude.publish(Float32(data=avg_altitude))
            self.get_logger().info(f"Published Avg HW DVL: Vx={avg_vx:.3f}, Vy={avg_vy:.3f}, Vz={avg_vz:.3f}, Alt={avg_altitude:.2f} (N={self.measurement_count})")

            self.sum_vx = 0.0; self.sum_vy = 0.0; self.sum_vz = 0.0
            self.sum_altitude = 0.0; self.measurement_count = 0

        self.last_publish_time = now

    def read_and_accumulate_hardware_dvl(self):
        if self.simulate: return
        if not self.sock:
            self.get_logger().warn("No DVL connection, attempting to reconnect in read_and_accumulate...")
            self.connect()
            if not self.sock: self.get_logger().warn("DVL reconnect failed."); return

        try:
            data = self.sock.recv(4096)
            if len(data) == 0: self.disconnect(); return
            self.read_buffer += data
        except socket.timeout: return
        except socket.error as e: self.get_logger().error(f'DVL Socket error: {e}.'); self.disconnect(); return
        except Exception as e: self.get_logger().error(f'Unexpected DVL read error: {e}'); self.disconnect(); return

        while b'\n' in self.read_buffer:
            line, self.read_buffer = self.read_buffer.split(b'\n', 1)
            line_str = line.strip().decode('utf-8', errors='ignore')
            if not line_str: continue
            try:
                jsondata = json.loads(line_str)
                if self.csv_writer:
                    jsondata['timestamp_ros'] = self.get_clock().now().nanoseconds / 1e9
                    self.csv_writer.writerow(jsondata)
                if jsondata.get("velocity_valid", False):
                    vx = jsondata.get("vx", 0.0); vy = jsondata.get("vy", 0.0)
                    vz = jsondata.get("vz", 0.0); alt = jsondata.get("altitude", -1.0)
                    if abs(vx) < 10.0 and abs(vy) < 10.0 and abs(vz) < 5.0 and alt >= 0.0:
                        self.sum_vx += vx; self.sum_vy += vy; self.sum_vz += vz
                        self.sum_altitude += alt; self.measurement_count += 1
                    else: self.get_logger().warn(f"DVL reading sanity check fail: {vx},{vy},{vz},{alt}")
            except json.JSONDecodeError: self.get_logger().warn(f"JSON DVL decode error: {line_str[:100]}...")
            except Exception as e: self.get_logger().error(f"DVL JSON processing error: {e}")


    def setup_csv_writer(self):
        if self.csv_file: self.csv_file.close(); self.csv_file = None; self.csv_writer = None
        if self.save_locally and not self.simulate:
            try:
                filename = f"dvl_data_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}.csv"
                self.csv_file = open(filename, 'w', newline='')
                header = ['timestamp_ros', 'ts', 'time', 'vx', 'vy', 'vz', 'fom', 'altitude', 'velocity_valid', 'status',
                          'x_stddev', 'y_stddev', 'z_stddev', 'alt_stddev', 'rssi_bm1', 'rssi_bm2', 'rssi_bm3', 'rssi_bm4', 'gain_bm1', 'gain_bm2', 'gain_bm3', 'gain_bm4']
                self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=header, extrasaction='ignore')
                self.csv_writer.writeheader(); self.get_logger().info(f"Opened DVL data file: {filename}")
            except Exception as e: self.get_logger().error(f"Fail to open DVL CSV '{filename}': {e}"); self.csv_file=None; self.csv_writer=None

    def connect(self):
        if self.sock: self.disconnect()
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0); self.sock.connect((self.tcp_ip, self.tcp_port))
            self.sock.settimeout(self.read_timeout)
            self.get_logger().info(f'DVL Connected: {self.tcp_ip}:{self.tcp_port}'); self.read_buffer = b''
        except socket.timeout: self.get_logger().error(f'DVL Conn Timeout {self.tcp_ip}:{self.tcp_port}'); self.sock = None
        except Exception as e: self.get_logger().error(f'DVL Conn Fail {self.tcp_ip}:{self.tcp_port}: {e}'); self.sock = None

    def disconnect(self):
        if self.sock:
            try: self.sock.close(); self.get_logger().info("DVL socket closed.")
            except Exception as e: self.get_logger().warn(f"Error closing DVL socket: {e}")
            finally: self.sock = None

    def send_dvl_callback(self, msg):
         if self.simulate: self.get_logger().info("DVL sim mode, ignoring send_dvl_command."); return
         if not self.sock: self.get_logger().warn("DVL socket not connected for command."); return
         self.send_hardware_dvl_command(msg.data)

    def send_hardware_dvl_command(self, cmd_string):
        if not self.sock: self.get_logger().warn("No DVL socket for command."); return
        try:
            self.get_logger().info(f"Sending to DVL: {cmd_string}")
            self.sock.sendall(cmd_string.encode('utf-8') + b'\n')
        except socket.error as e: self.get_logger().error(f"DVL socket send error: {e}. Closing."); self.disconnect()
        except Exception as e: self.get_logger().error(f"Unexpected DVL send error: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down DVL node...")
        self.cleanup_resources()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DVLNode()
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    except Exception as e:
        if node: node.get_logger().fatal(f"DVL Node unhandled exception: {e}")
        else: print(f"DVL Node unhandled exception during creation: {e}")
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()