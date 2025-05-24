#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32
import socket
import time
import json
import csv
from datetime import datetime, timezone

class CTDNode(Node):
    def __init__(self):
        super().__init__('ctd_node')
        self.declare_parameter('simulate', True)
        self.declare_parameter('tcp_ip_ctd', "192.168.194.48")
        self.declare_parameter('tcp_port_ctd', 10001)
        self.declare_parameter('save_locally_ctd', True)
        self.declare_parameter('socket_read_timeout_ctd', 0.1)
        self.declare_parameter('hardware_data_process_interval', 0.1)
        self.declare_parameter('hardware_publish_interval', 1.0)

        self.simulate = self.get_parameter('simulate').get_parameter_value().bool_value
        self.tcp_ip = self.get_parameter('tcp_ip_ctd').get_parameter_value().string_value
        self.tcp_port = self.get_parameter('tcp_port_ctd').get_parameter_value().integer_value
        self.save_locally = self.get_parameter('save_locally_ctd').get_parameter_value().bool_value
        self.read_timeout = self.get_parameter('socket_read_timeout_ctd').get_parameter_value().double_value
        self.hw_process_interval = self.get_parameter('hardware_data_process_interval').get_parameter_value().double_value
        self.hw_publish_interval = self.get_parameter('hardware_publish_interval').get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publisher_pressure = self.create_publisher(Float32, 'pressure', 10)
        self.publisher_temperature = self.create_publisher(Float32, 'temperature', 10)
        self.publisher_conductivity = self.create_publisher(Float32, 'conductivity', 10)

        self.sock = None
        self.timer_hw_read = None
        self.timer_hw_publish = None
        self.read_buffer = b''

        self.sim_pressure = None; self.sim_temperature = None; self.sim_conductivity = None

        self.sum_pressure = 0.0; self.sum_temperature = 0.0; self.sum_conductivity = 0.0
        self.measurement_count = 0
        self.last_publish_time = self.get_clock().now()

        self.csv_writer = None; self.csv_file = None
        self.sim_pressure_sub = None; self.sim_temp_sub = None; self.sim_cond_sub = None
        self.setup_mode()
        self.get_logger().info("CTDNode initialized.")


    def parameters_callback(self, params):
        needs_reconfig = False
        for param in params:
            if param.name == 'simulate':
                new_simulate_val = param.get_parameter_value().bool_value
                if self.simulate != new_simulate_val:
                    self.simulate = new_simulate_val
                    needs_reconfig = True
            elif param.name == 'save_locally_ctd':
                new_save_val = param.get_parameter_value().bool_value
                if self.save_locally != new_save_val:
                    self.save_locally = new_save_val
                    self.setup_csv_writer()
            elif param.name == 'tcp_ip_ctd':
                new_val = param.get_parameter_value().string_value
                if self.tcp_ip != new_val: self.tcp_ip = new_val; needs_reconfig = True
            elif param.name == 'tcp_port_ctd':
                new_val = param.get_parameter_value().integer_value
                if self.tcp_port != new_val: self.tcp_port = new_val; needs_reconfig = True
            elif param.name == 'socket_read_timeout_ctd':
                new_val = param.get_parameter_value().double_value
                if self.read_timeout != new_val: self.read_timeout = new_val;
            elif param.name == 'hardware_data_process_interval':
                new_val = param.get_parameter_value().double_value
                if self.hw_process_interval != new_val: self.hw_process_interval = new_val; needs_reconfig = True
            elif param.name == 'hardware_publish_interval':
                new_val = param.get_parameter_value().double_value
                if self.hw_publish_interval != new_val: self.hw_publish_interval = new_val; needs_reconfig = True

        if needs_reconfig:
            self.get_logger().info("CTD parameters changed, reconfiguring node.")
            self.setup_mode()
        return SetParametersResult(successful=True)

    def cleanup_resources(self):
        if self.timer_hw_read: self.timer_hw_read.cancel(); self.timer_hw_read = None
        if self.timer_hw_publish: self.timer_hw_publish.cancel(); self.timer_hw_publish = None
        self.disconnect()
        if self.csv_file: self.csv_file.close(); self.csv_file = None; self.csv_writer = None
        if self.sim_pressure_sub: self.destroy_subscription(self.sim_pressure_sub); self.sim_pressure_sub = None
        if self.sim_temp_sub: self.destroy_subscription(self.sim_temp_sub); self.sim_temp_sub = None
        if self.sim_cond_sub: self.destroy_subscription(self.sim_cond_sub); self.sim_cond_sub = None
        self.sim_pressure = None; self.sim_temperature = None; self.sim_conductivity = None

    def setup_mode(self):
        self.cleanup_resources()
        if self.simulate:
            self.get_logger().info("CTD Sim: Subscribing to 'sim_pressure_raw'.")
            self.sim_pressure_sub = self.create_subscription(Float32, 'sim_pressure_raw', self.sim_pressure_callback, 10)
        else: 
            self.get_logger().info(f"CTD Hardware mode: Connecting to {self.tcp_ip}:{self.tcp_port}")
            self.connect()
            if self.sock:
                 self.timer_hw_read = self.create_timer(self.hw_process_interval, self.read_and_accumulate_hardware_ctd)
                 self.timer_hw_publish = self.create_timer(self.hw_publish_interval, self.publish_averaged_hardware_data)
            else:
                 self.get_logger().warn("CTD connection failed. Hardware timers not started.")
        self.setup_csv_writer()

    def sim_pressure_callback(self, msg):
        # Corrected Indentation and original logic preserved
        self.get_logger().info(f"CTD (Sim): Received sim_pressure_raw: {msg.data:.2f}") 
        self.sim_pressure = msg.data
        self.try_publish_simulated_ctd_data()

    def try_publish_simulated_ctd_data(self):
        if self.sim_pressure is not None:
            self.get_logger().info(f"CTD (Sim): Publishing /pressure: {float(self.sim_pressure):.2f}")
            self.publisher_pressure.publish(Float32(data=float(self.sim_pressure)))
            self.publisher_temperature.publish(Float32(data=float(self.sim_temperature if self.sim_temperature is not None else 0.0)))
            self.publisher_conductivity.publish(Float32(data=float(self.sim_conductivity if self.sim_conductivity is not None else 0.0)))
            # self.get_logger().debug(f"Published Sim CTD: Pressure={self.sim_pressure:.3f}") # Original debug log

    def publish_averaged_hardware_data(self):
        if self.simulate: return
        if self.measurement_count > 0:
            avg_pressure = self.sum_pressure / self.measurement_count
            avg_temperature = self.sum_temperature / self.measurement_count
            avg_conductivity = self.sum_conductivity / self.measurement_count

            self.publisher_pressure.publish(Float32(data=float(avg_pressure)))
            self.publisher_temperature.publish(Float32(data=float(avg_temperature)))
            self.publisher_conductivity.publish(Float32(data=float(avg_conductivity)))
            self.get_logger().info(f"Published Avg HW CTD: P={avg_pressure:.2f}dbar, T={avg_temperature:.2f}C, C={avg_conductivity:.2f}mS/cm (N={self.measurement_count})")

            self.sum_pressure = 0.0; self.sum_temperature = 0.0; self.sum_conductivity = 0.0
            self.measurement_count = 0
        self.last_publish_time = self.get_clock().now()

    def read_and_accumulate_hardware_ctd(self):
        if self.simulate: return
        if not self.sock:
            self.get_logger().warn("No CTD connection, attempting to reconnect...")
            self.connect()
            if not self.sock: self.get_logger().warn("CTD reconnect failed."); return
        try:
            data = self.sock.recv(1024) 
            if len(data) == 0: self.disconnect(); return
            self.read_buffer += data
        except socket.timeout: return 
        except socket.error as e: self.get_logger().error(f'CTD Socket error: {e}.'); self.disconnect(); return
        except Exception as e: self.get_logger().error(f'Unexpected CTD read error: {e}'); self.disconnect(); return

        while b'\n' in self.read_buffer:
            line, self.read_buffer = self.read_buffer.split(b'\n', 1)
            line_str = line.strip().decode('utf-8', errors='ignore')
            if not line_str: continue
            self.get_logger().debug(f"CTD Raw Line: {line_str}")
            try:
                parts = line_str.split(',')
                data_dict = {}
                for part in parts:
                    key_value = part.split('=')
                    if len(key_value) == 2:
                        data_dict[key_value[0].strip()] = float(key_value[1].strip())

                pressure = data_dict.get('P')
                temperature = data_dict.get('T')
                conductivity = data_dict.get('C')

                if pressure is not None and temperature is not None and conductivity is not None:
                    if self.csv_writer:
                        row_data = {'timestamp_ros': self.get_clock().now().nanoseconds / 1e9,
                                    'pressure_dbar': pressure, 'temperature_c': temperature, 'conductivity_ms_cm': conductivity}
                        self.csv_writer.writerow(row_data)

                    self.sum_pressure += pressure
                    self.sum_temperature += temperature
                    self.sum_conductivity += conductivity
                    self.measurement_count += 1
                else:
                    self.get_logger().warn(f"Could not parse all CTD values from line: {line_str}")

            except ValueError as e: self.get_logger().warn(f"CTD ValueError (likely float conversion): {e} from line: {line_str}")
            except Exception as e: self.get_logger().error(f"CTD data processing error: {e} from line: {line_str}")

    def setup_csv_writer(self):
        if self.csv_file: self.csv_file.close(); self.csv_file = None; self.csv_writer = None
        if self.save_locally and not self.simulate:
            try:
                filename = f"ctd_data_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}.csv"
                self.csv_file = open(filename, 'w', newline='')
                header = ['timestamp_ros', 'pressure_dbar', 'temperature_c', 'conductivity_ms_cm']
                self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=header, extrasaction='ignore')
                self.csv_writer.writeheader(); self.get_logger().info(f"Opened CTD data file: {filename}")
            except Exception as e: self.get_logger().error(f"Fail to open CTD CSV '{filename}': {e}"); self.csv_file=None; self.csv_writer=None

    def connect(self):
        if self.sock: self.disconnect()
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.tcp_ip, self.tcp_port))
            self.sock.settimeout(self.read_timeout)
            self.get_logger().info(f'CTD Connected: {self.tcp_ip}:{self.tcp_port}')
            self.read_buffer = b''
        except socket.timeout: self.get_logger().error(f'CTD Conn Timeout {self.tcp_ip}:{self.tcp_port}'); self.sock = None
        except Exception as e: self.get_logger().error(f'CTD Conn Fail {self.tcp_ip}:{self.tcp_port}: {e}'); self.sock = None

    def disconnect(self):
        if self.sock:
            try: self.sock.close(); self.get_logger().info("CTD socket closed.")
            except Exception as e: self.get_logger().warn(f"Error closing CTD socket: {e}")
            finally: self.sock = None

    def destroy_node(self):
        self.get_logger().info("Shutting down CTD node...")
        self.cleanup_resources()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CTDNode()
        rclpy.spin(node)
    except KeyboardInterrupt: 
        if node: node.get_logger().info("CTDNode interrupted by user.")
    except Exception as e:
        if node: node.get_logger().fatal(f"CTD Node unhandled exception: {e}")
        else: print(f"CTD Node unhandled exception during creation: {e}")
    finally:
        if node: 
            node.get_logger().info("Shutting down CTDNode.")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()