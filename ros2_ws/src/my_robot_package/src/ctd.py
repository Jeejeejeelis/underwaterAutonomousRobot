#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, String
import time
import math
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
import binascii

DEFAULT_ATMOSPHERIC_PRESSURE_BAR = 1.01325

class CTDNode(Node):
    def __init__(self):
        super().__init__('ctd_node')
        self.declare_parameter('simulate', True)
        self.declare_parameter('serial_port_ctd', "/dev/ttyUSB0")
        self.declare_parameter('baud_rate_ctd', 9600)
        self.declare_parameter('hardware_poll_interval', 1.0)
        self.declare_parameter('atmospheric_pressure_bar', DEFAULT_ATMOSPHERIC_PRESSURE_BAR)

        self.simulate = self.get_parameter('simulate').get_parameter_value().bool_value
        self.serial_port = self.get_parameter('serial_port_ctd').value
        self.baud_rate = self.get_parameter('baud_rate_ctd').value
        self.poll_interval = self.get_parameter('hardware_poll_interval').value
        self.atmospheric_pressure_bar = self.get_parameter('atmospheric_pressure_bar').value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publisher_temp = self.create_publisher(Float32, 'temperature', 10)
        self.publisher_pressure = self.create_publisher(Float32, 'pressure', 10)
        self.publisher_conductivity = self.create_publisher(Float32, 'conductivity', 10)
        self.command_sub = self.create_subscription(String, 'send_ctd_command', self.send_ctd_callback, 10)

        self.ser = None
        self.timer_hardware_poll = None
        self.sim_pressure_sub = None

        self.setup_mode()

    def parameters_callback(self, params):
        old_simulate_value = self.simulate
        needs_reconfig = False
        for param in params:
            if param.name == 'simulate':
                if self.simulate != param.value: self.simulate = param.value; needs_reconfig = True
            elif param.name == 'serial_port_ctd':
                if self.serial_port != param.value: self.serial_port = param.value; needs_reconfig = True
            elif param.name == 'baud_rate_ctd':
                if self.baud_rate != param.value: self.baud_rate = param.value; needs_reconfig = True
            elif param.name == 'hardware_poll_interval':
                if self.poll_interval != param.value: self.poll_interval = param.value; needs_reconfig = True
            elif param.name == 'atmospheric_pressure_bar':
                self.atmospheric_pressure_bar = param.value
        if needs_reconfig:
            self.get_logger().info("CTD parameters changed, reconfiguring node...")
            self.setup_mode()
        return SetParametersResult(successful=True)

    def cleanup_resources(self):
        if self.timer_hardware_poll is not None: self.timer_hardware_poll.cancel(); self.timer_hardware_poll = None
        if self.ser is not None and self.ser.is_open: self.ser.close(); self.ser = None
        if self.sim_pressure_sub is not None: self.destroy_subscription(self.sim_pressure_sub); self.sim_pressure_sub = None

    def setup_mode(self):
        self.cleanup_resources()
        if self.simulate:
            self.get_logger().info("CTD Sim: Subscribing to '/sim_pressure_raw'. Publishing T,C,P on /sim_pressure_raw update.")
            self.sim_pressure_sub = self.create_subscription(
                Float32,
                'sim_pressure_raw',
                self.simulated_pressure_and_publish_callback,
                10)
        else: # hardware mode
            self.get_logger().info(f"CTD Hardware mode: Connecting to {self.serial_port} at {self.baud_rate} baud.")
            try:
                self.ser = serial.Serial(self.serial_port, baudrate=self.baud_rate, timeout=1.0)
                self.get_logger().info(f"Opened CTD serial port {self.serial_port} successfully.")
                self.timer_hardware_poll = self.create_timer(self.poll_interval, self.read_hardware_ctd_values)
            except Exception as e:
                self.get_logger().error(f"Failed to open CTD serial port {self.serial_port}: {e}")


    def simulated_pressure_and_publish_callback(self, msg):
        gauge_pressure_dbar = msg.data
        self.publisher_pressure.publish(Float32(data=gauge_pressure_dbar))

        # Publish dummy temperature and conductivity
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        temp_val = 20.0 + math.sin(current_time_sec * 0.1) * 2
        cond_val = 3.5 + math.sin(current_time_sec * 0.2) * 0.2
        self.publisher_temp.publish(Float32(data=temp_val))
        self.publisher_conductivity.publish(Float32(data=cond_val))
        self.get_logger().debug(f"Published Sim CTD on update: P={gauge_pressure_dbar:.2f}dbar (gauge), T={temp_val:.2f}C, C={cond_val:.2f}mS/cm")

    def send_ctd_callback(self, msg):
        if self.simulate:
            self.get_logger().info("In simulation mode, ignoring send_ctd command.")
            return
        if self.ser is None or not self.ser.is_open:
             self.get_logger().warn("Serial port not available. Cannot send command to CTD.")
             return
        cmd_string = msg.data
        self.get_logger().info(f"Received command to send to CTD: {cmd_string}")
        self.send_ctd_hardware_command(cmd_string)

    def read_hardware_ctd_values(self):
        if self.ser is None or not self.ser.is_open:
             self.get_logger().warn("CTD serial port not available. Cannot poll CTD.")
             return
        try:
            request_cmd_bytes = bytes([0xff,0xff,0xff,0xff,0xaa,0x00,0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x6c])
            self.ser.reset_input_buffer()
            self.ser.write(request_cmd_bytes)
            response = self.ser.read(32)

            if not response:
                 self.get_logger().warn("No response received from CTD after request.")
                 return

            res, temperature, absolute_pressure_bar, conductivity = self.parse_ctd_response_hardware(response)

            if res == 0:
                gauge_pressure_bar = absolute_pressure_bar - self.atmospheric_pressure_bar
                gauge_pressure_dbar = gauge_pressure_bar * 10.0
                if gauge_pressure_dbar < 0: gauge_pressure_dbar = 0.0

                self.publisher_temp.publish(Float32(data=temperature))
                self.publisher_pressure.publish(Float32(data=gauge_pressure_dbar))
                self.publisher_conductivity.publish(Float32(data=conductivity))
                self.get_logger().info(f"Published CTD: T={temperature:.3f}C, P_gauge={gauge_pressure_dbar:.3f}dbar, C={conductivity:.3f}mS/cm (Abs P sensor: {absolute_pressure_bar:.3f}bar)")
            else:
                self.get_logger().warn("Failed to parse CTD response.")
        except Exception as e:
            self.get_logger().error(f"Error during CTD poll/read: {e}")
            self.cleanup_resources()


    def parse_ctd_response_hardware(self, ctd_response_bytes):
        try: ctd_response_str = ctd_response_bytes.decode('ascii', errors='ignore')
        except Exception as e: self.get_logger().warn(f"Failed to decode CTD response bytes: {e}"); return -1,0.0,0.0,0.0
        self.get_logger().debug(f"Raw CTD string for parsing: {ctd_response_str.strip()}")
        start_index = ctd_response_str.find('$AQCTD')
        if start_index == -1: return -1,0.0,0.0,0.0
        end_index = ctd_response_str.find('*', start_index)
        if end_index == -1: return -1,0.0,0.0,0.0
        data_part = ctd_response_str[start_index + len('$AQCTD'):end_index]
        values = data_part.strip(',').split(',')
        if len(values) < 3: self.get_logger().warn(f"Could not extract 3 values from CTD: '{data_part}'"); return -1,0.0,0.0,0.0
        try:
            temperature = float(values[0])
            absolute_pressure_bar = float(values[1])
            conductivity = float(values[2])
            return 0, temperature, absolute_pressure_bar, conductivity
        except ValueError as e: self.get_logger().warn(f"Convert CTD values error: {values} - {e}"); return -1,0.0,0.0,0.0
        except Exception as e: self.get_logger().error(f"Unexpected CTD parsing error: {e}"); return -1,0.0,0.0,0.0

    def send_ctd_hardware_command(self, cmd_string):
        if self.ser is None or not self.ser.is_open: self.get_logger().warn("No CTD serial."); return
        try:
            if cmd_string == "DISP_ON_CMD": byte_cmd = bytes([0xff,0xff,0xff,0xff,0xaa,0x00,0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x6c])
            else: byte_cmd = cmd_string.encode('ascii') + b'\r\n'
            self.get_logger().info(f"Sending to CTD: {byte_cmd!r}")
            self.ser.write(byte_cmd)
            time.sleep(0.2)
            if self.ser.in_waiting > 0:
                 response = self.ser.read(self.ser.in_waiting)
                 self.get_logger().info(f"CTD Response to '{cmd_string}': {response.decode('ascii',errors='ignore').strip()}")
            else: self.get_logger().info(f"No immediate response for CTD cmd '{cmd_string}'")
        except Exception as e: self.get_logger().error(f"CTD send error: {e}")

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
    except KeyboardInterrupt: pass
    except Exception as e:
        if node: node.get_logger().fatal(f"CTD Node unhandled exception: {e}")
        else: print(f"CTD Node unhandled exception during creation: {e}")
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()