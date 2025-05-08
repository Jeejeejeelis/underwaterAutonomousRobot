#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, String
import time
import traceback

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
import binascii
import math

class CTDNode(Node):
    def __init__(self):
        super().__init__('ctd_node')

        simulate_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Run in simulation mode if True, otherwise use hardware.'
        )
        serial_port_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Serial port for the CTD device (e.g., /dev/ttyUSB0).'
        )
        baud_rate_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Baud rate for the CTD serial communication.'
        )
        read_interval_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Interval in seconds for reading CTD data in hardware mode.'
        )

        self.declare_parameter('simulate', True, simulate_descriptor)
        self.declare_parameter('serial_port', "/dev/ttyUSB0", serial_port_descriptor)
        self.declare_parameter('baud_rate', 9600, baud_rate_descriptor)
        self.declare_parameter('read_interval_s', 1.0, read_interval_descriptor)

        self.simulate = self.get_parameter('simulate').get_parameter_value().bool_value
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.read_interval = self.get_parameter('read_interval_s').get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publisher_temp = self.create_publisher(Float32, 'temperature', 10)
        self.publisher_pressure = self.create_publisher(Float32, 'pressure', 10)
        self.publisher_conductivity = self.create_publisher(Float32, 'conductivity', 10)

        self.command_sub = self.create_subscription(String, 'send_ctd_command', self.send_ctd_command_callback, 10)

        self.ser = None
        self.timer = None
        self.sim_pressure_sub = None

        self.get_logger().info(f"CTD Node starting. Simulation mode: {self.simulate}")
        self.setup_mode()

    def send_ctd_command_callback(self, msg):
            """Callback function for the 'send_ctd_command' topic."""
            if self.simulate:
                self.get_logger().info("CTD: In simulation mode, ignoring send_ctd command.")
                return
            if self.ser is None or not self.ser.is_open:
                self.get_logger().warn("CTD: Serial port not available. Cannot send command.")
                return
            cmd_string = msg.data
            self.get_logger().info(f"CTD: Received command to send: {cmd_string}")
            self.send_ctd_to_hardware(cmd_string)

    def parameters_callback(self, params):
        """Handle dynamic changes to parameters."""
        old_simulate_value = self.simulate
        changed_hw_params = False
        changed_timer_params = False

        for param in params:
            if param.name == 'simulate':
                if param.type_ == Parameter.Type.BOOL:
                    self.simulate = param.value
                    self.get_logger().info(f"'simulate' parameter changed to: {self.simulate}")
                else:
                    self.get_logger().warn(f"Incorrect type for 'simulate' parameter. Expected bool.")
                    return SetParametersResult(successful=False)
            elif param.name == 'serial_port':
                self.serial_port = param.value
                changed_hw_params = True
            elif param.name == 'baud_rate':
                self.baud_rate = param.value
                changed_hw_params = True
            elif param.name == 'read_interval_s':
                self.read_interval = param.value
                changed_timer_params = True

        if old_simulate_value != self.simulate or changed_hw_params:
            self.get_logger().info("Simulation mode or hardware parameters changed, reconfiguring node...")
            self.setup_mode()
        elif changed_timer_params and not self.simulate:
            self.get_logger().info("CTD read interval changed, reconfiguring timer for hardware mode.")
            if self.timer: self.timer.cancel()
            if self.ser and self.ser.is_open:
                 self.timer = self.create_timer(self.read_interval, self.read_ctd_values)
                 self.get_logger().info(f"Restarted hardware timer with new interval: {self.read_interval}s")
            else:
                 self.get_logger().warn("Read interval changed, but serial port not open. Timer will be set up if hardware mode activates.")


        return SetParametersResult(successful=True)

    def cleanup_mode(self):
        """Stop timers, close serial ports, destroy simulation subscribers."""
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            self.get_logger().debug("Timer cancelled.")
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.get_logger().info("Closed serial port.")
        if self.sim_pressure_sub is not None:
            self.destroy_subscription(self.sim_pressure_sub)
            self.sim_pressure_sub = None
            self.get_logger().debug("Simulated pressure subscriber destroyed.")

    def setup_mode(self):
        """Sets up the node based on the simulation parameter."""
        self.cleanup_mode()

        if self.simulate:
            self.get_logger().info("CTD: Setting up simulation mode.")
            self.sim_pressure_sub = self.create_subscription(
                Float32,
                'simulated_pressure_dbar',
                self.simulated_pressure_callback,
                10)
            self.get_logger().info("CTD: Subscribed to /simulated_pressure_dbar")
            self.timer = self.create_timer(self.read_interval, self.publish_dummy_temp_cond_only)
            self.get_logger().info(f"CTD: Started timer for dummy T/C publishing every {self.read_interval}s.")
        else:
            self.get_logger().info("CTD: Setting up hardware mode.")
            if not SERIAL_AVAILABLE:
                self.get_logger().error("CTD: Pyserial not found, but simulate=False. Cannot connect to hardware.")
                return

            try:
                self.get_logger().info(f"CTD: Attempting to open serial port {self.serial_port} at {self.baud_rate} baud.")
                self.ser = serial.Serial(self.serial_port, baudrate=self.baud_rate, timeout=1.0)
                self.get_logger().info(f"CTD: Opened serial port {self.serial_port} successfully.")
                self.timer = self.create_timer(self.read_interval, self.read_ctd_values)
                self.get_logger().info(f"CTD: Started timer for reading hardware CTD values every {self.read_interval}s.")
            except serial.SerialException as e:
                self.get_logger().error(f"CTD: Failed to open serial port {self.serial_port}: {e}")
                self.ser = None
            except Exception as e:
                 self.get_logger().error(f"CTD: An unexpected error occurred during serial setup: {e}")
                 self.ser = None

    def simulated_pressure_callback(self, msg):
        """Receives pressure from simulator (expected in dbar) and publishes it."""
        self.publisher_pressure.publish(msg)
        # self.get_logger().debug(f"CTD (Sim): Relayed simulated pressure {msg.data:.2f} dbar to /pressure")

    def publish_dummy_temp_cond_only(self):
        """Publishes dummy values for Temperature and Conductivity. Pressure comes from callback."""
        temp_msg = Float32(data=15.0 + math.sin(time.time() * 0.15) * 5) # Dummy temperature
        cond_msg = Float32(data=3.0 + math.sin(time.time() * 0.25) * 0.5) # Dummy conductivity

        self.publisher_temp.publish(temp_msg)
        self.publisher_conductivity.publish(cond_msg)
        # self.get_logger().info(f"CTD (Sim): Published dummy T={temp_msg.data:.2f}, C={cond_msg.data:.2f}")

    def send_ctd_callback(self, msg):
        if self.simulate:
            self.get_logger().info("CTD: In simulation mode, ignoring send_ctd command.")
            return
        if self.ser is None or not self.ser.is_open:
             self.get_logger().warn("CTD: Serial port not available. Cannot send command.")
             return
        cmd_string = msg.data
        self.get_logger().info(f"CTD: Received command to send: {cmd_string}")
        self.send_ctd_to_hardware(cmd_string)

    def read_ctd_values(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("CTD: Serial port not available for poll. Attempting to reopen...")
            try:
                self.ser = serial.Serial(self.serial_port, baudrate=self.baud_rate, timeout=1.0)
                self.get_logger().info(f"CTD: Reopened serial port {self.serial_port} successfully.")
            except serial.SerialException as e:
                self.get_logger().error(f"CTD: Failed to reopen serial port {self.serial_port}: {e}")
                self.ser = None
            except Exception as e:
                 self.get_logger().error(f"CTD: Unexpected error reopening serial port: {e}")
                 self.ser = None
            return
        try:
            request_cmd_bytes = bytes([0xff,0xff,0xff,0xff,0xaa,0x00,0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x6c])
            self.get_logger().debug(f"CTD: Sending data request: {binascii.hexlify(request_cmd_bytes)}")
            self.ser.reset_input_buffer()
            self.ser.write(request_cmd_bytes)
            response = self.ser.read(32) 
            if not response:
                 self.get_logger().warn("CTD: No response received after request (timeout).")
                 return

            min_expected_len = 10
            if len(response) < min_expected_len:
                 self.get_logger().warn(f"CTD: Incomplete response ({len(response)} bytes, expected at least {min_expected_len}). Discarding. Raw: {binascii.hexlify(response)}")
                 return

            self.get_logger().debug(f"CTD: Raw Response: {binascii.hexlify(response)}")
            res, temperature, pressure, conductivity = self.parse_ctd_response(response)

            if res == 0: # Success
                self.publisher_temp.publish(Float32(data=temperature))
                self.publisher_pressure.publish(Float32(data=pressure))
                self.publisher_conductivity.publish(Float32(data=conductivity))
                # self.get_logger().info(f"CTD: Published T={temperature:.2f}, P(dbar)={pressure:.2f}, C={conductivity:.2f}")
            else:
                self.get_logger().warn("CTD: Parsing response indicated failure.")

        except serial.SerialException as e:
            self.get_logger().error(f"CTD: Serial error during poll/read: {e}. Port will be closed.")
            if self.ser and self.ser.is_open: self.ser.close()
            self.ser = None
        except Exception as e:
            self.get_logger().error(f"CTD: Unexpected error during poll/read: {e}")

    def parse_ctd_response(self, ctd_response_bytes):
        try:
            ctd_response_str = ""
            try:
                start_char_index = ctd_response_bytes.find(b'$')
                if start_char_index != -1:
                    ctd_response_str = ctd_response_bytes[start_char_index:].decode('ascii', errors='ignore')
                else:
                    ctd_response_str = ctd_response_bytes.decode('ascii', errors='ignore')
            except UnicodeDecodeError:
                self.get_logger().warn(f"CTD: Failed to decode response part as ASCII. Raw: {binascii.hexlify(ctd_response_bytes)}")
                return -1, 0.0, 0.0, 0.0
        except Exception as e:
            self.get_logger().error(f"CTD: Error processing raw bytes before parsing: {e}. Raw: {binascii.hexlify(ctd_response_bytes)}")
            return -1, 0.0, 0.0, 0.0

        start_token = '$AQCTD'
        start_index = ctd_response_str.find(start_token)
        if start_index == -1:
            self.get_logger().debug(f"CTD: Start string '{start_token}' not found in decoded chunk: '{ctd_response_str[:60]}...'")
            return -1, 0.0, 0.0, 0.0

        end_marker = '*'
        end_index = ctd_response_str.find(end_marker, start_index)
        if end_index == -1:
             self.get_logger().debug(f"CTD: End marker '{end_marker}' not found after start in: '{ctd_response_str[start_index:start_index+60]}...'")
             return -1, 0.0, 0.0, 0.0

        data_part = ctd_response_str[start_index + len(start_token) : end_index]
        values = data_part.strip(',').split(',')

        if len(values) < 3:
            self.get_logger().warn(f"CTD: Could not extract enough values (expected 3) from data: '{data_part}'. Values: {values}")
            return -1, 0.0, 0.0, 0.0

        try:
            temperature = float(values[0])
            pressure = float(values[1])
            conductivity = float(values[2])
            self.get_logger().debug(f"CTD: Parsed T={temperature}, P(dbar)={pressure}, C={conductivity}")
            return 0, temperature, pressure, conductivity
        except ValueError as e:
            self.get_logger().warn(f"CTD: Failed to convert values to float: {values} from '{data_part}'. Error: {e}")
            return -1, 0.0, 0.0, 0.0
        except Exception as e:
            self.get_logger().error(f"CTD: Unexpected error parsing values: {e}. Values: {values}")
            return -1, 0.0, 0.0, 0.0

    def send_ctd_to_hardware(self, cmd_string):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("CTD: Cannot send command, serial port not available.")
            return
        try:
            if cmd_string == "DISP_ON_CMD":
                 byte_cmd = bytes([0xff, 0xff, 0xff, 0xff, 0xaa, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6c])
                 self.get_logger().info(f"CTD: Sending '{cmd_string}' bytes: {binascii.hexlify(byte_cmd)}")
                 self.ser.write(byte_cmd)
            else:
                 self.get_logger().info(f"CTD: Sending generic string command: {cmd_string}")
                 self.ser.write(cmd_string.encode('ascii') + b'\r\n')

            time.sleep(0.2)
            if self.ser.in_waiting > 0:
                 response = self.ser.read(self.ser.in_waiting)
                 self.get_logger().info(f"CTD: Response to command '{cmd_string}': {response.decode('ascii', errors='ignore')}")
            else:
                 self.get_logger().info(f"CTD: No immediate response received for command '{cmd_string}' (this might be normal).")

        except serial.SerialException as e:
            self.get_logger().error(f"CTD: Serial error during send: {e}")
            if self.ser and self.ser.is_open: self.ser.close()
            self.ser = None
        except Exception as e:
            self.get_logger().error(f"CTD: Unexpected error during send: {e}")

    def destroy_node(self):
        self.get_logger().info("CTD Node shutting down...")
        self.cleanup_mode()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CTDNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("CTD: Keyboard interrupt, shutting down.")
    except Exception as e:
        if node:
            exc_info_str = traceback.format_exc()
            node.get_logger().fatal(f"CTD: Unhandled exception: {e}\n{exc_info_str}")
        else:
            print(f"CTD: Unhandled exception during node creation: {e}")
    finally:
        if node:
            node.cleanup_mode()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("CTD Node terminated.")

if __name__ == '__main__':
    main()