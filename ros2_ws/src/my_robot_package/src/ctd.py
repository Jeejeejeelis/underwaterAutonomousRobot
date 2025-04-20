#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, String
import time

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
import binascii
# import codecs
import math

class CTDNode(Node):
    def __init__(self):
        super().__init__('ctd_node')
        self.declare_parameter('simulate', True) # Default to simulation
        self.simulate = self.get_parameter('simulate').get_parameter_value().bool_value
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publisher_temp = self.create_publisher(Float32, 'temperature', 10)
        self.publisher_pressure = self.create_publisher(Float32, 'pressure', 10)
        self.publisher_conductivity = self.create_publisher(Float32, 'conductivity', 10)

        self.command_sub = self.create_subscription(String, 'send_ctd_command', self.send_ctd_callback, 10)

        self.ser = None
        self.timer = None
        self.sim_pressure_sub = None

        self.setup_mode()

    def parameters_callback(self, params):
        """Handle dynamic changes to the 'simulate' parameter."""
        old_simulate_value = self.simulate
        for param in params:
            if param.name == 'simulate':
                if param.type_ == Parameter.Type.BOOL:
                    self.simulate = param.value
                    self.get_logger().info(f"'simulate' parameter changed to: {self.simulate}")
                else:
                    self.get_logger().warn(f"Incorrect type for 'simulate' parameter: {param.type_}. Expected bool.")
                    return SetParametersResult(successful=False)

        if old_simulate_value != self.simulate:
            self.get_logger().info("Simulation mode changed, reconfiguring node...")
            self.setup_mode() # This will handle cleanup and setup

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
            self.get_logger().info("Setting up simulation mode.")
            self.sim_pressure_sub = self.create_subscription(
                Float32,
                'simulated_pressure',
                self.simulated_pressure_callback,
                10)
            self.get_logger().info("Subscribed to /simulated_pressure")
            self.timer = self.create_timer(1.0, self.publish_dummy_temp_cond_only)
            self.get_logger().info("Started timer for dummy Temperature/Conductivity publishing.")
        else:
            self.get_logger().info("Setting up hardware mode.")
            if not SERIAL_AVAILABLE:
                self.get_logger().error("Pyserial not found, but simulate=False. Cannot connect to hardware.")
                return

            try:
                self.ser = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=1.0)
                self.get_logger().info("Opened serial port /dev/ttyUSB0 successfully.")
                self.timer = self.create_timer(1.0, self.read_ctd_values)
                self.get_logger().info("Started timer for reading hardware CTD values.")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port /dev/ttyUSB0: {e}")
            except Exception as e:
                 self.get_logger().error(f"An unexpected error occurred during serial setup: {e}")

    def simulated_pressure_callback(self, msg):
        """Receives pressure from simulator and publishes it."""
        self.publisher_pressure.publish(msg)
        # self.get_logger().debug(f"Received simulated pressure {msg.data:.2f}, publishing to /pressure")

    def publish_dummy_temp_cond_only(self):
        """Publishes dummy values for Temperature and Conductivity."""
        temp_msg = Float32(data=20.0 + math.sin(time.time() * 0.1) * 2) # Some variation
        cond_msg = Float32(data=3.5 + math.sin(time.time() * 0.2) * 0.2) # Some variation

        self.publisher_temp.publish(temp_msg)
        self.publisher_conductivity.publish(cond_msg)
        # self.get_logger().info(f"Published dummy CTD: T={temp_msg.data:.2f}, C={cond_msg.data:.2f}")

    def send_ctd_callback(self, msg):
        if self.simulate:
            self.get_logger().info("In simulation mode, ignoring send_ctd command.")
            return
        if self.ser is None or not self.ser.is_open:
             self.get_logger().warn("Serial port not available. Cannot send command.")
             return
        cmd_string = msg.data
        self.get_logger().info(f"Received command to send: {cmd_string}")
        self.send_ctd(cmd_string)


    def read_ctd_values(self):
        if self.ser is None or not self.ser.is_open:
             self.get_logger().warn("Serial port not available. Cannot poll CTD.")
             return
        try:
            request_cmd_bytes = bytes([0xff,0xff,0xff,0xff,0xaa,0x00,0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x6c])
            self.get_logger().debug(f"Sending CTD data request: {binascii.hexlify(request_cmd_bytes)}")
            self.ser.reset_input_buffer()
            self.ser.write(request_cmd_bytes)
            response = self.ser.read(32)

            if not response:
                 self.get_logger().warn("No response received from CTD after request (timeout).")
                 return

            if len(response) < 32:
                 self.get_logger().warn(f"Incomplete response received from CTD ({len(response)}/32 bytes). Discarding.")
                 return

            self.get_logger().debug(f"Raw CTD Response: {response}") # Log raw bytes for debugging
            res, temperature, pressure, conductivity = self.parse_ctd_response(response)

            if res == 0: # Assuming 0 means success based on previous code
                self.publisher_temp.publish(Float32(data=temperature))
                self.publisher_pressure.publish(Float32(data=pressure))
                self.publisher_conductivity.publish(Float32(data=conductivity))
                # self.get_logger().info(f"Published CTD: T={temperature:.2f}, P={pressure:.2f}, C={conductivity:.2f}")
            else:
                self.get_logger().warn("Parsing CTD response indicated failure.")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error during CTD poll/read: {e}")
            self.cleanup_mode()
        except Exception as e:
            self.get_logger().error(f"Unexpected error during CTD poll/read: {e}")
            self.cleanup_mode()


    def parse_ctd_response(self, ctd_response_bytes):
        try:
            ctd_response_str = ctd_response_bytes.decode('ascii', errors='ignore')
        except Exception as e:
            self.get_logger().warn(f"Failed to decode CTD response: {e}")
            return -1, 0.0, 0.0, 0.0

        start_index = ctd_response_str.find('$AQCTD')
        if start_index == -1:
            self.get_logger().debug("CTD start string '$AQCTD' not found in response chunk.")
            return -1, 0.0, 0.0, 0.0

        end_index = ctd_response_str.find('*', start_index)
        if end_index == -1:
             self.get_logger().debug("CTD end marker '*' not found after start.")
             return -1, 0.0, 0.0, 0.0

        data_part = ctd_response_str[start_index + len('$AQCTD'):end_index]

        values = data_part.strip(',').split(',')
        if len(values) < 3:
            self.get_logger().warn(f"Could not extract enough values from CTD data: {values}")
            return -1, 0.0, 0.0, 0.0

        try:
            temperature = float(values[0])
            pressure = float(values[1])
            conductivity = float(values[2])
            self.get_logger().debug(f"Parsed CTD: T={temperature}, P={pressure}, C={conductivity}")
            return 0, temperature, pressure, conductivity
        except ValueError as e:
            self.get_logger().warn(f"Failed to convert CTD values to float: {values} - Error: {e}")
            return -1, 0.0, 0.0, 0.0
        except Exception as e:
            self.get_logger().error(f"Unexpected error during CTD parsing: {e}")
            return -1, 0.0, 0.0, 0.0


    def send_ctd(self, cmd_string):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Cannot send command, serial port not available.")
            return
        # (Rest of send_ctd logic...)
        try:
            if cmd_string == "DISP_ON_CMD": #check what commands ctd takes!
                 byte_cmd = bytes([0xff, 0xff, 0xff, 0xff, 0xaa, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6c])
                 self.get_logger().info(f"Sending DISP_ON_CMD bytes: {binascii.hexlify(byte_cmd)}")
                 self.ser.write(byte_cmd)
            else:
                 self.get_logger().info(f"Sending string command: {cmd_string}")
                 self.ser.write(cmd_string.encode('ascii') + b'\r\n') # Assume command needs newline

            time.sleep(0.2)
            if self.ser.in_waiting > 0:
                 response = self.ser.read(self.ser.in_waiting)
                 self.get_logger().info(f"Response to command '{cmd_string}': {response.decode('ascii', errors='ignore')}")
            else:
                 self.get_logger().info(f"No immediate response received for command '{cmd_string}'")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error during send: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error during CTD send: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CTDNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception: {e}")
        else:
            print(f"Unhandled exception during node creation: {e}")
    finally:

        if node:
            node.cleanup_mode()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()