#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
import time
import random
from datetime import datetime, timezone
import traceback

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

DEFAULT_AT_CMD_TIMEOUT_S = 2.0
DEFAULT_AT_CMD_INTERVAL_S = 1.5

class GPSNode(Node):
    def __init__(self):
        super().__init__('gnss_node')

        sim_desc = ParameterDescriptor(description='Run in simulation mode.')
        port_desc = ParameterDescriptor(description='Serial port for GNSS module (e.g., /dev/ttyUSB2).')
        baud_desc = ParameterDescriptor(description='Baud rate for GNSS serial communication.')
        req_interval_desc = ParameterDescriptor(description='Interval to request/process GNSS data (seconds).')
        cmd_interval_desc = ParameterDescriptor(description='Minimum interval between sending AT commands (seconds).')
        cmd_timeout_desc = ParameterDescriptor(description='Timeout for AT command responses (seconds).')

        self.declare_parameter('simulate', True, sim_desc)
        self.declare_parameter('serial_port', "/dev/ttyS0", port_desc)
        self.declare_parameter('baud_rate', 115200, baud_desc)
        self.declare_parameter('request_interval_s', 2.0, req_interval_desc)
        self.declare_parameter('command_interval_s', DEFAULT_AT_CMD_INTERVAL_S, cmd_interval_desc)
        self.declare_parameter('command_timeout_s', DEFAULT_AT_CMD_TIMEOUT_S, cmd_timeout_desc)

        self.simulate = self.get_parameter('simulate').get_parameter_value().bool_value
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.request_interval = self.get_parameter('request_interval_s').get_parameter_value().double_value
        self.command_interval = self.get_parameter('command_interval_s').get_parameter_value().double_value
        self.command_timeout = self.get_parameter('command_timeout_s').get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publisher = self.create_publisher(String, 'gnss/raw_data', 10)

        self.ser = None
        self.timer = None

        self.gps_initialized_hardware = False
        self.last_cmd_sent_time = 0.0
        self.init_step = 0

        self.get_logger().info(f"GNSS Node starting. Simulation mode: {self.simulate}")
        self.setup_mode()

    def parameters_callback(self, params):
        result = SetParametersResult(successful=True)
        should_setup_mode = False
        
        for param in params:
            if param.name == 'simulate' and self.simulate != param.value:
                self.simulate = param.value
                self.get_logger().info(f"GNSS 'simulate' changed to {self.simulate}")
                should_setup_mode = True
            elif param.name == 'serial_port' and self.serial_port != param.value:
                self.serial_port = param.value
                should_setup_mode = True
            elif param.name == 'baud_rate' and self.baud_rate != param.value:
                self.baud_rate = param.value
                should_setup_mode = True
            elif param.name == 'request_interval_s' and self.request_interval != param.value:
                self.request_interval = param.value
                if not should_setup_mode:
                    if self.timer: self.timer.cancel()
                    self.timer = self.create_timer(self.request_interval, self._timer_callback_logic)
                    self.get_logger().info(f"GNSS request interval changed to {self.request_interval}s. Timer reset.")
            elif param.name == 'command_interval_s':
                 self.command_interval = param.value
            elif param.name == 'command_timeout_s':
                 self.command_timeout = param.value
        
        if should_setup_mode:
            self.get_logger().info("GNSS parameters triggered full mode setup.")
            self.setup_mode()
            
        return result

    def setup_mode(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        if self.ser is not None and self.ser.is_open:
            if not self.simulate and self.gps_initialized_hardware:
                self.get_logger().info("GNSS: Attempting to turn off GPS (AT+CGPS=0) before re-setup.")
                self._send_at_command('AT+CGPS=0', 'OK', self.command_timeout / 2)
            self.ser.close()
            self.ser = None
            self.get_logger().info("GNSS: Closed serial port.")

        self.gps_initialized_hardware = False
        self.init_step = 0

        if self.simulate:
            self.get_logger().info("GNSS: Running in simulation mode.")
            self.timer = self.create_timer(self.request_interval, self.publish_dummy_gnss_data) # Renamed
        else:
            self.get_logger().info("GNSS: Running in hardware mode.")
            if not SERIAL_AVAILABLE:
                self.get_logger().error("GNSS: Pyserial not found, but simulate=False. Cannot connect to hardware.")
                return
            self.timer = self.create_timer(self.request_interval, self._timer_callback_logic)

    def _timer_callback_logic(self):
        """Main logic for timer callback, routes to sim or hw."""
        if self.simulate:
            self.publish_dummy_gnss_data()
        else:
            self.manage_hardware_gnss_session()


    def publish_dummy_gnss_data(self):
        lat_deg = 60
        lat_min = 11.0 + random.uniform(-0.05, 0.05)
        lon_deg = 24
        lon_min = 56.0 + random.uniform(-0.05, 0.05)
        time_utc = datetime.now(timezone.utc).strftime("%H%M%S.%f")[:9] # hhmmss.ss
        satellites = random.randint(7,12)
        hdop = random.uniform(0.8, 1.5)
        altitude_msl = random.uniform(10.0, 15.0)
        geoid_separation = random.uniform(19.5, 20.5)

        nmea_sentence = f"$GPGGA,{time_utc},{lat_deg:02d}{lat_min:07.4f},N,{lon_deg:03d}{lon_min:07.4f},E,1,{satellites:02d},{hdop:.1f},{altitude_msl:.1f},M,{geoid_separation:.1f},M,,"
        checksum_payload = nmea_sentence[1:]
        checksum = 0
        for char in checksum_payload:
            checksum ^= ord(char)
        dummy_data = f"{nmea_sentence}*{checksum:02X}"
        
        # self.get_logger().debug(f"GNSS (Sim): Publishing dummy data: {dummy_data}")
        self.publisher.publish(String(data=dummy_data))

    def _send_at_command(self, command, expected_response, timeout_sec):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn(f"GNSS: Serial port not available. Cannot send AT command: {command}")
            return False, ""

        rec_buff_bytes = b''
        try:
            self.get_logger().debug(f"GNSS: Sending AT: {command}")
            self.ser.reset_input_buffer() # Clear any old data
            self.ser.reset_output_buffer()
            self.ser.write((command + '\r\n').encode('ascii'))
            
            start_time = time.monotonic()
            response_received = False
            while (time.monotonic() - start_time) < timeout_sec:
                if self.ser.in_waiting > 0:
                    rec_buff_bytes += self.ser.read(self.ser.in_waiting)
                    rec_buff_str_for_check = rec_buff_bytes.decode('ascii', errors='ignore')
                    if expected_response in rec_buff_str_for_check or "OK" in rec_buff_str_for_check or "ERROR" in rec_buff_str_for_check:
                        response_received = True
                if response_received and self.ser.in_waiting == 0:
                    time.sleep(0.1)
                    if self.ser.in_waiting > 0: rec_buff_bytes += self.ser.read(self.ser.in_waiting)
                    break
                time.sleep(0.05)

            rec_buff_decoded = rec_buff_bytes.decode('ascii', errors='ignore').strip()
            log_response = rec_buff_decoded.replace('\r\n', ' | ').replace('\r', ' | ').replace('\n', ' | ')
            self.get_logger().debug(f"GNSS: Response to '{command}': {log_response}")

            if expected_response in rec_buff_decoded:
                return True, rec_buff_decoded
            else:
                self.get_logger().warn(f"GNSS: AT command '{command}' did not return '{expected_response}'. Full response: {log_response}")
                return False, rec_buff_decoded

        except serial.SerialException as e:
            self.get_logger().error(f"GNSS: Serial error during AT '{command}': {e}")
            if self.ser and self.ser.is_open: self.ser.close()
            self.ser = None
            self.gps_initialized_hardware = False
            self.init_step = 0
            return False, ""
        except Exception as e:
             self.get_logger().error(f"GNSS: Unexpected error during AT '{command}': {e}")
             return False, ""

    def manage_hardware_gnss_session(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().info("GNSS: Hardware mode - serial port not open. Attempting to open...")
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.5)
                self.get_logger().info(f"GNSS: Serial port {self.serial_port} opened successfully.")
                self.init_step = 0
                self.gps_initialized_hardware = False
            except serial.SerialException as e:
                self.get_logger().error(f"GNSS: Failed to open serial port {self.serial_port}: {e}. Will retry later.")
                self.ser = None
                return
            except Exception as e:
                 self.get_logger().error(f"GNSS: Unexpected error opening serial port: {e}")
                 self.ser = None
                 return

        now_monotonic = time.monotonic()
        if (now_monotonic - self.last_cmd_sent_time) < self.command_interval:
             # self.get_logger().debug(f"GNSS: Waiting {self.command_interval - (now_monotonic - self.last_cmd_sent_time):.2f}s before next command...")
             return

        success = False
        response_data = ""

        if self.init_step == 0:
             self.get_logger().info('GNSS: Starting GPS session initialization sequence...')
             self.init_step = 1

        elif self.init_step == 1:
             self.get_logger().info('GNSS: Sending AT+CGPS=0 (Turn off GPS engine)...')
             success, response_data = self._send_at_command('AT+CGPS=0', 'OK', self.command_timeout)
             if success: self.init_step = 2

        elif self.init_step == 2:
            self.get_logger().info('GNSS: Sending AT+CGPS=1 (Turn on GPS engine)...')
            success, response_data = self._send_at_command('AT+CGPS=1', 'OK', self.command_timeout)
            if success:
                self.init_step = 3
                self.gps_initialized_hardware = True
                self.get_logger().info('GNSS: GPS engine initialized successfully.')

        elif self.init_step == 3 and self.gps_initialized_hardware:
            self.get_logger().debug('GNSS: Requesting GPS info (AT+CGPSINFO)...')
            success, response_data = self._send_at_command('AT+CGPSINFO', '+CGPSINFO:', self.command_timeout)
            if success and '+CGPSINFO:' in response_data:
                info_payload = ""
                for line in response_data.splitlines():
                    line = line.strip()
                    if line.startswith('+CGPSINFO:'):
                        info_payload = line.split('+CGPSINFO:', 1)[-1].strip()
                        break
                
                if not info_payload or ',,,,,,' in info_payload or all(c == ',' for c in info_payload.replace(" ", "")): # Check for empty or all-comma payload
                     self.get_logger().info('GNSS: Fix not ready yet or data is empty.')
                else:
                    self.get_logger().info(f"GNSS: Data received: {info_payload}")
                    self.publisher.publish(String(data=info_payload))
            elif success:
                self.get_logger().warn(f"GNSS: AT+CGPSINFO responded but no +CGPSINFO line found. Response: {response_data}")

        else:
             self.get_logger().warn(f"GNSS: In unexpected initialization step ({self.init_step}) or not initialized. Resetting.")
             self.init_step = 0
             self.gps_initialized_hardware = False

        self.last_cmd_sent_time = now_monotonic
    def destroy_node(self):
        self.get_logger().info("GNSS Node shutting down...")
        if self.timer:
            self.timer.cancel()
        if self.ser is not None and self.ser.is_open:
            try:
                 if not self.simulate and self.gps_initialized_hardware :
                     self.get_logger().info("GNSS: Sending AT+CGPS=0 (Turn off GPS) before closing port...")
                     self._send_at_command('AT+CGPS=0', 'OK', 1.0)
            except Exception as e:
                 self.get_logger().warn(f"GNSS: Failed to send final AT+CGPS=0: {e}")
            finally:
                 self.ser.close()
                 self.get_logger().info("GNSS: Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GPSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("GNSS: Keyboard interrupt, shutting down.")
    except Exception as e:
        if node:
            exc_info_str = traceback.format_exc()
            node.get_logger().fatal(f"GNSS: Unhandled exception: {e}\n{exc_info_str}")
        else:
            print(f"GNSS: Unhandled exception during node creation: {e}")
            traceback.print_exc()
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("GNSS Node terminated.")

if __name__ == '__main__':
    main()