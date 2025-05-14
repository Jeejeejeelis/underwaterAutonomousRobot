#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
import time

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

class GPSNode(Node):
    def __init__(self):
        super().__init__('gnss_node')

        self.declare_parameter('simulate', True)
        self.declare_parameter('serial_port_gnss', "/dev/ttyUSB2") # Specific port for GNSS
        self.declare_parameter('baud_rate_gnss', 115200)
        self.declare_parameter('timer_period_gnss', 2.0) # How often to request/publish GPS data

        self.simulate = self.get_parameter('simulate').get_parameter_value().bool_value
        self.serial_port = self.get_parameter('serial_port_gnss').value
        self.baud_rate = self.get_parameter('baud_rate_gnss').value
        self.timer_period = self.get_parameter('timer_period_gnss').value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publisher = self.create_publisher(String, 'gps_data', 10) # Publishes raw NMEA or +CGPSINFO like string

        self.ser = None
        self.timer = None

        self.gps_initialized_hardware = False # For hardware init sequence
        self.last_cmd_time = time.time()
        self.init_step = 0 # 0: start, 1: turn off, 2: turn on, 3: ready (for hardware)

        self.setup_mode()

    def parameters_callback(self, params):
        old_simulate = self.simulate
        old_serial_port = self.serial_port
        old_baud_rate = self.baud_rate
        old_timer_period = self.timer_period

        for param in params:
            if param.name == 'simulate': self.simulate = param.value
            elif param.name == 'serial_port_gnss': self.serial_port = param.value
            elif param.name == 'baud_rate_gnss': self.baud_rate = param.value
            elif param.name == 'timer_period_gnss': self.timer_period = param.value

        if (old_simulate != self.simulate or
            old_serial_port != self.serial_port or
            old_baud_rate != self.baud_rate or
            old_timer_period != self.timer_period):
            self.get_logger().info("GNSS parameters changed, reconfiguring node.")
            self.setup_mode()
        return SetParametersResult(successful=True)

    def cleanup_resources(self):
        if self.timer: self.timer.cancel(); self.timer = None
        if self.ser and self.ser.is_open:
            if not self.simulate: # Try to turn off GPS if it was on for hardware
                try:
                    self.get_logger().info("Attempting to turn off GPS module (AT+CGPS=0) before closing port.")
                    self.send_at_command_hardware('AT+CGPS=0', 'OK', 1.0)
                except Exception as e:
                    self.get_logger().warn(f"Failed to send AT+CGPS=0 on cleanup: {e}")
            self.ser.close()
            self.ser = None
        self.gps_initialized_hardware = False
        self.init_step = 0

    def setup_mode(self):
        self.cleanup_resources()
        if self.simulate:
            self.get_logger().info("GNSS Simulation mode: Publishing dummy NMEA-like data.")
            self.timer = self.create_timer(self.timer_period, self.publish_simulated_gnss_data)
        else:
            self.get_logger().info(f"GNSS Hardware mode: Connecting to {self.serial_port} at {self.baud_rate} baud.")
            if not SERIAL_AVAILABLE:
                self.get_logger().error("Pyserial not found, but simulate=False. Cannot connect to GNSS hardware.")
                return
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
                self.get_logger().info("GNSS serial port opened successfully.")
                self.timer = self.create_timer(self.timer_period, self.request_hardware_gps_position)
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open GNSS serial port {self.serial_port}: {e}")
            except Exception as e:
                 self.get_logger().error(f"An unexpected error occurred during GNSS serial setup: {e}")


    def publish_simulated_gnss_data(self):
        # Simulate a GPGGA NMEA string or a raw +CGPSINFO string
        # Using the +CGPSINFO format from the original gnss.py for consistency
        # Format: latitude,N/S,longitude,E/W,date,time,altitude,speed,course
        # Example: 6011.5234,N,02456.7498,E,160425,123519.00,545.4,0.0,,
        # Let's make it slightly dynamic
        lat_deg = 60 + (time.time() % 1000) / 10000.0
        lon_deg = 24 + (time.time() % 1000) / 10000.0
        lat_nmea = f"{int(lat_deg*100):04d}.{int((lat_deg*100 - int(lat_deg*100))*10000):04d}" # DDMM.MMMM
        lon_nmea = f"{int(lon_deg*100):05d}.{int((lon_deg*100 - int(lon_deg*100))*10000):04d}" # DDDMM.MMMM

        # Create a more complete +CGPSINFO like string, or a GPGGA.
        # For simplicity, using the original dummy string structure.
        sim_data = f"{lat_nmea},N,{lon_nmea},E,{time.strftime('%d%m%y,%H%M%S.00', time.gmtime())},50.0,0.0,"
        self.get_logger().debug(f"Publishing Simulated GNSS (+CGPSINFO like): {sim_data}")
        self.publisher.publish(String(data=sim_data))

    def send_at_command_hardware(self, command, expected_response, timeout_sec):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn(f"GNSS serial port not available. Cannot send AT: {command}")
            return False, ""
        rec_buff_bytes = b''
        try:
            self.get_logger().debug(f"Sending AT to GNSS: {command}")
            self.ser.reset_input_buffer()
            self.ser.write((command + '\r\n').encode('ascii'))
            start_time = time.monotonic()
            while (time.monotonic() - start_time) < timeout_sec:
                if self.ser.in_waiting > 0:
                    rec_buff_bytes += self.ser.read(self.ser.in_waiting)
                    # Early exit if expected response or ERROR is found
                    if expected_response.encode('ascii') in rec_buff_bytes or b"ERROR" in rec_buff_bytes:
                        break
                time.sleep(0.05) # Small delay to allow data to arrive

            rec_buff_decoded = rec_buff_bytes.decode('ascii', errors='ignore').strip()
            self.get_logger().debug(f"GNSS Response to '{command}': {rec_buff_decoded}")

            if expected_response in rec_buff_decoded:
                return True, rec_buff_decoded
            else:
                self.get_logger().warn(f"GNSS AT command '{command}' failed or unexpected response. Got: {rec_buff_decoded}")
                return False, rec_buff_decoded
        except serial.SerialException as e:
            self.get_logger().error(f"GNSS serial error during AT command '{command}': {e}")
            self.cleanup_resources() # Try to reset
            return False, ""
        except Exception as e:
             self.get_logger().error(f"Unexpected error during GNSS AT command '{command}': {e}")
             return False, ""

    def request_hardware_gps_position(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("GNSS serial port not available. Trying to reopen...")
            self.setup_mode() # Attempt to re-establish
            return

        now = time.time()
        if now - self.last_cmd_time < 1.5: # Basic command pacing
             return

        success = False
        response_data = ""

        if self.init_step == 0: # Start initialization
             self.get_logger().info('Starting GNSS hardware initialization...')
             self.init_step = 1

        elif self.init_step == 1: # Turn off GPS (ensure clean state)
             success, _ = self.send_at_command_hardware('AT+CGPS=0', 'OK', 2.0)
             if success: self.init_step = 2
             else: self.init_step = 0 # Retry init

        elif self.init_step == 2: # Turn on GPS
            success, _ = self.send_at_command_hardware('AT+CGPS=1', 'OK', 2.0)
            if success:
                self.init_step = 3
                self.gps_initialized_hardware = True
                self.get_logger().info('GNSS module initialized for hardware mode.')
            else: self.init_step = 0 # Retry init


        elif self.init_step == 3 and self.gps_initialized_hardware: # Ready to request data
            self.get_logger().debug('Requesting GNSS info (AT+CGPSINFO)...')
            success, response_data = self.send_at_command_hardware('AT+CGPSINFO', '+CGPSINFO:', 2.0)
            if success:
                info_part = response_data.split('+CGPSINFO:', 1)[-1].strip()
                if ',,,,,,' in info_part or info_part == "": # Check for empty fix
                     self.get_logger().info('GNSS fix not ready yet.')
                else:
                    self.get_logger().info(f"GNSS data received: {info_part}")
                    self.publisher.publish(String(data=info_part))
            # else: stay in init_step 3 and try again
        else: # Should not happen, reset
             self.get_logger().warn("GNSS in unexpected state, resetting initialization.")
             self.init_step = 0
             self.gps_initialized_hardware = False

        self.last_cmd_time = now

    def destroy_node(self):
        self.get_logger().info("Shutting down GNSS node...")
        self.cleanup_resources()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GPSNode()
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    except Exception as e:
        if node: node.get_logger().fatal(f"GNSS Node unhandled exception: {e}")
        else: print(f"GNSS Node unhandled exception during creation: {e}")
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()