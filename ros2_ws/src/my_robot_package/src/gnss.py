#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
import time
# Import serial only if not simulating
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

class GPSNode(Node):
    def __init__(self):
        super().__init__('gnss_node') # Corrected node name

        self.declare_parameter('simulate', True)
        self.declare_parameter('serial_port', "/dev/ttyUSB2")
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timer_period', 2.0) # How often to request GPS data

        self.simulate = self.get_parameter('simulate').get_parameter_value().bool_value
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publisher = self.create_publisher(String, 'gps_data', 10)

        self.ser = None
        self.timer = None

        self.gps_initialized = False
        self.last_cmd_time = time.time()
        self.init_step = 0 # 0: start, 1: turn off, 2: turn on, 3: ready

        self.setup_mode()

    def parameters_callback(self, params):
        """Callback for dynamic parameter changes."""
        needs_reconnect = False
        needs_timer_reset = False

        for param in params:
             if param.name == 'simulate':
                if self.simulate != param.value:
                    self.simulate = param.value
                    needs_reconnect = True
             elif param.name == 'serial_port':
                if self.serial_port != param.value:
                    self.serial_port = param.value
                    needs_reconnect = True
             elif param.name == 'baud_rate':
                if self.baud_rate != param.value:
                    self.baud_rate = param.value
                    needs_reconnect = True
             elif param.name == 'timer_period':
                 if self.timer_period != param.value:
                     self.timer_period = param.value
                     needs_timer_reset = True # Need to recreate timer

        if needs_reconnect:
            self.get_logger().info("GNSS parameters changed, re-establishing connection/mode.")
            self.setup_mode() # Handles timer reset too
        elif needs_timer_reset:
            self.get_logger().info("GNSS timer period changed, resetting timer.")
            if self.timer:
                self.timer.cancel()
            if self.simulate:
                 self.timer = self.create_timer(self.timer_period, self.publish_dummy_data)
            elif self.ser:
                 self.timer = self.create_timer(self.timer_period, self.request_gps_position)


        return SetParametersResult(successful=True)

    def setup_mode(self):
        """Sets up the node based on the simulation parameter."""
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.get_logger().info("Closed GNSS serial port.")

        self.gps_initialized = False
        self.init_step = 0

        if self.simulate:
            self.get_logger().info("Running GNSS in simulation mode, publishing dummy data.")
            self.timer = self.create_timer(self.timer_period, self.publish_dummy_data)
        else:
            if not SERIAL_AVAILABLE:
                self.get_logger().error("Pyserial not found, but simulate=False. Cannot connect to GNSS hardware.")
                return

            try:
                self.get_logger().info(f"Attempting to open GNSS serial port {self.serial_port} at {self.baud_rate} baud.")
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
                self.get_logger().info("GNSS serial port opened successfully.")
                self.timer = self.create_timer(self.timer_period, self.request_gps_position)
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port {self.serial_port}: {e}")
                self.ser = None
            except Exception as e:
                 self.get_logger().error(f"An unexpected error occurred during GNSS serial setup: {e}")


    def publish_dummy_data(self):
        dummy_data = f"$GPGGA,123519.00,{6011.5234},N,{2456.7498},E,1,08,0.9,545.4,M,46.9,M,,*47"
        lat_lon = "Lat: 60.192059, Lon: 24.945831"
        self.get_logger().debug("Publishing dummy GNSS data: " + lat_lon)
        self.publisher.publish(String(data=lat_lon))

    def send_at(self, command, expected_response, timeout_sec):
        """Sends AT command, reads response, checks for expected string."""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn(f"Serial port not available. Cannot send AT command: {command}")
            return False, ""

        rec_buff_bytes = b''
        try:
            self.get_logger().debug(f"Sending AT command: {command}")
            self.ser.reset_input_buffer()
            self.ser.write((command + '\r\n').encode('ascii'))
            start_time = time.monotonic()
            while (time.monotonic() - start_time) < timeout_sec:
                if self.ser.in_waiting > 0:
                    rec_buff_bytes += self.ser.read(self.ser.in_waiting)
                    rec_buff_str = rec_buff_bytes.decode('ascii', errors='ignore')
                    if expected_response in rec_buff_str or "ERROR" in rec_buff_str:
                         break
                else:
                    time.sleep(0.05)


            rec_buff_decoded = rec_buff_bytes.decode('ascii', errors='ignore').strip()
            self.get_logger().debug(f"Response to '{command}': {rec_buff_decoded}")

            if expected_response in rec_buff_decoded:
                return True, rec_buff_decoded
            else:
                self.get_logger().warn(f"AT command '{command}' failed or did not return '{expected_response}'. Response: {rec_buff_decoded}")
                return False, rec_buff_decoded

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error during AT command '{command}': {e}")
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = None
            self.gps_initialized = False
            self.init_step = 0
            return False, ""
        except Exception as e:
             self.get_logger().error(f"Unexpected error during AT command '{command}': {e}")
             return False, ""


    def request_gps_position(self):
        """Timer callback: Manages GNSS initialization and data requests."""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("GNSS serial port not available. Trying to reopen...")
            self.setup_mode()
            return

        now = time.time()
        if now - self.last_cmd_time < 1.5:
             self.get_logger().debug("Waiting before sending next GNSS command...")
             return

        success = False
        response_data = ""

        if self.init_step == 0:
             self.get_logger().info('Starting GPS session initialization...')
             self.init_step = 1

        elif self.init_step == 1:
             self.get_logger().info('Sending AT+CGPS=0 (Turn off GPS)...')
             success, response_data = self.send_at('AT+CGPS=0', 'OK', 2)
             if success:
                 self.init_step = 2

        elif self.init_step == 2:
            self.get_logger().info('Sending AT+CGPS=1 (Turn on GPS)...')
            success, response_data = self.send_at('AT+CGPS=1', 'OK', 2)
            if success:
                self.init_step = 3
                self.gps_initialized = True
                self.get_logger().info('GNSS module initialized.')

        elif self.init_step == 3 and self.gps_initialized:
            self.get_logger().debug('Requesting GPS info (AT+CGPSINFO)...')
            success, response_data = self.send_at('AT+CGPSINFO', '+CGPSINFO:', 2)
            if success:
                # Example response: +CGPSINFO: 6011.5234,N,02456.7498,E,160425,123519.00,545.4,0.0,
                # Check if the response contains valid coordinates (not just commas)
                # Remove the prefix "+CGPSINFO: "
                info_part = response_data.split('+CGPSINFO:', 1)[-1].strip()
                if ',,,,,,' in info_part or info_part == "":
                     self.get_logger().info('GNSS fix not ready yet.')
                else:
                    self.get_logger().info(f"GNSS data received: {info_part}")
                    self.publisher.publish(String(data=info_part))

        else:
             self.get_logger().warn("GNSS in unexpected state, resetting initialization.")
             self.init_step = 0
             self.gps_initialized = False


        self.last_cmd_time = now

    def destroy_node(self):
        self.get_logger().info("Shutting down GNSS node...")
        if self.timer:
            self.timer.cancel()
        if self.ser is not None and self.ser.is_open:
            try:
                 self.get_logger().info("Sending AT+CGPS=0 before closing port...")
                 self.send_at('AT+CGPS=0', 'OK', 1)
            except Exception as e:
                 self.get_logger().warn(f"Failed to send final AT+CGPS=0: {e}")
            finally:
                 self.ser.close()
                 self.get_logger().info("Closed GNSS serial port.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GPSNode()
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
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()