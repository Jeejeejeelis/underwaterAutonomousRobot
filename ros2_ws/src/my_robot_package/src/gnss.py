import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher = self.create_publisher(String, 'gps_data', 10)
        
        # Add simulation parameter (default: False)
        self.declare_parameter('simulate', False)
        self.simulate = self.get_parameter('simulate').value

        if self.simulate:
            self.get_logger().info("Running GNSS in simulation mode, publishing dummy data.")
            self.timer = self.create_timer(1.0, self.publish_dummy_data)
        else:
            self.ser = serial.Serial("/dev/ttyUSB2", 115200)
            self.timer = self.create_timer(1.0, self.get_gps_position)

    def publish_dummy_data(self):
        dummy_data = "Lat: 60.192059, Lon: 24.945831"
        self.get_logger().info("Publishing dummy GNSS data: " + dummy_data)
        self.publisher.publish(String(data=dummy_data))
        
    def send_at(self, command, back, timeout):
        rec_buff = ''
        self.ser.write((command + '\r\n').encode())
        time.sleep(timeout)
        if self.ser.in_waiting:
            time.sleep(0.01)
            rec_buff = self.ser.read(self.ser.in_waiting())
        if back not in rec_buff.decode():
            self.get_logger().info(f"{command} ERROR")
            self.get_logger().info(f"{command} back:\t{rec_buff.decode()}")
            return 0
        else:
            self.get_logger().info(rec_buff.decode())
            return 1

    def get_gps_position(self):
        rec_null = True
        answer = 0
        self.get_logger().info('Start GPS session...')
        self.send_at('AT+CGPS=0', 'OK', 1)
        self.send_at('AT+CGPS=1', 'OK', 1)
        time.sleep(2)
        while rec_null:
            rec_buff = ''
            answer = self.send_at('AT+CGPSINFO', '+CGPSINFO: ', 1)
            if answer == 1:
                if ',,,,,,' in rec_buff.decode():
                    self.get_logger().info('GPS is not ready')
                    rec_null = False
                    time.sleep(1)
                else:
                    gps_data = rec_buff.decode()
                    self.get_logger().info(f"GPS data: {gps_data}")
                    self.publisher.publish(String(data=gps_data))
                    rec_null = False
            else:
                self.get_logger().info(f"Error: {answer}")
                self.send_at('AT+CGPS=0', 'OK', 1)
                return False
            time.sleep(1.5)

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()