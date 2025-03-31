import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import time
import serial
import binascii
import codecs
import math

class CTDNode(Node):
    def __init__(self):
        super().__init__('ctd_node')
        simulate = self.declare_parameter('simulate', False).value  # Use a ROS2 parameter
        self.publisher_temp = self.create_publisher(Float32, 'temperature', 10)
        self.publisher_pressure = self.create_publisher(Float32, 'pressure', 10)
        self.publisher_conductivity = self.create_publisher(Float32, 'conductivity', 10)
        self.subscription = self.create_subscription(String, 'send_ctd_command', self.send_ctd_callback, 10)

        if simulate:
            self.get_logger().info("Running in simulation mode, publishing dummy CTD data.")
            self.timer = self.create_timer(1.0, self.publish_dummy_data)
        else:
            import serial
            self.ser = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=3.0)
            self.timer = self.create_timer(1.0, self.read_ctd_values)

    def publish_dummy_data(self):
        # Publish some dummy values
        self.publisher_temp.publish(Float32(data=20.0))
        self.publisher_pressure.publish(Float32(data=1013.25))
        self.publisher_conductivity.publish(Float32(data=3.5))

    def send_ctd_callback(self, msg):
        cmd_string = msg.data
        self.send_ctd(cmd_string)

    def read_ctd_values(self):
        response = self.ser.read(32)
        res, temperature, pressure, conductivity = self.parse_ctd_response(response)
        if res != -1:
            self.publisher_temp.publish(Float32(data=temperature))
            self.publisher_pressure.publish(Float32(data=pressure))
            self.publisher_conductivity.publish(Float32(data=conductivity))

    def parse_ctd_response(self, ctd_response):
        result = str(ctd_response).find('$AQCTD')
        if result == -1:
            self.get_logger().info("CTD start string not found")
            return -1, 0, 0, 0
        try:
            values = str(ctd_response).split(',')
            temperature = float(values[1])
            pressure = float(values[2])
            tmp_conductivity = values[3].replace('\r', '')
            conductivity = float(tmp_conductivity.split('*')[0])
            return 0, temperature, pressure, conductivity
        except:
            self.get_logger().info("Fails to extract CTD values")
            return -1, 0, 0, 0

    def send_ctd(self, cmd_string):
        self.ser.write(serial.to_bytes([0xff, 0xff, 0xff, 0xff, 0xaa, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6c]))
        response = self.ser.read(32)
        self.read_ctd_values(response)

def main(args=None):
    rclpy.init(args=args)
    node = CTDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()