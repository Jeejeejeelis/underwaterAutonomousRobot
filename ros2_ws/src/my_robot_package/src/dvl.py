import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import socket
import time
import os
import logging
import json
import csv
from datetime import datetime, timezone
from functools import wraps, partial
import asyncio

TCP_IP = "192.168.194.95"
TCP_PORT = 16171
deviceid = "SUB"
save_locally = True
dataJson = b''

def async_wrap(func):
    @wraps(func)
    async def run(*args, loop=None, executor=None, **kwargs):
        if loop is None:
            loop = asyncio.get_event_loop()
        pfunc = partial(func, *args, **kwargs)
        return await loop.run_in_executor(executor, pfunc)
    return run

class DVLNode(Node):
    def __init__(self):
        super().__init__('dvl_node')
        self.publisher_vx = self.create_publisher(Float32, 'vx', 10)
        self.publisher_vy = self.create_publisher(Float32, 'vy', 10)
        self.publisher_vz = self.create_publisher(Float32, 'vz', 10)
        self.publisher_altitude = self.create_publisher(Float32, 'altitude', 10)
        self.subscription = self.create_subscription(String, 'send_dvl_command', self.send_dvl_callback, 10)
        
        # Add simulation parameter (default: False)
        self.declare_parameter('simulate', False)
        self.simulate = self.get_parameter('simulate').value

        if self.simulate:
            self.get_logger().info("Running DVL in simulation mode, publishing dummy data.")
            self.timer = self.create_timer(1.0, self.publish_dummy_data)
        else:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connect(TCP_IP, TCP_PORT)
            self.timer = self.create_timer(1.0, self.read_dvl)

    def publish_dummy_data(self):
        # Publish dummy values for testing
        dummy_vx = 1.0
        dummy_vy = 2.0
        dummy_vz = 0.5
        dummy_altitude = 10.0
        self.get_logger().info("Publishing dummy DVL data")
        self.publisher_vx.publish(Float32(data=dummy_vx))
        self.publisher_vy.publish(Float32(data=dummy_vy))
        self.publisher_vz.publish(Float32(data=dummy_vz))
        self.publisher_altitude.publish(Float32(data=dummy_altitude))

    @async_wrap
    def connect(self, host, port):
        try:
            self.sock.connect((host, port))
            self.get_logger().info('Successful Connection to DVL')
        except:
            self.get_logger().info('Connection Failed DVL')

    def send_dvl_callback(self, msg):
        cmd_string = msg.data
        self.send_dvl(cmd_string)

    def read_dvl(self):
        global dataJson, deviceid, save_locally
        vx = 0
        vy = 0
        vz = 0
        altitude = 0
        measurement_cnt = 0
        dataJson = b''
        rawdata = b''
        data = b''
        time_delta = 0
        self.get_logger().info('Read DVL')
        while True:
            self.get_logger().info('Read DVL - Loop')
            rawdata = b''
            while not b'\n' in rawdata:
                try:
                    data = self.sock.recv(1)
                    if len(data) == 0:
                        self.get_logger().info('DVL socket lost, no data, reconnecting')
                        self.connect(TCP_IP, TCP_PORT)
                        continue
                except:
                    self.get_logger().info('Lost connection to DVL, reconnecting')
                    self.connect(TCP_IP, TCP_PORT)
                    continue
                rawdata = rawdata + data

            rawdata = dataJson + rawdata
            dataJson = b''
            strdata = rawdata.decode("utf-8").split('\n')
            dJson = strdata[1]
            strdata = strdata[0]
            self.get_logger().info("DVL message received")
            jsondata = json.loads(strdata)

            if "time" in jsondata:
                try:
                    if save_locally:
                        self.write_csv_data(jsondata)
                except:
                    self.get_logger().info('Fails to write CSV')

                time_delta += jsondata["time"] / 1000.0
                if jsondata["velocity_valid"]:
                    vx += jsondata["vx"]
                    vy += jsondata["vy"]
                    vz += jsondata["vz"]
                    altitude += jsondata["altitude"]
                    measurement_cnt += 1
                    if time_delta >= 1.0:
                        message = {
                            "deviceid": deviceid,
                            "timestamp": datetime.now(timezone.utc).isoformat(),
                            "vx": vx / measurement_cnt,
                            "vy": vy / measurement_cnt,
                            "vz": vz / measurement_cnt,
                            "altitude": altitude / measurement_cnt,
                            "measurements": measurement_cnt
                        }
                        time_delta = 0
                        measurement_cnt = 0
                        vx = 0
                        vy = 0
                        vz = 0
                        self.get_logger().info("Message collected")
                        self.publisher_vx.publish(Float32(data=message["vx"]))
                        self.publisher_vy.publish(Float32(data=message["vy"]))
                        self.publisher_vz.publish(Float32(data=message["vz"]))
                        self.publisher_altitude.publish(Float32(data=message["altitude"]))

            if "ts" in jsondata:
                self.get_logger().info("IMU message received")
                try:
                    if save_locally:
                        self.write_csv_data(jsondata)
                except:
                    self.get_logger().info('Fails to write CSV')

    def send_dvl(self, cmd_string):
        self.sock.sendall(cmd_string.encode())
        response = self.sock.recv(1024)
        self.get_logger().info(f"Response: {response}")

    def write_csv_data(self, jsondata):
        # Implement your CSV writing logic here
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DVLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()