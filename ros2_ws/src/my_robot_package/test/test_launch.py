import os
import unittest
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing

def generate_test_description():
    # Launch each node with simulation enabled.
    ctd_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='ctd.py',
        name='ctd_node',        # Corrected name
        output='screen',
        parameters=[{'simulate': True}]
    )
    dvl_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='dvl.py',
        name='dvl_node',        # Corrected name
        output='screen',
        parameters=[{'simulate': True}]
    )
    gnss_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='gnss.py',
        name='gnss_node',       # Corrected name
        output='screen',
        parameters=[{'simulate': True}]
    )

    return launch.LaunchDescription([
        ctd_node,
        dvl_node,
        gnss_node,
        launch_testing.actions.ReadyToTest()
    ]), {
        'ctd_node': ctd_node,
        'dvl_node': dvl_node,
        'gnss_node': gnss_node
    }

# Dummy subscriber for CTD node (listens to 'temperature')
class DummyCTDSubscriber(Node):
    def __init__(self):
        super().__init__('dummy_ctd_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'temperature', # Topic name from ctd.py
            self.listener_callback,
            10)
        self.received_data = [] # Store multiple messages
        self.get_logger().info("Dummy CTD subscriber started.")


    def listener_callback(self, msg):
        self.received_data.append(msg.data)
        self.get_logger().info(f"CTD - Received temperature: {msg.data}")

# Dummy subscriber for DVL node (listens to 'vx')
class DummyDVLSubscriber(Node):
    def __init__(self):
        super().__init__('dummy_dvl_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'vx',             # Topic name from dvl.py
            self.listener_callback,
            10)
        self.received_data = [] # Store multiple messages
        self.get_logger().info("Dummy DVL subscriber started.")

    def listener_callback(self, msg):
        self.received_data.append(msg.data)
        self.get_logger().info(f"DVL - Received vx: {msg.data}")

# Dummy subscriber for GNSS node (listens to 'gps_data')
class DummyGNSSSubscriber(Node):
    def __init__(self):
        super().__init__('dummy_gnss_subscriber')
        self.subscription = self.create_subscription(
            String,
            'gps_data',       # Topic name from gnss.py
            self.listener_callback,
            10)
        self.received_data = [] # Store multiple messages
        self.get_logger().info("Dummy GNSS subscriber started.")

    def listener_callback(self, msg):
        self.received_data.append(msg.data)
        self.get_logger().info(f"GNSS - Received gps_data: {msg.data}")

# --- Test Classes ---
# Use a single context for initializing rclpy once for all tests
@launch_testing.post_shutdown_test()
class TestAllNodesSimulated(unittest.TestCase):

    def test_ctd_dummy_data_publishing(self, proc_output, ctd_node, dvl_node, gnss_node):
        rclpy.init()
        try:
            node = DummyCTDSubscriber()
            # Spin for a few seconds to allow messages to arrive
            end_time = time.time() + 5.0
            while time.time() < end_time:
                 rclpy.spin_once(node, timeout_sec=0.1)
                 if len(node.received_data) > 0:
                     break # Exit loop once data is received

            self.assertGreater(len(node.received_data), 0, "No dummy temperature data received from CTD node.")
            # Check if received data is float (basic check)
            self.assertIsInstance(node.received_data[0], float)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_dvl_dummy_data_publishing(self, proc_output, ctd_node, dvl_node, gnss_node):
        rclpy.init()
        try:
            node = DummyDVLSubscriber()
            end_time = time.time() + 5.0
            while time.time() < end_time:
                 rclpy.spin_once(node, timeout_sec=0.1)
                 if len(node.received_data) > 0:
                     break

            self.assertGreater(len(node.received_data), 0, "No dummy vx data received from DVL node.")
            self.assertIsInstance(node.received_data[0], float)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_gnss_dummy_data_publishing(self, proc_output, ctd_node, dvl_node, gnss_node):
        rclpy.init()
        try:
            node = DummyGNSSSubscriber()
            end_time = time.time() + 5.0
            while time.time() < end_time:
                 rclpy.spin_once(node, timeout_sec=0.1)
                 if len(node.received_data) > 0:
                     break

            self.assertGreater(len(node.received_data), 0, "No dummy gps_data received from GNSS node.")
            self.assertIsInstance(node.received_data[0], str)
            # Check if the string contains expected parts
            self.assertIn("Lat:", node.received_data[0])
            self.assertIn("Lon:", node.received_data[0])
        finally:
            node.destroy_node()
            rclpy.shutdown()