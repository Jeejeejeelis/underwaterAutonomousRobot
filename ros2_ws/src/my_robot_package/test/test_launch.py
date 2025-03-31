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
        name='ctd_node',
        output='screen',
        parameters=[{'simulate': True}]
    )
    dvl_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='dvl.py',
        name='dvl_node',
        output='screen',
        parameters=[{'simulate': True}]
    )
    gnss_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='gnss.py',
        name='gnss_node',
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
            'temperature',
            self.listener_callback,
            10)
        self.received_data = None

    def listener_callback(self, msg):
        self.received_data = msg.data
        self.get_logger().info(f"CTD - Received temperature: {msg.data}")

# Dummy subscriber for DVL node (listens to 'vx')
class DummyDVLSubscriber(Node):
    def __init__(self):
        super().__init__('dummy_dvl_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'vx',
            self.listener_callback,
            10)
        self.received_data = None

    def listener_callback(self, msg):
        self.received_data = msg.data
        self.get_logger().info(f"DVL - Received vx: {msg.data}")

# Dummy subscriber for GNSS node (listens to 'gps_data')
class DummyGNSSSubscriber(Node):
    def __init__(self):
        super().__init__('dummy_gnss_subscriber')
        self.subscription = self.create_subscription(
            String,
            'gps_data',
            self.listener_callback,
            10)
        self.received_data = None

    def listener_callback(self, msg):
        self.received_data = msg.data
        self.get_logger().info(f"GNSS - Received gps_data: {msg.data}")

# Test for CTD node dummy data publishing
class TestCTDNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = DummyCTDSubscriber()
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_ctd_dummy_data_publishing(self):
        start_time = time.time()
        # Wait up to 5 seconds for dummy data
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if self.node.received_data is not None:
                break
        self.assertIsNotNone(self.node.received_data, "No dummy temperature data was received")
        # Expecting dummy temperature of 20.0 as defined in simulation mode of ctd.py
        self.assertEqual(self.node.received_data, 20.0, "The dummy temperature value does not match the expected value")

# Test for DVL node dummy data publishing
class TestDVLNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = DummyDVLSubscriber()
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_dvl_dummy_data_publishing(self):
        start_time = time.time()
        # Wait up to 5 seconds for dummy data
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if self.node.received_data is not None:
                break
        self.assertIsNotNone(self.node.received_data, "No dummy DVL data was received")
        # Expecting dummy vx value of 1.0 as defined in simulation mode of dvl.py
        self.assertEqual(self.node.received_data, 1.0, "The dummy vx value does not match the expected value")

# Test for GNSS node dummy data publishing
class TestGNSSNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = DummyGNSSSubscriber()
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_gnss_dummy_data_publishing(self):
        start_time = time.time()
        # Wait up to 5 seconds for dummy data
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if self.node.received_data is not None:
                break
        self.assertIsNotNone(self.node.received_data, "No dummy GNSS data was received")
        # Expecting dummy GPS data as defined in simulation mode of gnss.py
        expected_gps = "Lat: 60.192059, Lon: 24.945831"
        self.assertEqual(self.node.received_data, expected_gps, "The dummy GNSS data does not match the expected value")