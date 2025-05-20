#!/usr/bin/env python3
import os
import unittest
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32 # Added Int32 for encoder

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing

# Define default values that might be used across nodes for consistency in tests
DEFAULT_NEUTRAL_TICKS = 10890 # Max ticks / 2 =>  21 781 / 2 = 10 890

def generate_test_description():
    # Common parameters for nodes, assuming wall time for tests
    common_parameters = [{'use_sim_time': False}]

    # Sensor nodes (as before)
    ctd_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='ctd.py',
        name='ctd_node',
        output='screen',
        parameters=[{'simulate': True}] + common_parameters
    )
    dvl_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='dvl.py',
        name='dvl_node',
        output='screen',
        parameters=[{'simulate': True}] + common_parameters
    )
    gnss_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='gnss.py',
        name='gnss_node',
        output='screen',
        parameters=[{'simulate': True}] + common_parameters
    )

    # New nodes for motor control and simulation
    float_simulator_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='float_simulator.py',
        name='float_simulator_node',
        output='screen',
        parameters=[
            {'motor_control_mode_active': True}, # Test interaction with motorControl
            {'neutral_buoyancy_ticks': DEFAULT_NEUTRAL_TICKS},
            # other float_simulator params can use defaults or be set here
        ] + common_parameters
    )

    init_neutral_buoyancy_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='init_neutral_buoyancy.py',
        name='init_neutral_buoyancy_node',
        output='screen',
        parameters=[
            {'neutral_buoyancy_encoder_target': DEFAULT_NEUTRAL_TICKS}
        ] + common_parameters
    )

    motor_controller_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='motorControl.py',
        name='motor_controller_node',
        output='screen',
        parameters=[
            {'motor_control_active': True}, # Activate its logic (will use mock GPIO)
            {'neutral_buoyancy_ticks': DEFAULT_NEUTRAL_TICKS},
        ] + common_parameters
    )

    return launch.LaunchDescription([
        ctd_node,
        dvl_node,
        gnss_node,
        float_simulator_node,
        init_neutral_buoyancy_node,
        motor_controller_node,
        launch_testing.actions.ReadyToTest()
    ]), {
        'ctd_node': ctd_node,
        'dvl_node': dvl_node,
        'gnss_node': gnss_node,
        'float_simulator_node': float_simulator_node,
        'init_neutral_buoyancy_node': init_neutral_buoyancy_node,
        'motor_controller_node': motor_controller_node,
    }

# --- Dummy Subscribers/Publishers for new tests ---

class DummyEncoderSubscriber(Node):
    def __init__(self, topic_name='/current_encoder_position'):
        super().__init__('dummy_encoder_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            topic_name,
            self.listener_callback,
            10)
        self.received_data = []
        self.get_logger().info(f"Dummy Encoder subscriber started on {topic_name}.")

    def listener_callback(self, msg):
        self.received_data.append(msg.data)
        self.get_logger().info(f"EncoderSub - Received encoder data: {msg.data}")

class DummyVzSubscriber(Node):
    def __init__(self, topic_name='/sim_dvl_raw_vz'): # Assuming this is the vertical speed from simulator
        super().__init__('dummy_vz_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            topic_name,
            self.listener_callback,
            10)
        self.received_data = []
        self.get_logger().info(f"Dummy Vz subscriber started on {topic_name}.")

    def listener_callback(self, msg):
        self.received_data.append(msg.data)
        self.get_logger().info(f"VzSub - Received Vz data: {msg.data}")

class DummyEncoderPublisher(Node):
    def __init__(self, topic_name='/current_encoder_position'):
        super().__init__('dummy_encoder_publisher')
        self.publisher_ = self.create_publisher(Int32, topic_name, 10)
        self.get_logger().info(f"Dummy Encoder publisher started on {topic_name}.")

    def publish(self, value):
        msg = Int32()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f"EncoderPub - Published encoder data: {value}")


# --- Existing Dummy Subscribers (CTD, DVL, GNSS) ---
class DummyCTDSubscriber(Node):
    def __init__(self):
        super().__init__('dummy_ctd_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'temperature', # Assuming ctd_node publishes 'temperature'
            self.listener_callback,
            10)
        self.received_data = []
        self.get_logger().info("Dummy CTD subscriber started.")

    def listener_callback(self, msg):
        self.received_data.append(msg.data)
        self.get_logger().info(f"CTD - Received temperature: {msg.data}")

class DummyDVLSubscriber(Node):
    def __init__(self):
        super().__init__('dummy_dvl_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'vx', # Assuming dvl_node publishes 'vx'
            self.listener_callback,
            10)
        self.received_data = []
        self.get_logger().info("Dummy DVL subscriber started.")

    def listener_callback(self, msg):
        self.received_data.append(msg.data)
        self.get_logger().info(f"DVL - Received vx: {msg.data}")

class DummyGNSSSubscriber(Node):
    def __init__(self):
        super().__init__('dummy_gnss_subscriber')
        self.subscription = self.create_subscription(
            String, # Assuming gnss_node publishes String for 'gps_data'
            'gps_data',
            self.listener_callback,
            10)
        self.received_data = []
        self.get_logger().info("Dummy GNSS subscriber started.")

    def listener_callback(self, msg):
        self.received_data.append(msg.data)
        self.get_logger().info(f"GNSS - Received gps_data: {msg.data}")


# --- Test Classes ---
@launch_testing.post_shutdown_test()
class TestAllNodesSimulated(unittest.TestCase):

    # Existing tests for CTD, DVL, GNSS (ensure topics match what nodes publish)
    def test_ctd_dummy_data_publishing(self, proc_output, ctd_node): # Removed unused dvl_node, gnss_node
        rclpy.init()
        try:
            node = DummyCTDSubscriber()
            end_time = time.time() + 5.0
            while time.time() < end_time and not node.received_data:
                 rclpy.spin_once(node, timeout_sec=0.1)
            self.assertGreater(len(node.received_data), 0, "No dummy temperature data received from CTD node.")
            if node.received_data: self.assertIsInstance(node.received_data[0], float)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_dvl_dummy_data_publishing(self, proc_output, dvl_node): # Removed unused ctd_node, gnss_node
        rclpy.init()
        try:
            node = DummyDVLSubscriber()
            end_time = time.time() + 5.0
            while time.time() < end_time and not node.received_data:
                 rclpy.spin_once(node, timeout_sec=0.1)
            self.assertGreater(len(node.received_data), 0, "No dummy vx data received from DVL node.")
            if node.received_data: self.assertIsInstance(node.received_data[0], float)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_gnss_dummy_data_publishing(self, proc_output, gnss_node): # Removed unused ctd_node, dvl_node
        rclpy.init()
        try:
            node = DummyGNSSSubscriber()
            end_time = time.time() + 5.0
            while time.time() < end_time and not node.received_data:
                 rclpy.spin_once(node, timeout_sec=0.1)
            self.assertGreater(len(node.received_data), 0, "No dummy gps_data received from GNSS node.")
            if node.received_data:
                self.assertIsInstance(node.received_data[0], str)
                self.assertIn("Lat:", node.received_data[0])
                self.assertIn("Lon:", node.received_data[0])
        finally:
            node.destroy_node()
            rclpy.shutdown()

    # --- New tests for motor and simulator nodes ---

    def test_init_neutral_buoyancy_publishes_encoder(self, proc_output, init_neutral_buoyancy_node):
        rclpy.init()
        try:
            node = DummyEncoderSubscriber(topic_name='/current_encoder_position')
            end_time = time.time() + 7.0 # Give it a bit more time to initialize and publish
            initial_value_received = False
            while time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.received_data:
                    # With mock GPIO, it should publish its starting value (0) or the target if the mock was more advanced.
                    # Given the current mock, it will likely publish 0 repeatedly or get stuck.
                    # We are checking if *any* Int32 is published.
                    self.assertIsInstance(node.received_data[0], int)
                    initial_value_received = True
                    break
            self.assertTrue(initial_value_received, "No encoder data received from init_neutral_buoyancy_node.")
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_motor_controller_publishes_encoder(self, proc_output, motor_controller_node):
        rclpy.init()
        try:
            node = DummyEncoderSubscriber(topic_name='/current_encoder_position')
            end_time = time.time() + 5.0
            initial_value_received = False
            while time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.received_data:
                    # Expected to publish its initial encoder count (neutral_buoyancy_ticks).
                    self.assertIsInstance(node.received_data[0], int)
                    # self.assertEqual(node.received_data[0], DEFAULT_NEUTRAL_TICKS) # This might be too strict if init node overrides it first
                    initial_value_received = True
                    break
            self.assertTrue(initial_value_received, "No encoder data received from motor_controller_node.")
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_float_simulator_responds_to_encoder(self, proc_output, float_simulator_node):
        rclpy.init()
        encoder_pub_node = None
        vz_sub_node = None
        try:
            encoder_pub_node = DummyEncoderPublisher(topic_name='/current_encoder_position')
            vz_sub_node = DummyVzSubscriber(topic_name='/sim_dvl_raw_vz') # Check correct Vz topic

            # Let nodes initialize and simulator to publish initial Vz (likely 0)
            time.sleep(2) # Allow initial publications
            rclpy.spin_once(encoder_pub_node, timeout_sec=0.1)
            rclpy.spin_once(vz_sub_node, timeout_sec=0.1)
            vz_sub_node.received_data.clear() # Clear any initial Vz

            # Publish an encoder position that should cause ascent (e.g., less than neutral)
            # Min ticks is 0, neutral is DEFAULT_NEUTRAL_TICKS (e.g. 10890)
            # Max speed at limits is 0.5 m/s
            # Speed = ( (neutral - encoder) / (neutral - min) ) * max_speed
            # If encoder = 0, speed = max_speed (0.5)
            test_encoder_value_ascent = DEFAULT_NEUTRAL_TICKS // 2 # Should cause positive Vz
            encoder_pub_node.publish(test_encoder_value_ascent)

            end_time = time.time() + 5.0
            ascent_vz_received = False
            while time.time() < end_time:
                rclpy.spin_once(encoder_pub_node, timeout_sec=0.05) # Keep publishing
                rclpy.spin_once(vz_sub_node, timeout_sec=0.1)
                if vz_sub_node.received_data:
                    # self.get_logger().info(f"Simulator Vz received: {vz_sub_node.received_data[-1]}")
                    if vz_sub_node.received_data[-1] > 0.01: # Check for significant positive speed
                        ascent_vz_received = True
                        break
                    # else: # Keep clearing if it's still zero from before
                        # vz_sub_node.received_data.clear()
                encoder_pub_node.publish(test_encoder_value_ascent) # Re-publish periodically

            self.assertTrue(ascent_vz_received, "Float simulator did not show positive Vz for ascent encoder value.")
            if vz_sub_node.received_data : self.assertGreater(vz_sub_node.received_data[-1], 0.0, "Expected positive Vz for ascent.")

            # Clear and test descent
            vz_sub_node.received_data.clear()
            time.sleep(0.5)

            test_encoder_value_descent = DEFAULT_NEUTRAL_TICKS + ( (float_simulator_node.get_parameter('max_encoder_ticks').value - DEFAULT_NEUTRAL_TICKS) // 2 )
            encoder_pub_node.publish(int(test_encoder_value_descent))

            end_time = time.time() + 5.0
            descent_vz_received = False
            while time.time() < end_time:
                rclpy.spin_once(encoder_pub_node, timeout_sec=0.05)
                rclpy.spin_once(vz_sub_node, timeout_sec=0.1)
                if vz_sub_node.received_data:
                     # self.get_logger().info(f"Simulator Vz received: {vz_sub_node.received_data[-1]}")
                    if vz_sub_node.received_data[-1] < -0.01: # Check for significant negative speed
                        descent_vz_received = True
                        break
                    # else:
                        # vz_sub_node.received_data.clear()
                encoder_pub_node.publish(int(test_encoder_value_descent))


            self.assertTrue(descent_vz_received, "Float simulator did not show negative Vz for descent encoder value.")
            if vz_sub_node.received_data : self.assertLess(vz_sub_node.received_data[-1], 0.0, "Expected negative Vz for descent.")

        finally:
            if encoder_pub_node: encoder_pub_node.destroy_node()
            if vz_sub_node: vz_sub_node.destroy_node()
            rclpy.shutdown()