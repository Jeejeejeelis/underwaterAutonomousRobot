#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
import time

try:
    import RPi.GPIO as GPIO
    REAL_GPIO_AVAILABLE = True
except (RuntimeError, ModuleNotFoundError, ImportError):
    REAL_GPIO_AVAILABLE = False
    # ... (GPIOMock remains the same) ...
    class GPIOMock:
        BCM = "BCM_MODE"; OUT = "OUT_MODE"; IN = "IN_MODE"; PUD_UP = "PUD_UP_MODE"
        LOW = False; HIGH = True; BOTH = "BOTH_EDGES"
        _mock_motor_pin_config_ref = None
        def __init__(self):
            self.pin_states = {}
            self.log = []
            if GPIOMock._mock_motor_pin_config_ref is None:
                 try: GPIOMock._mock_motor_pin_config_ref = MOTOR_PIN_CONFIG
                 except NameError: pass
        def setmode(self, mode): self.log.append(f"Mock: setmode({mode})")
        def setup(self, pin, mode, pull_up_down=None): self.log.append(f"Mock: setup({pin}, {mode}, pud={pull_up_down})")
        def output(self, pin, state): self.pin_states[pin] = state; self.log.append(f"Mock: output({pin}, {state})")
        def input(self, pin):
            config_to_use = GPIOMock._mock_motor_pin_config_ref
            if config_to_use:
                 if pin == config_to_use.get('UP_LIMIT_PIN', -1) : return self.HIGH 
                 if pin == config_to_use.get('DOWN_LIMIT_PIN', -1) : return self.HIGH
            return self.LOW
        def add_event_detect(self, pin, edge, callback=None, bouncetime=None): self.log.append(f"Mock: add_event_detect({pin})")
        def remove_event_detect(self, pin): self.log.append(f"Mock: remove_event_detect({pin})")
        def cleanup(self): self.log.append("Mock: cleanup()")
    GPIO = GPIOMock()
    if not REAL_GPIO_AVAILABLE: print("Mock RPi.GPIO initialized for motorControl.py as real GPIO is not available.")


MOTOR_PIN_CONFIG = {
    'UP_PIN': 5,
    'DOWN_PIN': 6,
    'ENCODER_PIN': 24,
    'UP_LIMIT_PIN': 19,
    'DOWN_LIMIT_PIN': 26,
}
if not REAL_GPIO_AVAILABLE:
    if GPIOMock._mock_motor_pin_config_ref is None:
        GPIOMock._mock_motor_pin_config_ref = MOTOR_PIN_CONFIG

current_encoder_count = 0 
motor_movement_direction = 0

def encoder_tick_callback(channel):
    global current_encoder_count, motor_movement_direction
    if motor_movement_direction == 1:
        current_encoder_count += 1
    elif motor_movement_direction == -1:
        current_encoder_count -= 1

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.get_logger().info("MotorControllerNode starting...")

        self.declare_parameter('motor_control_active', True)
        self.declare_parameter('neutral_buoyancy_ticks', 10876)
        self.declare_parameter('max_encoder_ticks', 21753)
        self.declare_parameter('min_encoder_ticks', 0)
        self.declare_parameter('depth_tolerance_m', 0.1)
        self.declare_parameter('control_loop_rate_hz', 10.0)

        self.motor_control_active = self.get_parameter('motor_control_active').get_parameter_value().bool_value
        self.neutral_ticks = self.get_parameter('neutral_buoyancy_ticks').get_parameter_value().integer_value
        self.max_ticks = self.get_parameter('max_encoder_ticks').get_parameter_value().integer_value
        self.min_ticks = self.get_parameter('min_encoder_ticks').get_parameter_value().integer_value
        self.depth_tolerance = self.get_parameter('depth_tolerance_m').get_parameter_value().double_value
        
        global current_encoder_count
        current_encoder_count = self.neutral_ticks 
        self.get_logger().info(f"MotorControl: Initial current_encoder_count set to neutral_ticks: {current_encoder_count}")

        self.target_depth_m = None
        self.current_depth_m = None
        self.motor_is_moving = False

        self.initial_encoder_value_received = False
        self.target_depth_value_received = False
        self.current_depth_value_received = False
        self.ready_to_control = False

        self.target_depth_sub = self.create_subscription(Float32, '/target_depth', self.target_depth_callback, 10)
        self.current_depth_sub = self.create_subscription(Float32, 'drone_depth', self.current_depth_callback, 10) 
        self.initial_encoder_sub = self.create_subscription(Int32, '/current_encoder_position', self.initial_encoder_callback, rclpy.qos.qos_profile_sensor_data)
        self.encoder_publisher = self.create_publisher(Int32, '/current_encoder_position', 10)
        self.motor_status_publisher = self.create_publisher(Bool, '/motor_moving_status', 10)

        loop_rate = self.get_parameter('control_loop_rate_hz').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / loop_rate, self.control_loop)
        self.get_logger().info(f"MotorControllerNode initialized. Waiting for inputs to become ready...")

    def _try_set_ready_to_control(self):
        if not self.ready_to_control and self.initial_encoder_value_received and self.target_depth_value_received and self.current_depth_value_received:
            self.ready_to_control = True
            self.get_logger().info("MotorControl: All initial inputs received. Ready to control motor.")
            if self.motor_control_active:
                self.get_logger().info("MotorControl: Initializing GPIOs for this node's use.")
                self._init_gpio()

    def _init_gpio(self):
        # ... (function is the same) ...
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(MOTOR_PIN_CONFIG['ENCODER_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(MOTOR_PIN_CONFIG['ENCODER_PIN'], GPIO.BOTH, callback=encoder_tick_callback, bouncetime=2)
            GPIO.setup(MOTOR_PIN_CONFIG['UP_LIMIT_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(MOTOR_PIN_CONFIG['DOWN_LIMIT_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self._gpio_initialized_flag = True
            self.get_logger().info("GPIO initialized successfully for MotorControllerNode.")
        except Exception as e:
            self.get_logger().fatal(f"CRITICAL: GPIO initialization failed in MotorControllerNode: {e}")
            self.motor_control_active = False 

    def initial_encoder_callback(self, msg):
        # ... (function is the same) ...
        global current_encoder_count
        if not self.initial_encoder_value_received:
            self.get_logger().info(f"MotorControl: Received initial encoder position: {msg.data}")
            current_encoder_count = msg.data
            self.initial_encoder_value_received = True
            self._try_set_ready_to_control()

    def target_depth_callback(self, msg):
        # ... (function is the same) ...
        self.target_depth_m = msg.data
        if not self.target_depth_value_received:
            self.get_logger().info(f"MotorControl: Received first target depth: {msg.data:.2f}m")
            self.target_depth_value_received = True
            self._try_set_ready_to_control()

    def current_depth_callback(self, msg):
        # --- FIX: Ensure current_depth_m is always positive ---
        # The /drone_depth topic provides a negative Z value.
        # We need a positive depth to correctly compare with the positive target_depth_m.
        self.current_depth_m = abs(msg.data)
        
        if not self.current_depth_value_received:
            self.current_depth_value_received = True
            self._try_set_ready_to_control()

    def control_loop(self):
        # ... (The rest of the file is the same) ...
        global current_encoder_count, motor_movement_direction
        
        enc_msg = Int32()
        enc_msg.data = int(round(current_encoder_count)) 
        self.encoder_publisher.publish(enc_msg)

        motor_stat_msg = Bool()
        motor_stat_msg.data = self.motor_is_moving
        self.motor_status_publisher.publish(motor_stat_msg)

        if not self.motor_control_active or not self.ready_to_control:
            return
        
        if self.target_depth_m is None or self.current_depth_m is None:
            if self.motor_is_moving: self._stop_motor_gpio()
            return
        
        depth_error = self.target_depth_m - self.current_depth_m

        if abs(depth_error) <= self.depth_tolerance:
            if self.motor_is_moving:
                self.get_logger().info(f"MotorControl: Target depth reached. Stopping motor.")
                self._stop_motor_gpio()
        else:
            if depth_error > 0:
                if current_encoder_count < self.max_ticks:
                    self._command_descend_gpio()
                else:
                    if self.motor_is_moving: self._stop_motor_gpio()
            else:
                if current_encoder_count > self.min_ticks:
                    self._command_ascent_gpio()
                else:
                    if self.motor_is_moving: self._stop_motor_gpio()

    def _command_ascent_gpio(self): 
        global motor_movement_direction
        if not self.motor_is_moving or motor_movement_direction != -1:
            self.get_logger().info("MotorControl: Commanding ASCEND.")
            GPIO.output(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.LOW)
            GPIO.output(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.HIGH)
            motor_movement_direction = -1
            self.motor_is_moving = True

    def _command_descend_gpio(self): 
        global motor_movement_direction
        if not self.motor_is_moving or motor_movement_direction != 1:
            self.get_logger().info("MotorControl: Commanding DESCEND.")
            GPIO.output(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.LOW)
            GPIO.output(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.HIGH)
            motor_movement_direction = 1
            self.motor_is_moving = True

    def _stop_motor_gpio(self):
        global motor_movement_direction
        if self.motor_is_moving:
            self.get_logger().info("MotorControl: Commanding STOP.")
            GPIO.output(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.LOW)
            GPIO.output(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.LOW)
            motor_movement_direction = 0
            self.motor_is_moving = False

    def on_shutdown(self):
        self.get_logger().info("Shutting down MotorControllerNode...") 
        if self.motor_control_active and hasattr(self, '_gpio_initialized_flag'):
            self.get_logger().info("Attempting GPIO cleanup for MotorControllerNode.") 
            self._stop_motor_gpio()
            GPIO.cleanup()
            self.get_logger().info("GPIO cleanup complete for MotorControllerNode.")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorControllerNode() 
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info('Keyboard interrupt, shutting down MotorControllerNode...')
    except Exception as e:
        if node: node.get_logger().fatal(f"Unhandled exception in MotorControllerNode: {e}")
        else: print(f"Exception before node was initialized in MotorControllerNode: {e}")
    finally:
        if node:
            node.on_shutdown() 
            node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()