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
    class GPIOMock:
        BCM = "BCM_MODE"; OUT = "OUT_MODE"; IN = "IN_MODE"; PUD_UP = "PUD_UP_MODE"
        LOW = False; HIGH = True; BOTH = "BOTH_EDGES"
        _mock_motor_pin_config_ref = None

        def __init__(self):
            self.pin_states = {}
            self.log = []
            if GPIOMock._mock_motor_pin_config_ref is None:
                 try:
                     GPIOMock._mock_motor_pin_config_ref = MOTOR_PIN_CONFIG
                 except NameError:
                     pass


        def setmode(self, mode): self.log.append(f"Mock: setmode({mode})")
        def setup(self, pin, mode, pull_up_down=None): self.log.append(f"Mock: setup({pin}, {mode}, pud={pull_up_down})")
        def output(self, pin, state): self.pin_states[pin] = state; self.log.append(f"Mock: output({pin}, {state})")
        def input(self, pin):
            config_to_use = GPIOMock._mock_motor_pin_config_ref
            if config_to_use:
                 if pin in [config_to_use['UP_LIMIT_PIN'], config_to_use['DOWN_LIMIT_PIN']]: return self.HIGH
            return self.LOW
        def add_event_detect(self, pin, edge, callback=None, bouncetime=None): self.log.append(f"Mock: add_event_detect({pin})")
        def remove_event_detect(self, pin): self.log.append(f"Mock: remove_event_detect({pin})")
        def cleanup(self): self.log.append("Mock: cleanup()")
    GPIO = GPIOMock()
    print("Mock RPi.GPIO initialized for motorControl.py as real GPIO is not available.")


MOTOR_PIN_CONFIG = {
    'UP_PIN': 5,        # Wire: Brown. Physical Action: Piston IN, Volume DECREASE, UAV DESCENDS.
    'DOWN_PIN': 6,      # Wire: Red.   Physical Action: Piston OUT, Volume INCREASE, UAV ASCENDS.
    'ENCODER_PIN': 24,
    'UP_LIMIT_PIN': 19, # Green wire. Hit when piston is fully IN (min volume).
    'DOWN_LIMIT_PIN': 26, # Yellow wire. Hit when piston is fully OUT (max volume).
}
if not REAL_GPIO_AVAILABLE:
    if GPIOMock._mock_motor_pin_config_ref is None:
        GPIOMock._mock_motor_pin_config_ref = MOTOR_PIN_CONFIG


# Encoder and Motor State
# Encoder INCREASES as piston moves IN (towards UP_LIMIT_PIN, volume decrease)
# Encoder DECREASES as piston moves OUT (towards DOWN_LIMIT_PIN, volume increase)
current_encoder_count = 0
motor_movement_direction = 0  # 1 for Piston IN (Descend, Encoder INC), -1 for Piston OUT (Ascend, Encoder DEC)

def encoder_tick_callback(channel):
    global current_encoder_count, motor_movement_direction
    if motor_movement_direction == 1:    # Piston IN (Descend) -> Encoder INCREASES
        current_encoder_count += 1
    elif motor_movement_direction == -1: # Piston OUT (Ascend) -> Encoder DECREASES
        current_encoder_count -= 1

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        self.declare_parameter('motor_control_active', True)
        self.declare_parameter('neutral_buoyancy_ticks', 10876)
        self.declare_parameter('max_encoder_ticks', 21753)
        self.declare_parameter('min_encoder_ticks', 0)
        self.declare_parameter('depth_kp', 0.1)
        self.declare_parameter('depth_tolerance_m', 0.1)
        self.declare_parameter('control_loop_rate_hz', 10.0)

        self.motor_control_active = self.get_parameter('motor_control_active').value
        self.neutral_ticks = self.get_parameter('neutral_buoyancy_ticks').value
        self.max_ticks = self.get_parameter('max_encoder_ticks').value
        self.min_ticks = self.get_parameter('min_encoder_ticks').value
        self.depth_kp = self.get_parameter('depth_kp').value
        self.depth_tolerance = self.get_parameter('depth_tolerance_m').value
        
        global current_encoder_count
        current_encoder_count = self.neutral_ticks

        self.target_depth_m = None
        self.current_depth_m = None
        self.motor_is_moving = False

        if not REAL_GPIO_AVAILABLE:
            self.get_logger().warn("RPi.GPIO library not found. Motor control will use MOCK GPIO and will not function on hardware.")
            # self.motor_control_active = False
        
        if self.motor_control_active:
            self.get_logger().info("Motor Controller node is ACTIVE and will attempt to use GPIOs.")
            self._init_gpio()
        else:
            self.get_logger().info("Motor Controller node is INACTIVE. No GPIO operations will be performed.")

        self.target_depth_sub = self.create_subscription(
            Float32, '/target_depth', self.target_depth_callback, 10)
        self.current_depth_sub = self.create_subscription(
            Float32, '/drone_depth', self.current_depth_callback, 10)
        self.initial_encoder_sub = self.create_subscription(
            Int32, '/current_encoder_position', self.initial_encoder_callback, rclpy.qos.qos_profile_sensor_data)

        self.encoder_publisher = self.create_publisher(Int32, '/current_encoder_position', 10)
        self.motor_status_publisher = self.create_publisher(Bool, '/motor_moving_status', 10)

        loop_period = 1.0 / self.get_parameter('control_loop_rate_hz').value
        self.timer = self.create_timer(loop_period, self.control_loop)

        self.get_logger().info(f"MotorControllerNode initialized. Active: {self.motor_control_active}. Loop rate: {1.0/loop_period} Hz.")
        self.get_logger().info(f"Pins: DESCEND_PIN (Piston IN, Volume DEC)={MOTOR_PIN_CONFIG['UP_PIN']}, ASCEND_PIN (Piston OUT, Volume INC)={MOTOR_PIN_CONFIG['DOWN_PIN']}")

    def _init_gpio(self):
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(MOTOR_PIN_CONFIG['ENCODER_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(MOTOR_PIN_CONFIG['UP_LIMIT_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(MOTOR_PIN_CONFIG['DOWN_LIMIT_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(MOTOR_PIN_CONFIG['ENCODER_PIN'], GPIO.BOTH, callback=encoder_tick_callback, bouncetime=2)
            time.sleep(1) #IMPORTANT, otherwise nothing works!
            self.get_logger().info("GPIO initialized successfully for MotorControllerNode.")
        except Exception as e:
            self.get_logger().error(f"CRITICAL: GPIO initialization failed in MotorControllerNode: {e}")
            self.motor_control_active = False

    def initial_encoder_callback(self, msg):
        global current_encoder_count
        is_first_update_from_default = (current_encoder_count == self.neutral_ticks and msg.data != self.neutral_ticks)
        if abs(current_encoder_count - msg.data) > 5 or is_first_update_from_default:
            # self.get_logger().info(f"Received initial/updated encoder count: {msg.data}. Updating internal count from {current_encoder_count}.")
            current_encoder_count = msg.data
        # Unsubscribe form the topics:
        # self.destroy_subscription(self.initial_encoder_sub) 
        # self.initial_encoder_sub = None

    def target_depth_callback(self, msg):
        self.target_depth_m = msg.data

    def current_depth_callback(self, msg):
        self.current_depth_m = msg.data

    def control_loop(self):
        global current_encoder_count
        
        enc_msg = Int32()
        enc_msg.data = current_encoder_count
        self.encoder_publisher.publish(enc_msg)

        motor_stat_msg = Bool()
        motor_stat_msg.data = self.motor_is_moving
        self.motor_status_publisher.publish(motor_stat_msg)

        if not self.motor_control_active:
            if self.motor_is_moving: self._stop_motor_gpio()
            return

        if self.target_depth_m is None or self.current_depth_m is None:
            if self.motor_is_moving: self._stop_motor_gpio()
            self.get_logger().debug("Motor controller waiting for valid current and target depth to begin control.", throttle_duration_sec=5)
            return


        depth_error = self.target_depth_m - self.current_depth_m
        upper_limit_physical_hit = GPIO.input(MOTOR_PIN_CONFIG['UP_LIMIT_PIN']) == GPIO.LOW
        lower_limit_physical_hit = GPIO.input(MOTOR_PIN_CONFIG['DOWN_LIMIT_PIN']) == GPIO.LOW

        if abs(depth_error) <= self.depth_tolerance:
            if self.motor_is_moving:
                self.get_logger().info(f"Target depth reached (Error: {depth_error:.2f}m). Stopping motor at encoder {current_encoder_count}.")
                self._stop_motor_gpio()
        else:
            if depth_error > 0:
                if upper_limit_physical_hit or current_encoder_count >= self.max_ticks:
                    if self.motor_is_moving:
                        self.get_logger().warn(f"Cannot go deeper: Upper limit physically hit ({upper_limit_physical_hit}) or max encoder ticks ({current_encoder_count}>={self.max_ticks}). Stopping motor.")
                        self._stop_motor_gpio()
                else:
                    self._command_descend_gpio()
            else:
                if lower_limit_physical_hit or current_encoder_count <= self.min_ticks:
                    if self.motor_is_moving:
                        self.get_logger().warn(f"Cannot go shallower: Lower limit physically hit ({lower_limit_physical_hit}) or min encoder ticks ({current_encoder_count}<={self.min_ticks}). Stopping motor.")
                        self._stop_motor_gpio()
                else:
                    self._command_ascent_gpio()

        # Safety override if limit switches are hit while motor is commanded in that direction
        if upper_limit_physical_hit and motor_movement_direction == 1:
            self.get_logger().warn(f"Upper limit switch ACTIVE while commanding Piston IN! Stopping motor. Encoder: {current_encoder_count}")
            self._stop_motor_gpio()
            current_encoder_count = self.max_ticks

        if lower_limit_physical_hit and motor_movement_direction == -1:
            self.get_logger().warn(f"Lower limit switch ACTIVE while commanding Piston OUT! Stopping motor. Encoder: {current_encoder_count}")
            self._stop_motor_gpio()
            current_encoder_count = self.min_ticks

    def _command_ascent_gpio(self):
        global motor_movement_direction
        if not self.motor_is_moving or motor_movement_direction != -1:
            self.get_logger().debug(f"CMD ASCEND: Piston OUT, Encoder DEC. Activating DOWN_PIN ({MOTOR_PIN_CONFIG['DOWN_PIN']}). CurrentEnc: {current_encoder_count}")
            GPIO.output(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.LOW)
            GPIO.output(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.HIGH)
            motor_movement_direction = -1
            self.motor_is_moving = True

    def _command_descend_gpio(self):
        global motor_movement_direction
        if not self.motor_is_moving or motor_movement_direction != 1:
            self.get_logger().debug(f"CMD DESCEND: Piston IN, Encoder INC. Activating UP_PIN ({MOTOR_PIN_CONFIG['UP_PIN']}). CurrentEnc: {current_encoder_count}")
            GPIO.output(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.LOW)
            GPIO.output(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.HIGH)
            motor_movement_direction = 1
            self.motor_is_moving = True

    def _stop_motor_gpio(self):
        global motor_movement_direction
        if self.motor_is_moving:
            self.get_logger().debug(f"CMD STOP MOTOR. Encoder: {current_encoder_count}")
        GPIO.output(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.LOW)
        GPIO.output(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.LOW)
        motor_movement_direction = 0
        self.motor_is_moving = False

    def on_shutdown(self):
        self.get_logger().info("Shutting down MotorControllerNode...")
        if self.motor_control_active and REAL_GPIO_AVAILABLE :
            self.get_logger().info("Attempting GPIO cleanup for MotorControllerNode.")
            self._stop_motor_gpio()
            try:
                GPIO.remove_event_detect(MOTOR_PIN_CONFIG['ENCODER_PIN'])
            except Exception as e:
                 self.get_logger().warn(f"Could not remove event detect for encoder: {e}")
            GPIO.cleanup()
            self.get_logger().info("GPIO cleanup complete for MotorControllerNode.")
        else:
            self.get_logger().info("Motor control was inactive or no real GPIO, no hardware cleanup needed from this node.")

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        global current_encoder_count, motor_movement_direction
        current_encoder_count = 0
        motor_movement_direction = 0
        
        node = MotorControllerNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt, shutting down...')

    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception in MotorControllerNode: {e}")
        else:
            print(f"Exception before node was initialized: {e}")
            
    finally:
        # This is the proper ROS2 shutdown sequence for a launched node.
        # 1. The node cleans up its own resources (GPIO, etc.).
        # 2. The node is explicitly destroyed.
        # 3. rclpy.shutdown() is NOT called here. The launch service manages it.
        if node:
            node.get_logger().info("Cleaning up and destroying node...")
            node.on_shutdown()
            node.destroy_node()

        print("motor_controller_node has been shutdown cleanly.")