#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
import time

# Attempt to import RPi.GPIO
try:
    import RPi.GPIO as GPIO
    REAL_GPIO_AVAILABLE = True
except (RuntimeError, ModuleNotFoundError, ImportError):
    REAL_GPIO_AVAILABLE = False
    # Minimal Mock GPIO
    class GPIOMock:
        BCM = "BCM_MODE"; OUT = "OUT_MODE"; IN = "IN_MODE"; PUD_UP = "PUD_UP_MODE"
        LOW = False; HIGH = True; BOTH = "BOTH_EDGES"
        # Store MOTOR_PIN_CONFIG if possible, or use fixed vals for mock input
        # For simplicity, the mock input might need to be smarter or get config.
        # For now, it uses a global which is not ideal for a class.
        _mock_motor_pin_config_ref = None

        def __init__(self):
            self.pin_states = {}
            self.log = []
            if GPIOMock._mock_motor_pin_config_ref is None: # Try to get pin config for input simulation
                 # This is a bit of a hack for the standalone mock to access pin names for simulation
                 # In a real scenario, the mock might be passed the config or be more advanced.
                 try:
                     GPIOMock._mock_motor_pin_config_ref = MOTOR_PIN_CONFIG
                 except NameError: # MOTOR_PIN_CONFIG might not be defined when class is defined
                     pass


        def setmode(self, mode): self.log.append(f"Mock: setmode({mode})")
        def setup(self, pin, mode, pull_up_down=None): self.log.append(f"Mock: setup({pin}, {mode}, pud={pull_up_down})")
        def output(self, pin, state): self.pin_states[pin] = state; self.log.append(f"Mock: output({pin}, {state})")
        def input(self, pin):
            # Simulate limit switches: not pressed = HIGH with PUD_UP
            # Try to access the actual pin config if available
            config_to_use = GPIOMock._mock_motor_pin_config_ref
            if config_to_use:
                 if pin in [config_to_use['UP_LIMIT_PIN'], config_to_use['DOWN_LIMIT_PIN']]: return self.HIGH
            # Fallback if MOTOR_PIN_CONFIG wasn't available during class init or here
            # This part of the mock is tricky without instance-specific config.
            # For testing, one might need to override this input method.
            return self.LOW # Default for encoder or other inputs if not a known limit switch
        def add_event_detect(self, pin, edge, callback=None, bouncetime=None): self.log.append(f"Mock: add_event_detect({pin})")
        def remove_event_detect(self, pin): self.log.append(f"Mock: remove_event_detect({pin})")
        def cleanup(self): self.log.append("Mock: cleanup()")
    GPIO = GPIOMock()
    print("Mock RPi.GPIO initialized for motorControl.py as real GPIO is not available.")


# GPIO Pin Configuration (Matches motorTest.py)
# Comments updated to reflect new understanding:
MOTOR_PIN_CONFIG = {
    'UP_PIN': 5,        # Wire: Brown. Physical Action: Piston IN, Volume DECREASE, UAV DESCENDS.
    'DOWN_PIN': 6,      # Wire: Red.   Physical Action: Piston OUT, Volume INCREASE, UAV ASCENDS.
    'ENCODER_PIN': 27,
    'UP_LIMIT_PIN': 26, # Green wire. Hit when piston is fully IN (min volume).
    'DOWN_LIMIT_PIN': 19, # Yellow wire. Hit when piston is fully OUT (max volume).
}
if not REAL_GPIO_AVAILABLE: # Make mock aware of pin config if it wasn't set before
    if GPIOMock._mock_motor_pin_config_ref is None:
        GPIOMock._mock_motor_pin_config_ref = MOTOR_PIN_CONFIG


# Encoder and Motor State
# Convention: Encoder INCREASES as piston moves IN (towards UP_LIMIT_PIN, volume decrease)
# Encoder DECREASES as piston moves OUT (towards DOWN_LIMIT_PIN, volume increase)
current_encoder_count = 0 # This will be updated by initial_encoder_callback
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
        self.declare_parameter('neutral_buoyancy_ticks', 10890)
        self.declare_parameter('max_encoder_ticks', 21780) # Corresponds to UP_LIMIT_PIN (min volume)
        self.declare_parameter('min_encoder_ticks', 0)      # Corresponds to DOWN_LIMIT_PIN (max volume)
        self.declare_parameter('depth_kp', 0.1) # This Kp is for simple decision, not true PID for motor position yet
        self.declare_parameter('depth_tolerance_m', 0.1)
        self.declare_parameter('control_loop_rate_hz', 10.0)

        self.motor_control_active = self.get_parameter('motor_control_active').value
        self.neutral_ticks = self.get_parameter('neutral_buoyancy_ticks').value
        self.max_ticks = self.get_parameter('max_encoder_ticks').value
        self.min_ticks = self.get_parameter('min_encoder_ticks').value
        self.depth_kp = self.get_parameter('depth_kp').value # Currently not used in a P-control way for motor position
        self.depth_tolerance = self.get_parameter('depth_tolerance_m').value
        
        global current_encoder_count
        current_encoder_count = self.neutral_ticks # Assume starts at neutral until updated by init_neutral_buoyancy

        self.target_depth_m = None
        self.current_depth_m = None
        self.motor_is_moving = False

        if not REAL_GPIO_AVAILABLE:
            self.get_logger().warn("RPi.GPIO library not found. Motor control will use MOCK GPIO and will not function on hardware.")
            # self.motor_control_active = False # Keep as per parameter for testing structure
        
        if self.motor_control_active:
            self.get_logger().info("Motor Controller node is ACTIVE and will attempt to use GPIOs.")
            self._init_gpio()
        else:
            self.get_logger().info("Motor Controller node is INACTIVE. No GPIO operations will be performed.")

        self.target_depth_sub = self.create_subscription(
            Float32, '/target_depth', self.target_depth_callback, 10)
        self.current_depth_sub = self.create_subscription(
            Float32, '/current_depth', self.current_depth_callback, 10)
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
            GPIO.setup(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.OUT, initial=GPIO.LOW)   # Pin for Piston IN / DESCEND
            GPIO.setup(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.OUT, initial=GPIO.LOW) # Pin for Piston OUT / ASCEND
            GPIO.setup(MOTOR_PIN_CONFIG['ENCODER_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(MOTOR_PIN_CONFIG['UP_LIMIT_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(MOTOR_PIN_CONFIG['DOWN_LIMIT_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(MOTOR_PIN_CONFIG['ENCODER_PIN'], GPIO.BOTH, callback=encoder_tick_callback, bouncetime=2)
            self.get_logger().info("GPIO initialized successfully for MotorControllerNode.")
        except Exception as e:
            self.get_logger().error(f"CRITICAL: GPIO initialization failed in MotorControllerNode: {e}")
            self.motor_control_active = False

    def initial_encoder_callback(self, msg):
        global current_encoder_count
        # Update if significantly different, or if it's the first meaningful update
        is_first_update_from_default = (current_encoder_count == self.neutral_ticks and msg.data != self.neutral_ticks)
        if abs(current_encoder_count - msg.data) > 5 or is_first_update_from_default:
            self.get_logger().info(f"Received initial/updated encoder count: {msg.data}. Updating internal count from {current_encoder_count}.")
            current_encoder_count = msg.data
        # Consider unsubscribing if this should truly be a one-shot update from init_neutral_buoyancy
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
            return

        depth_error = self.target_depth_m - self.current_depth_m
        
        # UP_LIMIT_PIN is hit when piston is IN (min volume, max encoder ticks)
        # DOWN_LIMIT_PIN is hit when piston is OUT (max volume, min encoder ticks)
        upper_limit_physical_hit = GPIO.input(MOTOR_PIN_CONFIG['UP_LIMIT_PIN']) == GPIO.LOW
        lower_limit_physical_hit = GPIO.input(MOTOR_PIN_CONFIG['DOWN_LIMIT_PIN']) == GPIO.LOW

        # Decision logic for motor control
        if abs(depth_error) <= self.depth_tolerance:
            if self.motor_is_moving:
                self.get_logger().info(f"Target depth reached (Error: {depth_error:.2f}m). Stopping motor at encoder {current_encoder_count}.")
                self._stop_motor_gpio()
        else:
            if depth_error > 0: # Need to go deeper (DESCEND)
                # To DESCEND: Piston IN, Encoder INCREASES, use UP_PIN (GPIO 5)
                if upper_limit_physical_hit or current_encoder_count >= self.max_ticks:
                    if self.motor_is_moving: # Only stop if it was trying to move
                        self.get_logger().warn(f"Cannot go deeper: Upper limit physically hit ({upper_limit_physical_hit}) or max encoder ticks ({current_encoder_count}>={self.max_ticks}). Stopping motor.")
                        self._stop_motor_gpio()
                else:
                    self._command_descend_gpio() # Piston IN, Encoder INC
            else: # Need to go shallower (ASCEND, depth_error < 0)
                # To ASCEND: Piston OUT, Encoder DECREASES, use DOWN_PIN (GPIO 6)
                if lower_limit_physical_hit or current_encoder_count <= self.min_ticks:
                    if self.motor_is_moving:
                        self.get_logger().warn(f"Cannot go shallower: Lower limit physically hit ({lower_limit_physical_hit}) or min encoder ticks ({current_encoder_count}<={self.min_ticks}). Stopping motor.")
                        self._stop_motor_gpio()
                else:
                    self._command_ascent_gpio() # Piston OUT, Encoder DEC
        
        # Safety override if limit switches are hit while motor is commanded in that direction
        # motor_movement_direction: 1 for Piston IN (Descend), -1 for Piston OUT (Ascend)
        if upper_limit_physical_hit and motor_movement_direction == 1: # Moving IN (towards upper limit) and hit it
            self.get_logger().warn(f"Upper limit switch ACTIVE while commanding Piston IN! Stopping motor. Encoder: {current_encoder_count}")
            self._stop_motor_gpio()
            current_encoder_count = self.max_ticks # Correct encoder if slightly overshot

        if lower_limit_physical_hit and motor_movement_direction == -1: # Moving OUT (towards lower limit) and hit it
            self.get_logger().warn(f"Lower limit switch ACTIVE while commanding Piston OUT! Stopping motor. Encoder: {current_encoder_count}")
            self._stop_motor_gpio()
            current_encoder_count = self.min_ticks # Correct encoder if slightly overshot

    def _command_ascent_gpio(self): # Piston OUT, Volume INC, ASCEND, Encoder DEC
        global motor_movement_direction
        # Activates DOWN_PIN (GPIO 6, Red wire)
        if not self.motor_is_moving or motor_movement_direction != -1:
            self.get_logger().debug(f"CMD ASCEND: Piston OUT, Encoder DEC. Activating DOWN_PIN ({MOTOR_PIN_CONFIG['DOWN_PIN']}). CurrentEnc: {current_encoder_count}")
            GPIO.output(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.LOW)   # Ensure other direction is off
            GPIO.output(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.HIGH) # Activate Piston OUT
            motor_movement_direction = -1 # For encoder_tick_callback: DECREMENT
            self.motor_is_moving = True

    def _command_descend_gpio(self): # Piston IN, Volume DEC, DESCEND, Encoder INC
        global motor_movement_direction
        # Activates UP_PIN (GPIO 5, Brown wire)
        if not self.motor_is_moving or motor_movement_direction != 1:
            self.get_logger().debug(f"CMD DESCEND: Piston IN, Encoder INC. Activating UP_PIN ({MOTOR_PIN_CONFIG['UP_PIN']}). CurrentEnc: {current_encoder_count}")
            GPIO.output(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.LOW) # Ensure other direction is off
            GPIO.output(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.HIGH)  # Activate Piston IN
            motor_movement_direction = 1 # For encoder_tick_callback: INCREMENT
            self.motor_is_moving = True

    def _stop_motor_gpio(self):
        global motor_movement_direction
        if self.motor_is_moving: # Only log if it was actually moving
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
    # Reset globals for cleaner state if script is re-run in some interactive environments
    global current_encoder_count, motor_movement_direction
    current_encoder_count = 0 # Will be set by parameter or initial_encoder_callback
    motor_movement_direction = 0
    
    motor_controller_node = None
    try:
        motor_controller_node = MotorControllerNode()
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        if motor_controller_node: motor_controller_node.get_logger().info('Keyboard interrupt, shutting down...')
        else: print('Keyboard interrupt before node fully initialized.')
    except Exception as e:
        if motor_controller_node: motor_controller_node.get_logger().error(f"Unhandled exception: {e}")
        else: print(f"Unhandled exception before node fully initialized: {e}")
    finally:
        if motor_controller_node:
            motor_controller_node.on_shutdown()
            if hasattr(motor_controller_node, 'get_node_names') and motor_controller_node.get_node_names() and rclpy.ok():
                motor_controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("rclpy shutdown complete for motor_controller_node.")

if __name__ == '__main__':
    main()