#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool # Import Bool
import time

# ... (GPIOMock and MOTOR_GLOBALS sections remain the same) ...
try:
    import RPi.GPIO as GPIO
    REAL_GPIO_AVAILABLE = True
except (RuntimeError, ModuleNotFoundError) as e:
    print(f"Error importing RPi.GPIO: {e}")
    print("This script is intended to run on a Raspberry Pi with RPi.GPIO installed.")
    GPIO = None
    REAL_GPIO_AVAILABLE = False
except ImportError:
    print("RPi.GPIO library not found, using a mock for basic testing if not on RPI.")
    REAL_GPIO_AVAILABLE = False
    class GPIOMock:
        BCM = "BCM_MODE"; OUT = "OUTPUT_MODE"; IN = "INPUT_MODE"; PUD_UP = "PULL_UP_DOWN_UP"
        LOW = False; HIGH = True; BOTH = "BOTH_EDGES"
        _mock_motor_pin_config_ref = None

        def __init__(self):
            self._pin_modes = {}; self._pin_states = {}; self._event_callbacks = {}
            self._limit_switch_states = {}
            print("Mock RPi.GPIO initialized for InitNeutralBuoyancyNode.")
            if GPIOMock._mock_motor_pin_config_ref is None:
                try:
                    GPIOMock._mock_motor_pin_config_ref = MOTOR_GLOBALS
                    self._limit_switch_states[MOTOR_GLOBALS.get('UP_LIMIT_PIN')] = self.HIGH
                    self._limit_switch_states[MOTOR_GLOBALS.get('DOWN_LIMIT_PIN')] = self.HIGH
                except NameError:
                    pass

        def setmode(self, mode): pass
        def setup(self, pin, mode, pull_up_down=None):
            self._pin_modes[pin] = mode
            if mode == self.OUT: self._pin_states[pin] = self.LOW
        def output(self, pin, state):
            if pin in self._pin_modes and self._pin_modes[pin] == self.OUT: self._pin_states[pin] = state
        def input(self, pin):
            if pin in self._limit_switch_states:
                return self._limit_switch_states[pin]
            return self.LOW
        def add_event_detect(self, pin, edge, callback=None, bouncetime=None): self._event_callbacks[pin] = callback
        def cleanup(self): 
            print("SIMULATION: GPIO.cleanup() called.")
            pass
        def remove_event_detect(self, pin):
            if pin in self._event_callbacks: del self._event_callbacks[pin]
        
        def trigger_limit_switch(self, pin):
            if pin in self._limit_switch_states:
                self._limit_switch_states[pin] = self.LOW
                print(f"SIMULATION: Limit switch on pin {pin} triggered (LOW).")
                time.sleep(0.1)
                self._limit_switch_states[pin] = self.HIGH

    if GPIO is None and not REAL_GPIO_AVAILABLE: GPIO = GPIOMock()

MOTOR_GLOBALS = {
    'UP_PIN': 5,
    'DOWN_PIN': 6,
    'ENCODER_PIN': 24,
    'UP_LIMIT_PIN': 19,
    'DOWN_LIMIT_PIN': 26,

    'current_encoder_count': 0,
    'motor_moving': False,
    'movement_direction': 0,
    'initialization_complete': False,
    'neutral_buoyancy_encoder_target': 10876, 
}
if not REAL_GPIO_AVAILABLE:
    if GPIOMock._mock_motor_pin_config_ref is None:
        GPIOMock._mock_motor_pin_config_ref = MOTOR_GLOBALS

def encoder_callback(channel):
    if MOTOR_GLOBALS['movement_direction'] == 1:
        MOTOR_GLOBALS['current_encoder_count'] += 1
    elif MOTOR_GLOBALS['movement_direction'] == -1:
        MOTOR_GLOBALS['current_encoder_count'] -= 1
        
class InitNeutralBuoyancyNode(Node):
    def __init__(self):
        super().__init__('init_neutral_buoyancy_node')

        self.get_logger().info('Neutral Buoyancy Initialization Node Started.')
        self.encoder_publisher = self.create_publisher(Int32, '/current_encoder_position', 10)

        # --- CHANGE: Add publisher for the start signal ---
        # Use TRANSIENT_LOCAL to ensure the message is saved for any late-joining subscribers
        qos_profile = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.sim_start_publisher = self.create_publisher(Bool, '/simulation/start', qos_profile)

        try:
            GPIO.setmode(GPIO.BCM)
            # ... (rest of GPIO setup is the same) ...
            GPIO.setup(MOTOR_GLOBALS['UP_PIN'], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(MOTOR_GLOBALS['DOWN_PIN'], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(MOTOR_GLOBALS['ENCODER_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(MOTOR_GLOBALS['DOWN_LIMIT_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(MOTOR_GLOBALS['UP_LIMIT_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(MOTOR_GLOBALS['ENCODER_PIN'], GPIO.BOTH, callback=encoder_callback, bouncetime=2)
            time.sleep(1)
            self.get_logger().info(f"GPIO initialized. PistonIN/Descend Pin: {MOTOR_GLOBALS['UP_PIN']}, PistonOUT/Ascend Pin: {MOTOR_GLOBALS['DOWN_PIN']}")

        except Exception as e:
            self.get_logger().error(f"Error during GPIO setup: {e}. Node will not function correctly.")
            MOTOR_GLOBALS['initialization_complete'] = True
            return

        self.state = 'INITIALIZING_TO_LOWER_LIMIT'
        self.get_logger().info(f"Initial state: {self.state}")
        self.timer_period = 0.05
        self.simulation_timeout_sec = 5.0
        self.simulation_start_time = time.time()
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    # ... (control_loop and other methods remain the same) ...
    def control_loop(self):
        if MOTOR_GLOBALS['initialization_complete']:
            if self.timer:
                self.publish_encoder_count()
                self.timer.cancel()
                self.timer = None
            return

        self.publish_encoder_count()

        if self.state == 'INITIALIZING_TO_LOWER_LIMIT':
            self._command_piston_out()
            
            limit_hit = GPIO.input(MOTOR_GLOBALS['DOWN_LIMIT_PIN']) == GPIO.LOW
            sim_timeout = not REAL_GPIO_AVAILABLE and (time.time() - self.simulation_start_time > self.simulation_timeout_sec)

            if limit_hit or sim_timeout:
                if sim_timeout:
                    self.get_logger().warn("Simulation timeout reached. Forcing transition to next state.")
                self.get_logger().info("Lower limit switch HIT (Piston OUT, Max Volume).")
                self.stop_motor()
                MOTOR_GLOBALS['current_encoder_count'] = 0
                self.get_logger().info(f"Encoder count reset to {MOTOR_GLOBALS['current_encoder_count']}.")
                self.publish_encoder_count()
                self.state = 'MOVING_TO_NEUTRAL_BUOYANCY'
                self.get_logger().info(f"State changed to: {self.state}")

        elif self.state == 'MOVING_TO_NEUTRAL_BUOYANCY':
            if MOTOR_GLOBALS['current_encoder_count'] < MOTOR_GLOBALS['neutral_buoyancy_encoder_target']:
                self._command_piston_in()
            else:
                self.stop_motor()
                MOTOR_GLOBALS['initialization_complete'] = True
                self.get_logger().info(f"Neutral buoyancy position nominally reached at encoder count: {MOTOR_GLOBALS['current_encoder_count']}.")
                self.publish_encoder_count()
                if self.timer:
                    self.timer.cancel()
                    self.timer = None
                return

            if GPIO.input(MOTOR_GLOBALS['UP_LIMIT_PIN']) == GPIO.LOW:
                self.stop_motor()
                MOTOR_GLOBALS['initialization_complete'] = True
                self.get_logger().warn(f"UPPER limit switch hit unexpectedly at encoder {MOTOR_GLOBALS['current_encoder_count']}. Initialization stopped.")
                self.publish_encoder_count()
                if self.timer:
                    self.timer.cancel()
                    self.timer = None
                return

    def _command_piston_in(self):
        if not MOTOR_GLOBALS['motor_moving'] or MOTOR_GLOBALS['movement_direction'] != 1:
            GPIO.output(MOTOR_GLOBALS['DOWN_PIN'], GPIO.LOW)
            GPIO.output(MOTOR_GLOBALS['UP_PIN'], GPIO.HIGH)
            MOTOR_GLOBALS['motor_moving'] = True
            MOTOR_GLOBALS['movement_direction'] = 1

    def _command_piston_out(self):
        if not MOTOR_GLOBALS['motor_moving'] or MOTOR_GLOBALS['movement_direction'] != -1:
            GPIO.output(MOTOR_GLOBALS['UP_PIN'], GPIO.LOW)
            GPIO.output(MOTOR_GLOBALS['DOWN_PIN'], GPIO.HIGH)
            MOTOR_GLOBALS['motor_moving'] = True
            MOTOR_GLOBALS['movement_direction'] = -1

    def stop_motor(self):
        GPIO.output(MOTOR_GLOBALS['UP_PIN'], GPIO.LOW)
        GPIO.output(MOTOR_GLOBALS['DOWN_PIN'], GPIO.LOW)
        MOTOR_GLOBALS['motor_moving'] = False
        MOTOR_GLOBALS['movement_direction'] = 0

    def publish_encoder_count(self):
        msg = Int32()
        msg.data = MOTOR_GLOBALS['current_encoder_count']
        self.encoder_publisher.publish(msg)

    def on_shutdown(self):
        self.get_logger().info('Shutting down InitNeutralBuoyancyNode...')
        if REAL_GPIO_AVAILABLE:
            self.get_logger().info('Stopping motor and cleaning up GPIOs.')
            self.stop_motor()
            GPIO.cleanup()
        else:
            self.get_logger().info('Simulation mode: No hardware cleanup needed.')


def main(args=None):
    rclpy.init(args=args)
    init_node = None
    try:
        MOTOR_GLOBALS['current_encoder_count'] = 0
        MOTOR_GLOBALS['motor_moving'] = False
        MOTOR_GLOBALS['movement_direction'] = 0
        MOTOR_GLOBALS['initialization_complete'] = False

        init_node = InitNeutralBuoyancyNode()
        
        while rclpy.ok() and not MOTOR_GLOBALS.get('initialization_complete', False):
            rclpy.spin_once(init_node, timeout_sec=0.1)
        
        if MOTOR_GLOBALS.get('initialization_complete'):
            init_node.get_logger().info("Initialization complete.")
            # --- CHANGE: Publish the start signal for the simulator ---
            init_node.get_logger().info("Publishing simulation start signal.")
            start_msg = Bool()
            start_msg.data = True
            init_node.sim_start_publisher.publish(start_msg)
            # Give it a moment to publish before shutting down
            time.sleep(0.5)
        else:
            init_node.get_logger().warn("RCLPY shutting down before initialization was complete.")

    except KeyboardInterrupt:
        if init_node:
            init_node.get_logger().info('Keyboard interrupt received.')
    finally:
        if init_node:
            init_node.on_shutdown()
            init_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("init_neutral_buoyancy_node process has terminated.")


if __name__ == '__main__':
    main()