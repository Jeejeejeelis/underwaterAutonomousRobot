#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

try:
    import RPi.GPIO as GPIO
    REAL_GPIO_AVAILABLE = True
except (RuntimeError, ModuleNotFoundError) as e:
    print(f"Error importing RPi.GPIO: {e}")
    print("This script is intended to run on a Raspberry Pi with RPi.GPIO installed.")
    GPIO = None
    REAL_GPIO_AVAILABLE = False
except ImportError:
    print("RPi.GPIO library not found, using a mock for basic testing if not on RPi.")
    REAL_GPIO_AVAILABLE = False
    class GPIOMock:
        BCM = "BCM_MODE"; OUT = "OUTPUT_MODE"; IN = "INPUT_MODE"; PUD_UP = "PULL_UP_DOWN_UP"
        LOW = False; HIGH = True; BOTH = "BOTH_EDGES"
        _mock_motor_pin_config_ref = None
        def __init__(self):
            self._pin_modes = {}; self._pin_states = {}; self._event_callbacks = {}
            print("Mock RPi.GPIO initialized for InitNeutralBuoyancyNode.")
            if GPIOMock._mock_motor_pin_config_ref is None:
                 try: GPIOMock._mock_motor_pin_config_ref = MOTOR_GLOBALS
                 except NameError: pass
        def setmode(self, mode): pass
        def setup(self, pin, mode, pull_up_down=None):
            self._pin_modes[pin] = mode
            if mode == self.OUT: self._pin_states[pin] = self.LOW
        def output(self, pin, state):
            if pin in self._pin_modes and self._pin_modes[pin] == self.OUT: self._pin_states[pin] = state
        def input(self, pin):
            config_to_use = GPIOMock._mock_motor_pin_config_ref
            if config_to_use:
                if pin == config_to_use.get('UP_LIMIT_PIN') or pin == config_to_use.get('DOWN_LIMIT_PIN'):
                    return self.HIGH
            return self.LOW
        def add_event_detect(self, pin, edge, callback=None, bouncetime=None): self._event_callbacks[pin] = callback
        def cleanup(self): pass
        def remove_event_detect(self, pin):
            if pin in self._event_callbacks: del self._event_callbacks[pin]
    if GPIO is None and not REAL_GPIO_AVAILABLE: GPIO = GPIOMock()


MOTOR_GLOBALS = {
    'UP_PIN': 5,        # Wire: Brown. Physical Action: Piston IN, Volume DECREASE, UAV DESCENDS.
    'DOWN_PIN': 6,      # Wire: Red.   Physical Action: Piston OUT, Volume INCREASE, UAV ASCENDS.
    'ENCODER_PIN': 24,
    'UP_LIMIT_PIN': 19, # Green wire. Hit when piston is fully IN (min volume, max encoder ticks).
    'DOWN_LIMIT_PIN': 26, # Yellow wire. Hit when piston is fully OUT (max volume, min encoder ticks = 0).

    'current_encoder_count': 0,
    'motor_moving': False,
    # movement_direction: 1 for Piston IN (Encoder INC), -1 for Piston OUT (Encoder DEC)
    'movement_direction': 0,
    'initialization_complete': False,
    'neutral_buoyancy_encoder_target': 10876, # Target encoder count for neutral buoyancy. This should be axactly halfway for the piston!
}
if not REAL_GPIO_AVAILABLE:
    if GPIOMock._mock_motor_pin_config_ref is None:
        GPIOMock._mock_motor_pin_config_ref = MOTOR_GLOBALS

# Encoder INCREASES as piston moves IN (towards UP_LIMIT_PIN)
# Encoder DECREASES as piston moves OUT (towards DOWN_LIMIT_PIN)
def encoder_callback(channel):
    if MOTOR_GLOBALS['movement_direction'] == 1:
        MOTOR_GLOBALS['current_encoder_count'] += 1
    elif MOTOR_GLOBALS['movement_direction'] == -1:
        MOTOR_GLOBALS['current_encoder_count'] -= 1

class InitNeutralBuoyancyNode(Node):
    def __init__(self):
        super().__init__('init_neutral_buoyancy_node')

        if not REAL_GPIO_AVAILABLE:
            self.get_logger().warn("RPi.GPIO not available or using mock. Actual motor control for initialization will not function.")
        else:
            self.get_logger().info("RPi.GPIO found.")

        self.get_logger().info('Neutral Buoyancy Initialization Node Started.')
        self.encoder_publisher = self.create_publisher(Int32, '/current_encoder_position', 10)

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(MOTOR_GLOBALS['UP_PIN'], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(MOTOR_GLOBALS['DOWN_PIN'], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(MOTOR_GLOBALS['ENCODER_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(MOTOR_GLOBALS['DOWN_LIMIT_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(MOTOR_GLOBALS['UP_LIMIT_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)

            GPIO.add_event_detect(MOTOR_GLOBALS['ENCODER_PIN'], GPIO.BOTH, callback=encoder_callback, bouncetime=2)
            time.sleep(1) #Important, otherwise nothing works!
            self.get_logger().info(f"GPIO initialized. PistonIN/Descend Pin: {MOTOR_GLOBALS['UP_PIN']}, PistonOUT/Ascend Pin: {MOTOR_GLOBALS['DOWN_PIN']}")

        except Exception as e:
            self.get_logger().error(f"Error during GPIO setup: {e}. Node will not function correctly.")
            MOTOR_GLOBALS['initialization_complete'] = True
            return

        self.state = 'INITIALIZING_TO_LOWER_LIMIT'
        self.get_logger().info(f"Initial state: {self.state}")
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.control_loop)

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
            
            if GPIO.input(MOTOR_GLOBALS['DOWN_LIMIT_PIN']) == GPIO.LOW:
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
                # MOTOR_GLOBALS['current_encoder_count'] = self.max_ticks
                self.get_logger().warn(f"UPPER limit switch (Piston IN, Min Volume) hit unexpectedly at encoder {MOTOR_GLOBALS['current_encoder_count']} while seeking neutral. Initialization stopped.")
                self.publish_encoder_count()
                if self.timer:
                    self.timer.cancel()
                    self.timer = None
                return

    def _command_piston_in(self):
        """Commands piston IN. Volume DECREASES, UAV DESCENDS. Encoder INCREMENTS. Activates UP_PIN (GPIO 5)."""
        if not MOTOR_GLOBALS['motor_moving'] or MOTOR_GLOBALS['movement_direction'] != 1:
            # self.get_logger().debug("Commanding Piston IN (Encoder INC)")
            GPIO.output(MOTOR_GLOBALS['DOWN_PIN'], GPIO.LOW)
            GPIO.output(MOTOR_GLOBALS['UP_PIN'], GPIO.HIGH)
            MOTOR_GLOBALS['motor_moving'] = True
            MOTOR_GLOBALS['movement_direction'] = 1

    def _command_piston_out(self):
        """Commands piston OUT. Volume INCREASES, UAV ASCENDS. Encoder DECREMENTS. Activates DOWN_PIN (GPIO 6)."""
        if not MOTOR_GLOBALS['motor_moving'] or MOTOR_GLOBALS['movement_direction'] != -1:
            # self.get_logger().debug("Commanding Piston OUT (Encoder DEC)")
            GPIO.output(MOTOR_GLOBALS['UP_PIN'], GPIO.LOW)
            GPIO.output(MOTOR_GLOBALS['DOWN_PIN'], GPIO.HIGH)
            MOTOR_GLOBALS['motor_moving'] = True
            MOTOR_GLOBALS['movement_direction'] = -1

    def stop_motor(self):
        # self.get_logger().debug("Commanding motor STOP")
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
            self.get_logger().info('Attempting GPIO cleanup.')
            self.stop_motor()
            if MOTOR_GLOBALS.get('ENCODER_PIN') is not None:
                 try:
                    GPIO.remove_event_detect(MOTOR_GLOBALS['ENCODER_PIN'])
                 except Exception as e:
                    self.get_logger().warn(f"Could not remove event detect for encoder: {e}")
            GPIO.cleanup()
            self.get_logger().info('GPIO cleanup complete.')
        else:
            self.get_logger().info('GPIO was not initialized with real RPi.GPIO or using mock, no hardware cleanup needed.')

def main(args=None):
    rclpy.init(args=args)
    MOTOR_GLOBALS['current_encoder_count'] = 0
    MOTOR_GLOBALS['motor_moving'] = False
    MOTOR_GLOBALS['movement_direction'] = 0
    MOTOR_GLOBALS['initialization_complete'] = False

    init_node = InitNeutralBuoyancyNode()
    
    try:
        if MOTOR_GLOBALS['initialization_complete']:
            init_node.get_logger().error("Initialization marked complete prematurely, likely due to GPIO setup error in constructor.")
        elif not hasattr(init_node, 'timer') or init_node.timer is None :
            init_node.get_logger().error("Timer not created. Node cannot proceed with initialization logic.")
            MOTOR_GLOBALS['initialization_complete'] = True
        else:
            while rclpy.ok() and not MOTOR_GLOBALS['initialization_complete']:
                rclpy.spin_once(init_node, timeout_sec=0.1)
                if init_node.timer is None and not MOTOR_GLOBALS['initialization_complete']:
                    init_node.get_logger().warn("Timer stopped, but initialization not marked complete. Check logic or errors.")
                    break
            
            if MOTOR_GLOBALS['initialization_complete']:
                init_node.publish_encoder_count()
                init_node.get_logger().info(f"Initialization to neutral buoyancy complete at {MOTOR_GLOBALS['current_encoder_count']} ticks. Waiting for mission command...")
                time.sleep(0.5)
            else:
                if rclpy.ok():
                     init_node.get_logger().warn("Initialization sequence did not complete successfully.")

    except KeyboardInterrupt:
        init_node.get_logger().info('Keyboard interrupt, shutting down init_neutral_buoyancy_node...')
    except Exception as e:
        init_node.get_logger().error(f"Unhandled exception in init_neutral_buoyancy_node main operation: {e}")
    finally:
        init_node.get_logger().info(f"Entering finally block for {init_node.get_name()}.")
        if hasattr(init_node, 'on_shutdown'):
             init_node.on_shutdown()

        if hasattr(init_node, 'destroy_node') and hasattr(init_node, 'get_node_names') and init_node.get_node_names() and rclpy.ok():
            init_node.get_logger().info(f"Destroying node {init_node.get_name()}.")
            init_node.destroy_node()
        
        if rclpy.ok():
            print("Shutting down rclpy context for InitNeutralBuoyancyNode.")
            rclpy.shutdown()
        
        node_name = "init_neutral_buoyancy_node"
        if hasattr(init_node, 'get_name'):
            node_name = init_node.get_name()
        print(f"{node_name} (Python process) is terminating.")

if __name__ == '__main__':
    main()