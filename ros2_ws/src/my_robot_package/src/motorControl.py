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

        # Flags to ensure all initial data is received before controlling motor
        self.initial_encoder_value_received = False
        self.target_depth_value_received = False # To track if we've ever received a target
        self.current_depth_value_received = False # To track if we've ever received a current depth
        self.ready_to_control = False

        if not REAL_GPIO_AVAILABLE:
            self.get_logger().warn("RPi.GPIO library not found. Motor control will use MOCK GPIO.")
        
        if self.motor_control_active:
            self.get_logger().info("Motor Controller node is ACTIVE. GPIOs will be initialized when ready.")
            # Defer _init_gpio() until ready_to_control or make it safer
        else:
            self.get_logger().info("Motor Controller node is INACTIVE. No GPIO operations will be performed.")

        self.target_depth_sub = self.create_subscription(
            Float32, '/target_depth', self.target_depth_callback, 10)
        self.current_depth_sub = self.create_subscription(
            Float32, 'drone_depth', self.current_depth_callback, 10) 
        self.initial_encoder_sub = self.create_subscription(
            Int32, '/current_encoder_position', self.initial_encoder_callback, rclpy.qos.qos_profile_sensor_data)

        self.encoder_publisher = self.create_publisher(Int32, '/current_encoder_position', 10)
        self.motor_status_publisher = self.create_publisher(Bool, '/motor_moving_status', 10)

        loop_rate = self.get_parameter('control_loop_rate_hz').get_parameter_value().double_value
        loop_period = 1.0 / loop_rate if loop_rate > 0 else 0.1
        self.timer = self.create_timer(loop_period, self.control_loop)

        self.get_logger().info(f"MotorControllerNode initialized. Loop rate: {1.0/loop_period:.2f} Hz.")
        self.get_logger().info(f"Pins: PistonIN/DESCEND_PIN={MOTOR_PIN_CONFIG['UP_PIN']}, PistonOUT/ASCEND_PIN={MOTOR_PIN_CONFIG['DOWN_PIN']}")
        # GPIO init is now deferred or made safe

    def _try_set_ready_to_control(self):
        if not self.ready_to_control and \
           self.initial_encoder_value_received and \
           self.target_depth_value_received and \
           self.current_depth_value_received:
            
            self.ready_to_control = True
            self.get_logger().info("MotorControl: All initial inputs received. Ready to control motor.")
            if self.motor_control_active and not hasattr(self, '_gpio_initialized_flag'): # Check if GPIOs need init
                self.get_logger().info("MotorControl: Initializing GPIOs now that node is ready.")
                self._init_gpio()


    def _init_gpio(self):
        try:
            GPIO.setmode(GPIO.BCM) # This might still cause warning if init_neutral_buoyancy ran
            GPIO.setup(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(MOTOR_PIN_CONFIG['ENCODER_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(MOTOR_PIN_CONFIG['ENCODER_PIN'], GPIO.BOTH, callback=encoder_tick_callback, bouncetime=2)
            # Limit switches are inputs, less likely to conflict if re-setupped by init_neutral
            GPIO.setup(MOTOR_PIN_CONFIG['UP_LIMIT_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(MOTOR_PIN_CONFIG['DOWN_LIMIT_PIN'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self._gpio_initialized_flag = True # Mark GPIOs as initialized by this node
            time.sleep(0.1) 
            self.get_logger().info("GPIO initialized successfully for MotorControllerNode.")
        except Exception as e:
            self.get_logger().error(f"CRITICAL: GPIO initialization failed in MotorControllerNode: {e}")
            self.motor_control_active = False 

    def initial_encoder_callback(self, msg):
        global current_encoder_count
        self.get_logger().info(f"MotorControl: initial_encoder_callback received {msg.data}. Old internal count: {current_encoder_count}")
        current_encoder_count = msg.data
        self.initial_encoder_value_received = True
        self._try_set_ready_to_control()
        # Do not destroy subscription; let it continuously update from init_node then this node's own publisher

    def target_depth_callback(self, msg):
        self.get_logger().info(f"MotorControl: target_depth_callback received /target_depth: {msg.data:.2f}m")
        self.target_depth_m = msg.data
        self.target_depth_value_received = True
        self._try_set_ready_to_control()

    def current_depth_callback(self, msg):
        self.get_logger().info(f"MotorControl: current_depth_callback received /drone_depth: {msg.data:.2f}m")
        self.current_depth_m = msg.data
        self.current_depth_value_received = True
        self._try_set_ready_to_control()

    def control_loop(self):
        global current_encoder_count, motor_movement_direction
        
        self.get_logger().info(
            f"MotorControl LOOP: Ready={self.ready_to_control}, Active={self.motor_control_active}, TargetD={self.target_depth_m}, "
            f"CurrentD={self.current_depth_m}, Encoder={current_encoder_count}, MotorMoving={self.motor_is_moving}"
        )
        
        enc_msg = Int32()
        enc_msg.data = int(round(current_encoder_count)) 
        self.encoder_publisher.publish(enc_msg)

        motor_stat_msg = Bool()
        motor_stat_msg.data = self.motor_is_moving
        self.motor_status_publisher.publish(motor_stat_msg)

        if not self.motor_control_active:
            if self.motor_is_moving: self._stop_motor_gpio() # Should only be called if GPIOs are this node's
            return

        if not self.ready_to_control:
            self.get_logger().info("MotorControl: Not yet ready to control (waiting for initial encoder, target depth, or current depth).", throttle_duration_sec=5)
            # DO NOT stop motor here, init_neutral_buoyancy might be using it
            return
        
        # From here, self.ready_to_control is True
        if not hasattr(self, '_gpio_initialized_flag') and self.motor_control_active and REAL_GPIO_AVAILABLE:
            self.get_logger().warn("MotorControl: GPIOs were not initialized by this node but trying to control. This might be an issue if init_neutral_buoyancy didn't run or cleanup GPIOs properly.")
            # Attempt to initialize GPIOs if not done yet, though this might be too late or cause issues
            # self._init_gpio() # Or better, ensure init_gpio is called when ready_to_control is true.

        if self.target_depth_m is None or self.current_depth_m is None: # Should not happen if ready_to_control is true
            if self.motor_is_moving: self._stop_motor_gpio()
            self.get_logger().warn("MotorControl: Depths became None after being ready. This is unexpected. Stopping motor.", throttle_duration_sec=5)
            return
        
        self.get_logger().info("MotorControl: Proceeding with control logic.")

        depth_error = self.target_depth_m - self.current_depth_m
        upper_limit_physical_hit = GPIO.input(MOTOR_PIN_CONFIG['UP_LIMIT_PIN']) == GPIO.LOW if REAL_GPIO_AVAILABLE and hasattr(self, '_gpio_initialized_flag') else False
        lower_limit_physical_hit = GPIO.input(MOTOR_PIN_CONFIG['DOWN_LIMIT_PIN']) == GPIO.LOW if REAL_GPIO_AVAILABLE and hasattr(self, '_gpio_initialized_flag') else False

        if abs(depth_error) <= self.depth_tolerance:
            if self.motor_is_moving:
                self.get_logger().info(f"MotorControl: Target depth ({self.target_depth_m:.2f}m) reached (Error: {depth_error:.2f}m). Stopping motor at encoder {current_encoder_count}.")
                self._stop_motor_gpio()
        else:
            self.get_logger().info(f"MotorControl: Depth error: {depth_error:.2f}m. Target: {self.target_depth_m:.2f}m, Current: {self.current_depth_m:.2f}m")
            if depth_error > 0: 
                self.get_logger().info("MotorControl: Attempting to go deeper.")
                if upper_limit_physical_hit:
                    if self.motor_is_moving: self.get_logger().warn(f"MotorControl: Upper limit physically hit! Stopping."); self._stop_motor_gpio()
                elif current_encoder_count >= self.max_ticks:
                     if self.motor_is_moving: self.get_logger().warn(f"MotorControl: Max encoder ticks ({current_encoder_count}>={self.max_ticks}) reached! Stopping."); self._stop_motor_gpio()
                else:
                    self._command_descend_gpio() 
            else: 
                self.get_logger().info("MotorControl: Attempting to go shallower.")
                if lower_limit_physical_hit:
                    if self.motor_is_moving: self.get_logger().warn(f"MotorControl: Lower limit physically hit! Stopping."); self._stop_motor_gpio()
                elif current_encoder_count <= self.min_ticks:
                    if self.motor_is_moving: self.get_logger().warn(f"MotorControl: Min encoder ticks ({current_encoder_count}<={self.min_ticks}) reached! Stopping."); self._stop_motor_gpio()
                else:
                    self._command_ascent_gpio() 

        if REAL_GPIO_AVAILABLE and hasattr(self, '_gpio_initialized_flag'): 
            if upper_limit_physical_hit and motor_movement_direction == 1: 
                self.get_logger().warn(f"MotorControl: SAFETY - Upper limit switch ACTIVE while commanding Piston IN! Stopping. Encoder: {current_encoder_count}")
                self._stop_motor_gpio()
                current_encoder_count = self.max_ticks 

            if lower_limit_physical_hit and motor_movement_direction == -1: 
                self.get_logger().warn(f"MotorControl: SAFETY - Lower limit switch ACTIVE while commanding Piston OUT! Stopping. Encoder: {current_encoder_count}")
                self._stop_motor_gpio()
                current_encoder_count = self.min_ticks 

    def _command_ascent_gpio(self): 
        global motor_movement_direction
        if not (hasattr(self, '_gpio_initialized_flag') and self._gpio_initialized_flag): return

        if not self.motor_is_moving or motor_movement_direction != -1:
            self.get_logger().info(f"MotorControl: CMD ASCEND (Piston OUT). Encoder target DEC. CurrentEnc: {current_encoder_count}")
            if REAL_GPIO_AVAILABLE: GPIO.output(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.LOW); GPIO.output(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.HIGH)
            else: self.get_logger().warn("Mock GPIO: Simulating ASCEND command.")
            motor_movement_direction = -1
            self.motor_is_moving = True

    def _command_descend_gpio(self): 
        global motor_movement_direction
        if not (hasattr(self, '_gpio_initialized_flag') and self._gpio_initialized_flag): return

        if not self.motor_is_moving or motor_movement_direction != 1:
            self.get_logger().info(f"MotorControl: CMD DESCEND (Piston IN). Encoder target INC. CurrentEnc: {current_encoder_count}")
            if REAL_GPIO_AVAILABLE: GPIO.output(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.LOW); GPIO.output(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.HIGH)
            else: self.get_logger().warn("Mock GPIO: Simulating DESCEND command.")
            motor_movement_direction = 1
            self.motor_is_moving = True

    def _stop_motor_gpio(self):
        global motor_movement_direction
        if not (hasattr(self, '_gpio_initialized_flag') and self._gpio_initialized_flag): return

        if self.motor_is_moving: self.get_logger().info(f"MotorControl: CMD STOP MOTOR. Encoder: {current_encoder_count}")
        if REAL_GPIO_AVAILABLE: GPIO.output(MOTOR_PIN_CONFIG['UP_PIN'], GPIO.LOW); GPIO.output(MOTOR_PIN_CONFIG['DOWN_PIN'], GPIO.LOW)
        else: self.get_logger().warn("Mock GPIO: Simulating STOP command.")
        motor_movement_direction = 0
        self.motor_is_moving = False

    def on_shutdown(self):
        self.get_logger().info("Shutting down MotorControllerNode...") 
        if self.motor_control_active and REAL_GPIO_AVAILABLE and hasattr(self, '_gpio_initialized_flag') : # Only cleanup if this node init'd them
            self.get_logger().info("Attempting GPIO cleanup for MotorControllerNode.") 
            self._stop_motor_gpio()
            try:
                GPIO.remove_event_detect(MOTOR_PIN_CONFIG['ENCODER_PIN'])
            except Exception as e:
                 self.get_logger().warn(f"Could not remove event detect for encoder: {e}")
            GPIO.cleanup() # This cleans up all channels this script set up
            self.get_logger().info("GPIO cleanup complete for MotorControllerNode.")
        else:
            self.get_logger().info("Motor control was inactive, no real GPIO, or GPIOs not initialized by this node. No hardware cleanup needed from this node.")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        global current_encoder_count, motor_movement_direction
        motor_movement_direction = 0
        node = MotorControllerNode() 
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info('Keyboard interrupt, shutting down MotorControllerNode...')
    except Exception as e:
        if node: node.get_logger().fatal(f"Unhandled exception in MotorControllerNode: {e}")
        else: print(f"Exception before node was initialized in MotorControllerNode: {e}")
    finally:
        if node:
            node.get_logger().info("MotorControllerNode entering finally block for cleanup.")
            node.on_shutdown() 
            node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()
        print("motor_controller_node process has terminated.")

if __name__ == '__main__':
    main()