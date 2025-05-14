#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from rclpy.parameter import Parameter
import message_filters

# Conversion factor: 1 dbar of GAUGE pressure is approximately 0.992 meters of seawater depth.
DBAR_TO_METERS_FACTOR = 0.992

class DepthCalculatorNode(Node):
    def __init__(self):
        super().__init__('depth_calculator_node')
        self.get_logger().info("Depth Calculator Node (Sea Surface Z=0, Negative Depths) started.")

        self.declare_parameter('pressure_to_depth_factor', DBAR_TO_METERS_FACTOR)
        self.conversion_factor = self.get_parameter('pressure_to_depth_factor').value

        self.pressure_sub = message_filters.Subscriber(self, Float32, 'pressure')
        self.dvl_altitude_sub = message_filters.Subscriber(self, Float32, 'altitude')

        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.pressure_sub, self.dvl_altitude_sub],
            queue_size=10,
            slop=0.2,
            allow_headerless=True
        )
        self.time_synchronizer.registerCallback(self.synchronized_callback)

        # Publishers for depths (negative Z from sea surface Z=0)
        self.drone_depth_publisher = self.create_publisher(Float32, 'drone_depth', 10)
        self.seafloor_depth_publisher = self.create_publisher(Float32, 'seafloor_depth', 10)

        self.get_logger().info("Subscribed to '/pressure' (gauge dbar) and '/altitude' (m AGL).")
        self.get_logger().info("Publishing to '/drone_depth' (negative Z) and '/seafloor_depth' (negative Z).")
        self.get_logger().info("Coordinate system: Sea Surface Z=0, Z decreases (becomes more negative) with depth.")

    def synchronized_callback(self, pressure_msg, dvl_altitude_msg):
        gauge_pressure_dbar = pressure_msg.data
        dvl_altitude_agl_m = dvl_altitude_msg.data # Drone's height above seafloor

        drone_depth_positive_m = gauge_pressure_dbar * self.conversion_factor
        if drone_depth_positive_m < 0:
            self.get_logger().warn(f"Calculated negative drone depth ({drone_depth_positive_m:.2f}m) from gauge pressure {gauge_pressure_dbar:.2f}dbar. Clamping to 0 before negation.")
            drone_depth_positive_m = 0.0
        
        drone_depth_z_negative = -drone_depth_positive_m
        
        seafloor_depth_positive_m = -1.0
        seafloor_depth_z_negative = 0.0

        if dvl_altitude_agl_m >= 0:
            seafloor_depth_positive_m = drone_depth_positive_m + dvl_altitude_agl_m
            seafloor_depth_z_negative = -seafloor_depth_positive_m
        else:
            self.get_logger().debug(f"DVL altitude is negative ({dvl_altitude_agl_m:.2f}m), seafloor_depth will not be accurate or will be 0. Drone depth will still be published based on pressure.")

        self.drone_depth_publisher.publish(Float32(data=drone_depth_z_negative))
        
        if dvl_altitude_agl_m >=0:
            self.seafloor_depth_publisher.publish(Float32(data=seafloor_depth_z_negative))

        self.get_logger().debug(
            f"P_gauge={gauge_pressure_dbar:.2f}dbar, DVL_Alt_AGL={dvl_altitude_agl_m:.2f}m -> "
            f"Drone_Z={drone_depth_z_negative:.2f}m, Seafloor_Z published: {seafloor_depth_z_negative if dvl_altitude_agl_m >=0 else 'N/A'}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthCalculatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f"DepthCalculatorNode unhandled exception: {e}")
        else:
            print(f"DepthCalculatorNode unhandled exception during creation: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()