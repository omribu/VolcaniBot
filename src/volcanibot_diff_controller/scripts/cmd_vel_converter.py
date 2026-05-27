#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelConverter(Node):
    def __init__(self):
        super().__init__('cmd_vel_converter')
        
        # Subscribe to unstamped cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish stamped cmd_vel to controller
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diffdrive_base_controller/cmd_vel',
            10
        )
        
        self.get_logger().info('CmdVel converter started: /cmd_vel -> /diffdrive_base_controller/cmd_vel')
    
    def cmd_vel_callback(self, msg):
        # Convert Twist to TwistStamped
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = msg
        
        self.publisher.publish(stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    converter = CmdVelConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
