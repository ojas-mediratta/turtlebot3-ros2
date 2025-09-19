#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
# NEW: Import the constants
from . import constants as const

class RotateRobotNode(Node):
    def __init__(self):
        super().__init__('rotate_robot_node')
        self._coord_subscriber = self.create_subscription(
            Point,
            const.OBJECT_COORD_TOPIC,
            self._coord_callback,
            10
        )
        self._vel_publisher = self.create_publisher(Twist, const.CMD_VEL_TOPIC, 10)
        self.get_logger().info('rotate_robot_node has been started and is using configuration file.')

    def _coord_callback(self, msg):
        center_x = const.IMAGE_WIDTH / 2.0
        cx = msg.x
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        if cx == -1.0:
            self._vel_publisher.publish(twist_msg)
            return

        error = center_x - cx
        twist_msg.angular.z = const.TURN_SPEED * (error / center_x)
        self._vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RotateRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()