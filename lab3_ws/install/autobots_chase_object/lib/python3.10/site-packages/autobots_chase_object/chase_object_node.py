import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import time

# Import the constants file
from . import constants as const

class ChaseObjectNode(Node):
    def __init__(self):
        super().__init__('chase_object_node')

        # --- PI Controller State Variables ---
        self.angular_integral = 0.0
        self.linear_integral = 0.0
        self.last_time = time.time()

        # --- Subscriber and Publisher ---
        self._location_subscriber = self.create_subscription(
            Point,
            const.OBJECT_LOCATION_TOPIC,
            self._location_callback,
            10
        )
        self._vel_publisher = self.create_publisher(Twist, const.CMD_VEL_TOPIC, 10)
        
        self.get_logger().info('chase_object_node has been started.')

    def _location_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        actual_distance = msg.x
        actual_angle = msg.y

        # If the object is not found, stop the robot and reset integrals
        if actual_distance == -1.0:
            self.stop_robot()
            return
        
        # --- Angular PI Controller (for turning) ---
        angular_error = 0.0 - actual_angle
        self.angular_integral += angular_error * dt
        # Clamp the integral term to prevent windup
        self.angular_integral = max(min(self.angular_integral, const.INTEGRAL_CLAMP_ANGULAR), -const.INTEGRAL_CLAMP_ANGULAR)
        angular_velocity = (const.ANGULAR_KP * angular_error) + (const.ANGULAR_KI * self.angular_integral)

        # --- Linear PI Controller (for forward/backward) ---
        linear_error = actual_distance - const.DESIRED_DISTANCE_M
        self.linear_integral += linear_error * dt
        # Clamp the integral term
        self.linear_integral = max(min(self.linear_integral, const.INTEGRAL_CLAMP_LINEAR), -const.INTEGRAL_CLAMP_LINEAR)
        linear_velocity = (const.LINEAR_KP * linear_error) + (const.LINEAR_KI * self.linear_integral)

        # --- Publish the final velocity command ---
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = -angular_velocity
        self._vel_publisher.publish(twist_msg)

    def stop_robot(self):
        # Publish a zero-velocity Twist message
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self._vel_publisher.publish(twist_msg)
        # Reset integrals when the robot stops
        self.angular_integral = 0.0
        self.linear_integral = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = ChaseObjectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()