import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import time, math
from . import constants as const

class ChaseObjectNode(Node):
    def __init__(self):
        super().__init__('chase_object_node')

        # Params
        self.timeout_s = getattr(const, "NO_TARGET_TIMEOUT_S", 0.3)   # add to constants if you like
        self.pub_rate_hz = getattr(const, "CMD_PUB_RATE_HZ", 20.0)
        self.max_lin = getattr(const, "MAX_LINEAR", 0.4)
        self.max_ang = getattr(const, "MAX_ANGULAR", 1.5)

        # State
        self.angular_integral = 0.0
        self.linear_integral  = 0.0
        self.last_loop_time   = time.time()
        self.last_target_time = 0.0
        self.have_target      = False
        self.cmd_lin = 0.0
        self.cmd_ang = 0.0

        # IO
        self._location_subscriber = self.create_subscription(
            Point, const.OBJECT_LOCATION_TOPIC, self._location_callback, 10
        )
        self._vel_publisher = self.create_publisher(Twist, const.CMD_VEL_TOPIC, 10)

        # Periodic publisher + watchdog
        self.timer = self.create_timer(1.0 / self.pub_rate_hz, self._tick)

        self.get_logger().info('chase_object_node has been started.')

    def _location_callback(self, msg: Point):
        now = time.time()
        dt = max(1e-3, now - self.last_loop_time)
        self.last_loop_time = now

        actual_distance = msg.x
        actual_angle    = msg.y

        # Treat invalid/no-detect as no target; do NOT compute new commands
        if (actual_distance == -1.0) or (not math.isfinite(actual_distance)) or (not math.isfinite(actual_angle)):
            self.have_target = False
            return

        self.have_target = True
        self.last_target_time = now

        # --- Angular PI ---
        angular_error = -actual_angle  # target heading = 0
        if abs(angular_error) < const.DEL_A:
            angular_error = 0.0
            self.angular_integral = 0.0

        self.angular_integral += angular_error * dt
        self.angular_integral = max(min(self.angular_integral, const.INTEGRAL_CLAMP_ANGULAR),
                                    -const.INTEGRAL_CLAMP_ANGULAR)
        ang = (const.ANGULAR_KP * angular_error) + (const.ANGULAR_KI * self.angular_integral)

        # --- Linear PI ---
        linear_error = actual_distance - const.DESIRED_DISTANCE_M
        if abs(linear_error) < const.DEL_D:
            linear_error = 0.0
            self.linear_integral = 0.0

        self.linear_integral += linear_error * dt
        self.linear_integral = max(min(self.linear_integral, const.INTEGRAL_CLAMP_LINEAR),
                                   -const.INTEGRAL_CLAMP_LINEAR)
        lin = (const.LINEAR_KP * linear_error) + (const.LINEAR_KI * self.linear_integral)

        # Clamp outputs
        self.cmd_lin = max(min(lin,  self.max_lin), -self.max_lin)
        self.cmd_ang = max(min(-ang, self.max_ang), -self.max_ang)  # keep your original sign convention

    def _tick(self):
        """Periodic publisher & safety watchdog."""
        now = time.time()
        stale = (now - self.last_target_time) > self.timeout_s

        if (not self.have_target) or stale:
            # Publish zero if no target or stale
            self._publish_twist(0.0, 0.0)
            # Reset control state to avoid windup-driven lurch when target reappears
            self.angular_integral = 0.0
            self.linear_integral  = 0.0
            self.cmd_lin = 0.0
            self.cmd_ang = 0.0
            return

        # Normal periodic publish of most recent command
        self._publish_twist(self.cmd_lin, self.cmd_ang)

    def _publish_twist(self, lin, ang):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self._vel_publisher.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = ChaseObjectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
