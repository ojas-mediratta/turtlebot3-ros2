import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from . import constants as const

class GetObjectRangeNode(Node):
    def __init__(self):
        super().__init__('get_object_range_node')
        self.last_angle = None
        self.last_scan = None
        
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._angle_subscriber = self.create_subscription(Float32, const.OBJECT_ANGLE_TOPIC, self._angle_callback, 10)
        self._scan_subscriber = self.create_subscription(LaserScan, const.LIDAR_TOPIC, self._scan_callback, qos_profile=sensor_qos_profile)
        self._location_publisher = self.create_publisher(Point, const.OBJECT_LOCATION_TOPIC, 10)
        self.get_logger().info('get_object_range_node (Angle Corrected) has been started.')

    def _angle_callback(self, msg):
        self.last_angle = msg.data

    def _scan_callback(self, msg):
        self.last_scan = msg
        if self.last_angle is None or self.last_scan is None or math.isnan(self.last_angle):
            return

        # --- ANGLE NORMALIZATION FIX ---
        # The camera gives a negative angle for the left side, but the LIDAR
        # scan is indexed from 0 to 2*PI. We need to convert the angle.
        angle_from_camera = self.last_angle
        if angle_from_camera < 0:
            angle_from_camera += 2 * math.pi

        # Now, use the normalized angle for the calculation
        index = int((angle_from_camera - self.last_scan.angle_min) / self.last_scan.angle_increment)

        if 0 <= index < len(self.last_scan.ranges):
            distance = self.last_scan.ranges[index]
            if not math.isinf(distance) and distance > 0.01:
                # IMPORTANT: We still publish the ORIGINAL angle (-pi to +pi)
                # because the chase_object controller is expecting it.
                location_msg = Point(x=distance, y=self.last_angle, z=0.0)
                self._location_publisher.publish(location_msg)
            else:
                self._publish_not_found()
        else:
            self._publish_not_found()

    def _publish_not_found(self):
        location_msg = Point(x=-1.0, y=-1.0, z=0.0)
        self._location_publisher.publish(location_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GetObjectRangeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()