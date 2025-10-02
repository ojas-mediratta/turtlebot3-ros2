import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

from . import constants as const

class DetectObjectNode(Node):
    def __init__(self):
        super().__init__('detect_object_node')
        self.bridge = CvBridge()

        # Camera intrinsics (optional; populated from CameraInfo)
        self.fx = None
        self.cx = None

        # --- QoS Profile & Subscribers ---
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self._image_subscriber = self.create_subscription(
            CompressedImage, const.RAW_IMAGE_TOPIC, self._image_callback, image_qos_profile
        )

        # Subscribe to CameraInfo if your camera publishes it
        self._cinfo_subscriber = self.create_subscription(
            CameraInfo, const.CAMERA_INFO_TOPIC, self._caminfo_callback, 10
        )

        # --- Publishers ---
        self._angle_publisher = self.create_publisher(Float32, const.OBJECT_ANGLE_TOPIC, 10)
        self._processed_image_publisher = self.create_publisher(CompressedImage, const.PROCESSED_IMAGE_TOPIC, 10)

        self.get_logger().info('detect_object_node has been started.')

    def _caminfo_callback(self, msg: CameraInfo):
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = float(msg.k[0])
        self.cx = float(msg.k[2])

    def _image_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        h, w = frame.shape[:2]
        min_area = const.MIN_AREA_RATIO * (w * h)

        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)

        # Two-range HSV mask (e.g., for red wrap-around)
        mask1 = cv2.inRange(hsv, (const.H_LOW,  const.S_LOW,  const.V_LOW),
                                  (const.H_HIGH, const.S_HIGH, const.V_HIGH))
        mask2 = cv2.inRange(hsv, (const.H_LOW2,  const.S_LOW2,  const.V_LOW2),
                                  (const.H_HIGH2, const.S_HIGH2, const.V_HIGH2))
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((6, 6), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found = False
        if cnts:
            largest = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > min_area:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx_pix = int(M["m10"] / M["m00"])
                    cy_pix = int(M["m01"] / M["m00"])

                    # --- Angle computation ---
                    if self.fx is not None and self.cx is not None:
                        # Use pinhole model if intrinsics are known
                        x_norm = (cx_pix - self.cx) / self.fx
                        angle_radians = math.atan2(x_norm, 1.0)
                    else:
                        # Fallback: use HFOV with the ACTUAL image width
                        center_x = w / 2.0
                        pixel_error = cx_pix - center_x
                        angle_radians = pixel_error * (const.HORIZONTAL_FOV_RADIANS / w)

                    self._angle_publisher.publish(Float32(data=float(angle_radians)))

                    # Draw for visualization
                    cv2.drawContours(frame, [largest], -1, (0, 255, 0), 3)
                    cv2.circle(frame, (cx_pix, cy_pix), 7, (0, 0, 255), -1)
                    found = True

        if not found:
            self._angle_publisher.publish(Float32(data=float('nan')))

        processed_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
        self._processed_image_publisher.publish(processed_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DetectObjectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
