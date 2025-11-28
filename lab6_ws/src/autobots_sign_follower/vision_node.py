#!/usr/bin/env python3
"""Vision node: loads existing classifier and performs sign classification on request.

Subscribes:
  - `/image_raw/compressed` (sensor_msgs/CompressedImage)
  - `/classify_trigger` (std_msgs/Bool)  -- when True, run classification on latest image

Publishes:
  - `/sign_class` (std_msgs/Int32)
  - `/sign_confidence` (std_msgs/Float32)
"""
import os
import pickle
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32, Float32
from cv_bridge import CvBridge


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Parameters
        self.declare_parameter('model_path', None)
        self.declare_parameter('img_size', [128, 128])
        self.declare_parameter('confidence_threshold', 0.5)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        img_size = self.get_parameter('img_size').get_parameter_value().integer_array_value
        # Tunable confidence threshold for accepting a predicted class
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # Resolve default model path if not provided
        if not model_path:
            # model expected at lab6_ws/model_final.pkl relative to this file
            pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
            model_path = os.path.join(pkg_dir, 'model_final.pkl')

        # Import local preprocessing utilities implemented as part of lab6 (part 1).
        # Prefer the package-local `utils.py`. Fall back to other known locations
        # in the repository (e.g. `lab6_re/supplemental/utils.py`) so the node can
        # still run if the code was left in the separate lab6-re folder.
        try:
            from .utils import crop_sign, extract_features, IMG_SIZE
        except Exception:
            try:
                from autobots_sign_follower.utils import crop_sign, extract_features, IMG_SIZE
            except Exception:
                try:
                    from lab6_re.supplemental.utils import crop_sign, extract_features, IMG_SIZE
                except Exception as e:
                    self.get_logger().error(f'Failed to import local vision utilities: {e}')
                    raise

        self.crop_sign = crop_sign
        self.extract_features = extract_features

        # Load model
        if not os.path.isfile(model_path):
            self.get_logger().warning(f'Model file not found at {model_path}. Vision node will still run but classification will fail.')
            self.model = None
        else:
            with open(model_path, 'rb') as f:
                self.model = pickle.load(f)
            self.get_logger().info(f'Loaded model from {model_path}')

        # State
        self.latest_frame = None
        self.bridge = CvBridge()

        # Subscribers
        qos_profile = 1
        self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_cb, qos_profile)
        self.create_subscription(Bool, '/classify_trigger', self.trigger_cb, 10)

        # Publishers
        self.pub_class = self.create_publisher(Int32, '/sign_class', 10)
        self.pub_conf = self.create_publisher(Float32, '/sign_confidence', 10)

        self.get_logger().info('vision_node started')

    def image_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.latest_frame = frame
        except Exception as e:
            self.get_logger().warning(f'Failed to decode compressed image: {e}')

    def trigger_cb(self, msg: Bool):
        if not msg.data:
            return
        # Only classify if we have a recent frame
        if self.latest_frame is None:
            self.get_logger().warning('Classification requested but no image received yet')
            return

        frame = self.latest_frame.copy()
        cropped = self.crop_sign(frame)
        if cropped is None:
            # Class 0: empty wall
            self.pub_class.publish(Int32(data=0))
            self.pub_conf.publish(Float32(data=1.0))
            self.get_logger().info('crop_sign returned None -> publishing class 0')
            return

        # Extract features exactly as training did
        feats = self.extract_features(cropped)
        feats = np.array(feats).reshape(1, -1)

        if self.model is None:
            self.get_logger().warning('No model loaded; cannot classify')
            return

        try:
            pred = int(self.model.predict(feats)[0])
            # Try to get confidence via predict_proba when available
            conf = 0.0
            if hasattr(self.model, 'predict_proba'):
                probs = self.model.predict_proba(feats)[0]
                # map classes (model trained on classes 1-5) into indices
                # If model doesn't include class 0, handle gracefully
                try:
                    cls_index = list(self.model.classes_).index(pred)
                    conf = float(probs[cls_index])
                except Exception:
                    conf = float(max(probs))
            else:
                conf = 1.0

            # If confidence below threshold, treat as no-detection (class 0)
            if conf < self.confidence_threshold:
                self.pub_class.publish(Int32(data=0))
                self.pub_conf.publish(Float32(data=conf))
                self.get_logger().info(f'Prediction confidence {conf:.2f} below threshold {self.confidence_threshold:.2f} -> publishing class 0')
                return

            self.pub_class.publish(Int32(data=pred))
            self.pub_conf.publish(Float32(data=conf))
            self.get_logger().info(f'Published sign_class={pred} (conf={conf:.2f})')
        except Exception as e:
            self.get_logger().error(f'Classification failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
