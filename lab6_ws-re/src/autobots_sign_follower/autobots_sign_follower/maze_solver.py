import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import pickle
import os
import math
import time
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Import your utils
from . import utils

# --- STATE DEFINITIONS ---
STATE_DRIVE = 0
STATE_SCAN  = 1
STATE_TURN  = 2
STATE_GOAL  = 3
STATE_ALIGN = 4 

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')
        
        # --- 1. Configuration Constants ---
        self.STOP_DISTANCE = 0.55
        self.DRIVE_SPEED = 0.12
        self.KP_CENTER = 0.8
        self.KP_HEADING = 2.0 
        
        # Alignment Constants
        self.ALIGN_TOLERANCE = 0.03
        self.ALIGN_SPEED = 0.15
        
        # New Adaptive Timeout Variables
        self.last_align_error = 100.0
        self.align_stuck_counter = 0
        self.ALIGN_STUCK_LIMIT = 20 # ~2 seconds of no progress (at 10Hz)
        
        # Turning PID Constants
        self.TURN_KP = 1.5
        self.TURN_KI = 0.02
        self.turn_integral = 0.0
        
        # Smart History
        self.last_turn_direction = -90 # Default to Right if no history
        
        # --- 2. Load Model ---
        pkg_share = get_package_share_directory('autobots_sign_follower')
        model_path = os.path.join(pkg_share, 'models', 'svm_rbf_fall25.pkl')
        
        try:
            with open(model_path, 'rb') as f:
                self.model = pickle.load(f)
            self.get_logger().info(f"Model loaded: {model_path}")
        except Exception as e:
            self.get_logger().error(f"FATAL: Model load failed: {e}")
            self.model = None

        # --- 3. State Variables ---
        self.state = STATE_DRIVE
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.lidar_ranges = []
        self.scan_start_time = 0
        self.last_log_time = 0
        
        self.desired_heading = None
        
        # --- 4. ROS Setup ---
        self.bridge = CvBridge()
        
        lidar_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 1)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=lidar_qos_profile)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/debug/model_view/compressed', 10)
        
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Maze Solver Brain Initialized! Waiting for sensors...")

    # --- SENSOR CALLBACKS ---
    
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        self.lidar_ranges = msg.ranges

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            debug_image = cv_image.copy()
            self.draw_status(debug_image)

            if self.state == STATE_SCAN:
                if (self.get_clock().now().nanoseconds - self.scan_start_time) > 1e9:
                    self.process_sign(cv_image, debug_image)
            
            self.publish_debug_image(debug_image)

        except Exception as e:
            self.get_logger().error(f"Vision error: {e}")

    def get_state_name(self):
        if self.state == STATE_DRIVE: return "DRIVE"
        if self.state == STATE_SCAN: return "SCAN"
        if self.state == STATE_TURN: return "TURN"
        if self.state == STATE_GOAL: return "GOAL"
        if self.state == STATE_ALIGN: return "ALIGN"
        return "UNKNOWN"

    def draw_status(self, image):
        cv2.putText(image, f"STATE: {self.get_state_name()}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        deg = math.degrees(self.current_yaw)
        cv2.putText(image, f"Yaw: {deg:.1f}", (10, 110), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # --- CORE LOGIC ---

    def process_sign(self, image, debug_image):
        if self.model is None: return

        cropped = utils.crop_sign(image)
        sign_class = 0
        sign_name = "None"
        
        if cropped is not None:
            features = utils.extract_features(cropped)
            features = features.reshape(1, -1)
            sign_class = int(self.model.predict(features)[0])
            
            names = {0: "Empty", 1: "Left", 2: "Right", 3: "Do Not Enter", 4: "Stop", 5: "GOAL"}
            sign_name = names.get(sign_class, "Unknown")
            
            cv2.putText(debug_image, f"DETECTED: {sign_name} ({sign_class})", (10, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            try:
                h, w, _ = cropped.shape
                if h < debug_image.shape[0] and w < debug_image.shape[1]:
                    debug_image[debug_image.shape[0]-h:, debug_image.shape[1]-w:] = cropped
                    cv2.rectangle(debug_image, (debug_image.shape[1]-w, debug_image.shape[0]-h), 
                                 (debug_image.shape[1], debug_image.shape[0]), (0, 255, 0), 2)
            except: pass
            
            self.get_logger().info(f">>> SVM PREDICTION: Class {sign_class} ({sign_name}) <<<")
        else:
            cv2.putText(debug_image, "NO SIGN FOUND", (10, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            self.get_logger().info(">>> No Sign Detected (Class 0) <<<")

        if sign_class == 5:
            self.state = STATE_GOAL
        elif sign_class == 1:
            self.last_turn_direction = 90 # Cache Left
            self.initiate_turn(90)
        elif sign_class == 2:
            self.last_turn_direction = -90 # Cache Right
            self.initiate_turn(-90)
        elif sign_class in [3, 4]:
            self.initiate_turn(180)
        else:
            self.get_logger().warn(f"Nothing detected! Using cached default: {self.last_turn_direction} deg")
            self.initiate_turn(self.last_turn_direction)

    def publish_debug_image(self, cv_image):
        msg = self.bridge.cv2_to_compressed_imgmsg(cv_image)
        self.debug_pub.publish(msg)

    def initiate_turn(self, degrees):
        self.target_yaw = self.current_yaw + math.radians(degrees)
        if self.target_yaw > math.pi: self.target_yaw -= 2 * math.pi
        elif self.target_yaw < -math.pi: self.target_yaw += 2 * math.pi 
        
        self.turn_integral = 0.0 
        self.state = STATE_TURN
        self.desired_heading = None 
        self.get_logger().info(f"Turning {degrees} degrees... Target: {self.target_yaw:.2f}")

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle

    def get_lidar_slice(self, n, percentage, slice_width=5):
        idx = int(n * percentage)
        start = max(0, idx - slice_width)
        end = min(n, idx + slice_width)
        vals = [x for x in self.lidar_ranges[start:end] if x > 0.01]
        return np.mean(vals) if vals else 10.0

    def control_loop(self):
        if not self.lidar_ranges: return
        n = len(self.lidar_ranges)
        if n < 100: return

        twist = Twist()

        # --- STATE: DRIVE ---
        if self.state == STATE_DRIVE:
            if self.desired_heading is None:
                self.desired_heading = self.current_yaw
            
            idx_front = 0
            idx_lf = int(n * 0.05)
            idx_rf = int(n * 0.95)
            
            r_front = self.lidar_ranges[idx_front] if self.lidar_ranges[idx_front] > 0.01 else 10.0
            r_lf = self.lidar_ranges[idx_lf] if self.lidar_ranges[idx_lf] > 0.01 else 10.0
            r_rf = self.lidar_ranges[idx_rf] if self.lidar_ranges[idx_rf] > 0.01 else 10.0
            
            front_dist = min(r_front, r_lf, r_rf)
            angle_error = r_lf - r_rf

            if front_dist < self.STOP_DISTANCE:
                self.get_logger().info(f"Wall detected ({front_dist:.2f}m)!")
                
                if abs(angle_error) > 0.03: 
                    self.get_logger().info(f"Misaligned (Err: {angle_error:.3f}). Switching to ALIGN.")
                    self.state = STATE_ALIGN
                    self.last_align_error = 100.0 # Reset tracking
                    self.align_stuck_counter = 0
                else:
                    self.get_logger().info("Aligned. Scanning.")
                    self.state = STATE_SCAN
                    self.scan_start_time = self.get_clock().now().nanoseconds
                
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return

            heading_err = self.normalize_angle(self.desired_heading - self.current_yaw)
            
            left_dist = self.get_lidar_slice(n, 0.25)
            right_dist = self.get_lidar_slice(n, 0.75)
            
            center_error = 0.0
            if left_dist < 1.0 and right_dist < 1.0:
                center_error = (left_dist - right_dist)
            elif left_dist < 1.0: 
                center_error = (left_dist - 0.5)
            elif right_dist < 1.0: 
                center_error = (0.5 - right_dist)
            
            center_error = max(min(center_error, 1.0), -1.0)
            
            twist.linear.x = self.DRIVE_SPEED
            twist.angular.z = (heading_err * self.KP_HEADING) + (center_error * self.KP_CENTER)
            self.cmd_pub.publish(twist)

        # --- STATE: ALIGN (Adaptive Timeout) ---
        elif self.state == STATE_ALIGN:
            idx_lf = int(n * 0.05)
            idx_rf = int(n * 0.95)
            r_lf = self.lidar_ranges[idx_lf] if self.lidar_ranges[idx_lf] > 0.01 else 10.0
            r_rf = self.lidar_ranges[idx_rf] if self.lidar_ranges[idx_rf] > 0.01 else 10.0
            
            current_error = abs(r_lf - r_rf)
            
            if current_error >= self.last_align_error - 0.001: 
                self.align_stuck_counter += 1
            else:
                self.align_stuck_counter = 0 
            
            self.last_align_error = current_error
            
            # Use cached turn direction on timeout
            if self.align_stuck_counter > self.ALIGN_STUCK_LIMIT:
                self.get_logger().warn(f"Alignment Stuck! Executing fallback turn ({self.last_turn_direction} deg).")
                self.initiate_turn(self.last_turn_direction)
                return

            align_error_signed = r_lf - r_rf
            self.log_throttle(f"Aligning... Err: {align_error_signed:.3f} | Stuck: {self.align_stuck_counter}")
            
            if current_error < self.ALIGN_TOLERANCE:
                self.get_logger().info("Alignment Success.")
                self.state = STATE_SCAN
                self.scan_start_time = self.get_clock().now().nanoseconds
                twist.angular.z = 0.0
            else:
                cmd = align_error_signed * 1.5 
                cmd = max(min(cmd, self.ALIGN_SPEED), -self.ALIGN_SPEED)
                twist.angular.z = cmd
                twist.linear.x = 0.0
            
            self.cmd_pub.publish(twist)

        # --- STATE: SCAN ---
        elif self.state == STATE_SCAN:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

        # --- STATE: TURN ---
        elif self.state == STATE_TURN:
            twist.linear.x = 0.0
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            
            self.turn_integral += yaw_error
            self.turn_integral = max(min(self.turn_integral, 0.5), -0.5)
            pid_output = (self.TURN_KP * yaw_error) + (self.TURN_KI * self.turn_integral)
            
            if pid_output > 0:
                angular_speed = max(min(pid_output, 0.6), 0.25)
            else:
                angular_speed = min(max(pid_output, -0.6), -0.25)
            
            self.log_throttle(f"Turning... Error: {yaw_error:.2f} rad")
            
            if abs(yaw_error) < 0.08:
                self.get_logger().info("Turn Complete.")
                self.state = STATE_DRIVE
                self.desired_heading = None 
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return

            twist.angular.z = angular_speed
            self.cmd_pub.publish(twist)

        # --- STATE: GOAL ---
        elif self.state == STATE_GOAL:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            
            front_dist = self.lidar_ranges[0]
            if front_dist > 1.0:
                self.get_logger().info("Goal Reset: Moving!")
                self.state = STATE_DRIVE
                self.desired_heading = None

    def log_throttle(self, msg):
        now = time.time()
        if now - self.last_log_time > 1.0:
            self.get_logger().info(msg)
            self.last_log_time = now

def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()