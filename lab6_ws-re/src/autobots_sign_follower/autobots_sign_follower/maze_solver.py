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
import glob
import math
import time
import random
from collections import Counter
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
STATE_WIGGLE = 5
STATE_VERIFY_GOAL = 6 

# Scan Sub-states
SCAN_CENTER = 0
SCAN_LEFT   = 1
SCAN_RIGHT  = 2

# Verify Sub-states
VERIFY_INIT = 0
VERIFY_WIGGLE_1 = 1 
VERIFY_WIGGLE_2 = 2 
VERIFY_WIGGLE_3 = 3 
VERIFY_WIGGLE_4 = 4 
VERIFY_WIGGLE_5 = 5 
VERIFY_DONE     = 6

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')
        
        # --- 1. Configuration Constants ---
        self.STOP_DISTANCE = 0.55
        self.SAFETY_DISTANCE = 0.25
        self.DRIVE_SPEED = 0.12
        self.KP_CENTER = 0.8
        self.KP_HEADING = 2.0 
        
        # Alignment Constants
        self.ALIGN_TOLERANCE = 0.04
        self.ALIGN_SPEED = 0.15
        self.ALIGN_STUCK_LIMIT = 25
        
        # Emergency Unstuck Constants
        self.EMERGENCY_TIMEOUT = 4.0
        self.emergency_start_time = 0
        self.is_in_emergency = False
        
        # Turning PID Constants
        self.TURN_KP = 1.5
        self.TURN_KI = 0.02
        
        # Smart History
        self.last_turn_direction = -90 
        
        # Scan Logic Variables
        self.scan_stage = SCAN_CENTER
        self.scan_timer_start = 0
        self.align_stuck_counter = 0
        self.last_align_error = 100.0
        self.last_log_time = 0
        self.desired_heading = None
        self.wiggle_target = 0.0
        self.wiggle_description = "" # Debug text for wiggles
        self.accumulated_wiggle = 0.0 # Track total wiggle offset
        self.turn_description = "" # e.g., "Left (-90)"
        
        # Verification Variables
        self.verify_stage = VERIFY_INIT
        self.verify_votes = []
        
        # --- 2. Dynamic Model Loading ---
        pkg_share = get_package_share_directory('autobots_sign_follower')
        models_dir = os.path.join(pkg_share, 'models')
        model_files = glob.glob(os.path.join(models_dir, '*.pkl'))
        
        if not model_files:
            self.get_logger().error(f"FATAL: No .pkl model found in {models_dir}")
            self.model = None
        else:
            model_path = model_files[0]
            try:
                with open(model_path, 'rb') as f:
                    self.model = pickle.load(f)
                self.get_logger().info(f"Model loaded: {os.path.basename(model_path)}")
            except Exception as e:
                self.get_logger().error(f"FATAL: Failed to load model: {e}")
                self.model = None

        # --- 3. State Variables ---
        self.state = STATE_DRIVE
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.turn_integral = 0.0
        self.lidar_ranges = []
        
        # --- 4. ROS Setup ---
        self.bridge = CvBridge()
        
        lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 1)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=lidar_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/debug/model_view/compressed', 10)
        
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Maze Solver Initialized!")

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
            # Only process every 3rd frame to reduce lag
            # (Simple counter or time check could be added here)
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            
            # Only create debug image if we are actually going to publish it
            # Reduce resolution for speed
            debug_image = cv2.resize(cv_image, (320, 240))
            self.draw_status(debug_image)

            if self.state == STATE_VERIFY_GOAL:
                 if (self.get_clock().now().nanoseconds - self.scan_timer_start) > 0.5e9:
                    self.run_verification_step(cv_image, debug_image) # Use full res for logic
            
            elif self.state == STATE_SCAN:
                if (self.get_clock().now().nanoseconds - self.scan_timer_start) > 1e9:
                    self.run_scan_logic(cv_image, debug_image)
            
            self.publish_debug_image(debug_image)
        except Exception as e:
            self.get_logger().error(f"Vision error: {e}")

    # --- HELPER FUNCTIONS ---

    def get_state_name(self):
        states = {0: "DRIVE", 1: "SCAN", 2: "TURN", 3: "GOAL", 4: "ALIGN", 5: "WIGGLE", 6: "VERIFY"}
        name = states.get(self.state, "UNKNOWN")
        
        if self.state == STATE_TURN:
            name += f" ({self.turn_description})"
        return name

    def draw_status(self, image):
        # Smaller font (0.5) for speed and less clutter
        state_text = f"ST: {self.get_state_name()}"
        cv2.putText(image, state_text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        deg = math.degrees(self.current_yaw)
        cv2.putText(image, f"Y: {deg:.1f}", (5, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def publish_debug_image(self, cv_image):
        msg = self.bridge.cv2_to_compressed_imgmsg(cv_image)
        self.debug_pub.publish(msg)

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

    def get_augmented_votes(self, image):
        """Smart Augmentation Voting Strategy"""
        if self.model is None: return []
        
        images_to_check = [image]
        
        # Milder brightness/contrast
        images_to_check.append(cv2.convertScaleAbs(image, alpha=1.0, beta=20))  # Slight Bright
        images_to_check.append(cv2.convertScaleAbs(image, alpha=1.1, beta=0))   # Slight Contrast
        images_to_check.append(cv2.GaussianBlur(image, (3,3), 0))               # Slight Blur
        
        # Random Crop (Zoom in slightly)
        h, w, _ = image.shape
        crop_size = int(min(h, w) * 0.9)
        y_start = (h - crop_size) // 2
        x_start = (w - crop_size) // 2
        cropped_zoom = image[y_start:y_start+crop_size, x_start:x_start+crop_size]
        images_to_check.append(cv2.resize(cropped_zoom, (w, h)))

        votes = []
        for i, img in enumerate(images_to_check):
            cropped = utils.crop_sign(img)
            if cropped is not None:
                features = utils.extract_features(cropped)
                features = features.reshape(1, -1)
                base_pred = int(self.model.predict(features)[0])
                votes.append(base_pred)
                
                # --- CLASS-SPECIFIC AUGMENTATION VOTING ---
                # Add extra votes for invariant transformations based on initial prediction
                
                # Goal (5): Rotation Invariant (0, 90, 180, 270)
                if base_pred == 5:
                    for k in [1, 2, 3]: # 90, 180, 270
                        rot_img = np.rot90(cropped, k)
                        f_rot = utils.extract_features(rot_img).reshape(1, -1)
                        votes.append(int(self.model.predict(f_rot)[0]))
                
                # Goal (5), Stop (4), Do Not Enter (3), Empty (0): Flip Invariant
                if base_pred in [0, 3, 4, 5]:
                    # Horizontal Flip
                    flip_h = cv2.flip(cropped, 1)
                    f_h = utils.extract_features(flip_h).reshape(1, -1)
                    votes.append(int(self.model.predict(f_h)[0]))
                    
                # All Classes: Vertical Flip Invariant (Upside down left is still left)
                # (Actually, Left/Right might look weird upside down, but let's stick to Stop/Goal/DNE)
                if base_pred in [0, 3, 4, 5]:
                    # Vertical Flip
                    flip_v = cv2.flip(cropped, 0)
                    f_v = utils.extract_features(flip_v).reshape(1, -1)
                    votes.append(int(self.model.predict(f_v)[0]))

        return votes

    # --- VERIFICATION LOGIC ---

    def run_verification_step(self, image, debug_image):
        current_votes = self.get_augmented_votes(image)
        self.verify_votes.extend(current_votes)
        
        vote_text = f"V: {len(self.verify_votes)}"
        cv2.putText(debug_image, vote_text, (5, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if self.verify_stage == VERIFY_INIT:
            self.verify_stage = VERIFY_WIGGLE_1
            self.initiate_wiggle(-20, "Verify -20") 
        elif self.verify_stage == VERIFY_WIGGLE_1:
            self.verify_stage = VERIFY_WIGGLE_2
            self.initiate_wiggle(10, "Verify -10") 
        elif self.verify_stage == VERIFY_WIGGLE_2:
            self.verify_stage = VERIFY_WIGGLE_3
            self.initiate_wiggle(10, "Verify 0") 
        elif self.verify_stage == VERIFY_WIGGLE_3:
            self.verify_stage = VERIFY_WIGGLE_4
            self.initiate_wiggle(10, "Verify +10") 
        elif self.verify_stage == VERIFY_WIGGLE_4:
            self.verify_stage = VERIFY_WIGGLE_5
            self.initiate_wiggle(10, "Verify +20") 
        elif self.verify_stage == VERIFY_WIGGLE_5:
            self.conclude_verification()

    def conclude_verification(self):
        if not self.verify_votes:
            self.state = STATE_SCAN
            self.scan_stage = SCAN_CENTER
            self.accumulated_wiggle = 0.0 # Reset wiggle tracking
            return

        counts = Counter(self.verify_votes)
        most_common, count = counts.most_common(1)[0]
        total_votes = len(self.verify_votes)
        confidence = count / total_votes
        
        self.get_logger().info(f"VERIFICATION RESULTS: {counts} (Conf: {confidence:.2f})")
        
        if most_common == 5 and confidence > 0.6: 
            self.get_logger().info("GOAL CONFIRMED!")
            self.state = STATE_GOAL
        else:
            self.get_logger().warn(f"Goal Rejected! Correcting Angle.")
            # Correct accumulated wiggle before scanning again
            if abs(self.accumulated_wiggle) > 1.0:
                 self.initiate_turn(-self.accumulated_wiggle, "Undo Wiggle") 
            else:
                 self.state = STATE_SCAN
                 self.scan_stage = SCAN_CENTER
            
            self.accumulated_wiggle = 0.0

    # --- SCANNING LOGIC ---

    def run_scan_logic(self, image, debug_image):
        votes = self.get_augmented_votes(image)
        final_decision = 0
        if votes:
            final_decision = Counter(votes).most_common(1)[0][0]
            
        if final_decision != 0:
             cv2.putText(debug_image, f"Vote: {final_decision}", (5, 75), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if final_decision != 0:
            if abs(self.accumulated_wiggle) > 1.0:
                 self.get_logger().info(f"Correcting wiggle ({self.accumulated_wiggle} deg) before action.")
                 self.initiate_turn(-self.accumulated_wiggle, "Center Action")
                 self.pending_decision = final_decision
                 self.state = STATE_TURN 
                 return 

            self.execute_decision(final_decision)
            return

        # Wiggle Logic
        if self.scan_stage == SCAN_CENTER:
            self.scan_stage = SCAN_LEFT
            self.initiate_wiggle(20, "Scan Left") 

        elif self.scan_stage == SCAN_LEFT:
            self.scan_stage = SCAN_RIGHT
            self.initiate_wiggle(-40, "Scan Right") 

        elif self.scan_stage == SCAN_RIGHT:
            self.get_logger().warn("Scan failed. Correcting & Fallback.")
            self.initiate_wiggle(20, "Center")
            self.scan_stage = 99 # Flag for fallback

    def execute_decision(self, sign_class):
        if sign_class == 5:
            self.state = STATE_VERIFY_GOAL
            self.verify_stage = VERIFY_INIT
            self.verify_votes = []
            self.scan_timer_start = self.get_clock().now().nanoseconds
            self.accumulated_wiggle = 0.0 # Reset
            
        elif sign_class == 1:
            self.last_turn_direction = 90 
            corrected_turn = 90 - self.accumulated_wiggle
            self.initiate_turn(corrected_turn, "Left")
            
        elif sign_class == 2:
            self.last_turn_direction = -90 
            corrected_turn = -90 - self.accumulated_wiggle
            self.initiate_turn(corrected_turn, "Right")
            
        elif sign_class in [3, 4]:
            corrected_turn = 180 - self.accumulated_wiggle
            self.initiate_turn(corrected_turn, "U-Turn")
            
        else:
            self.initiate_turn(self.last_turn_direction, "Fallback")

    # --- MOTION CONTROL ---

    def initiate_turn(self, degrees, desc="Turn"):
        self.target_yaw = self.current_yaw + math.radians(degrees)
        if self.target_yaw > math.pi: self.target_yaw -= 2 * math.pi
        elif self.target_yaw < -math.pi: self.target_yaw += 2 * math.pi 
        
        self.turn_integral = 0.0
        self.state = STATE_TURN
        self.desired_heading = None 
        self.turn_description = f"{desc} ({degrees}d)" # Explicit debug text
        self.accumulated_wiggle = 0.0 
        self.get_logger().info(f"Turning: {self.turn_description}")

    def initiate_wiggle(self, degrees, desc="Wiggle"):
        self.target_yaw = self.current_yaw + math.radians(degrees)
        if self.target_yaw > math.pi: self.target_yaw -= 2 * math.pi
        elif self.target_yaw < -math.pi: self.target_yaw += 2 * math.pi
        
        self.turn_integral = 0.0
        self.wiggle_description = desc
        self.accumulated_wiggle += degrees # Track total offset
        
        if self.state == STATE_VERIFY_GOAL:
             pass 
        else:
             self.state = STATE_WIGGLE
             
        self.get_logger().info(f"Wiggle {degrees} ({desc})")

    def control_loop(self):
        if not self.lidar_ranges: return
        n = len(self.lidar_ranges)
        if n < 100: return
        twist = Twist()
        
        # SAFETY
        front_cone_indices = list(range(0, 20)) + list(range(n-20, n))
        min_safety_dist = 10.0
        for i in front_cone_indices:
            if i < n and self.lidar_ranges[i] > 0.01:
                if self.lidar_ranges[i] < min_safety_dist:
                    min_safety_dist = self.lidar_ranges[i]
        
        if min_safety_dist < self.SAFETY_DISTANCE:
            if not self.is_in_emergency:
                self.is_in_emergency = True
                self.emergency_start_time = time.time()
                self.get_logger().warn(f"EMERGENCY: {min_safety_dist:.2f}m")

            if (time.time() - self.emergency_start_time) > self.EMERGENCY_TIMEOUT:
                 self.get_logger().warn("Stuck! Escaping.")
                 twist.linear.x = -0.1
                 twist.angular.z = random.uniform(-0.5, 0.5) 
                 self.cmd_pub.publish(twist)
                 return

            twist.linear.x = -0.05 
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return
        else:
            self.is_in_emergency = False 

        # --- STATE: DRIVE ---
        if self.state == STATE_DRIVE:
            if self.desired_heading is None:
                self.desired_heading = self.current_yaw
            
            idx_lf = int(n * 0.05)
            idx_rf = int(n * 0.95)
            r_front = self.lidar_ranges[0] if self.lidar_ranges[0] > 0.01 else 10.0
            r_lf = self.lidar_ranges[idx_lf] if self.lidar_ranges[idx_lf] > 0.01 else 10.0
            r_rf = self.lidar_ranges[idx_rf] if self.lidar_ranges[idx_rf] > 0.01 else 10.0
            
            front_dist = min(r_front, r_lf, r_rf)
            angle_error = r_lf - r_rf

            if front_dist < self.STOP_DISTANCE:
                self.get_logger().info(f"Wall ({front_dist:.2f}m).")
                
                # CRITICAL FIX: Corner Check
                is_solid_wall = (r_lf < 1.0) and (r_rf < 1.0)
                
                if is_solid_wall and abs(angle_error) > 0.03:
                    self.get_logger().info(f"Misaligned. Aligning.")
                    self.state = STATE_ALIGN
                    self.last_align_error = 100.0
                    self.align_stuck_counter = 0
                    self.align_start_time = time.time()
                else:
                    self.get_logger().info("Ready to Scan.")
                    self.state = STATE_SCAN
                    self.scan_stage = SCAN_CENTER 
                    self.scan_timer_start = self.get_clock().now().nanoseconds
                    self.accumulated_wiggle = 0.0
                
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return

            # Centering
            heading_err = self.normalize_angle(self.desired_heading - self.current_yaw)
            left_dist = self.get_lidar_slice(n, 0.25)
            right_dist = self.get_lidar_slice(n, 0.75)
            
            center_error = 0.0
            if left_dist < 1.0 and right_dist < 1.0:
                center_error = (left_dist - right_dist)
            elif left_dist < 1.0: center_error = (left_dist - 0.5)
            elif right_dist < 1.0: center_error = (0.5 - right_dist)
            
            center_error = max(min(center_error, 1.0), -1.0)
            twist.linear.x = self.DRIVE_SPEED
            twist.angular.z = (heading_err * self.KP_HEADING) + (center_error * self.KP_CENTER)
            self.cmd_pub.publish(twist)

        # --- STATE: ALIGN ---
        elif self.state == STATE_ALIGN:
            idx_lf = int(n * 0.05)
            idx_rf = int(n * 0.95)
            r_lf = self.lidar_ranges[idx_lf] if self.lidar_ranges[idx_lf] > 0.01 else 10.0
            r_rf = self.lidar_ranges[idx_rf] if self.lidar_ranges[idx_rf] > 0.01 else 10.0
            
            # Abort if wall disappears (Corner case)
            if r_lf > 1.2 or r_rf > 1.2: 
                self.get_logger().warn("Lost wall. Scan.")
                self.state = STATE_SCAN
                self.scan_stage = SCAN_CENTER
                self.scan_timer_start = self.get_clock().now().nanoseconds
                self.accumulated_wiggle = 0.0
                return

            current_error = abs(r_lf - r_rf)
            align_error_signed = r_lf - r_rf
            
            if current_error >= self.last_align_error - 0.001: 
                self.align_stuck_counter += 1
            else: self.align_stuck_counter = 0
            self.last_align_error = current_error
            
            if self.align_stuck_counter > self.ALIGN_STUCK_LIMIT:
                self.get_logger().warn("Align Stuck. Scan.")
                self.state = STATE_SCAN
                self.scan_stage = SCAN_CENTER
                self.scan_timer_start = self.get_clock().now().nanoseconds
                self.accumulated_wiggle = 0.0
                return

            if current_error < self.ALIGN_TOLERANCE:
                self.get_logger().info("Aligned.")
                self.state = STATE_SCAN
                self.scan_stage = SCAN_CENTER
                self.scan_timer_start = self.get_clock().now().nanoseconds
                self.accumulated_wiggle = 0.0
                twist.angular.z = 0.0
            else:
                cmd = align_error_signed * 1.5 
                cmd = max(min(cmd, self.ALIGN_SPEED), -self.ALIGN_SPEED)
                twist.angular.z = cmd
            
            self.cmd_pub.publish(twist)

        # --- STATE: SCAN / VERIFY ---
        elif self.state == STATE_SCAN or self.state == STATE_VERIFY_GOAL:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

        # --- STATE: WIGGLE / TURN ---
        elif self.state == STATE_TURN or self.state == STATE_WIGGLE:
            twist.linear.x = 0.0
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            
            self.turn_integral += yaw_error
            self.turn_integral = max(min(self.turn_integral, 0.5), -0.5)
            pid_output = (self.TURN_KP * yaw_error) + (self.TURN_KI * self.turn_integral)
            
            if pid_output > 0: angular_speed = max(min(pid_output, 0.6), 0.25)
            else: angular_speed = min(max(pid_output, -0.6), -0.25)
            
            if abs(yaw_error) < 0.08:
                self.get_logger().info("Turn Done.")
                
                if self.state == STATE_WIGGLE:
                    if self.scan_stage == 99:
                        self.state = STATE_SCAN 
                        self.execute_decision(0)
                    else:
                        self.scan_timer_start = self.get_clock().now().nanoseconds
                        if self.verify_stage > 0:
                             self.state = STATE_VERIFY_GOAL
                        else:
                             self.state = STATE_SCAN
                else:
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
            if self.lidar_ranges[0] > 1.0: 
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