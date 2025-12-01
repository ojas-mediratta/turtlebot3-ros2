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
import threading # Added for sound thread
from collections import Counter
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Try importing TurtleBot3 Sound message
try:
    from turtlebot3_msgs.msg import Sound
    HAS_SOUND_MSG = True
except ImportError:
    HAS_SOUND_MSG = False

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
STATE_PREPARE_GOAL = 7 # NEW: Align and backup before verification
STATE_PREPARE_UTURN = 8 # NEW: Align and approach before verifying U-turn

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
        self.SAFETY_DISTANCE = 0.20 # Reduced from 0.25
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
        
        # Anti-Flip-Flop Logic
        self.last_strong_decision = 0
        self.last_strong_time = 0.0
        self.strong_threshold = 0.65  # Confidence required to set a strong precedent
        self.conflict_timeout = 4.0   # Seconds to reject conflicting weak signals
        
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
        self.verify_failure_count = 0 # Track consecutive failures to detect corner traps
        # Decision hysteresis / cooldown (seconds)
        self.last_scan_decision = 0
        self.last_scan_time = 0.0
        self.decision_cooldown_until = 0.0
        self.decision_hysteresis_time = 2.0  # require same decision to persist within this window
        self.decision_cooldown_seconds = 2.0  # after executing a decision, ignore new ones for this many seconds
        # Pending decision (used when a corrective turn is performed first)
        self.pending_decision = None
        # Settle after turn to avoid moving while still rotating
        self.settle_after_turn = 0.35
        self.settle_until = 0.0
        
        # Music/Sound State
        self.music_playing = False
        self.music_thread = None

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
        
        # Sound Publisher
        if HAS_SOUND_MSG:
            self.sound_pub = self.create_publisher(Sound, '/sound', 10)
            self.get_logger().info("Audio System: TurtleBot3 /sound topic enabled.")
        else:
            self.sound_pub = None
            self.get_logger().warn("Audio System: turtlebot3_msgs not found. Sound disabled.")
        
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
        states = {
            0: "DRIVE", 1: "SCAN", 2: "TURN", 3: "GOAL", 
            4: "ALIGN", 5: "WIGGLE", 6: "VERIFY", 7: "PREP_GOAL", 8: "PREP_UTURN"
        }
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
    
    def get_front_wall_angle(self):
        """
        Calculates the angle of the wall in front of the robot using LiDAR.
        Returns the angle in degrees (relative to the robot's current heading).
        Standard wall is ~90 degrees.
        Returns None if fitting fails or insufficient data.
        """
        if not self.lidar_ranges: return None
        n = len(self.lidar_ranges)
        
        # Use a NARROWER cone of +/- 12 indices for robust fitting
        indices = list(range(0, 12)) + list(range(n-12, n))
        
        xs = []
        ys = []
        
        for i in indices:
            r = self.lidar_ranges[i]
            if 0.1 < r < 2.0: # Filter for valid, close points
                # Calculate angle for this ray
                if i < n/2:
                    theta = i * (2 * math.pi / n)
                else:
                    theta = (i - n) * (2 * math.pi / n)
                
                # Convert to Cartesian (x=forward, y=left)
                xs.append(r * math.cos(theta))
                ys.append(r * math.sin(theta))
                
        if len(xs) < 10:
            return None # Not enough points
            
        try:
            # Fit line x = my + c
            # We use y as independent variable because the wall is roughly lateral
            # (vertical line in x-y plot), so slope in y vs x might be infinite.
            # slope 'm' here represents the 'tilt' of the wall.
            m, c = np.polyfit(ys, xs, 1)
            
            # The vector running along the wall (to the left, +y direction)
            # has components (dx, dy) = (m, 1)
            # Angle of this vector relative to robot x-axis:
            angle_rad = math.atan2(1, m)
            
            # atan2(1, m) result is always in (0, pi) because y=1 is positive.
            # This corresponds perfectly to a "Left" turning angle (0 to 180).
            return math.degrees(angle_rad)
            
        except Exception as e:
            self.get_logger().warn(f"Wall Angle Fit Error: {e}")
            return None

    def get_dominant_color_group(self, image):
        """
        Determines if the sign is in the RED group (Stop, U-Turn, Goal)
        or the BLUE/GREEN group (Left, Right).
        Returns 'RED', 'BLUE', or 'NONE'.
        
        IMPROVED: Wider ranges to catch all variations.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # --- RED RANGES (Strict to avoid Brown) ---
        # Brown/Wood: Hue 10-25, Low-Med Saturation/Value.
        # We target bright Red paint: High Saturation, High Value.
        
        # Lower Red (0-9)
        lower_red1 = np.array([0, 70, 70])
        upper_red1 = np.array([9, 255, 255])
        # Upper Red (171-180)
        lower_red2 = np.array([171, 70, 70])
        upper_red2 = np.array([180, 255, 255])
        
        mask_r1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_r2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_r1 | mask_r2
        
        # --- BLUE/GREEN/YELLOW RANGES ---
        # Catch light yellow/green/cyan/blue paint
        # Broad range to catch all non-red signals
        lower_blue_green = np.array([20, 50, 60]) 
        upper_blue_green = np.array([160, 255, 255])
        
        mask_blue = cv2.inRange(hsv, lower_blue_green, upper_blue_green)
        
        r_count = cv2.countNonZero(mask_red)
        b_count = cv2.countNonZero(mask_blue)
        
        # Threshold: require at least some pixels to make a call
        total_pixels = image.shape[0] * image.shape[1]
        if r_count < (total_pixels * 0.05) and b_count < (total_pixels * 0.05):
            return 'NONE' # Not enough color to decide
        
        if r_count > b_count: return 'RED'
        return 'BLUE'

    def get_color_presence(self, image):
        """
        Analyzes the image for Blue/Green vs Red content.
        Returns two booleans: (is_blue_present, is_red_present)
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # --- RED RANGES (Strict to avoid Brown) ---
        # Brown/Wood: Hue 10-25, Low-Med Saturation/Value.
        # We target bright Red paint: High Saturation, High Value.
        
        # Lower Red (0-9)
        lower_red1 = np.array([0, 70, 70])
        upper_red1 = np.array([9, 255, 255])
        # Upper Red (171-180)
        lower_red2 = np.array([171, 70, 70])
        upper_red2 = np.array([180, 255, 255])
        
        mask_r1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_r2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_r1 | mask_r2
        
        # --- BLUE/GREEN/YELLOW RANGES ---
        # Catch light yellow/green/cyan/blue paint
        # Broad range to catch all non-red signals
        lower_blue_green = np.array([20, 50, 60]) 
        upper_blue_green = np.array([160, 255, 255])
        
        mask_blue = cv2.inRange(hsv, lower_blue_green, upper_blue_green)
        
        r_count = cv2.countNonZero(mask_red)
        b_count = cv2.countNonZero(mask_blue)
        
        total_pixels = image.shape[0] * image.shape[1]
        
        # Threshold: 5% of the cropped area
        is_red_present = r_count > (total_pixels * 0.05)
        is_blue_present = b_count > (total_pixels * 0.05)
        
        return is_blue_present, is_red_present

    def play_goal_music(self):
        """Starts the physical buzzer thread."""
        if self.sound_pub is None: return
        
        if not self.music_playing:
            self.music_playing = True
            self.music_thread = threading.Thread(target=self._play_physical_buzzer_tune)
            self.music_thread.start()
            self.get_logger().info("Starting Physical Buzzer Sequence!")

    def stop_music(self):
        self.music_playing = False
        if self.music_thread:
            self.music_thread.join(timeout=0.2)
            self.music_thread = None
            
            # Ensure buzzer is OFF
            if self.sound_pub:
                msg_off = Sound()
                msg_off.value = 0
                self.sound_pub.publish(msg_off)

    def _play_physical_buzzer_tune(self):
        """
        Plays a rhythmic beat using the TB3 Buzzer (ON/OFF).
        Sound(value=1) is ON, Sound(value=0) is OFF.
        """
        msg_on = Sound()
        msg_on.value = 1
        msg_off = Sound()
        msg_off.value = 0
        
        while self.music_playing and rclpy.ok():
            # Pattern: Beep... Beep... Beep-Beep (Suspense-ish)
            
            # 1. Beep
            self.sound_pub.publish(msg_on)
            time.sleep(0.15)
            self.sound_pub.publish(msg_off)
            time.sleep(0.3)
            if not self.music_playing: break
            
            # 2. Beep
            self.sound_pub.publish(msg_on)
            time.sleep(0.15)
            self.sound_pub.publish(msg_off)
            time.sleep(0.3)
            if not self.music_playing: break
            
            # 3. Double Beep
            self.sound_pub.publish(msg_on)
            time.sleep(0.1)
            self.sound_pub.publish(msg_off)
            time.sleep(0.1)
            self.sound_pub.publish(msg_on)
            time.sleep(0.1)
            self.sound_pub.publish(msg_off)
            
            # Pause before loop
            time.sleep(0.8)

    def get_augmented_votes(self, image):
        """
        Smart Augmentation Voting Strategy with Strict Color Checks
        """
        if self.model is None: return []
        
        images_to_check = [image]
        
        # 1. Restore mild Brightness/Contrast (Fixes Goal vs UTurn)
        images_to_check.append(cv2.convertScaleAbs(image, alpha=1.0, beta=20))  # Slight Bright
        images_to_check.append(cv2.convertScaleAbs(image, alpha=1.1, beta=0))   # Slight Contrast
        
        votes = []
        for img in images_to_check:
            # Crop
            cropped = utils.crop_sign(img)
            if cropped is None: continue

            # --- COLOR CHECK ---
            is_blue, is_red = self.get_color_presence(cropped)

            # A. Normal Detection
            features = utils.extract_features(cropped)
            f_vec = features.reshape(1, -1)
            pred = int(self.model.predict(f_vec)[0])
            
            # --- FILTER BAD VOTES BASED ON COLOR ---
            # RULE: A Red class (3,4,5) MUST NOT have significant Blue/Green
            if pred in [0, 3, 4, 5] and is_blue:
                self.get_logger().info(f"DEBUG: Rejected {pred} because Blue Detected")
                pass # Reject
            # RULE: A Blue class (1,2) MUST have Blue
            elif pred in [1, 2] and not is_blue:
                self.get_logger().info(f"DEBUG: Rejected {pred} because No Blue Detected")
                pass # Reject
            else:
                votes.append(pred)
                # GOAL BOOST: If Goal is detected and passes color check, weight it higher
                if pred == 5:
                    votes.append(5)
                    votes.append(5)
            
            # B. Flipped Detection
            flip_c = cv2.flip(cropped, 1)
            f_flip = utils.extract_features(flip_c).reshape(1, -1)
            pred_flip = int(self.model.predict(f_flip)[0])
            
            # Filter Flipped Prediction by Color too
            if pred_flip in [0, 3, 4, 5] and is_blue: continue 
            if pred_flip in [1, 2] and not is_blue: continue 

            # Swap Logic
            if pred_flip == 1: votes.append(2)
            elif pred_flip == 2: votes.append(1)
            elif pred_flip != 0: 
                votes.append(pred_flip)
                if pred_flip == 5:
                    votes.append(5)
                    votes.append(5)
                
        return votes

    # --- VERIFICATION LOGIC ---

    def run_verification_step(self, image, debug_image):
        # Trigger Music
        self.play_goal_music()

        current_votes = self.get_augmented_votes(image)
        self.verify_votes.extend(current_votes)
        
        vote_text = f"V: {len(self.verify_votes)}"
        cv2.putText(debug_image, vote_text, (5, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if self.verify_stage == VERIFY_INIT:
            self.verify_stage = VERIFY_WIGGLE_1
            self.initiate_wiggle(-10, "Verify -10") # Reduced Wiggle
        elif self.verify_stage == VERIFY_WIGGLE_1:
            self.verify_stage = VERIFY_WIGGLE_2
            self.initiate_wiggle(5, "Verify -5") # Reduced Wiggle
        elif self.verify_stage == VERIFY_WIGGLE_2:
            self.verify_stage = VERIFY_WIGGLE_3
            self.initiate_wiggle(5, "Verify 0") 
        elif self.verify_stage == VERIFY_WIGGLE_3:
            self.verify_stage = VERIFY_WIGGLE_4
            self.initiate_wiggle(5, "Verify +5") # Reduced Wiggle
        elif self.verify_stage == VERIFY_WIGGLE_4:
            self.verify_stage = VERIFY_WIGGLE_5
            self.initiate_wiggle(5, "Verify +10") # Reduced Wiggle
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
        
        # Kept the 0.6 threshold as we now have more reliable votes
        if most_common == 5 and confidence > 0.6: 
            self.get_logger().info("GOAL CONFIRMED!")
            self.state = STATE_GOAL
            self.verify_failure_count = 0 # Reset on success
        
        # CHECK FOR U-TURN (Class 3 or 4)
        elif most_common in [3, 4] and confidence > 0.5:
            self.get_logger().info(f"U-Turn Confirmed ({most_common}). Executing.")
            self.verify_failure_count = 0
            self.stop_music()
            
            # Execute Smart U-Turn
            wall_angle = self.get_front_wall_angle()
            if wall_angle is not None:
                corrected_turn = wall_angle + 90
            else:
                corrected_turn = 180 - self.accumulated_wiggle
            self.initiate_turn(corrected_turn, "U-Turn (Verified)")
            
        # CHECK FOR LEFT/RIGHT (Class 1 or 2) - Correction
        elif most_common in [1, 2] and confidence > 0.5:
            self.get_logger().info(f"Correction: It was actually {most_common}. Executing.")
            self.verify_failure_count = 0
            self.stop_music()
            
            wall_angle = self.get_front_wall_angle()
            if most_common == 1: # Left
                deg = wall_angle if wall_angle else (90 - self.accumulated_wiggle)
                self.initiate_turn(deg, "Left (Verified)")
            else: # Right
                deg = (wall_angle - 180) if wall_angle else (-90 - self.accumulated_wiggle)
                self.initiate_turn(deg, "Right (Verified)")

        else:
            self.get_logger().warn(f"Goal Rejected! Correcting Angle.")
            self.verify_failure_count += 1
            self.stop_music() # Stop music if failed
            
            # Corner Escape Logic: If we fail verification twice in a row,
            # we are likely stuck facing a corner or bad angle.
            if self.verify_failure_count >= 2:
                self.get_logger().warn("Stuck in verification loop (Corner?). Forcing Turn.")
                # Turn 35 degrees to break corner alignment symmetry
                self.initiate_turn(35, "Corner Escape") 
                self.verify_failure_count = 0
                self.accumulated_wiggle = 0.0
                return
            
            # Correct accumulated wiggle before scanning again
            if abs(self.accumulated_wiggle) > 1.0:
                 self.initiate_turn(-self.accumulated_wiggle, "Undo Wiggle") 
            else:
                 self.state = STATE_SCAN
                 self.scan_stage = SCAN_CENTER
            
            self.accumulated_wiggle = 0.0

    # --- SCANNING LOGIC ---

    def run_scan_logic(self, image, debug_image):
        # Respect decision cooldown: ignore fresh decisions while cooling down
        now = time.time()
        if now < self.decision_cooldown_until:
            # throttle this log to avoid spamming
            self.log_throttle("Decision cooldown active, ignoring scan result.")
            return

        votes = self.get_augmented_votes(image)
        final_decision = 0
        confidence = 0.0
        
        if votes:
            counts = Counter(votes)
            final_decision, count = counts.most_common(1)[0]
            confidence = count / len(votes)

        if final_decision != 0:
            cv2.putText(debug_image, f"Vote: {final_decision} ({confidence:.2f})", (5, 75), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # --- CONFLICT CHECK (Anti-Flip-Flop) ---
            # If we recently saw a STRONG turn (Right/Left), reject weak opposing signals.
            if final_decision in [1, 2] and self.last_strong_decision in [1, 2]:
                time_since_strong = now - self.last_strong_time
                if time_since_strong < self.conflict_timeout:
                    if final_decision != self.last_strong_decision:
                        # Conflict detected!
                        if confidence < self.strong_threshold:
                            self.get_logger().warn(f"Rejecting weak {final_decision} vs recent strong {self.last_strong_decision}")
                            return # IGNORE this decision
                        else:
                            self.get_logger().warn("Strong conflict! Overwriting precedent.")

            # Hysteresis: require the same non-zero decision to be observed twice
            if final_decision == self.last_scan_decision and (now - self.last_scan_time) <= self.decision_hysteresis_time:
                # Confirmed decision — act on it
                if abs(self.accumulated_wiggle) > 1.0:
                    self.get_logger().info(f"Correcting wiggle ({self.accumulated_wiggle} deg) before action.")
                    self.initiate_turn(-self.accumulated_wiggle, "Center Action")
                    self.pending_decision = final_decision
                    self.state = STATE_TURN 
                    # reset last_scan to avoid immediate re-trigger
                    self.last_scan_decision = 0
                    self.last_scan_time = 0.0
                    return 
                
                # Update Strong Decision History if confident
                if final_decision in [1, 2] and confidence > self.strong_threshold:
                    self.last_strong_decision = final_decision
                    self.last_strong_time = now
                    self.get_logger().info(f"Set Strong Precedent: {final_decision}")

                self.execute_decision(final_decision)
                # execute_decision will set cooldown; clear last_scan
                self.last_scan_decision = 0
                self.last_scan_time = 0.0
                return
            else:
                # Store this observation and wait for confirmation
                self.last_scan_decision = final_decision
                self.last_scan_time = now
                self.get_logger().info(f"Seen {final_decision}, waiting for confirmation.")
                return
        
        # --- NEW: Immediate Fallback on Class 0 after Strong Vote ---
        # If we see NOTHING (0) but recently had a STRONG vote (Left/Right),
        # Assume we lost track and execute the turn blindly but smartly.
        if final_decision == 0 and self.last_strong_decision in [1, 2]:
             time_since_strong = now - self.last_strong_time
             # If it's been less than 3 seconds since we saw a strong sign
             if time_since_strong < 3.0:
                 self.get_logger().warn(f"Lost sign! Executing recent strong decision: {self.last_strong_decision}")
                 self.execute_decision(self.last_strong_decision)
                 self.last_scan_decision = 0
                 return

        # Wiggle Logic
        if self.scan_stage == SCAN_CENTER:
            self.scan_stage = SCAN_LEFT
            self.initiate_wiggle(15, "Scan Left") # Reduced Wiggle

        elif self.scan_stage == SCAN_LEFT:
            self.scan_stage = SCAN_RIGHT
            self.initiate_wiggle(-30, "Scan Right") # Reduced Wiggle

        elif self.scan_stage == SCAN_RIGHT:
            self.get_logger().warn("Scan failed. Correcting & Fallback.")
            self.initiate_wiggle(15, "Center")
            self.scan_stage = 99 # Flag for fallback

    def execute_decision(self, sign_class):
        if sign_class == 5:
            # Instead of Verify immediately, go to Prepare state
            self.state = STATE_PREPARE_GOAL
            self.accumulated_wiggle = 0.0 
            self.play_goal_music() # Start music early!
            self.get_logger().info("GOAL DETECTED: Positioning for Verification...")
            
        elif sign_class == 1: # Left
            self.last_turn_direction = 90
            self.verify_failure_count = 0
            
            # Smart Turn Logic
            wall_angle = self.get_front_wall_angle()
            if wall_angle is not None:
                corrected_turn = wall_angle
                self.get_logger().info(f"Smart Turn Left: {wall_angle:.1f}d")
            else:
                corrected_turn = 90 - self.accumulated_wiggle
                
            self.initiate_turn(corrected_turn, "Left")
            
        elif sign_class == 2: # Right
            self.last_turn_direction = -90 
            self.verify_failure_count = 0
            
            # Smart Turn Logic
            wall_angle = self.get_front_wall_angle()
            if wall_angle is not None:
                corrected_turn = wall_angle - 180
                self.get_logger().info(f"Smart Turn Right: {corrected_turn:.1f}d")
            else:
                corrected_turn = -90 - self.accumulated_wiggle
                
            self.initiate_turn(corrected_turn, "Right")
            
        elif sign_class in [3, 4]: # U-Turn
            self.verify_failure_count = 0 
            self.state = STATE_PREPARE_UTURN
            self.accumulated_wiggle = 0.0
            self.get_logger().info("U-TURN DETECTED: Positioning for Verification...")
            
        else:
            self.verify_failure_count = 0 # Reset
            
            # --- SMARTER FALLBACK: Check walls before turning ---
            # If we missed a sign, try to turn parallel to the front wall 
            # OR default to last direction but check for side obstacles.
            wall_angle = self.get_front_wall_angle()
            fallback_turn = self.last_turn_direction
            
            if wall_angle is not None:
                # If there is a wall in front, turn parallel to it.
                # wall_angle points 'Left'.
                # If last turn was Left (90), use wall_angle.
                # If last turn was Right (-90), use wall_angle - 180.
                if self.last_turn_direction > 0:
                    fallback_turn = wall_angle
                else:
                    fallback_turn = wall_angle - 180
                self.get_logger().info(f"Smart Fallback (Wall): {fallback_turn:.1f}d")
            else:
                pass 

            self.initiate_turn(fallback_turn, "Fallback")
            
        # For non-goal decisions, start a short cooldown
        if sign_class != 5 and sign_class not in [3, 4]:
            self.decision_cooldown_until = time.time() + self.decision_cooldown_seconds

    # --- MOTION CONTROL ---

    def initiate_turn(self, degrees, desc="Turn"):
        self.target_yaw = self.current_yaw + math.radians(degrees)
        if self.target_yaw > math.pi: self.target_yaw -= 2 * math.pi
        elif self.target_yaw < -math.pi: self.target_yaw += 2 * math.pi 
        
        self.turn_integral = 0.0
        self.state = STATE_TURN
        self.desired_heading = None 
        self.turn_description = f"{desc} ({degrees:.1f}d)" # Explicit debug text
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
        
        # --- REFINED SAFETY CHECK (Front Cone + Side Proximity) ---
        # Instead of 360 panic, we define a "danger zone".
        
        # 1. Front Cone (e.g., +/- 60 degrees) - Strict Check
        # Indices: [0..n/6] and [5n/6..n] roughly
        front_cone_idx = list(range(0, int(n/6))) + list(range(int(5*n/6), n))
        min_front_dist = 10.0
        for i in front_cone_idx:
            if self.lidar_ranges[i] > 0.01 and self.lidar_ranges[i] < min_front_dist:
                min_front_dist = self.lidar_ranges[i]
                
        # 2. Overall Proximity - Looser Check
        # Check everything else for extremely close objects
        min_any_dist = 10.0
        for i in range(n):
            if self.lidar_ranges[i] > 0.01 and self.lidar_ranges[i] < min_any_dist:
                min_any_dist = self.lidar_ranges[i]

        # Trigger emergency if:
        # A. Front is blocked < SAFETY_DISTANCE (0.20m)
        # B. ANY part is blocked < PANIC_DISTANCE (0.08m)
        PANIC_DISTANCE = 0.08
        
        is_emergency = False
        if min_front_dist < self.SAFETY_DISTANCE:
            is_emergency = True
            self.get_logger().warn(f"EMERGENCY FRONT: {min_front_dist:.2f}m")
        elif min_any_dist < PANIC_DISTANCE:
            is_emergency = True
            self.get_logger().warn(f"EMERGENCY SIDE/REAR: {min_any_dist:.2f}m")
            
        # Exception: Allow closer approach in PREPARE_GOAL/UTURN if aligned
        if (self.state == STATE_PREPARE_GOAL or self.state == STATE_PREPARE_UTURN) and min_front_dist > 0.15:
            is_emergency = False

        if is_emergency:
            if not self.is_in_emergency:
                self.is_in_emergency = True
                self.emergency_start_time = time.time()

            if (time.time() - self.emergency_start_time) > self.EMERGENCY_TIMEOUT:
                 self.get_logger().warn("Stuck! Escaping.")
                 twist.linear.x = -0.1
                 # Add rotation to wiggle out
                 twist.angular.z = random.uniform(-0.5, 0.5) 
                 self.cmd_pub.publish(twist)
                 return

            twist.linear.x = -0.05 # Back up slowly
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return
        else:
            self.is_in_emergency = False 

        # --- STATE: DRIVE ---
        if self.state == STATE_DRIVE:
            # If we're still settling from a recent turn, don't drive forward yet
            if time.time() < self.settle_until:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return
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
        
        # --- STATE: PREPARE GOAL/UTURN (Align & Backup) ---
        elif self.state == STATE_PREPARE_GOAL or self.state == STATE_PREPARE_UTURN:
            # 1. Get Wall Angle (Target: 90 degrees)
            wall_angle = self.get_front_wall_angle()
            ang_err = 0.0
            if wall_angle is not None:
                ang_err = wall_angle - 90.0 # e.g., 100 - 90 = 10 (Turn Left)
            
            # 2. Get Front Distance 
            # Target: 0.55m for Goal, 0.65m for U-Turn (better context)
            target_dist = 0.55 if self.state == STATE_PREPARE_GOAL else 0.65
            
            # Use a slightly wider cone to be robust
            idx_l = int(n * 0.02)
            idx_r = int(n * 0.98)
            f_dist = min(self.lidar_ranges[0], self.lidar_ranges[idx_l], self.lidar_ranges[idx_r])
            if f_dist > 10.0: f_dist = target_dist # Invalid read check
            
            dist_err = f_dist - target_dist
            
            # Control Logic
            aligned = abs(ang_err) < 5.0
            positioned = abs(dist_err) < 0.05
            
            if aligned and positioned:
                self.get_logger().info(f"Positioned (D:{f_dist:.2f}, A:{wall_angle:.1f}). Verifying.")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                
                # Transition to Verification
                self.state = STATE_VERIFY_GOAL
                self.verify_stage = VERIFY_INIT
                self.verify_votes = []
                self.scan_timer_start = self.get_clock().now().nanoseconds
                return
            
            # Corrections
            # Angular P-Control
            twist.angular.z = 0.02 * ang_err
            # Clamp angular
            twist.angular.z = max(min(twist.angular.z, 0.3), -0.3)
            
            # Linear P-Control
            # Only apply linear if we are somewhat aligned to avoid weird arcs
            # REDUCED SPEED for safer positioning
            if abs(ang_err) < 20.0:
                twist.linear.x = 0.4 * dist_err 
                # Clamp linear (Very slow for safety)
                twist.linear.x = max(min(twist.linear.x, 0.04), -0.04)
                
                # OBSTACLE GUARD: Don't drive forward if blocked
                if twist.linear.x > 0 and f_dist < 0.3:
                    twist.linear.x = 0.0
            else:
                twist.linear.x = 0.0
                
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
                    # If a pending decision was set (e.g., we corrected a wiggle
                    # and stored the desired action), execute it now. Otherwise
                    # return to DRIVE and settle briefly before moving.
                    if self.pending_decision is not None:
                        pd = self.pending_decision
                        self.pending_decision = None
                        self.get_logger().info(f"Executing pending decision: {pd}")
                        self.execute_decision(pd)
                    else:
                        self.state = STATE_DRIVE
                        self.desired_heading = None
                        self.settle_until = time.time() + self.settle_after_turn

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