# ~/autobots/lab3_ws/src/autobots_chase_object/autobots_chase_object/constants.py
import math

# --- ROS Topic Names ---
RAW_IMAGE_TOPIC = '/image_raw/compressed'
PROCESSED_IMAGE_TOPIC = '/image_processed/compressed'
LIDAR_TOPIC = '/scan'
OBJECT_ANGLE_TOPIC = '/object_angle'
OBJECT_LOCATION_TOPIC = '/object_location'
CMD_VEL_TOPIC = '/cmd_vel'
CAMERA_INFO_TOPIC = "/camera/camera_info"

# --- TF FRAMES ---
CAMERA_FRAME = 'camera_link'
LIDAR_FRAME = 'base_scan'

# --- Image Processing Parameters ---
IMAGE_WIDTH = 640
HORIZONTAL_FOV_DEGREES = 62.2
HORIZONTAL_FOV_RADIANS = HORIZONTAL_FOV_DEGREES * (math.pi / 180)
# HSV color thresholds for a red object
H_LOW, S_LOW, V_LOW = 0, 120, 80
H_HIGH, S_HIGH, V_HIGH = 12, 255, 255
H_LOW2, S_LOW2, V_LOW2 = 170, 120, 70
H_HIGH2, S_HIGH2, V_HIGH2 = 179, 255, 255
# Area filter threshold as a ratio of the total image area
MIN_AREA_RATIO = 0.02

# --- Robot Control Parameters ---
# --- PID Gains ---
ANGULAR_KP = 0.045
ANGULAR_KI = 0.0
LINEAR_KP = 0.7
LINEAR_KI = 0.05
# Desired distance from the object in meters
DESIRED_DISTANCE_M = 0.5
# Integral windup clamp limits
INTEGRAL_CLAMP_ANGULAR = 1.0
INTEGRAL_CLAMP_LINEAR = 0.5

# --- NEW: Controller Deadbands (Tolerances) ---
# If the error is smaller than these values, the robot will stop.
# Angular tolerance in radians (e.g., 2 degrees)
DEL_A = math.radians(0.0001)
# Linear tolerance in meters (e.g., 3 cm)
DEL_D = 0.03