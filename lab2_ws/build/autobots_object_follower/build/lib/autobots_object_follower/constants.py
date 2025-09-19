# ~/autobots/lab2_ws/src/autobots_object_follower/autobots_object_follower/constants.py

# --- ROS TOPIC NAMES ---
RAW_IMAGE_TOPIC = '/image_raw/compressed'
PROCESSED_IMAGE_TOPIC = '/image_processed/compressed'
OBJECT_COORD_TOPIC = '/object_px'
CMD_VEL_TOPIC = '/cmd_vel'

# --- IMAGE PROCESSING PARAMETERS ---
# HSV color thresholds for a red object
H_LOW = 0
S_LOW = 120
V_LOW = 80
H_HIGH = 12
S_HIGH = 255
V_HIGH = 255

# Second HSV band for red (which wraps around 180)
H_LOW2 = 170
S_LOW2 = 120
V_LOW2 = 80
H_HIGH2 = 179
S_HIGH2 = 255
V_HIGH2 = 255

# Area filter thresholds as a ratio of the total image area
MIN_AREA_RATIO = 0.005 # Lowered to detect smaller objects
MAX_AREA_RATIO = 0.50

# --- ROBOT CONTROL PARAMETERS ---
# The width of the camera image in pixels
IMAGE_WIDTH = 320

# How fast the robot turns. A higher value means faster turning.
TURN_SPEED = 1