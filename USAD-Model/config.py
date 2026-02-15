"""
USAD Configuration File
Contains all system parameters, lane definitions, and thresholds
"""

# ==================== ARDUINO CONFIGURATION ====================
ARDUINO_PORT = "COM6"
ARDUINO_BAUDRATE = 9600
ARDUINO_TIMEOUT = 1

# ==================== CAMERA CONFIGURATION ====================
CAMERA_SOURCE = 0  # 0 for laptop webcam, 1 for external camera, or path to video file
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30

# ==================== LANE DEFINITIONS ====================
# Define lane regions as polygons (x, y coordinates)
# Format: [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
# Scaled for 1280x720 resolution (2x scale from 640x480)
LANES = {
    "LANE1": {  # North
        "name": "North",
        "region": [(485, 0), (710, 0), (720, 190), (485, 190)],
        "stop_line": [(480, 195), (720, 190)],
        "direction": "vertical",
        "arduino_cmd": "LANE1"
    },
    "LANE2": {  # South
        "name": "South", 
        "region": [(465, 720), (730, 720), (720, 420), (480, 420)],
        "stop_line": [(480, 420), (720, 420)],
        "direction": "vertical",
        "arduino_cmd": "LANE2"
    },
    "LANE3": {  # East
        "name": "East",
        "region": [(1020, 180), (1040, 410), (720, 415), (720, 190)],
        "stop_line": [(720, 189), (720, 420)],
        "direction": "horizontal",
        "arduino_cmd": "LANE3"
    },
    "LANE4": {  # West
        "name": "West",
        "region": [(165, 195), (145 , 422), (480, 420), (480, 195)],
        "stop_line": [(480, 195), (480, 420)],
        "direction": "horizontal",
        "arduino_cmd": "LANE4"
    }
}

# Intersection center area (where vehicles should not stop)
INTERSECTION_CENTER = [(480, 195), (720, 190), (720, 420), (480, 420)]

# ==================== TIMING CONFIGURATION ====================
GREEN_TIME = 5  # seconds (matches Arduino)
YELLOW_TIME = 3  # seconds (matches Arduino)
RED_TIME = 8  # seconds (green + yellow for other lanes)

# Adaptive timing adjustments
MIN_GREEN_TIME = 3
MAX_GREEN_TIME = 15
CONGESTION_THRESHOLD = 5  # vehicles to trigger extended green

# ==================== VEHICLE DETECTION PARAMETERS ====================
# Color / size filtering for toy cars only
# Filter out tiny noise but keep toy cars
MIN_VEHICLE_AREA = 600  # pixels - minimum toy car size
MAX_VEHICLE_AREA = 20000  # pixels - maximum toy car size
BACKGROUND_HISTORY = 50  # not used (color detection)
BACKGROUND_THRESHOLD = 40  # not used (color detection)  
DETECT_SHADOWS = False  # not used (color detection)

# Focus detection on specific car colors (HSV ranges)
# Enable so mainly your cars (red, white, blue) are detected
ENABLE_COLOR_FILTERING = True

# Car color groups (HSV). Adjust if lighting changes a lot.
CAR_COLOR_RANGES = [
    # Blue car
    ((95, 80, 40), (135, 255, 255)),

    # Red car (two ranges because red wraps HSV hue)
    ((0, 120, 50), (10, 255, 255)),
    ((170, 120, 50), (180, 255, 255)),

    # White car: low saturation, high brightness
    ((0, 0, 180), (179, 60, 255)),
]

# Minimum fraction of the detection box that must match one
# of the car colors above. Slightly stricter to ignore road patches.
CAR_COLOR_MIN_RATIO = 0.08  # 8%

# Vehicle tracking
MAX_TRACKING_DISTANCE = 50  # pixels
VEHICLE_LOST_FRAMES = 10  # frames before vehicle is considered gone (~0.3s at 30fps)

# ==================== ACCIDENT DETECTION PARAMETERS ====================
STOPPED_TIME_THRESHOLD = 999999.0  # DISABLED - stationary toy cars, don't detect stopped vehicles
STOPPED_DISTANCE_THRESHOLD = 999999  # DISABLED
COLLISION_DISTANCE_THRESHOLD = 30  # pixels (proximity for collision detection)
ACCIDENT_CONFIDENCE_FRAMES = 30  # frames to confirm accident

# ==================== VIOLATION DETECTION PARAMETERS (E.Y.E.) ====================
# Red light violation
RED_LIGHT_CROSSING_THRESHOLD = 20  # pixels past stop line
RED_LIGHT_SPEED_THRESHOLD = 3  # pixels/frame minimum speed

# Yellow light abuse
YELLOW_ABUSE_SPEED_THRESHOLD = 8  # pixels/frame (speeding through yellow)
YELLOW_SAFE_DISTANCE = 50  # pixels before stop line (can safely stop)

# Illegal turn detection
ENABLE_ILLEGAL_TURN = False  # disable for stationary diorama
TURN_ANGLE_THRESHOLD = 45  # degrees deviation from lane direction
ILLEGAL_TURN_FRAMES = 5  # frames to confirm illegal turn

# ==================== LICENSE PLATE DETECTION ====================
ENABLE_LICENSE_PLATE_DETECTION = False  # Disabled - Tesseract not installed
LP_MIN_WIDTH = 80
LP_MAX_WIDTH = 300
LP_MIN_HEIGHT = 20
LP_MAX_HEIGHT = 100
LP_ASPECT_RATIO_MIN = 2.0
LP_ASPECT_RATIO_MAX = 6.0

# OCR Configuration
TESSERACT_CONFIG = '--psm 8 --oem 3 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'

# ==================== EMERGENCY NOTIFICATION ====================
EMERGENCY_HOTLINE = "911"
ENABLE_SMS_SIMULATION = True
ENABLE_CALL_SIMULATION = True
NOTIFICATION_COOLDOWN = 60  # seconds between repeat notifications

# ==================== LOGGING CONFIGURATION ====================
LOG_DIRECTORY = "logs"
EVENT_LOG_FILE = "traffic_events.csv"
VIOLATION_LOG_FILE = "violations.csv"
ACCIDENT_LOG_FILE = "accidents.csv"
ANALYTICS_UPDATE_INTERVAL = 300  # seconds (5 minutes)

# Event types
EVENT_TYPES = {
    "RED_LIGHT_VIOLATION": "Red Light Violation",
    "YELLOW_ABUSE": "Yellow Light Abuse",
    "ILLEGAL_TURN": "Illegal Turn",
    "ACCIDENT": "Accident Detected",
    "CONGESTION": "Lane Congestion",
    "EMERGENCY_NOTIFIED": "Emergency Services Notified"
}

# Vehicle types (based on size)
VEHICLE_TYPES = {
    "SMALL": (200, 2500),      # Small objects, motorcycle, small car
    "MEDIUM": (2500, 6000),    # Sedan, SUV
    "LARGE": (6000, 15000)     # Truck, bus
}

# ==================== DISPLAY CONFIGURATION ====================
DISPLAY_WINDOW_NAME = "USAD - AI Traffic Management System"
SHOW_DEBUG_INFO = True
SHOW_VEHICLE_IDS = True
SHOW_LANE_REGIONS = True
SHOW_VIOLATIONS = True

# Colors (BGR format for OpenCV)
COLOR_LANE1 = (255, 0, 0)      # Blue
COLOR_LANE2 = (0, 255, 0)      # Green
COLOR_LANE3 = (0, 0, 255)      # Red
COLOR_LANE4 = (255, 255, 0)    # Cyan
COLOR_ACCIDENT = (0, 0, 255)   # Red
COLOR_VIOLATION = (0, 165, 255)  # Orange
COLOR_VEHICLE = (255, 255, 255)  # White
COLOR_LICENSE_PLATE = (0, 255, 255)  # Yellow

# ==================== ANALYTICS CONFIGURATION ====================
ANALYTICS_TIME_WINDOWS = {
    "MORNING_RUSH": (7, 9),     # 7 AM - 9 AM
    "AFTERNOON": (12, 14),      # 12 PM - 2 PM
    "EVENING_RUSH": (17, 19),   # 5 PM - 7 PM
    "NIGHT": (22, 6)            # 10 PM - 6 AM
}

# High-risk thresholds
HIGH_RISK_VIOLATION_MULTIPLIER = 3.0  # 3x more violations = high risk
PEAK_HOURS_WINDOW = 60  # minutes for peak analysis

# ==================== SYSTEM BEHAVIOR ====================
AUTO_MODE_DEFAULT = True  # Start in automatic cycling mode
ENABLE_ADAPTIVE_TIMING = True  # Adjust green time based on congestion
ENABLE_ACCIDENT_PRIORITY = False  # DISABLED - stationary toy cars don't have real accidents
ACCIDENT_PRIORITY_DURATION = 30  # seconds to keep accident lane clear

# Debug modes
DEBUG_MODE = False
SAVE_DEBUG_FRAMES = False
DEBUG_FRAME_INTERVAL = 30  # Save every N frames

# Optional debug visualization for vehicle detection
# When enabled, opens extra OpenCV windows.
SHOW_FG_MASK = False
SHOW_COLOR_MASK = False

# Background subtractor adaptation (used only when motion-based detection is active)
# Small positive values keep the background slowly adapting.
BG_LEARNING_RATE = 0.001

# Prefer color segmentation when ENABLE_COLOR_FILTERING is on.
# This detects toy cars even when stationary (no motion required).
USE_COLOR_SEGMENTATION = True

# If True, only create detections when a blob's center is inside one of the lane polygons.
# Set to False to debug/visualize detections in the intersection area too.
REQUIRE_LANE_MEMBERSHIP_FOR_DETECTION = True
