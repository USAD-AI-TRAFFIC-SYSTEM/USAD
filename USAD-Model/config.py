"""USAD configuration."""

# Arduino
ARDUINO_PORT = "COM6"
ARDUINO_BAUDRATE = 9600
ARDUINO_TIMEOUT = 1

# Camera
CAMERA_SOURCE = 0  # 0 for laptop webcam, 1 for external camera, or path to video file
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30

# Lanes (polygons in 1280x720 coordinates)
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

# Intersection center
INTERSECTION_CENTER = [(480, 195), (720, 190), (720, 420), (480, 420)]

# Timing (seconds)
GREEN_TIME = 30  # seconds (matches Arduino)
YELLOW_TIME = 3  # seconds (matches Arduino)
RED_TIME = 30  # seconds (green + yellow for other lanes)

# Adaptive timing
MIN_GREEN_TIME = 3
MAX_GREEN_TIME = 30
CONGESTION_THRESHOLD = 2  # simulation: 2 cars = congested lane

# Simulation interpretation
SIM_NON_CONGESTED_CARS = 1
SIM_CONGESTED_CARS = 2

# Signal adjustments
ADAPTIVE_GREEN_EXTEND_SECONDS = 10  # extra seconds for congested lanes
ADAPTIVE_GREEN_REDUCE_SECONDS = 5  # reduce seconds for non-congested lanes

# Fallback
SIMULATE_SIGNALS_WHEN_NO_ARDUINO = True

# Vehicle detection (toy cars)
MIN_VEHICLE_AREA = 1000  # pixels - minimum toy car size (helps ignore lane stripe segments)
MAX_VEHICLE_AREA = 20000  # pixels - maximum toy car size
BACKGROUND_HISTORY = 50  # not used (color detection)
BACKGROUND_THRESHOLD = 40  # not used (color detection)  
DETECT_SHADOWS = False  # not used (color detection)

# Filter to intended car colors (HSV).
ENABLE_COLOR_FILTERING = True

# Prefer color segmentation as the primary detector (works for stationary cars).
USE_COLOR_SEGMENTATION = True

# Detect cars anywhere inside the intersection ROI (lanes + center).
REQUIRE_LANE_MEMBERSHIP_FOR_DETECTION = False
REQUIRE_INTERSECTION_ROI_FOR_DETECTION = True
INTERSECTION_ROI_DILATE_PX = 10

# Car colors (HSV)
CAR_COLOR_RANGES = [
    # Blue
    ((90, 60, 30), (140, 255, 255)),

    # Red (two ranges; hue wraps)
    ((0, 80, 40), (12, 255, 255)),
    ((168, 80, 40), (180, 255, 255)),

]

# Minimum fraction of bbox pixels matching a car color.
CAR_COLOR_MIN_RATIO = 0.02

# Vehicle tracking (bigger helps after moving cars by hand)
MAX_TRACKING_DISTANCE = 80  # pixels
# Prefer matching by overlap when cars are close (helps separate same-color cars).
MIN_TRACKING_IOU = 0.05
TRACKING_NEAR_DISTANCE_RATIO = 0.6
# Increase tolerance so brief segmentation dropouts don't delete cars.
VEHICLE_LOST_FRAMES = 60  # ~2s at 30fps

# Faster on-screen appearance.
MIN_TRACK_CONFIRM_FRAMES = 1

# Unconfirmed tracks expire faster so random shapes don't linger on-screen.
CANDIDATE_VEHICLE_LOST_FRAMES = 8

# Smoothing reduces jitter in the drawn box/ID location.
# 0.0 = no smoothing, 0.6–0.8 = stable.
TRACK_SMOOTHING_ALPHA = 0.7

# If a confirmed vehicle temporarily fails contour extraction, keep it alive if there
# are still enough car-colored pixels inside its last bbox.
TRACK_PRESENCE_MIN_RATIO = 0.02

# Lane membership tolerance (pixels)
LANE_MEMBERSHIP_TOLERANCE_PX = 35

# Tighten detection to cars only (reduces lane-marking false positives)
MIN_VEHICLE_BBOX_WIDTH = 18
MIN_VEHICLE_BBOX_HEIGHT = 18
MAX_VEHICLE_ASPECT_RATIO = 2.8
MIN_VEHICLE_EXTENT = 0.42  # contour area / bbox area (filters thin/line-like blobs)
MAX_VEHICLE_EXTENT = 0.95

# Additional blob shape filtering (lane-marking fragments/glare often fail this).
MIN_VEHICLE_SOLIDITY = 0.88

# Morph kernel used in color segmentation close step
COLOR_MASK_CLOSE_KERNEL = (3, 3)

# When two cars are bumper-to-bumper, color segmentation can merge them into a
# single elongated blob. Enable a light-weight split step to recover individual cars.
ENABLE_BLOB_SPLITTING = True
BLOB_SPLIT_MAX_EROSIONS = 4
BLOB_SPLIT_MIN_AREA = 8500

# ==================== ACCIDENT DETECTION PARAMETERS ====================
STOPPED_TIME_THRESHOLD = 999999.0  # DISABLED - stationary toy cars, don't detect stopped vehicles
STOPPED_DISTANCE_THRESHOLD = 999999  # DISABLED
COLLISION_DISTANCE_THRESHOLD = 30  # pixels (legacy / fallback)

# ==================== ON-SCREEN ALERT TIMING ====================
# If no cars are detected for this long, hide vehicle boxes + clear accident/violation alerts.
NO_CAR_CLEAR_SECONDS = 1.5

# Delay on-screen collision banner/markers by this many seconds after confirmation.
COLLISION_SCREEN_ALERT_DELAY_SECONDS = 0.0

# Delay on-screen violation markers/labels by this many seconds after a violation is detected.
VIOLATION_SCREEN_ALERT_DELAY_SECONDS = 0.0

# Collision rule for the simulation: treat vehicles as collided when the gap between
# their bounding boxes is 0–1 mm (physical distance).
# Because the camera measures pixels, we convert mm -> pixels using PIXELS_PER_MM.
COLLISION_DISTANCE_MM = 1.0
PIXELS_PER_MM = 5.0  # tune this once using a ruler in the camera view

# Collision should trigger only when the *objects* touch.
# Allow up to ~2px gap to tolerate segmentation noise.
COLLISION_OBJECT_GAP_PX = 2.0

# If object-gap can't be computed (e.g., segmentation dropout), only treat as a
# collision when the bboxes are essentially touching.
COLLISION_BBOX_TOUCH_PX = 2.0

# Keep strict contact detection: don't dilate masks.
COLLISION_MASK_DILATE_PX = 0

# Small hysteresis to avoid flicker.
COLLISION_RELEASE_EXTRA_PX = 1.0

# If two cars are close but there is a visible gap between the objects, treat them
# as STOPPED/QUEUED rather than a collision.
QUEUE_PROXIMITY_PX = 45.0

# If the region between two close cars is mostly black road (background), treat them as
# stopped/queued cars (not a collision accident).
ENABLE_GAP_BACKGROUND_CHECK = True
BLACK_GAP_GRAY_MAX = 70        # pixels <= this are considered "black background" (0-255)
BLACK_GAP_MIN_RATIO = 0.12     # suppress collision if a visible black gap exists
STOPPED_QUEUE_TTL_SECONDS = 2.0

# Collision persistence: keep a detected collision from flickering off if detection
# temporarily drops for a few frames.
STICKY_COLLISION_SECONDS = 1.0
COLLISION_REMOVE_AFTER_MISSED_FRAMES = 300

# Wait before classifying STOPPED / COLLISION.
# This reduces flicker and avoids triggering while detection is still stabilizing.
ACCIDENT_DETECTION_DELAY_SECONDS = 0.0
COLLISION_MIN_CONTACT_SECONDS = 0.0
QUEUE_MIN_SECONDS = 0.5

# Keep collision alert visible even if detections drop briefly.
COLLISION_ALERT_HOLD_SECONDS = 10.0

# When a collision is no longer true (cars separated or one disappears), remove the
# collision alert after this many seconds.
COLLISION_CLEAR_SECONDS = 5.0

# If two detections overlap too much, they are likely the same physical car
# (duplicate detection). Suppress collision for such pairs.
COLLISION_DUPLICATE_OVERLAP_MAX = 0.65

# Confirmation frames (smaller = faster on-screen alert)
ACCIDENT_CONFIDENCE_FRAMES = 30  # generic default
COLLISION_CONFIDENCE_FRAMES = 1

# How quickly to remove accidents when the condition is no longer observed.
# Prevents one-frame false positives from lingering on-screen.
ACCIDENT_REMOVE_AFTER_MISSED_FRAMES = 10
CONFIRMED_ACCIDENT_REMOVE_AFTER_MISSED_FRAMES = 30

# If True, require both vehicles to be moving slowly to count as a collision
REQUIRE_LOW_SPEED_FOR_COLLISION = False
COLLISION_MAX_SPEED_PX_PER_SEC = 999999.0

# If True, require at least one of the two vehicles to be moving above a minimal
# speed to count as a collision. Helps prevent collisions from static blobs.
# For toy-car dioramas, collisions should be based on physical contact (touching)
# and not necessarily motion; motion gating can hide true bumper-to-bumper contact.
REQUIRE_MOTION_FOR_COLLISION = False
COLLISION_MIN_SPEED_PX_PER_SEC = 8.0

# ==================== VIOLATION DETECTION PARAMETERS (E.Y.E.) ====================
# Red light violation
RED_LIGHT_CROSSING_THRESHOLD = 20  # pixels past stop line
RED_LIGHT_SPEED_THRESHOLD = 1.5  # pixels/frame minimum speed

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
    # Allow up to MAX_VEHICLE_AREA so large/merged blobs don't get labeled UNKNOWN.
    "LARGE": (6000, MAX_VEHICLE_AREA)     # Truck, bus
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
