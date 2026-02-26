"""USAD configuration."""

# Arduino
ARDUINO_PORT = "COM5"
ARDUINO_BAUDRATE = 9600
ARDUINO_TIMEOUT = 1

# Camera
CAMERA_SOURCE = 0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 20

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
GREEN_TIME = 30
YELLOW_TIME = 3
RED_TIME = 30

# Adaptive timing
MIN_GREEN_TIME = 3
MAX_GREEN_TIME = 30
CONGESTION_THRESHOLD = 2

# Simulation interpretation
SIM_NON_CONGESTED_CARS = 1
SIM_CONGESTED_CARS = 2

# Signal adjustments
ADAPTIVE_GREEN_EXTEND_SECONDS = 10  # extra seconds for congested lanes
ADAPTIVE_GREEN_REDUCE_SECONDS = 5  # reduce seconds for non-congested lanes

# Fallback
SIMULATE_SIGNALS_WHEN_NO_ARDUINO = True

# Vehicle detection (toy cars)
MIN_VEHICLE_AREA = 1000
MAX_VEHICLE_AREA = 20000
BACKGROUND_HISTORY = 50
BACKGROUND_THRESHOLD = 40
DETECT_SHADOWS = False

ENABLE_COLOR_FILTERING = True

USE_COLOR_SEGMENTATION = True

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

# When USE_COLOR_SEGMENTATION is enabled, we usually prevent motion-only blobs from
# spawning new tracks (to avoid ghosts). A roof-mounted white plate can reduce the
# color-mask blob enough to miss first-time detections, so allow motion to spawn a
# new track only if there is still some car-color present in the motion bbox.
MOTION_NEW_TRACK_MIN_COLOR_RATIO = 0.01

MAX_TRACKING_DISTANCE = 80  # pixels
MIN_TRACKING_IOU = 0.05
TRACKING_NEAR_DISTANCE_RATIO = 0.6
# Track removal: prefer time-based removal for consistent behavior even if FPS dips.
# Frame-based TTL is kept as a fallback.
VEHICLE_LOST_FRAMES = 25  # fallback (~<1s at 30fps)

# Remove tracks when they have not been observed for this many seconds.
# This ensures boxes disappear quickly (<1s) regardless of actual FPS.
TRACK_LOST_REMOVE_SECONDS = 0.85

# If intersection ROI gating is enabled, remove tracks after they remain outside for this long.
ROI_OUTSIDE_REMOVE_SECONDS = 0.85

MIN_TRACK_CONFIRM_FRAMES = 1

CANDIDATE_VEHICLE_LOST_FRAMES = 8

# Smoothing alpha: higher = more lag but less jitter. During collisions, higher is better.
# At collision-time, this heavily dampens segmentation mask changes.
TRACK_SMOOTHING_ALPHA = 0.50

# Extra stabilization when cars are close/touching.
# These are intentionally conservative: they only kick in when detections are near/overlapping.
TRACK_CROWD_DISTANCE_PX = 75.0
TRACK_CROWD_IOU = 0.05
TRACK_OVERLAP_IOU = 0.08

# Boost smoothing during crowded/overlap (higher = more stable, more lag).
TRACK_STABILITY_ALPHA_MULT = 1.35
TRACK_OVERLAP_ALPHA_MULT = 1.20

# During overlap, keep the box stable but DO NOT let it drift away.
TRACK_OVERLAP_MAX_DET_DEV_PX = 28

# Prevent distance-only matching when cars touch/overlap (reduces ID swaps/fly-away).
TRACK_MATCH_MIN_IOU_WHEN_CROWDED = 0.02
TRACK_MATCH_MIN_IOU_WHEN_OVERLAP = 0.06
TRACK_MATCH_MAX_DIST_WITHOUT_IOU_PX = 10.0

# Stationary lock: freeze bbox when the object isn't moving.
# Enter lock when per-frame center shift <= ENTER, exit when > EXIT.
TRACK_STATIONARY_ENTER_PX = 2.0
TRACK_STATIONARY_EXIT_PX = 6.0

# Keep collision tracks alive through brief detection failures (prevents alert drop).
COLLISION_TRACK_PREDICT_FOR_LOST_FRAMES = 60
COLLISION_TRACK_GRACE_SECONDS = 2.5

# Overlap behavior: keep bbox size stable and centered on smoothed track center.
TRACK_OVERLAP_LOCK_SIZE = True
TRACK_OVERLAP_CENTER_BOX = True

# Stronger jitter deadband + median filtering during overlap only.
TRACK_OVERLAP_DEADBAND_PX = 4
TRACK_OVERLAP_SIZE_DEADBAND_PX = 4
TRACK_STABILITY_BBOX_MEDIAN_WINDOW = 3
TRACK_STABILITY_CENTER_MEDIAN_WINDOW = 3
TRACK_OVERLAP_BBOX_MEDIAN_WINDOW = 5
TRACK_OVERLAP_CENTER_MEDIAN_WINDOW = 5

# Prediction parameters (used when a confirmed track misses a frame).
# Lower max_dt = less aggressive prediction; lower max_shift = less bbox jitter during collisions.
TRACK_PREDICT_MAX_DT = 0.15  # seconds (very conservative)
TRACK_PREDICT_MAX_SHIFT_PX = 8  # pixels (tight constraint)

# Cap bbox movement from segmentation noise (during active detection, not prediction).
TRACK_BBOX_MAX_SHIFT_PER_FRAME = 10  # pixels (limits jitter when updating with new detections)

TRACK_PRESENCE_MIN_RATIO = 0.02

LANE_MEMBERSHIP_TOLERANCE_PX = 35

MIN_VEHICLE_BBOX_WIDTH = 18
MIN_VEHICLE_BBOX_HEIGHT = 18
MAX_VEHICLE_ASPECT_RATIO = 2.8
MIN_VEHICLE_EXTENT = 0.42  # contour area / bbox area (filters thin/line-like blobs)
MAX_VEHICLE_EXTENT = 0.95

MIN_VEHICLE_SOLIDITY = 0.88

# Extra filtering to suppress small/non-vehicle blobs.
# - `FG_MASK_MIN_RATIO` rejects mostly-hollow motion blobs inside the bbox (motion detection).
# - `NEW_TRACK_MIN_AREA_SCALE` makes it harder for tiny blobs to spawn NEW tracks.
# - Small-blob strict settings apply tighter extent/solidity thresholds only to small areas.
FG_MASK_MIN_RATIO = 0.12
NEW_TRACK_MIN_AREA_SCALE = 1.10
SMALL_BLOB_STRICT_AREA_SCALE = 1.60
MIN_VEHICLE_CIRCULARITY = 0.04
MIN_VEHICLE_EXTENT_SMALL = MIN_VEHICLE_EXTENT + 0.08
MIN_VEHICLE_SOLIDITY_SMALL = min(0.99, MIN_VEHICLE_SOLIDITY + 0.04)

# Color segmentation morphology.
# Keep closing small (or disable with (1,1)) to avoid fusing two nearby cars into one blob.
COLOR_MASK_OPEN_KERNEL = (3, 3)
COLOR_MASK_CLOSE_KERNEL = (1, 1)

ENABLE_BLOB_SPLITTING = True
BLOB_SPLIT_MAX_EROSIONS = 4
BLOB_SPLIT_MIN_AREA = 8500

# Blob splitting (watershed) tuning.
# Higher DT ratio -> smaller sure-foreground peaks (often helps separate touching cars).
BLOB_SPLIT_USE_WATERSHED = True
BLOB_SPLIT_DT_THRESH_RATIO = 0.58
BLOB_SPLIT_BG_DILATE_ITERS = 2

# Accident detection
STOPPED_TIME_THRESHOLD = 999999.0
STOPPED_DISTANCE_THRESHOLD = 999999
COLLISION_DISTANCE_THRESHOLD = 30

# On-screen alert timing
NO_CAR_CLEAR_SECONDS = 1.5

COLLISION_SCREEN_ALERT_DELAY_SECONDS = 0.0

VIOLATION_SCREEN_ALERT_DELAY_SECONDS = 0.0

COLLISION_DISTANCE_MM = 1.0
PIXELS_PER_MM = 5.0

COLLISION_OBJECT_GAP_PX = 2.0

COLLISION_BBOX_TOUCH_PX = 2.0

COLLISION_MASK_DILATE_PX = 0

COLLISION_RELEASE_EXTRA_PX = 1.0

QUEUE_PROXIMITY_PX = 45.0

ENABLE_GAP_BACKGROUND_CHECK = True
BLACK_GAP_GRAY_MAX = 70
BLACK_GAP_MIN_RATIO = 0.12
STOPPED_QUEUE_TTL_SECONDS = 2.0

STICKY_COLLISION_SECONDS = 1.0
COLLISION_REMOVE_AFTER_MISSED_FRAMES = 300

ACCIDENT_DETECTION_DELAY_SECONDS = 0.0
COLLISION_MIN_CONTACT_SECONDS = 0.0
QUEUE_MIN_SECONDS = 0.5

# Collision alerts should clear immediately when resolved.
COLLISION_ALERT_HOLD_SECONDS = 0.0
COLLISION_CLEAR_SECONDS = 0.0

COLLISION_DUPLICATE_OVERLAP_MAX = 0.65

ACCIDENT_CONFIDENCE_FRAMES = 30  # generic default
COLLISION_CONFIDENCE_FRAMES = 1

ACCIDENT_REMOVE_AFTER_MISSED_FRAMES = 10
CONFIRMED_ACCIDENT_REMOVE_AFTER_MISSED_FRAMES = 30

REQUIRE_LOW_SPEED_FOR_COLLISION = False
COLLISION_MAX_SPEED_PX_PER_SEC = 999999.0

REQUIRE_MOTION_FOR_COLLISION = False
COLLISION_MIN_SPEED_PX_PER_SEC = 8.0

# Violation detection (E.Y.E.)
RED_LIGHT_CROSSING_THRESHOLD = 20  # pixels past stop line
RED_LIGHT_SPEED_THRESHOLD = 1.5

YELLOW_ABUSE_SPEED_THRESHOLD = 8  # pixels/frame (speeding through yellow)
YELLOW_SAFE_DISTANCE = 50  # pixels before stop line (can safely stop)

ENABLE_ILLEGAL_TURN = False
TURN_ANGLE_THRESHOLD = 45  # degrees deviation from lane direction
ILLEGAL_TURN_FRAMES = 5  # frames to confirm illegal turn

# License plate detection
ENABLE_LICENSE_PLATE_DETECTION = True
LP_MIN_WIDTH = 40  # lowered for toy car printed tape labels
LP_MAX_WIDTH = 250
LP_MIN_HEIGHT = 12  # lowered for toy car scale
LP_MAX_HEIGHT = 80
LP_ASPECT_RATIO_MIN = 1.8
LP_ASPECT_RATIO_MAX = 8.0  # more flexible for various tape sizes

# Roof-mounted plate constraint: search only the upper portion of the vehicle ROI.
# Keep these conservative to avoid scanning the entire car (performance).
LP_VEHICLE_PADDING_PX = 10
LP_ABOVE_VEHICLE_EXTRA_PX = 25
# Additional above-search as a fraction of vehicle bbox height.
# Useful when the plate is mounted above the detected car bbox.
LP_ABOVE_VEHICLE_EXTRA_RATIO = 0.85
LP_ROOF_Y_MAX_RATIO = 0.45
LP_ROOF_X_MARGIN_RATIO = 0.05

# Candidate/OCR tuning
LP_MAX_CANDIDATES_PER_VEHICLE = 3
LP_OCR_ROTATIONS = [0, 90, 180, 270]
LP_OCR_ALLOWLIST = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
LP_MIN_OCR_CONFIDENCE = 35.0  # percent

# TEST MODE: accept any alphanumeric OCR result (not strict AAA999).
LP_TEST_READ_ALL_ALNUM = True
LP_TEST_ALNUM_MIN_LEN = 1
LP_TEST_ALNUM_MAX_LEN = 20

# Performance: template OCR is fast (OpenCV-only) and avoids UI stalls.
LP_PREFER_TEMPLATE_OCR = False

# Non-blocking OCR pipeline (runs OCR in a background thread).
LP_ASYNC_OCR_ENABLE = True

# Multi-frame validation before OCR: require N consecutive detections.
LP_OCR_CONSECUTIVE_DETECTIONS = 2

# Controlled OCR cadence per vehicle (5–10 frames typical).
LP_OCR_EVERY_N_FRAMES = 3

# Performance safeguards.
LP_OCR_MAX_CALLS_PER_SECOND = 2.0
LP_OCR_JOB_QUEUE_SIZE = 16
LP_OCR_TIMEOUT_SECONDS = 2.50

# Preprocessing: upscale plate patch before OCR (2x or 3x recommended).
LP_OCR_UPSCALE = 2.0

# Cache: do not re-run OCR once confidence is high enough.
LP_OCR_CACHE_MIN_CONF = 60.0

# Tesseract can be slow; keep it conservative by default.
LP_TESSERACT_ROTATIONS = [0]
LP_TESSERACT_MAX_VARIANTS = 2

# Visualization
DRAW_LICENSE_PLATE_BBOX = True

# Temporal stability (avoid flicker): only confirm after consistent multi-frame votes.
LP_STABLE_MIN_OBSERVATIONS = 2
LP_STABLE_MIN_RATIO = 0.60
# With LP_MIN_OCR_CONFIDENCE=35% and 2 observations, the minimum possible score is 70.
# Keep the score threshold aligned so plates can actually confirm.
LP_STABLE_MIN_SCORE = 70.0
LP_TRACK_TTL_SECONDS = 12.0

# Plate bbox smoothing/tracking (display + consistent OCR ROI)
LP_BBOX_SMOOTHING_ALPHA = 0.35

# Vehicle detection (optional): merge attached white roof-plate pixels into the car blob.
# Default is OFF to preserve the existing object detection behavior.
VEHICLE_PLATE_MERGE_ENABLE = False
VEHICLE_PLATE_MERGE_DILATE_PX = 18

# Display-only stabilization (does NOT affect detection/tracking logic)
DISPLAY_BBOX_STABILIZATION = True
DISPLAY_BBOX_ALPHA = 0.20
DISPLAY_BBOX_DEADBAND_PX = 2

# OCR Configuration
TESSERACT_CONFIG = '--psm 8 --oem 3 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'

# Optional: explicit Tesseract executable path (helps when PATH isn't refreshed).
# Example: r"C:\\Program Files\\Tesseract-OCR\\tesseract.exe"
TESSERACT_CMD = r"C:\\Program Files\\Tesseract-OCR\\tesseract.exe"

# Template OCR fallback (OpenCV-only)
# Used automatically when EasyOCR isn't available.
LP_TEMPLATE_OCR_ENABLE = True
LP_TEMPLATE_OCR_SIZE = 36
LP_TEMPLATE_OCR_OPEN_KERNEL = 2
LP_TEMPLATE_OCR_CHAR_PAD = 2
LP_TEMPLATE_MIN_CHAR_SCORE = 0.45
LP_TEMPLATE_OCR_ROTATIONS = [0, 90, 180, 270]

# License plate OCR throttling (EasyOCR can be expensive)
LP_DETECT_EVERY_N_FRAMES = 4
LP_MAX_VEHICLES_PER_FRAME = 1
LP_PER_VEHICLE_COOLDOWN_SECONDS = 0.35

# Keep plate text stable across brief tracking/ID jitter while vehicle is moving.
LP_MEMORY_TTL_SECONDS = 4.0
LP_MEMORY_MATCH_IOU = 0.10
LP_MEMORY_MATCH_CENTER_PX = 90.0

# If a track already has a plate, wait this long before forcing another OCR pass.
LP_REOCR_HOLD_SECONDS = 1.5

# Event-time best-effort OCR caps (accidents/violations). Keep low to avoid lag.
LP_EVENT_MAX_VEHICLES_PER_ACCIDENT = 2

# Emergency notification
EMERGENCY_HOTLINE = "911"
ENABLE_SMS_SIMULATION = True
ENABLE_CALL_SIMULATION = True
NOTIFICATION_COOLDOWN = 60

# Logging
LOG_DIRECTORY = "logs"
EVENT_LOG_FILE = "traffic_events.csv"
VIOLATION_LOG_FILE = "violations.csv"
ACCIDENT_LOG_FILE = "accidents.csv"
PLATE_REGISTRY_FILE = "plates.txt"
PLATE_REGISTRY_FLUSH_INTERVAL_SECONDS = 1.0
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
    "LARGE": (6000, MAX_VEHICLE_AREA)     # Truck, bus
}

# Restrict detections/tracks to these type labels (set to None/empty to allow all).
# For the current toy-car setup, we only want the main object size and to ignore
# fingers/hand movement which usually shows up as SMALL/MEDIUM blobs.
ALLOWED_VEHICLE_TYPES = ("LARGE",)

# Display
DISPLAY_WINDOW_NAME = "USAD - AI Traffic Management System"
SHOW_DEBUG_INFO = True
SHOW_VEHICLE_IDS = True
SHOW_LANE_REGIONS = True
SHOW_VIOLATIONS = True

# Temporary OCR debug window (shows real cropped patch per vehicle)
LP_DEBUG_SHOW_OCR_PATCH = False
LP_DEBUG_WINDOW_NAME = "LP OCR Debug"
LP_DEBUG_PATCH_W = 240
LP_DEBUG_PATCH_H = 120

# FPS display smoothing
FPS_EMA_ALPHA = 0.15

# Colors (BGR format for OpenCV)
COLOR_LANE1 = (255, 0, 0)      # Blue
COLOR_LANE2 = (0, 255, 0)      # Green
COLOR_LANE3 = (0, 0, 255)      # Red
COLOR_LANE4 = (255, 255, 0)    # Cyan
COLOR_ACCIDENT = (0, 0, 255)   # Red
COLOR_VIOLATION = (0, 165, 255)  # Orange
COLOR_VEHICLE = (255, 255, 255)  # White
COLOR_LICENSE_PLATE = (0, 255, 255)  # Yellow (text label)
COLOR_LICENSE_PLATE_BBOX = (0, 255, 0)  # Green (dedicated OCR bbox)

# Analytics
ANALYTICS_TIME_WINDOWS = {
    "MORNING_RUSH": (7, 9),     # 7 AM - 9 AM
    "AFTERNOON": (12, 14),      # 12 PM - 2 PM
    "EVENING_RUSH": (17, 19),   # 5 PM - 7 PM
    "NIGHT": (22, 6)            # 10 PM - 6 AM
}

# High-risk thresholds
HIGH_RISK_VIOLATION_MULTIPLIER = 3.0  # 3x more violations = high risk
PEAK_HOURS_WINDOW = 60  # minutes for peak analysis

# System behavior
AUTO_MODE_DEFAULT = True  # Start in automatic cycling mode
ENABLE_ADAPTIVE_TIMING = True  # Adjust green time based on congestion
ENABLE_ACCIDENT_PRIORITY = False  # DISABLED - stationary toy cars don't have real accidents
ACCIDENT_PRIORITY_DURATION = 30  # seconds to keep accident lane clear

# Debug modes
DEBUG_MODE = False
SAVE_DEBUG_FRAMES = False
DEBUG_FRAME_INTERVAL = 30  # Save every N frames

# Optional debug visualization for vehicle detection
SHOW_FG_MASK = False
SHOW_COLOR_MASK = False

BG_LEARNING_RATE = 0.001
