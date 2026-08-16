"""USAD main application (camera loop + detection + control + UI overlay)."""

import cv2
import numpy as np
import time
import sys
import os
import importlib
import threading
from typing import Optional, Tuple
import config
from traffic_controller import TrafficController
from vehicle_detector import VehicleDetector
from accident_detector import AccidentDetector
from violation_detector import ViolationDetector
from license_plate_detector import LicensePlateDetector
from event_logger import EventLogger
from emergency_notifier import EmergencyNotifier


class LatestFrameGrabber:
    """Read latest frame from a VideoCapture in a background thread."""

    def __init__(self, cap: cv2.VideoCapture):
        self._cap = cap
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._latest: Optional[np.ndarray] = None
        self._latest_ts: float = 0.0

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="LatestFrameGrabber", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        t = self._thread
        if t is not None:
            t.join(timeout=1.0)

    def _run(self):
        while not self._stop.is_set():
            if self._cap is None:
                time.sleep(0.01)
                continue
            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.005)
                continue
            with self._lock:
                self._latest = frame
                self._latest_ts = time.time()

    def get_latest(self) -> Tuple[bool, Optional[np.ndarray], float]:
        with self._lock:
            if self._latest is None:
                return False, None, 0.0
            return True, self._latest, float(self._latest_ts)


class USAD:
    """Main USAD application controller"""
    
    def __init__(self):
        print("="*70)
        print("USAD - Urban Smart Adaptive Dispatcher")
        print("AI Traffic Management System")
        print("="*70)
        
        self.traffic_controller = TrafficController()
        self.vehicle_detector = VehicleDetector()
        self.accident_detector = AccidentDetector()
        self.violation_detector = ViolationDetector()
        self.license_plate_detector = LicensePlateDetector()
        self.event_logger = EventLogger()
        self.emergency_notifier = EmergencyNotifier()
        print(f"[Logs] Session logs and analytics: {os.path.abspath(config.LOG_DIRECTORY)}")
        
        # State variables
        self.current_active_lane = None
        self.lane_green_start_time = None
        self.lane_green_duration = config.GREEN_TIME
        self.current_phase = "GREEN"  # GREEN, YELLOW, RED
        self.phase_start_time = time.time()

        # Software control when Arduino is disconnected
        self.software_auto_mode = bool(config.AUTO_MODE_DEFAULT)
        self.arduino_connected = False
        
        # Performance metrics
        self.fps = 0
        self.frame_count = 0
        self.start_time = time.time()
        self._last_loop_ts: Optional[float] = None
        self._fps_ema: float = 0.0
        
        # Video capture
        self.cap = None
        self._grabber: Optional[LatestFrameGrabber] = None
        
        # Display settings
        self.is_fullscreen = False

        # When False, app.py renders the HUD instead of OpenCV.
        self.show_cv_panel = True

        # Store detected license plates directly on Camera 2
        self._detected_plates = {}

        # Detection/alert gating
        self._last_vehicle_seen_ts: float = time.time()
        self._no_car_idle_mode: bool = False
        
        # Config hot reload (disabled when running as bundled exe)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_path = os.path.join(script_dir, 'config.py')
        self._is_frozen = getattr(sys, 'frozen', False)  # True when running as PyInstaller exe
        if not self._is_frozen and os.path.exists(self.config_path):
            self.config_last_modified = os.path.getmtime(self.config_path)
        else:
            self.config_last_modified = 0
        self.config_check_interval = 1.0  # Check every 1 second
        self.last_config_check = time.time()

        # License plate OCR throttling (keeps EasyOCR from stalling frames)
        self._lp_last_attempt_ts = {}
        self._lp_frame_index = 0
        
    def initialize_camera(self) -> bool:
        """Initialize camera or video source"""
        print("\n[Camera] Initializing...")

        source = config.CAMERA_SOURCE
        backends = [
            ("DSHOW", cv2.CAP_DSHOW),
            ("MSMF", getattr(cv2, "CAP_MSMF", cv2.CAP_ANY)),
            ("ANY", cv2.CAP_ANY),
        ]

        self.cap = None
        for name, backend in backends:
            cap = cv2.VideoCapture(source, backend)
            if cap is not None and cap.isOpened():
                self.cap = cap
                print(f"[Camera] ✓ Opened source {source} using {name}")
                break

        if self.cap is None or not self.cap.isOpened():
            print(f"[ERROR] Could not open camera source: {source}")
            print("        Try closing other camera apps, or set CAMERA_SOURCE=1.")
            return False
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)
        
        print("[Camera] Warming up...")
        for i in range(10):
            ret, _ = self.cap.read()
            if not ret:
                if i == 0:
                    print(f"[ERROR] Camera cannot read frames. Make sure:")
                    print(f"        - Camera is not in use by another application")
                    print(f"        - Camera {config.CAMERA_SOURCE} is properly connected")
                    return False
                time.sleep(0.1)
        
        print(f"[Camera] ✓ Initialized ({config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT} @ {config.CAMERA_FPS}fps)")
        return True
    
    def initialize_arduino(self) -> bool:
        """Initialize Arduino connection"""
        print("\n[Arduino] Connecting to COM4...")
        
        if self.traffic_controller.connect():
            print(f"[Arduino] ✓ Connected on {config.ARDUINO_PORT}")
            if config.AUTO_MODE_DEFAULT:
                self.traffic_controller.set_auto_mode()
                print("[Arduino] ✓ Auto mode enabled")
            return True
        else:
            print("[Arduino] ✗ Failed to connect")
            print("[Arduino] System will run in simulation mode")
            return False
    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """Process a single frame"""
        is_camera_1 = (config.CAMERA_SOURCE == 1)
        is_camera_2 = (config.CAMERA_SOURCE == 2)

        if is_camera_1:
            vehicles = self.vehicle_detector.detect_vehicles(frame)
        else:
            vehicles = []

        now = time.time()
        if bool(getattr(self.vehicle_detector, "had_detections_this_frame", False)) or bool(
            getattr(self.vehicle_detector, "had_confirmed_updates_this_frame", False)
        ):
            self._last_vehicle_seen_ts = now
        no_car_clear_seconds = float(getattr(config, "NO_CAR_CLEAR_SECONDS", 2.0))
        no_car_clear_seconds = max(0.0, no_car_clear_seconds)

        enter_idle = (not vehicles) and ((now - float(self._last_vehicle_seen_ts)) >= no_car_clear_seconds)
        if enter_idle and not self._no_car_idle_mode:
            self.vehicle_detector.reset()
            self.accident_detector.reset()
            self.violation_detector.reset()
            self._no_car_idle_mode = True
        elif not enter_idle:
            self._no_car_idle_mode = False
        
        lane_counts = self.vehicle_detector.get_vehicle_count_by_lane(include_intersection=True)
        lane_counts_for_control = {k: int(lane_counts.get(k, 0)) for k in config.LANES.keys()}

        is_camera_1 = (config.CAMERA_SOURCE == 1)
        is_camera_2 = (config.CAMERA_SOURCE == 2)

        if is_camera_1:
            self.update_signal_cycle(lane_counts_for_control)
        
        if self._no_car_idle_mode or not is_camera_1:
            accidents = []
            confirmed_accidents = []
        else:
            accidents = self.accident_detector.detect_accidents(vehicles, frame=frame)
            confirmed_accidents = self.accident_detector.get_confirmed_accidents()
        
        for accident in confirmed_accidents:
            if not accident.notified:
                notified = self.emergency_notifier.notify_accident(accident)
                if notified:
                    self.event_logger.log_accident(accident, notified=True)
        
        if self._no_car_idle_mode or not is_camera_1:
            new_violations = []
        else:
            new_violations = self.violation_detector.detect_violations(vehicles)
        
        for violation in new_violations:
            self.event_logger.log_violation(violation)
            print(f"[VIOLATION] {violation.get_description()}", flush=True)
        
        if config.ENABLE_LICENSE_PLATE_DETECTION and is_camera_2:
            self._lp_frame_index += 1
            # Retrieve async OCR result from the dummy vehicle ID 999
            res = self.license_plate_detector.get_result(999)
            if res:
                plate_text, confidence, plate_bbox = res
                cleaned = "".join(c for c in plate_text if c.isalnum()).upper()
                print(f"[OCR RAW READ] Found text: '{plate_text}' -> Cleaned: '{cleaned}' (Conf: {confidence:.1f}%)", flush=True)
                if len(cleaned) == 6 and cleaned[:3].isalpha() and cleaned[3:].isdigit():
                    self._detected_plates[cleaned] = (plate_bbox, confidence, time.time())
                    print(f"  ↳ ✓ Format Match! Displaying plate: {cleaned}", flush=True)

            # Submit next frame asynchronously if queue is not full
            self.license_plate_detector.submit_async(
                999,
                frame,
                (0, 0, frame.shape[1], frame.shape[0])
            )
        
        self.update_traffic_control(lane_counts_for_control, accidents)
        frame = self.draw_interface(frame, vehicles, accidents, lane_counts)
        
        return frame

    def _is_arduino_connected(self) -> bool:
        return bool(self.traffic_controller.serial_port and self.traffic_controller.serial_port.is_open)

    def _lane_order(self):
        return list(config.LANES.keys())

    def _get_next_lane(self, lane_key: str) -> str:
        order = self._lane_order()
        if not order:
            return lane_key
        if lane_key not in order:
            return order[0]
        idx = order.index(lane_key)
        return order[(idx + 1) % len(order)]

    def _classify_lane_congestion(self, count: int) -> str:
        if count >= getattr(config, "SIM_CONGESTED_CARS", 2):
            return "CONGESTED"
        if count == getattr(config, "SIM_NON_CONGESTED_CARS", 1):
            return "NON-CONGESTED"
        return "EMPTY"

    def _compute_green_duration(self, lane_key: str, lane_counts: dict) -> int:
        """Compute green duration for a lane based on congestion state."""
        base = int(getattr(config, "GREEN_TIME", 5))
        min_green = int(getattr(config, "MIN_GREEN_TIME", 3))
        max_green = int(getattr(config, "MAX_GREEN_TIME", 15))

        count = int(lane_counts.get(lane_key, 0))
        state = self._classify_lane_congestion(count)

        if not getattr(config, "ENABLE_ADAPTIVE_TIMING", True):
            return max(min_green, min(max_green, base))

        if state == "CONGESTED":
            delta = int(getattr(config, "ADAPTIVE_GREEN_EXTEND_SECONDS", 3))
            return max(min_green, min(max_green, base + max(0, delta)))
        if state == "NON-CONGESTED":
            delta = int(getattr(config, "ADAPTIVE_GREEN_REDUCE_SECONDS", 2))
            return max(min_green, min(max_green, base - max(0, delta)))
        return min_green

    def _apply_signal_states(self, active_lane: str, phase: str):
        """Apply signal colors to violation detector (and UI)."""
        for lane in config.LANES.keys():
            if lane == active_lane:
                self.violation_detector.set_traffic_signal(lane, phase)
            else:
                self.violation_detector.set_traffic_signal(lane, "RED")

    def update_signal_cycle(self, lane_counts: dict):
        """Advance the traffic signal cycle. Simulates signals when Arduino is disconnected."""
        now = time.time()
        self.arduino_connected = self._is_arduino_connected()

        simulate = bool(getattr(config, "SIMULATE_SIGNALS_WHEN_NO_ARDUINO", True)) and not self.arduino_connected
        if not simulate:
            if self.software_auto_mode:
                arduino_green_time = float(getattr(config, "GREEN_TIME", 25))
                arduino_yellow_time = float(getattr(config, "YELLOW_TIME", 4))

                if self.current_active_lane is None:
                    self.current_active_lane = self._lane_order()[0]
                    self.current_phase = "GREEN"
                    self.phase_start_time = now
                    self.lane_green_duration = arduino_green_time
                    self._apply_signal_states(self.current_active_lane, "GREEN")
                    return

                if self.current_phase == "GREEN":
                    if now - self.phase_start_time >= arduino_green_time:
                        self.current_phase = "YELLOW"
                        self.phase_start_time = now
                        self._apply_signal_states(self.current_active_lane, "YELLOW")
                elif self.current_phase == "YELLOW":
                    if now - self.phase_start_time >= arduino_yellow_time:
                        next_lane = self._get_next_lane(self.current_active_lane)
                        self.current_active_lane = next_lane
                        self.current_phase = "GREEN"
                        self.phase_start_time = now
                        self.lane_green_duration = arduino_green_time
                        self._apply_signal_states(next_lane, "GREEN")
            return

        if not self.software_auto_mode and self.current_active_lane is None:
            self.activate_lane(self._lane_order()[0])
            self.lane_green_duration = self._compute_green_duration(self.current_active_lane, lane_counts)
            self._apply_signal_states(self.current_active_lane, "GREEN")
            return

        if self.current_active_lane is None:
            self.activate_lane(self._lane_order()[0])
            self.lane_green_duration = self._compute_green_duration(self.current_active_lane, lane_counts)
            self._apply_signal_states(self.current_active_lane, "GREEN")
            return

        if not self.software_auto_mode:
            self.current_phase = "GREEN"
            self._apply_signal_states(self.current_active_lane, "GREEN")
            return

        if self.current_phase == "GREEN":
            if now - self.phase_start_time >= float(self.lane_green_duration):
                self.current_phase = "YELLOW"
                self.phase_start_time = now
                self._apply_signal_states(self.current_active_lane, "YELLOW")
        elif self.current_phase == "YELLOW":
            if now - self.phase_start_time >= float(getattr(config, "YELLOW_TIME", 3)):
                next_lane = self._get_next_lane(self.current_active_lane)
                self.current_active_lane = next_lane
                self.current_phase = "GREEN"
                self.phase_start_time = now
                self.lane_green_duration = self._compute_green_duration(next_lane, lane_counts)
                self._apply_signal_states(next_lane, "GREEN")
    
    def update_traffic_control(self, lane_counts: dict, accidents: list):
        """Update traffic light control based on AI logic"""
        if not config.ENABLE_ADAPTIVE_TIMING:
            return
        
        if config.ENABLE_ACCIDENT_PRIORITY and accidents:
            for accident in accidents:
                if accident.confirmed and accident.lane and accident.lane in config.LANES:
                    if self.current_active_lane != accident.lane:
                        print(f"[AI] Accident detected in {accident.lane}, activating lane for clearance")
                        self.activate_lane(accident.lane)
                        self.lane_green_duration = config.ACCIDENT_PRIORITY_DURATION
                    return
        
        max_vehicles = 0
        congested_lane = None
        
        for lane_key, count in lane_counts.items():
            if count > max_vehicles:
                max_vehicles = count
                congested_lane = lane_key
        
        if max_vehicles >= config.CONGESTION_THRESHOLD:
            if self.current_active_lane == congested_lane:
                self.lane_green_duration = self._compute_green_duration(congested_lane, lane_counts)
            elif self._is_arduino_connected():
                print(f"[AI] Congestion detected in {congested_lane} ({max_vehicles} vehicles)")
                self.software_auto_mode = False
                self.activate_lane(congested_lane)
                self.lane_green_duration = self._compute_green_duration(congested_lane, lane_counts)
        else:
            if not self.software_auto_mode:
                print(f"[AI] Congestion cleared (max vehicles: {max_vehicles}), returning to AUTO mode")
                if self._is_arduino_connected():
                    self.traffic_controller.set_auto_mode()
                    self.current_active_lane = self._lane_order()[0]
                    self.current_phase = "GREEN"
                    self.phase_start_time = time.time()
                    self.lane_green_duration = float(getattr(config, "GREEN_TIME", 25))
                    self._apply_signal_states(self.current_active_lane, "GREEN")
                self.software_auto_mode = True
    
    def activate_lane(self, lane_key: str):
        """Activate a specific lane"""
        if lane_key not in config.LANES:
            return
        
        self.current_active_lane = lane_key
        self.lane_green_start_time = time.time()
        self.current_phase = "GREEN"
        self.phase_start_time = time.time()
        
        self.violation_detector.set_traffic_signal(lane_key, "GREEN")
        for other_lane in config.LANES.keys():
            if other_lane != lane_key:
                self.violation_detector.set_traffic_signal(other_lane, "RED")
        
        if self.traffic_controller.serial_port and self.traffic_controller.serial_port.is_open:
            self.traffic_controller.activate_lane_by_name(lane_key)
    
    def draw_interface(self, frame: np.ndarray, vehicles, accidents, lane_counts) -> np.ndarray:
        """Draw complete UI on frame"""
        is_camera_1 = (config.CAMERA_SOURCE == 1)
        is_camera_2 = (config.CAMERA_SOURCE == 2)

        if config.SHOW_LANE_REGIONS and is_camera_1:
            for lane_key, lane_data in config.LANES.items():
                region = np.array(lane_data["region"], dtype=np.int32)
                color = self._get_lane_color(lane_key)
                cv2.polylines(frame, [region], True, color, 2)
                
                center_x = int(np.mean([p[0] for p in region]))
                center_y = int(np.mean([p[1] for p in region]))
                cv2.putText(frame, lane_data["name"], (center_x - 30, center_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                
                stop_line = lane_data["stop_line"]
                cv2.line(frame, stop_line[0], stop_line[1], (0, 255, 255), 3)
        
        if is_camera_1:
            intersection = np.array(config.INTERSECTION_CENTER, dtype=np.int32)
            cv2.polylines(frame, [intersection], True, (255, 0, 255), 2)
        
        if not self._no_car_idle_mode and is_camera_1:
            frame = self.vehicle_detector.draw_vehicles(frame, vehicles)
            frame = self.accident_detector.draw_stopped_vehicles(frame, vehicles)
            frame = self.accident_detector.draw_accidents(frame)

            if config.SHOW_VIOLATIONS:
                frame = self.violation_detector.draw_violations(frame, recent_only=True)
        
        if config.ENABLE_LICENSE_PLATE_DETECTION and is_camera_2:
            try:
                now_ts = time.time()
                to_delete = []
                for plate_text, (plate_bbox, confidence, ts) in list(self._detected_plates.items()):
                    if (now_ts - ts) > 2.0:
                        to_delete.append(plate_text)
                    else:
                        frame = self.license_plate_detector.draw_license_plate(frame, plate_text, plate_bbox, confidence)
                for p in to_delete:
                    self._detected_plates.pop(p, None)
            except Exception:
                pass

        # ── Only draw the OpenCV HUD when running standalone (main.py directly).
        # ── When app.py sets self.show_cv_panel = False, this block is skipped
        # ── and all status info is rendered by app.py's CTk panel instead.
        if self.show_cv_panel:
            frame = self.draw_status_panel(frame, vehicles, accidents, lane_counts)
        
        return frame
    
    def draw_status_panel(self, frame: np.ndarray, vehicles, accidents, lane_counts) -> np.ndarray:
        """Draw status information panel"""
        height, width = frame.shape[:2]
        
        overlay = frame.copy()
        panel_height = 220
        cv2.rectangle(overlay, (0, 0), (width, panel_height), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.7, frame, 0.3, 0)
        
        y_offset = 25
        x_left = 20
        x_right = width // 2 + 20
        
        cv2.putText(frame, "USAD - AI TRAFFIC MANAGEMENT", (x_left, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        y_offset += 30
        
        arduino_status = "CONNECTED" if (self.traffic_controller.serial_port and 
                                        self.traffic_controller.serial_port.is_open) else "DISCONNECTED"
        status_color = (0, 255, 0) if arduino_status == "CONNECTED" else (0, 0, 255)
        cv2.putText(frame, f"Arduino: {arduino_status}", (x_left, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
        
        cv2.putText(frame, f"FPS: {self.fps:.1f} | Cam: {config.CAMERA_SOURCE}", (x_left + 250, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        y_offset += 25
        
        cv2.putText(frame, f"Vehicles: {len(vehicles)}", (x_left, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        lane_text = " | ".join([f"{lane}: {count}" for lane, count in lane_counts.items()])
        cv2.putText(frame, lane_text, (x_left + 150, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        active_lane = self.current_active_lane or "-"
        active_signal = self.violation_detector.current_signals.get(active_lane, "-") if self.current_active_lane else "-"
        now = time.time()
        remaining = 0.0
        if self.current_active_lane:
            if self.current_phase == "GREEN":
                remaining = max(0.0, float(self.lane_green_duration) - (now - self.phase_start_time))
            elif self.current_phase == "YELLOW":
                remaining = max(0.0, float(getattr(config, "YELLOW_TIME", 3)) - (now - self.phase_start_time))

        y_offset += 20
        cv2.putText(
            frame,
            f"Active: {active_lane} | Signal: {active_signal} | Remaining: {remaining:.1f}s",
            (x_left, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
        )

        y_offset += 20
        lane_states = []
        for lane_key in config.LANES.keys():
            count = int(lane_counts.get(lane_key, 0))
            state = self._classify_lane_congestion(count)
            lane_states.append(f"{lane_key}:{state}")

        if "INTERSECTION" in lane_counts:
            inter_count = int(lane_counts.get("INTERSECTION", 0))
            inter_state = self._classify_lane_congestion(inter_count)
            lane_states.append(f"INTERSECTION:{inter_state}")

        cv2.putText(
            frame,
            "Lane status: " + " | ".join(lane_states),
            (x_left, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1,
        )

        y_offset += 18
        durations = []
        for lane_key in config.LANES.keys():
            durations.append(f"{lane_key}:{self._compute_green_duration(lane_key, lane_counts)}s")
        cv2.putText(
            frame,
            "Adaptive green: " + " | ".join(durations),
            (x_left, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1,
        )
        
        y_offset += 25
        
        accident_count = len([a for a in accidents if a.confirmed])
        accident_color = (0, 0, 255) if accident_count > 0 else (255, 255, 255)
        cv2.putText(frame, f"Accidents: {accident_count}", (x_left, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, accident_color, 1)

        stopped_count = len(self.accident_detector.get_stopped_vehicle_ids())
        cv2.putText(
            frame, f"Stopped cars: {stopped_count}",
            (x_left + 140, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
        )
        
        violation_stats = self.violation_detector.get_statistics()
        cv2.putText(frame, f"Violations: {violation_stats['total_violations']}", (x_left + 200, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        
        y_offset += 25
        
        notif_count = self.emergency_notifier.get_notification_count()
        cv2.putText(frame, f"Emergency Calls: {notif_count}", (x_left, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        y_offset += 30
        
        # Controls
        cv2.putText(frame, "Controls: [Q]uit | [R]eset | [B]g reset | [A]uto | [1-4]Lanes | [S]tats | [F]ull | [C]am",
                   (x_left, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        return frame
    
    def _get_lane_color(self, lane_key: str):
        """Get color for lane based on key"""
        colors = {
            "LANE1": config.COLOR_LANE1,
            "LANE2": config.COLOR_LANE2,
            "LANE3": config.COLOR_LANE3,
            "LANE4": config.COLOR_LANE4
        }
        return colors.get(lane_key, (255, 255, 255))
    
    def handle_keyboard(self, key: int) -> bool:
        """Handle keyboard input"""
        if key == ord('q') or key == ord('Q') or key == 27:
            return False
        elif key == ord('r') or key == ord('R'):
            print("\n[System] Resetting...")
            self.vehicle_detector.reset(reset_background=False, verbose=False)
            self.accident_detector.reset()
            self.violation_detector.reset()
            self.emergency_notifier.reset()
            print("[System] ✓ Reset complete")
        elif key == ord('b') or key == ord('B'):
            print("\n[System] Resetting background learning...")
            self.vehicle_detector.reset(reset_background=True, verbose=True)
            print("[System] ✓ Background learning reset")
        elif key == ord('a') or key == ord('A'):
            print("\n[Control] Switching to AUTO mode")
            if self.traffic_controller.serial_port and self.traffic_controller.serial_port.is_open:
                self.traffic_controller.set_auto_mode()
            else:
                self.software_auto_mode = True
        elif key == ord('1'):
            print("\n[Control] Activating LANE1")
            self.software_auto_mode = False
            self.activate_lane("LANE1")
        elif key == ord('2'):
            print("\n[Control] Activating LANE2")
            self.software_auto_mode = False
            self.activate_lane("LANE2")
        elif key == ord('3'):
            print("\n[Control] Activating LANE3")
            self.software_auto_mode = False
            self.activate_lane("LANE3")
        elif key == ord('4'):
            print("\n[Control] Activating LANE4")
            self.software_auto_mode = False
            self.activate_lane("LANE4")
        elif key == ord('s') or key == ord('S'):
            self.print_statistics()
        elif key == ord('f') or key == ord('F'):
            self.toggle_fullscreen()
        
        # C - Cycle camera source
        elif key == ord('c') or key == ord('C'):
            self.cycle_camera()
        
        return True
    
    def print_statistics(self):
        """Print current statistics (from logged data so terminal matches analytics report)"""
        print("\n" + "="*70)
        print("CURRENT STATISTICS")
        print("="*70)

        v_analytics = self.event_logger.get_violation_analytics()
        a_analytics = self.event_logger.get_accident_analytics()

        total_v = v_analytics.get('total', 0)
        print(f"\nViolations: {total_v}")
        if v_analytics.get('by_type'):
            for vtype, count in v_analytics['by_type'].items():
                print(f"  - {vtype}: {count}")

        total_a = a_analytics.get('total', 0)
        emergency_n = a_analytics.get('emergency_notified', 0)
        print(f"\nActive Accidents (logged this session): {total_a}")
        if a_analytics.get('by_type'):
            for atype, count in a_analytics['by_type'].items():
                print(f"  - {atype}: {count}")

        print(f"\nEmergency Notifications: {emergency_n}")

        print("\nGenerating full analytics report...")
        self.event_logger.generate_report()

        print("="*70 + "\n")
    
    def toggle_fullscreen(self):
        """Toggle fullscreen mode"""
        self.is_fullscreen = not self.is_fullscreen
        if self.is_fullscreen:
            cv2.setWindowProperty(config.DISPLAY_WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            print("\n[Display] Fullscreen mode: ON")
        else:
            cv2.setWindowProperty(config.DISPLAY_WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
            print("\n[Display] Fullscreen mode: OFF")
    
    def check_config_reload(self):
        """Check if config file has been modified and reload if needed.
        Skipped when running as a bundled exe (no source files available)."""
        if self._is_frozen:
            return
        current_time = time.time()
        if current_time - self.last_config_check < self.config_check_interval:
            return
        self.last_config_check = current_time
        try:
            if not os.path.exists(self.config_path):
                return
            current_mtime = os.path.getmtime(self.config_path)
            if current_mtime != self.config_last_modified:
                print("\n[Config] Detected changes in config.py, reloading...")
                importlib.reload(config)
                self.config_last_modified = current_mtime
                print("[Config] ✓ Configuration reloaded successfully")
                print("[Config] Lane regions and settings updated")
        except Exception as e:
            print(f"[Config] Error reloading config: {e}")
    
    def cycle_camera(self):
        """Cycle to the next camera source configured in config.CAMERA_SOURCES"""
        if not hasattr(config, "CAMERA_SOURCES") or not config.CAMERA_SOURCES:
            print("\n[Camera] No CAMERA_SOURCES list defined in config.py")
            return

        orig_source = config.CAMERA_SOURCE
        try:
            current_idx = config.CAMERA_SOURCES.index(orig_source)
            next_idx = (current_idx + 1) % len(config.CAMERA_SOURCES)
            next_source = config.CAMERA_SOURCES[next_idx]
        except ValueError:
            next_source = config.CAMERA_SOURCES[0]

        if next_source == orig_source:
            print(f"\n[Camera] Only one camera source ({orig_source}) is available.")
            return

        print(f"\n[Camera] Switching from source {orig_source} to {next_source}...")
        
        # Stop background grabber thread
        if hasattr(self, "_grabber") and self._grabber is not None:
            try:
                self._grabber.stop()
            except Exception:
                pass
            self._grabber = None

        # Release the current camera
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

        # Helper function to open a camera source
        def open_source(src):
            backends = [
                ("DSHOW", cv2.CAP_DSHOW),
                ("MSMF", getattr(cv2, "CAP_MSMF", cv2.CAP_ANY)),
                ("ANY", cv2.CAP_ANY),
            ]
            for name, backend in backends:
                try:
                    cap = cv2.VideoCapture(src, backend)
                    if cap is not None and cap.isOpened():
                        # Set properties
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
                        cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)
                        # Warm up
                        for _ in range(5):
                            cap.read()
                        print(f"[Camera] ✓ Opened source {src} using {name}")
                        return cap
                except Exception:
                    continue
            return None

        new_cap = open_source(next_source)
        if new_cap is not None:
            self.cap = new_cap
            config.CAMERA_SOURCE = next_source
            print(f"[Camera] ✓ Switched to source {next_source}")
        else:
            print(f"[ERROR] Failed to open source {next_source}. Reverting to source {orig_source}...")
            reverted_cap = open_source(orig_source)
            if reverted_cap is not None:
                self.cap = reverted_cap
                config.CAMERA_SOURCE = orig_source
                print(f"[Camera] ✓ Reverted to source {orig_source}")
            else:
                print(f"[CRITICAL] Could not re-open original source {orig_source} either!")
                self.cap = None

        # Reset detectors on camera switch to prevent old frames' vehicle positions/states
        # from leaking into the new camera source feed.
        try:
            self.vehicle_detector.reset()
            self.accident_detector.reset()
            self.violation_detector.reset()
        except Exception:
            pass

    def run(self):
        """Main application loop"""
        if not self.initialize_camera():
            print("[ERROR] Camera initialization failed")
            return
        
        arduino_connected = self.initialize_arduino()
        self.arduino_connected = arduino_connected
        
        print("\n" + "="*70)
        print("SYSTEM READY")
        print("="*70)
        print("\nPress 'Q' to quit")
        print("Press 'R' to reset")
        print("Press 'B' to reset background learning")
        print("Press 'A' for auto mode")
        print("Press '1-4' to activate specific lanes")
        print("Press 'S' for statistics")
        print("Press 'F' to toggle fullscreen")
        print("\n" + "="*70 + "\n")
        
        cv2.namedWindow(config.DISPLAY_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(config.DISPLAY_WINDOW_NAME, 1280, 720)

        if (not self._is_arduino_connected()) and getattr(config, "SIMULATE_SIGNALS_WHEN_NO_ARDUINO", True):
            if self.current_active_lane is None and config.LANES:
                self.activate_lane(list(config.LANES.keys())[0])
        
        try:
            while True:
                if self.cap is None:
                    # Create black display error frame
                    error_frame = np.zeros((config.CAMERA_HEIGHT, config.CAMERA_WIDTH, 3), dtype=np.uint8)
                    cv2.putText(
                        error_frame,
                        f"CAMERA {config.CAMERA_SOURCE} DISCONNECTED / ERROR",
                        (config.CAMERA_WIDTH // 2 - 350, config.CAMERA_HEIGHT // 2 - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 0, 255),
                        2,
                    )
                    cv2.putText(
                        error_frame,
                        "Press [C] to switch camera source or [Q] to quit",
                        (config.CAMERA_WIDTH // 2 - 280, config.CAMERA_HEIGHT // 2 + 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (200, 200, 200),
                        2,
                    )
                    # Draw standard status panel on error frame
                    error_frame = self.draw_status_panel(error_frame, [], [], {})
                    cv2.imshow(config.DISPLAY_WINDOW_NAME, error_frame)
                    
                    self.check_config_reload()
                    
                    key = cv2.waitKey(100) & 0xFF
                    if key != 0xFF:
                        if not self.handle_keyboard(key):
                            break
                    continue

                if self._grabber is None:
                    self._grabber = LatestFrameGrabber(self.cap)
                    self._grabber.start()
                    self._last_frame_received_ts = time.time()

                ret, frame, ts = self._grabber.get_latest()
                if not ret or frame is None:
                    # If we haven't received any frame for more than 3 seconds, show error frame
                    if time.time() - self._last_frame_received_ts > 3.0:
                        error_frame = np.zeros((config.CAMERA_HEIGHT, config.CAMERA_WIDTH, 3), dtype=np.uint8)
                        cv2.putText(
                            error_frame,
                            f"CAMERA {config.CAMERA_SOURCE} READ ERROR (TIMEOUT)",
                            (config.CAMERA_WIDTH // 2 - 380, config.CAMERA_HEIGHT // 2 - 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.9,
                            (0, 0, 255),
                            2,
                        )
                        cv2.putText(
                            error_frame,
                            "Press [C] to switch camera source or [Q] to quit",
                            (config.CAMERA_WIDTH // 2 - 280, config.CAMERA_HEIGHT // 2 + 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (200, 200, 200),
                            2,
                        )
                        error_frame = self.draw_status_panel(error_frame, [], [], {})
                        cv2.imshow(config.DISPLAY_WINDOW_NAME, error_frame)
                        
                        self.check_config_reload()
                        
                        key = cv2.waitKey(100) & 0xFF
                        if key != 0xFF:
                            if not self.handle_keyboard(key):
                                break
                    else:
                        time.sleep(0.005)
                    continue

                self._last_frame_received_ts = time.time()

                now_loop = time.time()
                if self._last_loop_ts is not None:
                    dt = max(1e-6, now_loop - float(self._last_loop_ts))
                    inst = 1.0 / dt
                    alpha = float(getattr(config, "FPS_EMA_ALPHA", 0.15) or 0.15)
                    if alpha > 0:
                        alpha = max(0.01, min(0.5, alpha))
                        if self._fps_ema <= 0:
                            self._fps_ema = inst
                        else:
                            self._fps_ema = (1.0 - alpha) * self._fps_ema + alpha * inst
                        self.fps = float(self._fps_ema)
                self._last_loop_ts = now_loop
                
                processed_frame = self.process_frame(frame)
                self.check_config_reload()
                self.frame_count += 1
                
                cv2.imshow(config.DISPLAY_WINDOW_NAME, processed_frame)
                
                if cv2.getWindowProperty(config.DISPLAY_WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    print("\n[System] Window closed by user")
                    break
                
                key = cv2.waitKey(1) & 0xFF
                if not self.handle_keyboard(key):
                    break
                
        except KeyboardInterrupt:
            print("\n\n[System] Interrupted by user")
        
        finally:
            print("\n[System] Shutting down...")
            self.print_statistics()

            if self._grabber is not None:
                self._grabber.stop()
                self._grabber = None
            
            if self.cap:
                self.cap.release()
            
            cv2.destroyAllWindows()
            
            if arduino_connected:
                self.traffic_controller.disconnect()
            
            print("[System] ✓ Shutdown complete")
            print("\nThank you for using USAD!")


# Entry point
if __name__ == "__main__":
    print("\n" * 2)
    app = USAD()
    app.run()