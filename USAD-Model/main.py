"""
USAD - Urban Smart Adaptive Dispatcher
Main Application

AI-powered traffic management system with:
- Computer vision vehicle detection
- Accident detection with emergency notifications
- E.Y.E. violation detection system
- License plate OCR
- Arduino traffic light control
- Real-time analytics and logging
"""

import cv2
import numpy as np
import time
import sys
import os
import importlib
import config
from traffic_controller import TrafficController
from vehicle_detector import VehicleDetector
from accident_detector import AccidentDetector
from violation_detector import ViolationDetector
from license_plate_detector import LicensePlateDetector
from event_logger import EventLogger
from emergency_notifier import EmergencyNotifier


class USAD:
    """Main USAD application controller"""
    
    def __init__(self):
        print("="*70)
        print("USAD - Urban Smart Adaptive Dispatcher")
        print("AI Traffic Management System")
        print("="*70)
        
        # Initialize components
        self.traffic_controller = TrafficController()
        self.vehicle_detector = VehicleDetector()
        self.accident_detector = AccidentDetector()
        self.violation_detector = ViolationDetector()
        self.license_plate_detector = LicensePlateDetector()
        self.event_logger = EventLogger()
        self.emergency_notifier = EmergencyNotifier()
        
        # State variables
        self.current_active_lane = None
        self.lane_green_start_time = None
        self.lane_green_duration = config.GREEN_TIME
        self.current_phase = "GREEN"  # GREEN, YELLOW, RED
        self.phase_start_time = time.time()
        
        # Performance metrics
        self.fps = 0
        self.frame_count = 0
        self.start_time = time.time()
        
        # Video capture
        self.cap = None
        
        # Display settings
        self.is_fullscreen = False
        
        # Config hot reload
        self.config_last_modified = os.path.getmtime('USAD-Model/config.py')
        self.config_check_interval = 1.0  # Check every 1 second
        self.last_config_check = time.time()
        
    def initialize_camera(self) -> bool:
        """Initialize camera or video source"""
        print("\n[Camera] Initializing...")
        
        self.cap = cv2.VideoCapture(config.CAMERA_SOURCE)
        
        if not self.cap.isOpened():
            print(f"[ERROR] Could not open camera source: {config.CAMERA_SOURCE}")
            return False
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)
        
        print(f"[Camera] ✓ Initialized ({config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT} @ {config.CAMERA_FPS}fps)")
        return True
    
    def initialize_arduino(self) -> bool:
        """Initialize Arduino connection"""
        print("\n[Arduino] Connecting to COM6...")
        
        if self.traffic_controller.connect():
            print(f"[Arduino] ✓ Connected on {config.ARDUINO_PORT}")
            
            # Start in auto mode if configured
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
        # Detect vehicles
        vehicles = self.vehicle_detector.detect_vehicles(frame)
        
        # Get vehicle counts by lane
        lane_counts = self.vehicle_detector.get_vehicle_count_by_lane()
        
        # Detect accidents
        accidents = self.accident_detector.detect_accidents(vehicles)
        confirmed_accidents = self.accident_detector.get_confirmed_accidents()
        
        # Handle emergency notifications for confirmed accidents
        for accident in confirmed_accidents:
            if not accident.notified:
                notified = self.emergency_notifier.notify_accident(accident)
                if notified:
                    self.event_logger.log_accident(accident, notified=True)
        
        # Detect violations
        new_violations = self.violation_detector.detect_violations(vehicles)
        
        # Log new violations
        for violation in new_violations:
            self.event_logger.log_violation(violation)
        
        # License plate detection
        if config.ENABLE_LICENSE_PLATE_DETECTION:
            for vehicle in vehicles:
                if not vehicle.license_plate:
                    result = self.license_plate_detector.detect_license_plate(frame, vehicle.bbox)
                    if result:
                        plate_text, confidence, plate_bbox = result
                        vehicle.license_plate = plate_text
                        vehicle.license_plate_confidence = confidence
        
        # Intelligent traffic control
        self.update_traffic_control(lane_counts, accidents)
        
        # Draw everything on frame
        frame = self.draw_interface(frame, vehicles, accidents, lane_counts)
        
        return frame
    
    def update_traffic_control(self, lane_counts: dict, accidents: list):
        """Update traffic light control based on AI logic"""
        if not config.ENABLE_ADAPTIVE_TIMING:
            return
        
        # Check for accidents - give priority to accident lanes
        if config.ENABLE_ACCIDENT_PRIORITY and accidents:
            for accident in accidents:
                if accident.confirmed and accident.lane and accident.lane in config.LANES:
                    # Clear the accident lane
                    if self.current_active_lane != accident.lane:
                        print(f"[AI] Accident detected in {accident.lane}, activating lane for clearance")
                        self.activate_lane(accident.lane)
                        self.lane_green_duration = config.ACCIDENT_PRIORITY_DURATION
                    return
        
        # Adaptive timing based on congestion
        max_vehicles = 0
        congested_lane = None
        
        for lane_key, count in lane_counts.items():
            if count > max_vehicles:
                max_vehicles = count
                congested_lane = lane_key
        
        # If a lane has congestion, give it more green time
        if max_vehicles >= config.CONGESTION_THRESHOLD:
            if self.current_active_lane != congested_lane:
                print(f"[AI] Congestion detected in {congested_lane} ({max_vehicles} vehicles)")
                self.activate_lane(congested_lane)
                
                # Calculate extended green time
                extra_time = min(max_vehicles, config.MAX_GREEN_TIME - config.GREEN_TIME)
                self.lane_green_duration = config.GREEN_TIME + extra_time
                print(f"[AI] Extended green time: {self.lane_green_duration}s")
    
    def activate_lane(self, lane_key: str):
        """Activate a specific lane"""
        if lane_key not in config.LANES:
            return
        
        self.current_active_lane = lane_key
        self.lane_green_start_time = time.time()
        self.current_phase = "GREEN"
        self.phase_start_time = time.time()
        
        # Update violation detector
        self.violation_detector.set_traffic_signal(lane_key, "GREEN")
        for other_lane in config.LANES.keys():
            if other_lane != lane_key:
                self.violation_detector.set_traffic_signal(other_lane, "RED")
        
        # Send command to Arduino
        if self.traffic_controller.serial_port and self.traffic_controller.serial_port.is_open:
            self.traffic_controller.activate_lane_by_name(lane_key)
    
    def draw_interface(self, frame: np.ndarray, vehicles, accidents, lane_counts) -> np.ndarray:
        """Draw complete UI on frame"""
        # Draw lane regions
        if config.SHOW_LANE_REGIONS:
            for lane_key, lane_data in config.LANES.items():
                region = np.array(lane_data["region"], dtype=np.int32)
                color = self._get_lane_color(lane_key)
                cv2.polylines(frame, [region], True, color, 2)
                
                # Draw lane label
                center_x = int(np.mean([p[0] for p in region]))
                center_y = int(np.mean([p[1] for p in region]))
                cv2.putText(frame, lane_data["name"], (center_x - 30, center_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                
                # Draw stop line
                stop_line = lane_data["stop_line"]
                cv2.line(frame, stop_line[0], stop_line[1], (0, 255, 255), 3)
        
        # Draw intersection center
        intersection = np.array(config.INTERSECTION_CENTER, dtype=np.int32)
        cv2.polylines(frame, [intersection], True, (255, 0, 255), 2)
        
        # Draw vehicles
        frame = self.vehicle_detector.draw_vehicles(frame, vehicles)
        
        # Draw accidents
        frame = self.accident_detector.draw_accidents(frame)
        
        # Draw violations
        if config.SHOW_VIOLATIONS:
            frame = self.violation_detector.draw_violations(frame, recent_only=True)
        
        # Draw license plates
        if config.ENABLE_LICENSE_PLATE_DETECTION:
            for vehicle in vehicles:
                if vehicle.license_plate:
                    # Plate already drawn in vehicle drawing
                    pass
        
        # Draw status panel
        frame = self.draw_status_panel(frame, vehicles, accidents, lane_counts)
        
        return frame
    
    def draw_status_panel(self, frame: np.ndarray, vehicles, accidents, lane_counts) -> np.ndarray:
        """Draw status information panel"""
        height, width = frame.shape[:2]
        
        # Create semi-transparent overlay
        overlay = frame.copy()
        panel_height = 180
        cv2.rectangle(overlay, (0, 0), (width, panel_height), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.7, frame, 0.3, 0)
        
        y_offset = 25
        x_left = 20
        x_right = width // 2 + 20
        
        # System status
        cv2.putText(frame, "USAD - AI TRAFFIC MANAGEMENT", (x_left, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        y_offset += 30
        
        # Arduino status
        arduino_status = "CONNECTED" if (self.traffic_controller.serial_port and 
                                        self.traffic_controller.serial_port.is_open) else "DISCONNECTED"
        status_color = (0, 255, 0) if arduino_status == "CONNECTED" else (0, 0, 255)
        cv2.putText(frame, f"Arduino: {arduino_status}", (x_left, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
        
        # FPS
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (x_left + 250, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        y_offset += 25
        
        # Vehicle counts
        cv2.putText(frame, f"Vehicles: {len(vehicles)}", (x_left, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Lane counts
        lane_text = " | ".join([f"{lane}: {count}" for lane, count in lane_counts.items()])
        cv2.putText(frame, lane_text, (x_left + 150, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        y_offset += 25
        
        # Accidents
        accident_count = len([a for a in accidents if a.confirmed])
        accident_color = (0, 0, 255) if accident_count > 0 else (255, 255, 255)
        cv2.putText(frame, f"Accidents: {accident_count}", (x_left, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, accident_color, 1)
        
        # Violations
        violation_stats = self.violation_detector.get_statistics()
        cv2.putText(frame, f"Violations: {violation_stats['total_violations']}", (x_left + 200, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        
        y_offset += 25
        
        # Emergency notifications
        notif_count = self.emergency_notifier.get_notification_count()
        cv2.putText(frame, f"Emergency Calls: {notif_count}", (x_left, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        y_offset += 30
        
        # Controls
        cv2.putText(frame, "Controls: [Q]uit | [R]eset | [A]uto | [1-4]Lanes | [S]tats | [F]ullscreen",
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
        # Q - Quit
        if key == ord('q') or key == ord('Q') or key == 27:  # ESC
            return False
        
        # R - Reset
        elif key == ord('r') or key == ord('R'):
            print("\n[System] Resetting...")
            self.vehicle_detector.reset()
            self.accident_detector.reset()
            self.violation_detector.reset()
            self.emergency_notifier.reset()
            print("[System] ✓ Reset complete")
        
        # A - Auto mode
        elif key == ord('a') or key == ord('A'):
            print("\n[Control] Switching to AUTO mode")
            if self.traffic_controller.serial_port and self.traffic_controller.serial_port.is_open:
                self.traffic_controller.set_auto_mode()
        
        # 1-4 - Manual lane activation
        elif key == ord('1'):
            print("\n[Control] Activating LANE1")
            self.activate_lane("LANE1")
        elif key == ord('2'):
            print("\n[Control] Activating LANE2")
            self.activate_lane("LANE2")
        elif key == ord('3'):
            print("\n[Control] Activating LANE3")
            self.activate_lane("LANE3")
        elif key == ord('4'):
            print("\n[Control] Activating LANE4")
            self.activate_lane("LANE4")
        
        # S - Statistics
        elif key == ord('s') or key == ord('S'):
            self.print_statistics()
        
        # F - Toggle fullscreen
        elif key == ord('f') or key == ord('F'):
            self.toggle_fullscreen()
        
        return True
    
    def print_statistics(self):
        """Print current statistics"""
        print("\n" + "="*70)
        print("CURRENT STATISTICS")
        print("="*70)
        
        # Violations
        v_stats = self.violation_detector.get_statistics()
        print(f"\nViolations: {v_stats['total_violations']}")
        if v_stats.get('by_type'):
            for vtype, count in v_stats['by_type'].items():
                print(f"  - {vtype}: {count}")
        
        # Accidents
        accidents = self.accident_detector.get_confirmed_accidents()
        print(f"\nActive Accidents: {len(accidents)}")
        for accident in accidents:
            print(f"  - {accident.get_description()}")
        
        # Emergency notifications
        print(f"\nEmergency Notifications: {self.emergency_notifier.get_notification_count()}")
        
        # Generate full report
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
        """Check if config file has been modified and reload if needed"""
        current_time = time.time()
        
        # Only check every config_check_interval seconds
        if current_time - self.last_config_check < self.config_check_interval:
            return
        
        self.last_config_check = current_time
        
        try:
            current_mtime = os.path.getmtime('USAD-Model/config.py')
            
            if current_mtime != self.config_last_modified:
                print("\n[Config] Detected changes in config.py, reloading...")
                
                # Reload the config module
                importlib.reload(config)
                
                # Update the cached modification time
                self.config_last_modified = current_mtime
                
                print("[Config] ✓ Configuration reloaded successfully")
                print("[Config] Lane regions and settings updated")
                
        except Exception as e:
            print(f"[Config] Error reloading config: {e}")
    
    def run(self):
        """Main application loop"""
        # Initialize components
        if not self.initialize_camera():
            print("[ERROR] Camera initialization failed")
            return
        
        arduino_connected = self.initialize_arduino()
        
        print("\n" + "="*70)
        print("SYSTEM READY")
        print("="*70)
        print("\nPress 'Q' to quit")
        print("Press 'R' to reset")
        print("Press 'A' for auto mode")
        print("Press '1-4' to activate specific lanes")
        print("Press 'S' for statistics")
        print("Press 'F' to toggle fullscreen")
        print("\n" + "="*70 + "\n")
        
        # Create window with fullscreen capability
        cv2.namedWindow(config.DISPLAY_WINDOW_NAME, cv2.WINDOW_NORMAL)
        
        # Main loop
        try:
            while True:
                ret, frame = self.cap.read()
                
                if not ret:
                    print("[ERROR] Failed to read frame")
                    break
                
                # Process frame
                processed_frame = self.process_frame(frame)
                
                # Check for config file changes
                self.check_config_reload()
                
                # Calculate FPS
                self.frame_count += 1
                elapsed = time.time() - self.start_time
                if elapsed > 1.0:
                    self.fps = self.frame_count / elapsed
                    self.frame_count = 0
                    self.start_time = time.time()
                
                # Display
                cv2.imshow(config.DISPLAY_WINDOW_NAME, processed_frame)
                
                # Check if window was closed
                if cv2.getWindowProperty(config.DISPLAY_WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    print("\n[System] Window closed by user")
                    break
                
                # Handle keyboard
                key = cv2.waitKey(1) & 0xFF
                if not self.handle_keyboard(key):
                    break
                
        except KeyboardInterrupt:
            print("\n\n[System] Interrupted by user")
        
        finally:
            # Cleanup
            print("\n[System] Shutting down...")
            
            # Print final statistics
            self.print_statistics()
            
            # Release resources
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
