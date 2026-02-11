"""
Vehicle Detection and Tracking Module
Uses OpenCV background subtraction and contour detection to track vehicles
"""

import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional
import time
import config

class Vehicle:
    """Represents a tracked vehicle"""
    
    _next_id = 1
    
    def __init__(self, center: Tuple[int, int], bbox: Tuple[int, int, int, int], area: float):
        self.id = Vehicle._next_id
        Vehicle._next_id += 1
        
        self.center = center
        self.bbox = bbox  # (x, y, w, h)
        self.area = area
        
        # Tracking history
        self.positions = [center]
        self.timestamps = [time.time()]
        self.lost_frames = 0
        
        # Lane information
        self.current_lane = None
        self.crossed_stop_line = False
        
        # Vehicle classification
        self.vehicle_type = self._classify_vehicle(area)
        
        # License plate
        self.license_plate = None
        self.license_plate_confidence = 0.0
        
        # Status flags
        self.is_stopped = False
        self.stopped_time = 0
        self.last_moved_time = time.time()
        
        # Violation tracking
        self.violations = []
        
    def _classify_vehicle(self, area: float) -> str:
        """Classify vehicle by size"""
        for vtype, (min_area, max_area) in config.VEHICLE_TYPES.items():
            if min_area <= area <= max_area:
                return vtype
        return "UNKNOWN"
    
    def update(self, center: Tuple[int, int], bbox: Tuple[int, int, int, int], area: float):
        """Update vehicle position"""
        self.center = center
        self.bbox = bbox
        self.area = area
        
        self.positions.append(center)
        self.timestamps.append(time.time())
        
        # Keep only recent history
        if len(self.positions) > 30:
            self.positions.pop(0)
            self.timestamps.pop(0)
        
        self.lost_frames = 0
        
        # Check if vehicle is stopped
        if len(self.positions) >= 2:
            distance = np.linalg.norm(np.array(center) - np.array(self.positions[-2]))
            
            if distance < config.STOPPED_DISTANCE_THRESHOLD:
                if not self.is_stopped:
                    self.stopped_time = time.time()
                    self.is_stopped = True
            else:
                self.is_stopped = False
                self.last_moved_time = time.time()
    
    def get_speed(self) -> float:
        """Calculate vehicle speed in pixels/second"""
        if len(self.positions) < 2:
            return 0.0
        
        # Use last 5 positions
        positions = self.positions[-5:]
        timestamps = self.timestamps[-5:]
        
        if len(positions) < 2:
            return 0.0
        
        total_distance = 0
        for i in range(1, len(positions)):
            distance = np.linalg.norm(np.array(positions[i]) - np.array(positions[i-1]))
            total_distance += distance
        
        time_elapsed = timestamps[-1] - timestamps[0]
        if time_elapsed > 0:
            return total_distance / time_elapsed
        return 0.0
    
    def get_direction(self) -> Tuple[float, float]:
        """Get movement direction vector"""
        if len(self.positions) < 2:
            return (0, 0)
        
        start = np.array(self.positions[0])
        end = np.array(self.positions[-1])
        direction = end - start
        
        # Normalize
        norm = np.linalg.norm(direction)
        if norm > 0:
            return tuple(direction / norm)
        return (0, 0)
    
    def get_stopped_duration(self) -> float:
        """Get how long vehicle has been stopped"""
        if self.is_stopped:
            return time.time() - self.stopped_time
        return 0.0


class VehicleDetector:
    """Detects and tracks vehicles using background subtraction with fixed background"""
    
    def __init__(self):
        # Background subtractor - learns background then stays fixed
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=300,  # Learn from first 10 seconds (300 frames at 30fps)
            varThreshold=30,
            detectShadows=False
        )
        
        # Morphological kernels
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        
        # Tracked vehicles
        self.vehicles: Dict[int, Vehicle] = {}
        
        # Background learning control
        self.frame_count = 0
        self.background_ready = False
        self.learning_frames = 300  # 10 seconds at 30fps
        
    def detect_vehicles(self, frame: np.ndarray) -> List[Vehicle]:
        """
        Detect vehicles in frame using a fixed background model.
        
        Args:
            frame: Input frame (BGR)
            
        Returns:
            List of detected/tracked vehicles
        """
        # Build a stable background for the first N frames
        self.frame_count += 1
        if not self.background_ready:
            self.bg_subtractor.apply(frame, learningRate=1.0)
            if self.frame_count >= self.learning_frames:
                self.background_ready = True
            return []
        
        # Freeze background model, detect changes only
        fg_mask = self.bg_subtractor.apply(frame, learningRate=0.0)
        _, fg_mask = cv2.threshold(fg_mask, 200, 255, cv2.THRESH_BINARY)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, self.kernel_open)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, self.kernel_close)
        
        # Find contours
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Extract vehicle candidates and filter by lane regions
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if config.MIN_VEHICLE_AREA <= area <= config.MAX_VEHICLE_AREA:
                x, y, w, h = cv2.boundingRect(contour)
                center = (x + w // 2, y + h // 2)
                
                # Check if vehicle center is inside any lane region
                is_in_lane = False
                for lane_key, lane_data in config.LANES.items():
                    lane_region = np.array(lane_data["region"], dtype=np.int32)
                    result = cv2.pointPolygonTest(lane_region, center, False)
                    if result >= 0:
                        is_in_lane = True
                        break
                
                # Only add detection if it's inside a lane
                if is_in_lane:
                    detections.append({
                        'center': center,
                        'bbox': (x, y, w, h),
                        'area': area
                    })
        
        # Update tracked vehicles
        self._update_tracking(detections)
        
        # Increment lost frames and remove old vehicles
        to_remove = []
        for vehicle_id, vehicle in self.vehicles.items():
            vehicle.lost_frames += 1
            if vehicle.lost_frames > config.VEHICLE_LOST_FRAMES:
                to_remove.append(vehicle_id)
        
        for vehicle_id in to_remove:
            del self.vehicles[vehicle_id]
        
        return list(self.vehicles.values())
    
    def _update_tracking(self, detections: List[dict]):
        """Update vehicle tracking with new detections"""
        if not detections:
            return
        
        # Match detections to existing vehicles
        used_detections = set()
        
        for vehicle in self.vehicles.values():
            best_match = None
            best_distance = config.MAX_TRACKING_DISTANCE
            
            for i, detection in enumerate(detections):
                if i in used_detections:
                    continue
                
                distance = np.linalg.norm(
                    np.array(vehicle.center) - np.array(detection['center'])
                )
                
                if distance < best_distance:
                    best_distance = distance
                    best_match = i
            
            if best_match is not None:
                detection = detections[best_match]
                vehicle.update(
                    detection['center'],
                    detection['bbox'],
                    detection['area']
                )
                used_detections.add(best_match)
        
        # Create new vehicles for unmatched detections
        for i, detection in enumerate(detections):
            if i not in used_detections:
                vehicle = Vehicle(
                    detection['center'],
                    detection['bbox'],
                    detection['area']
                )
                self.vehicles[vehicle.id] = vehicle
    
    def get_vehicles_in_lane(self, lane_key: str) -> List[Vehicle]:
        """Get all vehicles in a specific lane"""
        if lane_key not in config.LANES:
            return []
        
        lane_region = np.array(config.LANES[lane_key]["region"], dtype=np.int32)
        
        # Apply padding to shrink lane detection area (prevents edge detection)
        if hasattr(config, 'LANE_PADDING') and config.LANE_PADDING > 0:
            # Calculate center of lane
            center_x = int(np.mean(lane_region[:, 0]))
            center_y = int(np.mean(lane_region[:, 1]))
            
            # Shrink polygon towards center by padding amount
            padded_region = []
            for point in lane_region:
                dx = point[0] - center_x
                dy = point[1] - center_y
                dist = np.sqrt(dx*dx + dy*dy)
                if dist > 0:
                    # Move point towards center by padding amount
                    scale = max(0, (dist - config.LANE_PADDING)) / dist
                    new_x = int(center_x + dx * scale)
                    new_y = int(center_y + dy * scale)
                    padded_region.append([new_x, new_y])
                else:
                    padded_region.append([point[0], point[1]])
            lane_region = np.array(padded_region, dtype=np.int32)
        
        vehicles_in_lane = []
        
        for vehicle in self.vehicles.values():
            # Check if vehicle center is inside lane polygon
            result = cv2.pointPolygonTest(lane_region, vehicle.center, False)
            if result >= 0:
                vehicle.current_lane = lane_key
                vehicles_in_lane.append(vehicle)
        
        return vehicles_in_lane
    
    def get_vehicle_count_by_lane(self) -> Dict[str, int]:
        """Get vehicle count for each lane"""
        counts = {lane: 0 for lane in config.LANES.keys()}
        
        for lane_key in config.LANES.keys():
            counts[lane_key] = len(self.get_vehicles_in_lane(lane_key))
        
        return counts
    
    def draw_vehicles(self, frame: np.ndarray, vehicles: List[Vehicle]) -> np.ndarray:
        """Draw vehicles on frame"""
        for vehicle in vehicles:
            x, y, w, h = vehicle.bbox
            
            # Draw bounding box
            color = config.COLOR_VEHICLE
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            
            # Draw center point
            cv2.circle(frame, vehicle.center, 4, color, -1)
            
            # Draw vehicle ID and info
            if config.SHOW_VEHICLE_IDS:
                label = f"ID:{vehicle.id} {vehicle.vehicle_type}"
                if vehicle.license_plate:
                    label += f" {vehicle.license_plate}"
                
                cv2.putText(frame, label, (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw trajectory
            if len(vehicle.positions) > 1:
                points = np.array(vehicle.positions, dtype=np.int32)
                cv2.polylines(frame, [points], False, color, 1)
        
        return frame
    
    def reset(self):
        """Reset detector and clear all vehicles"""
        self.vehicles.clear()
        Vehicle._next_id = 1
