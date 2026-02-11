"""
E.Y.E. (Eyeing Your Encounter) - Violation Detection Module
Detects unsafe and illegal vehicle behaviors using classical computer vision
"""

import cv2
import numpy as np
from typing import List, Dict, Optional, Tuple
import time
import config
from vehicle_detector import Vehicle


class Violation:
    """Represents a detected traffic violation"""
    
    _next_id = 1
    
    def __init__(self, violation_type: str, vehicle: Vehicle, lane: str, 
                 traffic_signal: str, location: Tuple[int, int]):
        self.id = Violation._next_id
        Violation._next_id += 1
        
        self.type = violation_type
        self.vehicle = vehicle
        self.lane = lane
        self.traffic_signal = traffic_signal  # "RED", "YELLOW", "GREEN"
        self.location = location
        self.timestamp = time.time()
        
        # Evidence
        self.speed = vehicle.get_speed()
        self.direction = vehicle.get_direction()
        self.license_plate = vehicle.license_plate
        
    def get_description(self) -> str:
        """Get violation description"""
        descriptions = {
            "RED_LIGHT_VIOLATION": f"Vehicle {self.vehicle.id} ran red light in {self.lane}",
            "YELLOW_ABUSE": f"Vehicle {self.vehicle.id} sped through yellow light in {self.lane}",
            "ILLEGAL_TURN": f"Vehicle {self.vehicle.id} made illegal turn in {self.lane}"
        }
        return descriptions.get(self.type, f"Unknown violation in {self.lane}")
    
    def to_dict(self) -> dict:
        """Convert to dictionary for logging"""
        return {
            'violation_id': self.id,
            'type': self.type,
            'vehicle_id': self.vehicle.id,
            'vehicle_type': self.vehicle.vehicle_type,
            'license_plate': self.license_plate or 'N/A',
            'lane': self.lane,
            'traffic_signal': self.traffic_signal,
            'speed': round(self.speed, 2),
            'location': self.location,
            'timestamp': self.timestamp
        }


class ViolationDetector:
    """E.Y.E. system for detecting traffic violations"""
    
    def __init__(self):
        self.violations: List[Violation] = []
        self.checked_vehicles: Dict[int, set] = {}  # vehicle_id -> set of violation types detected
        self.current_signals: Dict[str, str] = {lane: "GREEN" for lane in config.LANES.keys()}
        
    def set_traffic_signal(self, lane: str, signal: str):
        """Update traffic signal state for a lane"""
        if lane in self.current_signals:
            self.current_signals[lane] = signal
    
    def detect_violations(self, vehicles: List[Vehicle]) -> List[Violation]:
        """
        Detect all types of violations
        
        Args:
            vehicles: List of tracked vehicles
            
        Returns:
            List of detected violations
        """
        new_violations = []
        
        for vehicle in vehicles:
            # Initialize checked violations set for new vehicles
            if vehicle.id not in self.checked_vehicles:
                self.checked_vehicles[vehicle.id] = set()
            
            # Detect red light violations
            red_light_violation = self._detect_red_light_violation(vehicle)
            if red_light_violation:
                new_violations.append(red_light_violation)
            
            # Detect yellow light abuse
            yellow_abuse = self._detect_yellow_abuse(vehicle)
            if yellow_abuse:
                new_violations.append(yellow_abuse)
            
            # Detect illegal turns
            illegal_turn = self._detect_illegal_turn(vehicle)
            if illegal_turn:
                new_violations.append(illegal_turn)
        
        # Add to violations list
        self.violations.extend(new_violations)
        
        # Clean up old checked vehicles
        self._cleanup_old_vehicles(vehicles)
        
        return new_violations
    
    def _detect_red_light_violation(self, vehicle: Vehicle) -> Optional[Violation]:
        """Detect red light running"""
        # Skip if already detected for this vehicle
        if 'RED_LIGHT' in self.checked_vehicles.get(vehicle.id, set()):
            return None
        
        # Check if vehicle has a current lane
        if not vehicle.current_lane:
            return None
        
        lane_key = vehicle.current_lane
        
        # Check if signal is red
        if self.current_signals.get(lane_key) != "RED":
            return None
        
        # Check if vehicle crossed stop line
        if not vehicle.crossed_stop_line:
            stop_line = config.LANES[lane_key]["stop_line"]
            if self._crossed_line(vehicle, stop_line):
                vehicle.crossed_stop_line = True
                
                # Check if vehicle was moving (not stopped)
                speed = vehicle.get_speed()
                if speed > config.RED_LIGHT_SPEED_THRESHOLD:
                    # Create violation
                    violation = Violation(
                        "RED_LIGHT_VIOLATION",
                        vehicle,
                        lane_key,
                        "RED",
                        vehicle.center
                    )
                    
                    # Mark as checked
                    self.checked_vehicles[vehicle.id].add('RED_LIGHT')
                    
                    return violation
        
        return None
    
    def _detect_yellow_abuse(self, vehicle: Vehicle) -> Optional[Violation]:
        """Detect speeding through yellow light"""
        # Skip if already detected
        if 'YELLOW_ABUSE' in self.checked_vehicles.get(vehicle.id, set()):
            return None
        
        if not vehicle.current_lane:
            return None
        
        lane_key = vehicle.current_lane
        
        # Check if signal is yellow
        if self.current_signals.get(lane_key) != "YELLOW":
            return None
        
        # Check vehicle speed
        speed = vehicle.get_speed()
        if speed < config.YELLOW_ABUSE_SPEED_THRESHOLD:
            return None
        
        # Check distance from stop line
        stop_line = config.LANES[lane_key]["stop_line"]
        distance_to_line = self._distance_to_line(vehicle.center, stop_line)
        
        # If vehicle is far from stop line but speeding through yellow
        if distance_to_line > config.YELLOW_SAFE_DISTANCE:
            # Check if crossing stop line
            if self._crossed_line(vehicle, stop_line):
                violation = Violation(
                    "YELLOW_ABUSE",
                    vehicle,
                    lane_key,
                    "YELLOW",
                    vehicle.center
                )
                
                self.checked_vehicles[vehicle.id].add('YELLOW_ABUSE')
                return violation
        
        return None
    
    def _detect_illegal_turn(self, vehicle: Vehicle) -> Optional[Violation]:
        """Detect illegal turns based on direction"""
        if not config.ENABLE_ILLEGAL_TURN:
            return None
        # Skip if already detected
        if 'ILLEGAL_TURN' in self.checked_vehicles.get(vehicle.id, set()):
            return None
        
        if not vehicle.current_lane:
            return None
        
        # Need sufficient movement history
        if len(vehicle.positions) < config.ILLEGAL_TURN_FRAMES:
            return None
        
        lane_key = vehicle.current_lane
        lane_direction = config.LANES[lane_key]["direction"]
        
        # Calculate vehicle's actual direction
        vehicle_direction = vehicle.get_direction()
        
        # Calculate angle from expected direction
        if lane_direction == "vertical":
            # Expected: moving up (0, -1) or down (0, 1)
            # Check horizontal component
            horizontal_component = abs(vehicle_direction[0])
            
            if horizontal_component > 0.7:  # Strong horizontal movement
                # This is a turn - check if it's illegal
                # For now, we'll flag significant turns
                angle = np.arctan2(vehicle_direction[1], vehicle_direction[0])
                angle_deg = np.degrees(angle)
                
                if abs(angle_deg) > config.TURN_ANGLE_THRESHOLD:
                    violation = Violation(
                        "ILLEGAL_TURN",
                        vehicle,
                        lane_key,
                        self.current_signals.get(lane_key, "UNKNOWN"),
                        vehicle.center
                    )
                    
                    self.checked_vehicles[vehicle.id].add('ILLEGAL_TURN')
                    return violation
        
        elif lane_direction == "horizontal":
            # Expected: moving left (-1, 0) or right (1, 0)
            # Check vertical component
            vertical_component = abs(vehicle_direction[1])
            
            if vertical_component > 0.7:  # Strong vertical movement
                angle = np.arctan2(vehicle_direction[1], vehicle_direction[0])
                angle_deg = np.degrees(angle)
                
                if abs(90 - abs(angle_deg)) > config.TURN_ANGLE_THRESHOLD:
                    violation = Violation(
                        "ILLEGAL_TURN",
                        vehicle,
                        lane_key,
                        self.current_signals.get(lane_key, "UNKNOWN"),
                        vehicle.center
                    )
                    
                    self.checked_vehicles[vehicle.id].add('ILLEGAL_TURN')
                    return violation
        
        return None
    
    def _crossed_line(self, vehicle: Vehicle, stop_line: List[Tuple[int, int]]) -> bool:
        """Check if vehicle crossed stop line"""
        if len(vehicle.positions) < 2:
            return False
        
        # Create line segment from stop line
        line_pt1 = np.array(stop_line[0], dtype=np.float32)
        line_pt2 = np.array(stop_line[1], dtype=np.float32)
        
        # Check if vehicle trajectory crossed the line
        prev_pos = np.array(vehicle.positions[-2], dtype=np.float32)
        curr_pos = np.array(vehicle.positions[-1], dtype=np.float32)
        
        # Line intersection check
        crossed = self._line_intersects(prev_pos, curr_pos, line_pt1, line_pt2)
        
        return crossed
    
    def _line_intersects(self, p1: np.ndarray, p2: np.ndarray, 
                        q1: np.ndarray, q2: np.ndarray) -> bool:
        """Check if line segment p1-p2 intersects with q1-q2"""
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
        
        return ccw(p1, q1, q2) != ccw(p2, q1, q2) and ccw(p1, p2, q1) != ccw(p1, p2, q2)
    
    def _distance_to_line(self, point: Tuple[int, int], 
                         line: List[Tuple[int, int]]) -> float:
        """Calculate perpendicular distance from point to line"""
        p = np.array(point, dtype=np.float32)
        p1 = np.array(line[0], dtype=np.float32)
        p2 = np.array(line[1], dtype=np.float32)
        
        # Calculate distance
        d = np.abs(np.cross(p2 - p1, p1 - p)) / np.linalg.norm(p2 - p1)
        
        return d
    
    def _cleanup_old_vehicles(self, current_vehicles: List[Vehicle]):
        """Remove data for vehicles no longer tracked"""
        current_ids = {v.id for v in current_vehicles}
        old_ids = set(self.checked_vehicles.keys()) - current_ids
        
        for vehicle_id in old_ids:
            del self.checked_vehicles[vehicle_id]
    
    def get_violations_by_type(self, violation_type: str) -> List[Violation]:
        """Get violations of specific type"""
        return [v for v in self.violations if v.type == violation_type]
    
    def get_violations_by_lane(self, lane: str) -> List[Violation]:
        """Get violations in specific lane"""
        return [v for v in self.violations if v.lane == lane]
    
    def draw_violations(self, frame: np.ndarray, recent_only: bool = True) -> np.ndarray:
        """Draw violation markers on frame"""
        current_time = time.time()
        
        for violation in self.violations:
            # Only show recent violations (last 5 seconds)
            if recent_only and (current_time - violation.timestamp) > 5:
                continue
            
            x, y = violation.location
            color = config.COLOR_VIOLATION
            
            # Draw violation marker
            cv2.drawMarker(frame, (x, y), color, cv2.MARKER_STAR, 20, 2)
            
            # Draw violation label
            label = f"VIOLATION: {violation.type}"
            cv2.putText(frame, label, (x - 80, y - 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw vehicle ID
            vehicle_info = f"Vehicle {violation.vehicle.id}"
            if violation.license_plate:
                vehicle_info += f" ({violation.license_plate})"
            
            cv2.putText(frame, vehicle_info, (x - 80, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        return frame
    
    def get_statistics(self) -> dict:
        """Get violation statistics"""
        total = len(self.violations)
        
        by_type = {}
        for vtype in config.EVENT_TYPES.keys():
            if "VIOLATION" in vtype or "ABUSE" in vtype or "TURN" in vtype:
                count = len(self.get_violations_by_type(vtype))
                by_type[vtype] = count
        
        by_lane = {}
        for lane in config.LANES.keys():
            count = len(self.get_violations_by_lane(lane))
            by_lane[lane] = count
        
        return {
            'total_violations': total,
            'by_type': by_type,
            'by_lane': by_lane
        }
    
    def reset(self):
        """Reset detector"""
        self.violations.clear()
        self.checked_vehicles.clear()
        Violation._next_id = 1
