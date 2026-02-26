"""E.Y.E. (Eyeing Your Encounter) - traffic violation detection."""

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
        self.traffic_signal = traffic_signal
        self.location = location
        self.timestamp = time.time()
        
        # Snapshot vehicle state for logging/evidence.
        self.speed = vehicle.get_speed()
        self.direction = vehicle.get_direction()
        self.license_plate = vehicle.license_plate
        
    def get_description(self) -> str:
        """Get violation description"""
        plate = self.license_plate or getattr(self.vehicle, "license_plate", None) or "UNKNOWN-PLATE"
        lane = self.lane or "UNKNOWN-LANE"
        descriptions = {
            "RED_LIGHT_VIOLATION": f"{lane} car {plate} committed RED LIGHT VIOLATION",
            "YELLOW_ABUSE": f"{lane} car {plate} committed YELLOW LIGHT ABUSE",
            "ILLEGAL_TURN": f"{lane} car {plate} committed ILLEGAL TURN"
        }
        return descriptions.get(self.type, f"{lane} car {plate} committed UNKNOWN VIOLATION")
    
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
        # vehicle_id -> {"RED_LIGHT", "YELLOW_ABUSE", "ILLEGAL_TURN"} (prevents duplicates)
        self.checked_vehicles: Dict[int, set] = {}
        self.current_signals: Dict[str, str] = {lane: "GREEN" for lane in config.LANES.keys()}

        # Cache intersection centroid for oriented stop-line distance tests.
        try:
            pts = np.array(getattr(config, "INTERSECTION_CENTER", []), dtype=np.float32)
            if pts.size != 0:
                self._intersection_centroid = (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))
            else:
                self._intersection_centroid = (0.0, 0.0)
        except Exception:
            self._intersection_centroid = (0.0, 0.0)
        
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
            if vehicle.id not in self.checked_vehicles:
                self.checked_vehicles[vehicle.id] = set()
            
            red_light_violation = self._detect_red_light_violation(vehicle)
            if red_light_violation:
                new_violations.append(red_light_violation)
            
            yellow_abuse = self._detect_yellow_abuse(vehicle)
            if yellow_abuse:
                new_violations.append(yellow_abuse)
            
            illegal_turn = self._detect_illegal_turn(vehicle)
            if illegal_turn:
                new_violations.append(illegal_turn)
        
        self.violations.extend(new_violations)
        
        self._cleanup_old_vehicles(vehicles)
        
        return new_violations
    
    def _detect_red_light_violation(self, vehicle: Vehicle) -> Optional[Violation]:
        """Detect red light running"""
        if 'RED_LIGHT' in self.checked_vehicles.get(vehicle.id, set()):
            return None
        
        if not vehicle.current_lane:
            return None
        
        lane_key = vehicle.current_lane
        
        if self.current_signals.get(lane_key) != "RED":
            return None
        
        if not vehicle.crossed_stop_line:
            stop_line = config.LANES[lane_key]["stop_line"]
            if self._crossed_stop_line(vehicle, stop_line):
                vehicle.crossed_stop_line = True
                
                speed = vehicle.get_speed()
                if speed > config.RED_LIGHT_SPEED_THRESHOLD:
                    violation = Violation(
                        "RED_LIGHT_VIOLATION",
                        vehicle,
                        lane_key,
                        "RED",
                        vehicle.center
                    )
                    
                    self.checked_vehicles[vehicle.id].add('RED_LIGHT')
                    
                    return violation
        
        return None
    
    def _detect_yellow_abuse(self, vehicle: Vehicle) -> Optional[Violation]:
        """Detect speeding through yellow light"""
        if 'YELLOW_ABUSE' in self.checked_vehicles.get(vehicle.id, set()):
            return None
        
        if not vehicle.current_lane:
            return None
        
        lane_key = vehicle.current_lane
        
        if self.current_signals.get(lane_key) != "YELLOW":
            return None
        
        speed = vehicle.get_speed()
        if speed < config.YELLOW_ABUSE_SPEED_THRESHOLD:
            return None
        
        stop_line = config.LANES[lane_key]["stop_line"]
        distance_to_line = self._distance_to_line(vehicle.center, stop_line)
        
        # A vehicle far from the stop line should slow/stop; flag if it still crosses fast.
        if distance_to_line > config.YELLOW_SAFE_DISTANCE:
            if self._crossed_stop_line(vehicle, stop_line):
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
        if 'ILLEGAL_TURN' in self.checked_vehicles.get(vehicle.id, set()):
            return None
        
        if not vehicle.current_lane:
            return None
        
        if len(vehicle.positions) < config.ILLEGAL_TURN_FRAMES:
            return None
        
        lane_key = vehicle.current_lane
        lane_direction = config.LANES[lane_key]["direction"]
        
        vehicle_direction = vehicle.get_direction()
        
        # If a vehicle's movement strongly deviates from the lane axis, flag as a turn.
        if lane_direction == "vertical":
            horizontal_component = abs(vehicle_direction[0])
            
            if horizontal_component > 0.7:
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
            vertical_component = abs(vehicle_direction[1])
            
            if vertical_component > 0.7:
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
    
    def _signed_distance_to_stop_line(self, point: Tuple[float, float], stop_line: List[Tuple[int, int]]) -> float:
        """Signed perpendicular distance to stop line in pixels.

        Sign is oriented such that positive means "toward the intersection".
        """
        p = np.array(point, dtype=np.float32)
        p1 = np.array(stop_line[0], dtype=np.float32)
        p2 = np.array(stop_line[1], dtype=np.float32)
        v = p2 - p1
        denom = float(np.linalg.norm(v))
        if denom <= 1e-6:
            return 0.0

        # 2D cross product magnitude gives signed area; divide by |v| for distance.
        dist = float(np.cross(v, (p - p1)) / denom)

        # Orient sign so the intersection centroid is on the positive side.
        ic = np.array(self._intersection_centroid, dtype=np.float32)
        dist_ic = float(np.cross(v, (ic - p1)) / denom)
        if dist_ic < 0:
            dist = -dist
        return dist

    def _crossed_stop_line(self, vehicle: Vehicle, stop_line: List[Tuple[int, int]]) -> bool:
        """Check if vehicle crossed stop line.

        Uses a "past the line" signed-distance threshold to avoid missing crossings
        when FPS dips or when the vehicle center lands exactly on the line.
        """
        if len(vehicle.positions) < 2:
            return False

        # Use the raw observed centers in history for crossing tests.
        prev_pos = tuple(map(float, vehicle.positions[-2]))
        curr_pos = tuple(map(float, vehicle.positions[-1]))

        prev_d = self._signed_distance_to_stop_line(prev_pos, stop_line)
        curr_d = self._signed_distance_to_stop_line(curr_pos, stop_line)

        past_px = float(getattr(config, "RED_LIGHT_CROSSING_THRESHOLD", 0.0) or 0.0)
        past_px = max(0.0, past_px)

        # Primary: moved from not-past to past.
        if prev_d < past_px and curr_d >= past_px:
            return True

        # Fallback: strict segment intersection (catches weird geometry cases).
        line_pt1 = np.array(stop_line[0], dtype=np.float32)
        line_pt2 = np.array(stop_line[1], dtype=np.float32)
        p1 = np.array(prev_pos, dtype=np.float32)
        p2 = np.array(curr_pos, dtype=np.float32)
        return bool(self._line_intersects(p1, p2, line_pt1, line_pt2))
    
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
        delay = float(getattr(config, "VIOLATION_SCREEN_ALERT_DELAY_SECONDS", 0.0) or 0.0)
        delay = max(0.0, delay)
        
        for violation in self.violations:
            # Delay on-screen alerts to reduce flicker / false positives
            if (current_time - float(violation.timestamp)) < delay:
                continue

            # Only show recent violations (last 5 seconds)
            if recent_only and (current_time - violation.timestamp) > 5:
                continue
            
            x, y = map(int, violation.location)
            color = config.COLOR_VIOLATION
            
            cv2.drawMarker(frame, (x, y), color, cv2.MARKER_STAR, 20, 2)

            plate = str(violation.license_plate or getattr(violation.vehicle, "license_plate", None) or "N/A")
            lane = str(violation.lane or "N/A")
            label = violation.type.replace("_", " ")
            detail = f"{lane} | {plate}"
            cv2.putText(
                frame,
                label,
                (x - 80, y - 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )
            cv2.putText(
                frame,
                detail,
                (x - 80, y - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

            reason = violation.get_description()
            cv2.putText(
                frame,
                reason,
                (x - 80, y - 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                color,
                1,
            )
            
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
