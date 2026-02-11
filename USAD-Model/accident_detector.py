"""
Accident Detection Module
Detects stopped vehicles and collisions at intersections
"""

import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional
import time
import config
from vehicle_detector import Vehicle


class Accident:
    """Represents a detected accident"""
    
    _next_id = 1
    
    def __init__(self, accident_type: str, vehicles: List[Vehicle], location: Tuple[int, int]):
        self.id = Accident._next_id
        Accident._next_id += 1
        
        self.type = accident_type  # "STOPPED" or "COLLISION"
        self.vehicles = vehicles
        self.location = location
        self.detected_time = time.time()
        self.confidence_frames = 0
        self.confirmed = False
        self.notified = False
        
        # Affected lane
        self.lane = self._determine_lane()
        
    def _determine_lane(self) -> Optional[str]:
        """Determine which lane the accident is in"""
        for lane_key, lane_data in config.LANES.items():
            lane_region = np.array(lane_data["region"], dtype=np.int32)
            result = cv2.pointPolygonTest(lane_region, self.location, False)
            if result >= 0:
                return lane_key
        
        # Check if in intersection center
        intersection_region = np.array(config.INTERSECTION_CENTER, dtype=np.int32)
        result = cv2.pointPolygonTest(intersection_region, self.location, False)
        if result >= 0:
            return "INTERSECTION"
        
        return None
    
    def update(self):
        """Update accident state"""
        self.confidence_frames += 1
        
        if self.confidence_frames >= config.ACCIDENT_CONFIDENCE_FRAMES:
            self.confirmed = True
    
    def get_duration(self) -> float:
        """Get how long accident has been detected"""
        return time.time() - self.detected_time
    
    def get_description(self) -> str:
        """Get accident description"""
        vehicle_ids = [str(v.id) for v in self.vehicles]
        duration = self.get_duration()
        
        if self.type == "STOPPED":
            return f"Stopped vehicle(s) detected (IDs: {', '.join(vehicle_ids)}) for {duration:.1f}s in {self.lane}"
        elif self.type == "COLLISION":
            return f"Collision detected between vehicles (IDs: {', '.join(vehicle_ids)}) in {self.lane}"
        
        return f"Unknown accident type in {self.lane}"


class AccidentDetector:
    """Detects accidents using vehicle tracking data"""
    
    def __init__(self):
        self.accidents: Dict[int, Accident] = {}
        self.checked_vehicles: set = set()  # Vehicle IDs already involved in accidents
        
    def detect_accidents(self, vehicles: List[Vehicle]) -> List[Accident]:
        """
        Detect accidents from vehicle list
        
        Args:
            vehicles: List of tracked vehicles
            
        Returns:
            List of active accidents
        """
        new_accidents = []
        
        # Detect stopped vehicles
        stopped_accidents = self._detect_stopped_vehicles(vehicles)
        new_accidents.extend(stopped_accidents)
        
        # Detect collisions
        collision_accidents = self._detect_collisions(vehicles)
        new_accidents.extend(collision_accidents)
        
        # Add new accidents to tracking
        for accident in new_accidents:
            self.accidents[accident.id] = accident
            # Mark vehicles as checked
            for vehicle in accident.vehicles:
                self.checked_vehicles.add(vehicle.id)
        
        # Update existing accidents
        to_remove = []
        for accident_id, accident in self.accidents.items():
            accident.update()
            
            # Remove old unconfirmed accidents
            if not accident.confirmed and accident.get_duration() > 10:
                to_remove.append(accident_id)
            
            # Remove very old confirmed accidents
            if accident.confirmed and accident.get_duration() > 120:
                to_remove.append(accident_id)
        
        for accident_id in to_remove:
            accident = self.accidents[accident_id]
            for vehicle in accident.vehicles:
                self.checked_vehicles.discard(vehicle.id)
            del self.accidents[accident_id]
        
        return list(self.accidents.values())
    
    def _detect_stopped_vehicles(self, vehicles: List[Vehicle]) -> List[Accident]:
        """Detect vehicles that have been stopped too long"""
        accidents = []
        
        for vehicle in vehicles:
            # Skip if already in an accident
            if vehicle.id in self.checked_vehicles:
                continue
            
            # Check if stopped in intersection or lane
            if vehicle.is_stopped:
                stopped_duration = vehicle.get_stopped_duration()
                
                if stopped_duration >= config.STOPPED_TIME_THRESHOLD:
                    # Check if in intersection center
                    intersection_region = np.array(config.INTERSECTION_CENTER, dtype=np.int32)
                    in_intersection = cv2.pointPolygonTest(
                        intersection_region, vehicle.center, False
                    ) >= 0
                    
                    # Check if in any lane
                    in_lane = False
                    for lane_key, lane_data in config.LANES.items():
                        lane_region = np.array(lane_data["region"], dtype=np.int32)
                        if cv2.pointPolygonTest(lane_region, vehicle.center, False) >= 0:
                            in_lane = True
                            break
                    
                    if in_intersection or in_lane:
                        accident = Accident("STOPPED", [vehicle], vehicle.center)
                        accidents.append(accident)
        
        return accidents
    
    def _detect_collisions(self, vehicles: List[Vehicle]) -> List[Accident]:
        """Detect vehicle collisions based on proximity"""
        accidents = []
        
        # Check all pairs of vehicles
        for i, vehicle1 in enumerate(vehicles):
            if vehicle1.id in self.checked_vehicles:
                continue
            
            for vehicle2 in vehicles[i+1:]:
                if vehicle2.id in self.checked_vehicles:
                    continue
                
                # Calculate distance between vehicles
                distance = np.linalg.norm(
                    np.array(vehicle1.center) - np.array(vehicle2.center)
                )
                
                # Check if too close
                if distance < config.COLLISION_DISTANCE_THRESHOLD:
                    # Both vehicles should be stopped or moving slowly
                    speed1 = vehicle1.get_speed()
                    speed2 = vehicle2.get_speed()
                    
                    if speed1 < 2 and speed2 < 2:
                        # Calculate collision point
                        collision_point = (
                            (vehicle1.center[0] + vehicle2.center[0]) // 2,
                            (vehicle1.center[1] + vehicle2.center[1]) // 2
                        )
                        
                        accident = Accident("COLLISION", [vehicle1, vehicle2], collision_point)
                        accidents.append(accident)
        
        return accidents
    
    def get_confirmed_accidents(self) -> List[Accident]:
        """Get only confirmed accidents"""
        return [acc for acc in self.accidents.values() if acc.confirmed]
    
    def get_accidents_in_lane(self, lane_key: str) -> List[Accident]:
        """Get accidents in specific lane"""
        return [acc for acc in self.accidents.values() if acc.lane == lane_key]
    
    def has_accident_in_lane(self, lane_key: str) -> bool:
        """Check if lane has any accidents"""
        return any(acc.lane == lane_key and acc.confirmed for acc in self.accidents.values())
    
    def draw_accidents(self, frame: np.ndarray) -> np.ndarray:
        """Draw accident markers on frame"""
        for accident in self.accidents.values():
            if not accident.confirmed:
                continue
            
            x, y = accident.location
            
            # Draw accident marker
            color = config.COLOR_ACCIDENT
            cv2.drawMarker(frame, (x, y), color, cv2.MARKER_CROSS, 30, 3)
            
            # Draw circle
            cv2.circle(frame, (x, y), 40, color, 2)
            
            # Draw accident info
            label = f"ACCIDENT #{accident.id} - {accident.type}"
            cv2.putText(frame, label, (x - 80, y - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            duration = accident.get_duration()
            time_label = f"Duration: {duration:.1f}s"
            cv2.putText(frame, time_label, (x - 80, y - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw alert box
            cv2.rectangle(frame, (x - 100, y - 70), (x + 100, y + 70), color, -1)
            cv2.putText(frame, "EMERGENCY", (x - 70, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return frame
    
    def reset(self):
        """Reset detector and clear all accidents"""
        self.accidents.clear()
        self.checked_vehicles.clear()
        Accident._next_id = 1
