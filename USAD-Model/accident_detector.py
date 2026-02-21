"""Accident detection (stopped vehicles + collisions)."""

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
        
        self.type = accident_type
        self.vehicles = vehicles
        self.location = location
        self.detected_time = time.time()
        self.last_seen_time = self.detected_time
        self.confidence_frames = 0
        self.confirmed = False
        self.confirmed_time: Optional[float] = None
        self.notified = False
        self.missed_frames = 0

        # Confirmation behavior
        if self.type == "COLLISION":
            self._confirm_frames_required = int(getattr(config, "COLLISION_CONFIDENCE_FRAMES", 3))
        else:
            self._confirm_frames_required = int(getattr(config, "ACCIDENT_CONFIDENCE_FRAMES", 30))
        
        # Affected lane
        self.lane = self._determine_lane()
        
    def _determine_lane(self) -> Optional[str]:
        """Determine which lane the accident is in"""
        for lane_key, lane_data in config.LANES.items():
            lane_region = np.array(lane_data["region"], dtype=np.int32)
            result = cv2.pointPolygonTest(lane_region, self.location, False)
            if result >= 0:
                return lane_key
        
        intersection_region = np.array(config.INTERSECTION_CENTER, dtype=np.int32)
        result = cv2.pointPolygonTest(intersection_region, self.location, False)
        if result >= 0:
            return "INTERSECTION"
        
        return None
    
    def update(self, *, seen: bool):
        """Update accident state.

        Confidence should only build when the underlying condition is still observed.
        This prevents one-frame false positives from becoming "confirmed" later.
        """
        if seen:
            self.last_seen_time = time.time()
            self.missed_frames = 0
            self.confidence_frames += 1
            if (not self.confirmed) and (self.confidence_frames >= self._confirm_frames_required):
                self.confirmed = True
                self.confirmed_time = self.last_seen_time
        else:
            self.missed_frames += 1
    
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
        # Map an "accident signature" (type + vehicle IDs) to a stable Accident ID.
        # Used to accumulate confidence only while the condition persists.
        self._accident_key_to_id: Dict[Tuple[str, Tuple[int, ...]], int] = {}

        # Vehicle IDs currently involved in CONFIRMED accidents (suppresses duplicates)
        self.checked_vehicles: set = set()
        self.stopped_vehicle_last_seen: Dict[int, float] = {}

        # Pair-wise collision hysteresis (prevents flicker/downgrade when detection jitters)
        self._sticky_collision_until: Dict[Tuple[int, int], float] = {}

        # When a collision is no longer observed, remove it after a short clear delay.
        self._collision_resolved_since: Dict[Tuple[int, int], float] = {}

        # Time-based confirmation (wait 1–2s before classifying)
        self._vehicle_first_seen: Dict[int, float] = {}
        self._collision_contact_start: Dict[Tuple[int, int], float] = {}
        self._queue_close_start: Dict[Tuple[int, int], float] = {}

    def _accident_key(self, accident_type: str, vehicles: List[Vehicle]) -> Tuple[str, Tuple[int, ...]]:
        ids = tuple(sorted(int(v.id) for v in vehicles))
        return (str(accident_type), ids)
        
    def detect_accidents(self, vehicles: List[Vehicle], frame: Optional[np.ndarray] = None) -> List[Accident]:
        """
        Detect accidents from vehicle list
        
        Args:
            vehicles: List of tracked vehicles
            
        Returns:
            List of active accidents
        """
        # Detect stopped vehicles + collisions for this frame (evidence)
        frame_evidence: List[Accident] = []
        frame_evidence.extend(self._detect_stopped_vehicles(vehicles))
        frame_evidence.extend(self._detect_collisions(vehicles, frame=frame))

        now = time.time()
        current_vehicle_ids = set(int(v.id) for v in vehicles)
        vehicles_by_id: Dict[int, Vehicle] = {int(v.id): v for v in vehicles}

        def _bbox_gap_distance_px(b1, b2) -> float:
            x1, y1, w1, h1 = b1
            x2, y2, w2, h2 = b2
            left1, top1, right1, bottom1 = x1, y1, x1 + w1, y1 + h1
            left2, top2, right2, bottom2 = x2, y2, x2 + w2, y2 + h2

            dx = 0
            if right1 < left2:
                dx = left2 - right1
            elif right2 < left1:
                dx = left1 - right2

            dy = 0
            if bottom1 < top2:
                dy = top2 - bottom1
            elif bottom2 < top1:
                dy = top1 - bottom2

            return float(np.hypot(dx, dy))

        def _collision_threshold_px() -> float:
            """Pixel threshold for considering cars 'close enough' for collision logic.

            If mm-based calibration is configured, we still keep at least the legacy
            pixel threshold as a safety net. In practice, bbox jitter and segmentation
            can create a 5–20px gap even when cars are touching.
            """
            legacy = float(getattr(config, "COLLISION_DISTANCE_THRESHOLD", 30))
            mm = float(getattr(config, "COLLISION_DISTANCE_MM", 0.0) or 0.0)
            if mm > 0:
                ppm = float(getattr(config, "PIXELS_PER_MM", 0.0) or 0.0)
                if ppm > 0:
                    return max(legacy, mm * ppm)
            return legacy

        collision_clear_seconds = float(getattr(config, "COLLISION_CLEAR_SECONDS", 5.0))
        threshold_px = _collision_threshold_px()
        queue_proximity_px = float(getattr(config, "QUEUE_PROXIMITY_PX", 45.0))
        clear_gap_px = max(threshold_px, queue_proximity_px) + 15.0

        # Track first-seen times for gating
        for v in vehicles:
            self._vehicle_first_seen.setdefault(int(v.id), now)

        # Update/create tracked accidents based on evidence keys
        seen_keys: set = set()
        for evidence in frame_evidence:
            key = self._accident_key(evidence.type, evidence.vehicles)
            seen_keys.add(key)

            existing_id = self._accident_key_to_id.get(key)
            if existing_id is not None and existing_id in self.accidents:
                acc = self.accidents[existing_id]
                # Refresh dynamic fields (location might shift slightly with tracking)
                acc.location = evidence.location
                acc.vehicles = evidence.vehicles
                acc.update(seen=True)
            else:
                # Start a new tracked accident candidate
                evidence.last_seen_time = now
                evidence.confidence_frames = 1

                # If the configured confirmation threshold is 1 (common for collisions),
                # confirm immediately so the UI can alert on the same frame.
                if (not evidence.confirmed) and (evidence.confidence_frames >= int(getattr(evidence, "_confirm_frames_required", 1))):
                    evidence.confirmed = True
                    evidence.confirmed_time = now
                self.accidents[evidence.id] = evidence
                self._accident_key_to_id[key] = evidence.id

        # Age out accidents that are not seen this frame
        remove_if_missed = int(getattr(config, "ACCIDENT_REMOVE_AFTER_MISSED_FRAMES", 10))
        remove_if_missed_confirmed = int(getattr(config, "CONFIRMED_ACCIDENT_REMOVE_AFTER_MISSED_FRAMES", 30))
        remove_if_missed_collision = int(getattr(config, "COLLISION_REMOVE_AFTER_MISSED_FRAMES", 120))

        to_remove: List[int] = []
        for accident_id, accident in self.accidents.items():
            key = self._accident_key(accident.type, accident.vehicles)
            if key in seen_keys:
                # Reset "resolved" timers when collision evidence is present again.
                if accident.type == "COLLISION" and len(accident.vehicles) >= 2:
                    pair = (min(int(accident.vehicles[0].id), int(accident.vehicles[1].id)),
                            max(int(accident.vehicles[0].id), int(accident.vehicles[1].id)))
                    self._collision_resolved_since.pop(pair, None)
                continue

            # If collision cars separate or disappear, remove the collision alert after a short delay.
            if accident.type == "COLLISION" and len(accident.vehicles) >= 2:
                v1_id = int(accident.vehicles[0].id)
                v2_id = int(accident.vehicles[1].id)
                pair = (min(v1_id, v2_id), max(v1_id, v2_id))

                v1 = vehicles_by_id.get(v1_id)
                v2 = vehicles_by_id.get(v2_id)
                resolved = False

                # If one/both vehicles are no longer detected, consider collision resolved.
                if v1 is None or v2 is None:
                    resolved = True
                else:
                    gap_now = _bbox_gap_distance_px(v1.bbox, v2.bbox)
                    if gap_now > clear_gap_px:
                        resolved = True

                if resolved:
                    start = float(self._collision_resolved_since.get(pair, 0.0) or 0.0)
                    if start <= 0.0:
                        self._collision_resolved_since[pair] = now
                    elif (now - start) >= max(0.0, collision_clear_seconds):
                        to_remove.append(accident_id)
                        continue
                else:
                    self._collision_resolved_since.pop(pair, None)

            # Hold confirmed collision alerts on-screen briefly even if evidence disappears.
            if (
                accident.type == "COLLISION"
                and accident.confirmed
                and (now - float(accident.last_seen_time)) < float(getattr(config, "COLLISION_ALERT_HOLD_SECONDS", 3.0))
                and all(int(v.id) in current_vehicle_ids for v in accident.vehicles)
            ):
                continue

            accident.update(seen=False)

            if accident.type == "COLLISION":
                limit = remove_if_missed_collision
            else:
                limit = remove_if_missed_confirmed if accident.confirmed else remove_if_missed
            if accident.missed_frames >= limit:
                to_remove.append(accident_id)

        # Clean up stopped/queued vehicle markers
        ttl = float(getattr(config, "STOPPED_QUEUE_TTL_SECONDS", 2.0))
        to_delete = [vid for vid, ts in self.stopped_vehicle_last_seen.items() if (now - ts) > ttl]
        for vid in to_delete:
            del self.stopped_vehicle_last_seen[vid]

        # Clean up gating maps (avoid unbounded growth)
        delay = float(getattr(config, "ACCIDENT_DETECTION_DELAY_SECONDS", 1.5))
        alive_ids = set(int(v.id) for v in vehicles)
        self._vehicle_first_seen = {vid: ts for vid, ts in self._vehicle_first_seen.items() if vid in alive_ids or (now - ts) < max(5.0, delay * 2)}

        # Mark vehicles involved in confirmed accidents (suppresses duplicates)
        self.checked_vehicles.clear()
        for accident in self.accidents.values():
            if not accident.confirmed:
                continue
            for v in accident.vehicles:
                self.checked_vehicles.add(v.id)

        # Remove accidents flagged for removal and clean up key map
        if to_remove:
            reverse_key = {v: k for k, v in self._accident_key_to_id.items()}
            for accident_id in to_remove:
                acc = self.accidents.get(accident_id)
                if acc is None:
                    continue
                key = reverse_key.get(accident_id) or self._accident_key(acc.type, acc.vehicles)
                self._accident_key_to_id.pop(key, None)
                del self.accidents[accident_id]
        
        return list(self.accidents.values())

    def get_stopped_vehicle_ids(self) -> List[int]:
        """Vehicles currently marked as stopped/queued (not an accident)."""
        return list(self.stopped_vehicle_last_seen.keys())

    def draw_stopped_vehicles(self, frame: np.ndarray, vehicles: List[Vehicle]) -> np.ndarray:
        """Draw a small STOPPED marker on vehicles classified as queued/stopped."""
        stopped_ids = set(self.get_stopped_vehicle_ids())
        if not stopped_ids:
            return frame

        for vehicle in vehicles:
            if vehicle.id not in stopped_ids:
                continue
            x, y, w, h = map(int, vehicle.bbox)
            cv2.putText(
                frame,
                "STOPPED",
                (x, max(15, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )
        return frame
    
    def _detect_stopped_vehicles(self, vehicles: List[Vehicle]) -> List[Accident]:
        """Detect vehicles that have been stopped too long"""
        accidents = []
        
        for vehicle in vehicles:
            if vehicle.id in self.checked_vehicles:
                continue
            
            if vehicle.is_stopped:
                stopped_duration = vehicle.get_stopped_duration()
                
                if stopped_duration >= config.STOPPED_TIME_THRESHOLD:
                    intersection_region = np.array(config.INTERSECTION_CENTER, dtype=np.int32)
                    in_intersection = cv2.pointPolygonTest(
                        intersection_region, vehicle.center, False
                    ) >= 0
                    
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
    
    def _detect_collisions(self, vehicles: List[Vehicle], frame: Optional[np.ndarray] = None) -> List[Accident]:
        """Detect vehicle collisions based on proximity"""
        accidents = []

        def _overlap_over_min_area(b1, b2) -> float:
            x1, y1, w1, h1 = b1
            x2, y2, w2, h2 = b2
            if w1 <= 0 or h1 <= 0 or w2 <= 0 or h2 <= 0:
                return 0.0
            ax1, ay1, ax2, ay2 = x1, y1, x1 + w1, y1 + h1
            bx1, by1, bx2, by2 = x2, y2, x2 + w2, y2 + h2
            ix1, iy1 = max(ax1, bx1), max(ay1, by1)
            ix2, iy2 = min(ax2, bx2), min(ay2, by2)
            iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
            inter = float(iw * ih)
            if inter <= 0:
                return 0.0
            a1 = float(w1 * h1)
            a2 = float(w2 * h2)
            return inter / max(1.0, min(a1, a2))

        def _car_color_mask(bgr: np.ndarray) -> Optional[np.ndarray]:
            ranges = getattr(config, "CAR_COLOR_RANGES", None)
            if not ranges:
                return None
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            combined = None
            for (lower_hsv, upper_hsv) in ranges:
                lower = np.array(lower_hsv, dtype=np.uint8)
                upper = np.array(upper_hsv, dtype=np.uint8)
                m = cv2.inRange(hsv, lower, upper)
                combined = m if combined is None else cv2.bitwise_or(combined, m)
            if combined is None:
                return None
            # IMPORTANT: avoid MORPH_OPEN here (can create fake 1–3px gaps).
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
            return combined

        def _object_gap_distance_px(frame_img: np.ndarray, b1, b2) -> Optional[float]:
            """Compute min pixel gap between car-colored pixels inside two bboxes.

            Returns:
                0 if masks touch/overlap, >0 if there is a gap, None if unavailable.
            """
            if frame_img is None:
                return None
            if not bool(getattr(config, "ENABLE_COLOR_FILTERING", True)):
                return None

            x1, y1, w1, h1 = b1
            x2, y2, w2, h2 = b2
            if w1 <= 0 or h1 <= 0 or w2 <= 0 or h2 <= 0:
                return None

            # Union ROI with a small margin.
            margin = 6
            ux1 = int(max(0, min(x1, x2) - margin))
            uy1 = int(max(0, min(y1, y2) - margin))
            ux2 = int(min(frame_img.shape[1] - 1, max(x1 + w1, x2 + w2) + margin))
            uy2 = int(min(frame_img.shape[0] - 1, max(y1 + h1, y2 + h2) + margin))
            if ux2 <= ux1 or uy2 <= uy1:
                return None

            roi = frame_img[uy1:uy2, ux1:ux2]
            mask = _car_color_mask(roi)
            if mask is None or mask.size == 0:
                return None

            # Build per-vehicle masks restricted to each bbox region inside the union ROI.
            v1 = np.zeros_like(mask)
            v2 = np.zeros_like(mask)

            rx1, ry1 = int(x1 - ux1), int(y1 - uy1)
            rx2, ry2 = int(x1 + w1 - ux1), int(y1 + h1 - uy1)
            sx1, sy1 = int(x2 - ux1), int(y2 - uy1)
            sx2, sy2 = int(x2 + w2 - ux1), int(y2 + h2 - uy1)

            rx1, ry1 = max(0, rx1), max(0, ry1)
            rx2, ry2 = min(mask.shape[1], rx2), min(mask.shape[0], ry2)
            sx1, sy1 = max(0, sx1), max(0, sy1)
            sx2, sy2 = min(mask.shape[1], sx2), min(mask.shape[0], sy2)

            if rx2 <= rx1 or ry2 <= ry1 or sx2 <= sx1 or sy2 <= sy1:
                return None

            v1[ry1:ry2, rx1:rx2] = mask[ry1:ry2, rx1:rx2]
            v2[sy1:sy2, sx1:sx2] = mask[sy1:sy2, sx1:sx2]

            if cv2.countNonZero(v1) == 0 or cv2.countNonZero(v2) == 0:
                return None

            # Tolerate 1–2px segmentation holes at bumpers/edges.
            dilate_px = int(getattr(config, "COLLISION_MASK_DILATE_PX", 1) or 0)
            if dilate_px > 0:
                k = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE,
                    (2 * dilate_px + 1, 2 * dilate_px + 1),
                )
                v1 = cv2.dilate(v1, k)
                v2 = cv2.dilate(v2, k)

            # Distance from v1 pixels to nearest v2 pixel.
            inv2 = cv2.bitwise_not(v2)
            dist_to_v2 = cv2.distanceTransform(inv2, cv2.DIST_L2, 3)
            min_d1 = float(dist_to_v2[v1 > 0].min())

            inv1 = cv2.bitwise_not(v1)
            dist_to_v1 = cv2.distanceTransform(inv1, cv2.DIST_L2, 3)
            min_d2 = float(dist_to_v1[v2 > 0].min())

            return float(min(min_d1, min_d2))

        def bbox_gap_distance_px(b1, b2) -> float:
            # b = (x, y, w, h)
            x1, y1, w1, h1 = b1
            x2, y2, w2, h2 = b2
            left1, top1, right1, bottom1 = x1, y1, x1 + w1, y1 + h1
            left2, top2, right2, bottom2 = x2, y2, x2 + w2, y2 + h2

            dx = 0
            if right1 < left2:
                dx = left2 - right1
            elif right2 < left1:
                dx = left1 - right2

            dy = 0
            if bottom1 < top2:
                dy = top2 - bottom1
            elif bottom2 < top1:
                dy = top1 - bottom2

            return float(np.hypot(dx, dy))

        def gap_roi_between_bboxes(b1, b2):
            x1, y1, w1, h1 = b1
            x2, y2, w2, h2 = b2
            left1, top1, right1, bottom1 = x1, y1, x1 + w1, y1 + h1
            left2, top2, right2, bottom2 = x2, y2, x2 + w2, y2 + h2

            # Horizontal gap with vertical overlap
            overlap_top = max(top1, top2)
            overlap_bottom = min(bottom1, bottom2)
            overlap_left = max(left1, left2)
            overlap_right = min(right1, right2)

            if overlap_bottom > overlap_top:
                # There is vertical overlap; gap is horizontal if boxes are separated in X
                if right1 < left2:
                    return (right1, overlap_top, left2, overlap_bottom)
                if right2 < left1:
                    return (right2, overlap_top, left1, overlap_bottom)

            if overlap_right > overlap_left:
                # There is horizontal overlap; gap is vertical if boxes are separated in Y
                if bottom1 < top2:
                    return (overlap_left, bottom1, overlap_right, top2)
                if bottom2 < top1:
                    return (overlap_left, bottom2, overlap_right, top1)

            # Diagonal separation: use rectangle between nearest corners
            # Choose x-range between nearest vertical edges
            if right1 < left2:
                gx1, gx2 = right1, left2
            elif right2 < left1:
                gx1, gx2 = right2, left1
            else:
                gx1, gx2 = overlap_left, overlap_right

            # Choose y-range between nearest horizontal edges
            if bottom1 < top2:
                gy1, gy2 = bottom1, top2
            elif bottom2 < top1:
                gy1, gy2 = bottom2, top1
            else:
                gy1, gy2 = overlap_top, overlap_bottom

            return (gx1, gy1, gx2, gy2)

        def gap_is_mostly_black(frame_img: np.ndarray, roi) -> bool:
            if frame_img is None:
                return False
            x1, y1, x2, y2 = roi
            x1 = int(max(0, min(x1, x2)))
            x2 = int(max(0, max(x1, x2)))
            y1 = int(max(0, min(y1, y2)))
            y2 = int(max(0, max(y1, y2)))
            if x2 - x1 < 3 or y2 - y1 < 3:
                return False

            roi_img = frame_img[y1:y2, x1:x2]
            if roi_img.size == 0:
                return False

            gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
            black_max = int(getattr(config, "BLACK_GAP_GRAY_MAX", 70))
            black_ratio = float(np.mean(gray <= black_max))
            min_ratio = float(getattr(config, "BLACK_GAP_MIN_RATIO", 0.65))
            return black_ratio >= min_ratio

        def collision_threshold_px() -> float:
            # Prefer mm-based rule if configured
            legacy = float(getattr(config, "COLLISION_DISTANCE_THRESHOLD", 30))
            mm = float(getattr(config, "COLLISION_DISTANCE_MM", 0.0) or 0.0)
            if mm > 0:
                ppm = float(getattr(config, "PIXELS_PER_MM", 0.0) or 0.0)
                if ppm > 0:
                    return max(legacy, mm * ppm)
            return legacy

        # Strict touch rules (1–2px) for fast collision detection.
        object_touch_px = float(getattr(config, "COLLISION_OBJECT_GAP_PX", 1.0))
        bbox_touch_px = float(getattr(config, "COLLISION_BBOX_TOUCH_PX", 2.0))
        release_extra = float(getattr(config, "COLLISION_RELEASE_EXTRA_PX", 2.0))
        queue_proximity_px = float(getattr(config, "QUEUE_PROXIMITY_PX", 45.0))
        sticky_seconds = float(getattr(config, "STICKY_COLLISION_SECONDS", 2.0))
        delay = float(getattr(config, "ACCIDENT_DETECTION_DELAY_SECONDS", 1.5))
        min_contact_seconds = float(getattr(config, "COLLISION_MIN_CONTACT_SECONDS", 1.0))
        min_queue_seconds = float(getattr(config, "QUEUE_MIN_SECONDS", 1.0))

        threshold_px = collision_threshold_px()
        require_motion = bool(getattr(config, "REQUIRE_MOTION_FOR_COLLISION", True))
        min_motion_speed = float(getattr(config, "COLLISION_MIN_SPEED_PX_PER_SEC", 8.0))
        
        for i, vehicle1 in enumerate(vehicles):
            if vehicle1.id in self.checked_vehicles:
                continue
            
            for vehicle2 in vehicles[i+1:]:
                if vehicle2.id in self.checked_vehicles:
                    continue
                
                # Distance between vehicle bodies (bounding boxes)
                gap_px = bbox_gap_distance_px(vehicle1.bbox, vehicle2.bbox)

                # Suppress "collisions" between two detections that heavily overlap.
                # This typically means the same physical car was detected twice.
                overlap_max = float(getattr(config, "COLLISION_DUPLICATE_OVERLAP_MAX", 0.65))
                overlap_ratio = _overlap_over_min_area(vehicle1.bbox, vehicle2.bbox)
                if overlap_ratio >= overlap_max:
                    # High overlap can be a duplicate track OR a real collision after impact.
                    # Only suppress as duplicate when the two tracks are extremely co-located
                    # AND at least one of them is "new" / likely spawned by a split/jitter.
                    c1 = np.array(vehicle1.center, dtype=np.float32)
                    c2 = np.array(vehicle2.center, dtype=np.float32)
                    center_dist = float(np.linalg.norm(c1 - c2))
                    w1, h1 = vehicle1.bbox[2], vehicle1.bbox[3]
                    w2, h2 = vehicle2.bbox[2], vehicle2.bbox[3]
                    scale = float(max(1, min(w1, h1, w2, h2)))

                    now_ts = time.time()
                    delay = float(getattr(config, "ACCIDENT_DETECTION_DELAY_SECONDS", 1.5))
                    v1_seen = float(self._vehicle_first_seen.get(int(vehicle1.id), now_ts))
                    v2_seen = float(self._vehicle_first_seen.get(int(vehicle2.id), now_ts))
                    v1_is_new = (now_ts - v1_seen) < max(0.1, delay)
                    v2_is_new = (now_ts - v2_seen) < max(0.1, delay)
                    v1_unconfirmed = not bool(getattr(vehicle1, "confirmed", False))
                    v2_unconfirmed = not bool(getattr(vehicle2, "confirmed", False))

                    if center_dist <= (0.25 * scale) and (v1_is_new or v2_is_new or v1_unconfirmed or v2_unconfirmed):
                        continue

                pair_key = (min(vehicle1.id, vehicle2.id), max(vehicle1.id, vehicle2.id))
                now_ts = time.time()
                sticky_until = float(self._sticky_collision_until.get(pair_key, 0.0) or 0.0)

                # Use object-to-object gap (color mask) when available.
                obj_gap = None
                used_object_gap = False
                if frame is not None:
                    obj_gap = _object_gap_distance_px(frame, vehicle1.bbox, vehicle2.bbox)
                    used_object_gap = obj_gap is not None

                # HARD contact: trigger immediately for 1–2px gap, without waiting for
                # the global stabilization delay.
                hard_contact = False
                if obj_gap is not None:
                    hard_contact = obj_gap <= object_touch_px
                else:
                    # When object gap can't be computed, only accept near-touching bboxes.
                    hard_contact = gap_px <= bbox_touch_px

                # Global stabilization delay: wait before classifying anything for new tracks
                v1_seen = float(self._vehicle_first_seen.get(int(vehicle1.id), now_ts))
                v2_seen = float(self._vehicle_first_seen.get(int(vehicle2.id), now_ts))
                if (not hard_contact) and ((now_ts - v1_seen) < delay or (now_ts - v2_seen) < delay):
                    continue

                # Quick reject: not close enough to be either queued or collided.
                if gap_px > max(queue_proximity_px, threshold_px) + 10:
                    continue

                # If we can measure object gap: collision only if objects touch.
                if obj_gap is not None:
                    # If we recently had a collision, allow a little extra gap to
                    # avoid jitter flipping COLLISION -> STOPPED.
                    effective_touch = object_touch_px
                    if sticky_until > now_ts:
                        effective_touch = object_touch_px + max(0.0, release_extra)

                    if obj_gap <= effective_touch:
                        # Contact candidate: require it to persist for N seconds
                        start = float(self._collision_contact_start.get(pair_key, 0.0) or 0.0)
                        if start <= 0.0:
                            self._collision_contact_start[pair_key] = now_ts
                            # If contact persistence is enabled, wait until it elapses.
                            if min_contact_seconds > 0:
                                continue
                        if min_contact_seconds > 0 and (now_ts - start) < min_contact_seconds:
                            continue
                    else:
                        # Reset contact timer when not touching
                        self._collision_contact_start.pop(pair_key, None)

                        # There is visible space between objects -> STOPPED/queued only.
                        if gap_px <= queue_proximity_px:
                            qstart = float(self._queue_close_start.get(pair_key, 0.0) or 0.0)
                            if qstart <= 0.0:
                                self._queue_close_start[pair_key] = now_ts
                            if (now_ts - float(self._queue_close_start.get(pair_key, now_ts))) >= min_queue_seconds:
                                self.stopped_vehicle_last_seen[vehicle1.id] = now_ts
                                self.stopped_vehicle_last_seen[vehicle2.id] = now_ts
                        continue
                else:
                    # If we cannot measure object gap, clear timers so we don't latch incorrectly
                    self._collision_contact_start.pop(pair_key, None)
                    self._queue_close_start.pop(pair_key, None)

                    if sticky_until > now_ts:
                        # If we cannot measure object gap for a moment, keep collision alive briefly
                        # as long as bboxes are still essentially touching.
                        if gap_px > (bbox_touch_px + max(0.0, release_extra)):
                            continue

                    # Without object gap, only accept near-touching bboxes as collision.
                    if gap_px > bbox_touch_px:
                        continue

                # NOTE: bbox-only fallback is intentionally strict (<= ~1–2px).

                # Check if within collision gap threshold
                # - If we have object gap, it already enforced "touch" above.
                # - If we don't, we already enforced bbox_touch_px above.
                if (obj_gap is not None and gap_px <= max(queue_proximity_px, threshold_px)) or (obj_gap is None and gap_px <= bbox_touch_px):
                    # If both objects are essentially stationary, do not call it a collision.
                    # This suppresses false positives from static blobs (lane markings, shadows) and
                    # bumper-to-bumper queued cars.
                    if require_motion:
                        speed1 = float(vehicle1.get_speed())
                        speed2 = float(vehicle2.get_speed())
                        if max(speed1, speed2) < min_motion_speed:
                            now = time.time()
                            self.stopped_vehicle_last_seen[vehicle1.id] = now
                            self.stopped_vehicle_last_seen[vehicle2.id] = now
                            continue

                    # If there is a visible black background gap between them, treat as stopped/queued cars
                    # (not a collision accident).
                    if (
                        (not used_object_gap)
                        and frame is not None
                        and bool(getattr(config, "ENABLE_GAP_BACKGROUND_CHECK", True))
                    ):
                        roi = gap_roi_between_bboxes(vehicle1.bbox, vehicle2.bbox)
                        if gap_is_mostly_black(frame, roi):
                            now = time.time()
                            self.stopped_vehicle_last_seen[vehicle1.id] = now
                            self.stopped_vehicle_last_seen[vehicle2.id] = now
                            continue

                    if bool(getattr(config, "REQUIRE_LOW_SPEED_FOR_COLLISION", False)):
                        speed1 = vehicle1.get_speed()
                        speed2 = vehicle2.get_speed()
                        max_speed = float(getattr(config, "COLLISION_MAX_SPEED_PX_PER_SEC", 2.0))
                        if speed1 > max_speed or speed2 > max_speed:
                            continue

                    collision_point = (
                        (vehicle1.center[0] + vehicle2.center[0]) // 2,
                        (vehicle1.center[1] + vehicle2.center[1]) // 2,
                    )

                    # Refresh sticky collision window to prevent flicker.
                    self._sticky_collision_until[pair_key] = now_ts + max(0.0, sticky_seconds)

                    # Clear queue timer on collision
                    self._queue_close_start.pop(pair_key, None)

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
        # Large alert banner if any confirmed collision exists (after short on-screen delay)
        now = time.time()
        delay = float(getattr(config, "COLLISION_SCREEN_ALERT_DELAY_SECONDS", 0.0) or 0.0)
        delay = max(0.0, delay)
        confirmed_collisions = []
        for acc in self.accidents.values():
            if not (acc.confirmed and acc.type == "COLLISION"):
                continue
            t0 = float(acc.confirmed_time or acc.detected_time)
            if (now - t0) >= delay:
                confirmed_collisions.append(acc)
        if confirmed_collisions:
            height, width = frame.shape[:2]
            overlay = frame.copy()
            banner_h = 110
            cv2.rectangle(overlay, (0, 0), (width, banner_h), (0, 0, 255), -1)
            frame = cv2.addWeighted(overlay, 0.85, frame, 0.15, 0)

            latest = max(confirmed_collisions, key=lambda a: a.detected_time)
            ids = ", ".join(str(v.id) for v in latest.vehicles)
            cv2.putText(
                frame,
                "COLLISION DETECTED",
                (20, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.4,
                (255, 255, 255),
                3,
            )
            cv2.putText(
                frame,
                f"Vehicles: {ids}",
                (20, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2,
            )

        for accident in self.accidents.values():
            if not accident.confirmed:
                continue
            if accident.type == "COLLISION":
                t0 = float(accident.confirmed_time or accident.detected_time)
                if (now - t0) < delay:
                    continue
            
            x, y = map(int, accident.location)
            
            color = config.COLOR_ACCIDENT
            cv2.drawMarker(frame, (x, y), color, cv2.MARKER_CROSS, 30, 3)
            
            cv2.circle(frame, (x, y), 40, color, 2)
            
            label = f"ACCIDENT #{accident.id} - {accident.type}"
            cv2.putText(frame, label, (x - 80, y - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            duration = accident.get_duration()
            time_label = f"Duration: {duration:.1f}s"
            cv2.putText(frame, time_label, (x - 80, y - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            cv2.rectangle(frame, (x - 90, y - 55), (x + 90, y + 55), color, 2)
            cv2.putText(frame, "ALERT", (x - 35, y + 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        
        return frame
    
    def reset(self):
        """Reset detector and clear all accidents"""
        self.accidents.clear()
        self.checked_vehicles.clear()
        Accident._next_id = 1
