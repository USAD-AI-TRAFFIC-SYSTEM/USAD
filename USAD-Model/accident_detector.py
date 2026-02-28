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
        # Intersection should take precedence: when an accident occurs in the
        # middle zone, record it under the intersection/center status.
        try:
            intersection_region = np.array(config.INTERSECTION_CENTER, dtype=np.int32)
            result = cv2.pointPolygonTest(intersection_region, self.location, False)
            if result >= 0:
                return "INTERSECTION"
        except Exception:
            pass

        for lane_key, lane_data in config.LANES.items():
            lane_region = np.array(lane_data["region"], dtype=np.int32)
            result = cv2.pointPolygonTest(lane_region, self.location, False)
            if result >= 0:
                return lane_key
        
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

        # For multi-vehicle collisions, track resolution by the full involved ID set.
        self._collision_resolved_since_group: Dict[Tuple[int, ...], float] = {}

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
        missing_grace_seconds = float(getattr(config, "COLLISION_MISSING_VEHICLE_GRACE_SECONDS", 2.0) or 2.0)
        missing_grace_seconds = max(0.0, min(10.0, missing_grace_seconds))
        threshold_px = _collision_threshold_px()
        queue_proximity_px = float(getattr(config, "QUEUE_PROXIMITY_PX", 45.0))
        clear_gap_px = max(threshold_px, queue_proximity_px) + 15.0

        # Track first-seen times for gating
        for v in vehicles:
            self._vehicle_first_seen.setdefault(int(v.id), now)

        def _dedupe_keys_for_accident(accident_id: int, keep_key: Tuple[str, Tuple[int, ...]]):
            # Remove stale signature keys that point at the same accident.
            for k, v in list(self._accident_key_to_id.items()):
                if v == accident_id and k != keep_key:
                    self._accident_key_to_id.pop(k, None)

        # Update/create tracked accidents based on evidence.
        # Use accident IDs for "seen this frame" to avoid resetting/aging when collision
        # vehicle sets fluctuate (e.g., 2 -> 3 vehicles) while it is still the same event.
        seen_accident_ids: set[int] = set()

        for evidence in frame_evidence:
            # COLLISION: attempt to match to an existing collision accident by overlap,
            # so duration/ID remains stable even when the involved vehicle set changes.
            if evidence.type == "COLLISION":
                evidence_ids = {int(v.id) for v in evidence.vehicles}

                best_id: Optional[int] = None
                best_inter = 0
                best_dist = 1e9
                best_recent = 1e9
                max_match_dist = float(getattr(config, "COLLISION_MATCH_MAX_DISTANCE_PX", 90.0) or 90.0)
                # If tracking IDs churn during a real collision, vehicle-id intersection can be 0.
                # Allow a proximity match to a very recent collision accident to keep a stable
                # accident ID (prevents duplicate notifications) WITHOUT changing collision rules.
                max_match_age = float(getattr(config, "COLLISION_MATCH_MAX_AGE_SECONDS", 4.0) or 4.0)
                max_match_age = max(0.0, min(10.0, max_match_age))

                ex, ey = map(float, evidence.location)
                for accident_id, accident in self.accidents.items():
                    if accident.type != "COLLISION":
                        continue
                    acc_ids = {int(v.id) for v in accident.vehicles}
                    inter = len(evidence_ids.intersection(acc_ids))
                    recent = float(now - float(getattr(accident, "last_seen_time", 0.0) or 0.0))
                    if inter <= 0 and (max_match_age <= 0 or recent > max_match_age):
                        continue
                    ax, ay = map(float, accident.location)
                    dist = float(np.hypot(ax - ex, ay - ey))
                    if dist > max_match_dist:
                        continue

                    # Prefer higher overlap, then closer distance.
                    if (
                        inter > best_inter
                        or (inter == best_inter and dist < best_dist)
                        or (inter == best_inter and abs(dist - best_dist) < 1e-6 and recent < best_recent)
                    ):
                        best_id = int(accident_id)
                        best_inter = int(inter)
                        best_dist = float(dist)
                        best_recent = float(recent)

                if best_id is not None and best_id in self.accidents:
                    acc = self.accidents[best_id]
                    if best_inter <= 0:
                        # Fallback match (ID churn): replace the involved set with the
                        # current evidence set to avoid unbounded union growth.
                        acc.vehicles = list(evidence.vehicles)
                    else:
                        acc_ids = {int(v.id) for v in acc.vehicles}
                        union_ids = sorted(acc_ids.union(evidence_ids))
                        # Preserve vehicle identity across brief detection dropouts:
                        # if a vehicle isn't present in the current `vehicles` list (because
                        # it wasn't detected this frame), keep the prior Vehicle object so
                        # collision matching/notification does not churn.
                        updated: List[Vehicle] = []
                        for vid in union_ids:
                            vcur = vehicles_by_id.get(int(vid))
                            if vcur is not None:
                                updated.append(vcur)
                                continue
                            for vold in acc.vehicles:
                                try:
                                    if int(getattr(vold, "id", 0) or 0) == int(vid):
                                        updated.append(vold)
                                        break
                                except Exception:
                                    continue
                        acc.vehicles = updated
                    acc.location = evidence.location
                    acc.update(seen=True)
                    seen_accident_ids.add(best_id)

                    key = self._accident_key(acc.type, acc.vehicles)
                    self._accident_key_to_id[key] = best_id
                    _dedupe_keys_for_accident(best_id, key)
                    continue

            # Default: match by exact signature key
            key = self._accident_key(evidence.type, evidence.vehicles)
            existing_id = self._accident_key_to_id.get(key)
            if existing_id is not None and existing_id in self.accidents:
                acc = self.accidents[existing_id]
                # Refresh dynamic fields (location might shift slightly with tracking)
                acc.location = evidence.location
                acc.vehicles = evidence.vehicles
                acc.update(seen=True)
                seen_accident_ids.add(int(existing_id))
                _dedupe_keys_for_accident(int(existing_id), key)
            else:
                # Start a new tracked accident candidate
                evidence.last_seen_time = now
                evidence.confidence_frames = 1

                # If the configured confirmation threshold is 1 (common for collisions),
                # confirm immediately so the UI can alert on the same frame.
                if (not evidence.confirmed) and (
                    evidence.confidence_frames >= int(getattr(evidence, "_confirm_frames_required", 1))
                ):
                    evidence.confirmed = True
                    evidence.confirmed_time = now
                self.accidents[evidence.id] = evidence
                self._accident_key_to_id[key] = evidence.id
                seen_accident_ids.add(int(evidence.id))
                _dedupe_keys_for_accident(int(evidence.id), key)

        # Consolidate collision accidents so multi-vehicle pileups are represented
        # as a single collision event.
        #
        # Goals:
        # - Never have two active COLLISION accidents that share the same vehicle.
        # - When contact edges flicker, merge nearby/recent collision groups so a
        #   3+ vehicle pileup doesn't split into multiple accidents.
        def _merge_collision_accidents(keep_id: int, drop_id: int):
            if keep_id == drop_id:
                return
            keep = self.accidents.get(keep_id)
            drop = self.accidents.get(drop_id)
            if keep is None or drop is None:
                return
            if keep.type != "COLLISION" or drop.type != "COLLISION":
                return

            keep_ids = {int(getattr(v, "id", 0) or 0) for v in keep.vehicles}
            drop_ids = {int(getattr(v, "id", 0) or 0) for v in drop.vehicles}
            union_ids = sorted({vid for vid in (keep_ids.union(drop_ids)) if vid > 0})
            merged_vehicles: List[Vehicle] = []
            for vid in union_ids:
                vcur = vehicles_by_id.get(int(vid))
                if vcur is not None:
                    merged_vehicles.append(vcur)
                    continue
                # Prefer to retain an existing Vehicle object when missing from current frame.
                found = None
                for src in (keep.vehicles, drop.vehicles):
                    for v in src:
                        try:
                            if int(getattr(v, "id", 0) or 0) == int(vid):
                                found = v
                                break
                        except Exception:
                            continue
                    if found is not None:
                        break
                if found is not None:
                    merged_vehicles.append(found)
            keep.vehicles = merged_vehicles

            # Merge/refresh metadata.
            try:
                kx, ky = map(float, keep.location)
                dx, dy = map(float, drop.location)
                keep.location = (int(round((kx + dx) / 2.0)), int(round((ky + dy) / 2.0)))
            except Exception:
                pass
            keep.last_seen_time = max(float(getattr(keep, "last_seen_time", 0.0) or 0.0), float(getattr(drop, "last_seen_time", 0.0) or 0.0))
            keep.detected_time = min(float(getattr(keep, "detected_time", now) or now), float(getattr(drop, "detected_time", now) or now))
            keep.confidence_frames = max(int(getattr(keep, "confidence_frames", 0) or 0), int(getattr(drop, "confidence_frames", 0) or 0))
            keep.missed_frames = min(int(getattr(keep, "missed_frames", 0) or 0), int(getattr(drop, "missed_frames", 0) or 0))
            keep.notified = bool(getattr(keep, "notified", False) or getattr(drop, "notified", False))
            if bool(getattr(drop, "confirmed", False)):
                keep.confirmed = True
                t_keep = float(getattr(keep, "confirmed_time", 0.0) or 0.0)
                t_drop = float(getattr(drop, "confirmed_time", 0.0) or 0.0)
                if t_keep <= 0.0:
                    keep.confirmed_time = t_drop if t_drop > 0.0 else keep.last_seen_time
                elif t_drop > 0.0:
                    keep.confirmed_time = min(t_keep, t_drop)

            # Remove key mappings that point to the dropped accident.
            for k, v in list(self._accident_key_to_id.items()):
                if int(v) == int(drop_id):
                    self._accident_key_to_id.pop(k, None)

            # Remove the dropped accident.
            self.accidents.pop(drop_id, None)
            seen_accident_ids.discard(int(drop_id))

        # Pass 1: merge any collisions that share vehicles.
        collision_ids = [int(aid) for aid, a in self.accidents.items() if getattr(a, "type", None) == "COLLISION"]
        owner_by_vehicle: Dict[int, int] = {}
        for aid in sorted(collision_ids):
            acc = self.accidents.get(aid)
            if acc is None:
                continue
            vids = [int(getattr(v, "id", 0) or 0) for v in getattr(acc, "vehicles", [])]
            for vid in vids:
                if vid <= 0:
                    continue
                owner = owner_by_vehicle.get(vid)
                if owner is None or owner == aid:
                    owner_by_vehicle[vid] = aid
                    continue
                # Merge into the earlier/older accident id (stable).
                keep_id = int(min(owner, aid))
                drop_id = int(max(owner, aid))
                _merge_collision_accidents(keep_id, drop_id)
                # Ensure ownership maps to the keeper.
                owner_by_vehicle[vid] = keep_id

        # Pass 2: merge nearby + recent collision groups (handles edge flicker in 3+ pileups).
        merge_dist = float(getattr(config, "COLLISION_MATCH_MAX_DISTANCE_PX", 90.0) or 90.0)
        merge_age = float(getattr(config, "COLLISION_MATCH_MAX_AGE_SECONDS", 4.0) or 4.0)
        merge_dist = max(10.0, min(300.0, merge_dist))
        merge_age = max(0.0, min(10.0, merge_age))

        changed = True
        while changed:
            changed = False
            collisions = [
                (int(aid), self.accidents.get(int(aid)))
                for aid in list(self.accidents.keys())
                if self.accidents.get(int(aid)) is not None and self.accidents.get(int(aid)).type == "COLLISION"
            ]
            collisions = [(aid, acc) for aid, acc in collisions if acc is not None]
            collisions.sort(key=lambda t: float(getattr(t[1], "detected_time", now) or now))
            for i in range(len(collisions)):
                a_id, a = collisions[i]
                if a_id not in self.accidents:
                    continue
                a_recent = float(now - float(getattr(a, "last_seen_time", 0.0) or 0.0))
                if merge_age > 0 and a_recent > merge_age:
                    continue
                ax, ay = map(float, getattr(a, "location", (0, 0)))
                a_vids = {int(getattr(v, "id", 0) or 0) for v in getattr(a, "vehicles", [])}
                for j in range(i + 1, len(collisions)):
                    b_id, b = collisions[j]
                    if b_id not in self.accidents:
                        continue
                    b_recent = float(now - float(getattr(b, "last_seen_time", 0.0) or 0.0))
                    if merge_age > 0 and b_recent > merge_age:
                        continue
                    bx, by = map(float, getattr(b, "location", (0, 0)))
                    dist = float(np.hypot(ax - bx, ay - by))
                    if dist > merge_dist:
                        continue

                    b_vids = {int(getattr(v, "id", 0) or 0) for v in getattr(b, "vehicles", [])}
                    a_lane = getattr(a, "lane", None)
                    b_lane = getattr(b, "lane", None)

                    def _lane_compatible(l1, l2):
                        if not l1 or not l2:
                            return True
                        if l1 == l2:
                            return True
                        if l1 == "INTERSECTION" or l2 == "INTERSECTION":
                            return True
                        return False

                    # Merge if overlapping vehicles (should already be handled) OR very close in space/time
                    # and lane-compatible (avoid merging unrelated collisions from different lanes).
                    if a_vids.intersection(b_vids) or (dist <= (merge_dist * 0.75) and _lane_compatible(a_lane, b_lane)):
                        keep_id = int(min(a_id, b_id))
                        drop_id = int(max(a_id, b_id))
                        _merge_collision_accidents(keep_id, drop_id)
                        changed = True
                        break
                if changed:
                    break

        # Refresh the signature map for collisions after consolidation.
        for aid, acc in list(self.accidents.items()):
            if acc.type != "COLLISION":
                continue
            key = self._accident_key(acc.type, acc.vehicles)
            self._accident_key_to_id[key] = int(aid)
            _dedupe_keys_for_accident(int(aid), key)

        # Age out accidents that are not seen this frame
        remove_if_missed = int(getattr(config, "ACCIDENT_REMOVE_AFTER_MISSED_FRAMES", 10))
        remove_if_missed_confirmed = int(getattr(config, "CONFIRMED_ACCIDENT_REMOVE_AFTER_MISSED_FRAMES", 30))
        remove_if_missed_collision = int(getattr(config, "COLLISION_REMOVE_AFTER_MISSED_FRAMES", 120))

        to_remove: List[int] = []
        for accident_id, accident in self.accidents.items():
            if int(accident_id) in seen_accident_ids:
                # Reset "resolved" timers when collision evidence is present again.
                if accident.type == "COLLISION" and len(accident.vehicles) >= 2:
                    ids = tuple(sorted(int(v.id) for v in accident.vehicles))
                    if len(ids) == 2:
                        self._collision_resolved_since.pop((ids[0], ids[1]), None)
                    self._collision_resolved_since_group.pop(ids, None)
                continue

            # If collision cars separate or disappear, remove the collision alert after a short delay.
            if accident.type == "COLLISION" and len(accident.vehicles) >= 2:
                ids = tuple(sorted(int(v.id) for v in accident.vehicles))

                resolved = False
                resolved_due_to_missing = False
                live = []
                for vid in ids:
                    v = vehicles_by_id.get(int(vid))
                    if v is None:
                        resolved = True
                        resolved_due_to_missing = True
                        break
                    live.append(v)

                if not resolved:
                    # Consider resolved if ALL vehicles have separated beyond the clear gap.
                    min_gap = None
                    for i in range(len(live)):
                        for j in range(i + 1, len(live)):
                            g = _bbox_gap_distance_px(live[i].bbox, live[j].bbox)
                            min_gap = g if min_gap is None else min(min_gap, g)
                    if min_gap is not None and float(min_gap) > float(clear_gap_px):
                        resolved = True

                if resolved:
                    start = float(self._collision_resolved_since_group.get(ids, 0.0) or 0.0)
                    if start <= 0.0:
                        self._collision_resolved_since_group[ids] = now
                    else:
                        clear_seconds_eff = float(collision_clear_seconds)
                        if resolved_due_to_missing:
                            clear_seconds_eff = max(clear_seconds_eff, float(missing_grace_seconds))
                        if (now - start) >= max(0.0, clear_seconds_eff):
                            to_remove.append(accident_id)
                            continue
                else:
                    self._collision_resolved_since_group.pop(ids, None)

            # Hold confirmed collision alerts on-screen briefly even if evidence disappears.
            hold_seconds = float(getattr(config, "COLLISION_ALERT_HOLD_SECONDS", 3.0) or 0.0)
            if hold_seconds > 0 and (
                accident.type == "COLLISION"
                and accident.confirmed
                and (now - float(accident.last_seen_time)) < hold_seconds
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

        # Stability hint for tracking: when a collision is active, segmentation/masks can
        # jitter a lot (touching cars, merged blobs). Mark involved vehicles so bbox
        # stabilization can be boosted on subsequent frames.
        stability_seconds = float(getattr(config, "COLLISION_BBOX_STABILIZE_SECONDS", 1.75) or 0.0)
        if stability_seconds > 0:
            until = now + stability_seconds
            for accident in self.accidents.values():
                if not (accident.confirmed and accident.type == "COLLISION"):
                    continue
                for v in accident.vehicles:
                    vid = int(getattr(v, "id", 0) or 0)
                    if vid <= 0:
                        continue
                    vv = vehicles_by_id.get(vid)
                    if vv is None:
                        continue
                    try:
                        setattr(vv, "_collision_stability_until", float(until))
                    except Exception:
                        pass

        # Remove accidents flagged for removal and clean up key map
        if to_remove:
            for accident_id in to_remove:
                acc = self.accidents.get(accident_id)
                if acc is None:
                    continue
                for k, v in list(self._accident_key_to_id.items()):
                    if v == accident_id:
                        self._accident_key_to_id.pop(k, None)
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
        
        # Collect collision edges (vehicle id pairs) then group into connected components.
        collision_edges: List[Tuple[int, int]] = []

        for i, vehicle1 in enumerate(vehicles):
            # NOTE: do not suppress vehicles already in confirmed accidents here.
            # Collisions can expand (2 cars -> 3 cars) and we want the group to update.
            
            for vehicle2 in vehicles[i+1:]:
                
                
                # Distance between vehicle bodies (bounding boxes)
                gap_px = bbox_gap_distance_px(vehicle1.bbox, vehicle2.bbox)

                # Suppress "collisions" between two detections that heavily overlap.
                # This typically means the same physical car was detected twice (e.g.,
                # blob-splitting spawned two tracks from one car).
                # We suppress whenever the two tracks are co-located regardless of
                # whether they are new/confirmed — two confirmed tracks from the same
                # physical object must not register as a collision.
                overlap_max = float(getattr(config, "COLLISION_DUPLICATE_OVERLAP_MAX", 0.40))
                overlap_ratio = _overlap_over_min_area(vehicle1.bbox, vehicle2.bbox)
                if overlap_ratio >= overlap_max:
                    c1 = np.array(vehicle1.center, dtype=np.float32)
                    c2 = np.array(vehicle2.center, dtype=np.float32)
                    center_dist = float(np.linalg.norm(c1 - c2))
                    w1, h1 = vehicle1.bbox[2], vehicle1.bbox[3]
                    w2, h2 = vehicle2.bbox[2], vehicle2.bbox[3]
                    scale = float(max(1, min(w1, h1, w2, h2)))

                    # Two tracks whose centres are within 35 % of the smaller bbox
                    # dimension are almost certainly the same physical object.
                    if center_dist <= (0.35 * scale):
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

                    collision_edges.append((int(vehicle1.id), int(vehicle2.id)))

        if not collision_edges:
            return accidents

        # Group collisions so 3+ vehicle pileups are represented as one accident.
        vehicles_by_id = {int(v.id): v for v in vehicles}
        adj: Dict[int, set] = {}
        for a, b in collision_edges:
            adj.setdefault(a, set()).add(b)
            adj.setdefault(b, set()).add(a)

        seen = set()
        for vid in adj.keys():
            if vid in seen:
                continue
            stack = [vid]
            comp = set()
            while stack:
                cur = stack.pop()
                if cur in seen:
                    continue
                seen.add(cur)
                comp.add(cur)
                for nxt in adj.get(cur, set()):
                    if nxt not in seen:
                        stack.append(nxt)

            if len(comp) < 2:
                continue
            involved = [vehicles_by_id[c] for c in sorted(comp) if c in vehicles_by_id]
            if len(involved) < 2:
                continue
            cx = int(round(float(sum(v.center[0] for v in involved)) / float(len(involved))))
            cy = int(round(float(sum(v.center[1] for v in involved)) / float(len(involved))))
            accidents.append(Accident("COLLISION", involved, (cx, cy)))
        
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
            # Subtle pulse to make the alert feel "alive" without changing accident timing.
            pulse_speed = float(getattr(config, "COLLISION_BANNER_PULSE_HZ", 1.2) or 1.2)
            pulse_speed = max(0.0, pulse_speed)
            if pulse_speed > 0:
                a = 0.80 + 0.15 * (0.5 + 0.5 * float(np.sin(now * (2.0 * np.pi * pulse_speed))))
                a = float(max(0.65, min(0.95, a)))
            else:
                a = 0.85
            frame = cv2.addWeighted(overlay, a, frame, 1.0 - a, 0)
            cv2.rectangle(frame, (0, 0), (width - 1, banner_h - 1), (255, 255, 255), 2)

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
        self._accident_key_to_id.clear()
        self._collision_resolved_since.clear()
        self._collision_resolved_since_group.clear()
