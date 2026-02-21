"""Vehicle detection and tracking."""

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

        self._smooth_center = np.array(center, dtype=np.float32)
        self._smooth_bbox = np.array(bbox, dtype=np.float32)
        
        self.positions = [center]
        self.timestamps = [time.time()]
        self.lost_frames = 0

        self.seen_frames = 1
        self.confirmed = False
        
        self.current_lane = None
        self.crossed_stop_line = False
        
        self.vehicle_type = self._classify_vehicle(area)
        
        self.license_plate = None
        self.license_plate_confidence = 0.0
        
        self.is_stopped = False
        self.stopped_time = 0
        self.last_moved_time = time.time()
        
        self.violations = []

        # How many consecutive frames this track has been outside the intersection ROI.
        # Used to prevent ID churn when objects hover near ROI boundaries.
        self.roi_outside_frames = 0
        
    def _classify_vehicle(self, area: float) -> str:
        """Classify vehicle by size"""
        for vtype, (min_area, max_area) in config.VEHICLE_TYPES.items():
            if min_area <= area <= max_area:
                return vtype
        return "UNKNOWN"
    
    def update(self, center: Tuple[int, int], bbox: Tuple[int, int, int, int], area: float):
        """Update vehicle position"""
        # Smoothing helps with jitter, but large alpha values can make the bbox lag,
        # especially when two cars get close and segmentation changes quickly.
        base_alpha = float(getattr(config, "TRACK_SMOOTHING_ALPHA", 0.0) or 0.0)
        base_alpha = max(0.0, min(0.95, base_alpha))

        c = np.array(center, dtype=np.float32)
        b = np.array(bbox, dtype=np.float32)

        # Dynamic alpha: be more responsive on large moves.
        try:
            delta = float(np.linalg.norm(c - self._smooth_center))
        except Exception:
            delta = 0.0
        dynamic_px = float(getattr(config, "TRACK_DYNAMIC_ALPHA_DISTANCE_PX", 12.0) or 12.0)
        fast_alpha_cap = float(getattr(config, "TRACK_FAST_ALPHA_CAP", 0.35) or 0.35)
        fast_alpha_cap = max(0.0, min(0.95, fast_alpha_cap))

        alpha = base_alpha
        if delta >= dynamic_px:
            alpha = min(alpha, fast_alpha_cap)

        if alpha > 0:
            self._smooth_center = alpha * self._smooth_center + (1 - alpha) * c
            self._smooth_bbox = alpha * self._smooth_bbox + (1 - alpha) * b
        else:
            self._smooth_center = c
            self._smooth_bbox = b

        self.center = (int(self._smooth_center[0]), int(self._smooth_center[1]))
        sb = self._smooth_bbox
        new_bbox = (int(sb[0]), int(sb[1]), int(sb[2]), int(sb[3]))
        
        # Cap bbox movement per frame to dampen jitter from segmentation noise.
        max_shift = float(getattr(config, "TRACK_BBOX_MAX_SHIFT_PER_FRAME", 10))
        if max_shift > 0 and len(getattr(self, "positions", [])) > 0:
            old_bbox = self.bbox
            dx = new_bbox[0] - old_bbox[0]
            dy = new_bbox[1] - old_bbox[1]
            
            # Clamp movement magnitude to max_shift pixels
            if abs(dx) > max_shift:
                dx = max_shift if dx > 0 else -max_shift
            if abs(dy) > max_shift:
                dy = max_shift if dy > 0 else -max_shift
            
            new_bbox = (old_bbox[0] + dx, old_bbox[1] + dy, new_bbox[2], new_bbox[3])
        
        self.bbox = new_bbox
        self.area = area

        self.vehicle_type = self._classify_vehicle(area)
        
        self.positions.append(center)
        self.timestamps.append(time.time())
        
        # Keep only recent history
        if len(self.positions) > 30:
            self.positions.pop(0)
            self.timestamps.pop(0)
        
        self.lost_frames = 0

        # This frame is an observed update (not a prediction).
        setattr(self, "_predicted_only", False)

        self.seen_frames += 1
        confirm_frames = int(getattr(config, "MIN_TRACK_CONFIRM_FRAMES", 3))
        if self.seen_frames >= max(1, confirm_frames):
            self.confirmed = True
        
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

    def apply_prediction_update(self, now_ts: float):
        """Update center/bbox using a short-horizon prediction.

        This keeps bboxes responsive when detection briefly fails (e.g., cars touch/merge).
        It does NOT append to the motion history so speed/stop logic stays conservative.
        """
        max_dt = float(getattr(config, "TRACK_PREDICT_MAX_DT", 0.35) or 0.35)
        max_shift = int(getattr(config, "TRACK_PREDICT_MAX_SHIFT_PX", 35) or 35)
        max_dt = max(0.05, min(2.0, max_dt))
        max_shift = max(1, min(200, max_shift))

        pred = self.predict_center(now_ts, max_dt=max_dt)
        dx = int(pred[0] - self.center[0])
        dy = int(pred[1] - self.center[1])

        # Clamp to prevent runaway drift.
        if dx > max_shift:
            dx = max_shift
        elif dx < -max_shift:
            dx = -max_shift
        if dy > max_shift:
            dy = max_shift
        elif dy < -max_shift:
            dy = -max_shift

        if dx == 0 and dy == 0:
            return

        x, y, w, h = self.bbox
        self.center = (int(self.center[0] + dx), int(self.center[1] + dy))
        self.bbox = (int(x + dx), int(y + dy), int(w), int(h))

        # Keep smoothing state aligned so the next observed update doesn't "snap".
        try:
            self._smooth_center = np.array(self.center, dtype=np.float32)
            self._smooth_bbox = np.array(self.bbox, dtype=np.float32)
        except Exception:
            pass

        setattr(self, "_predicted_only", True)
    
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

    def get_velocity(self) -> Tuple[float, float]:
        """Estimate velocity (vx, vy) in pixels/second using recent history."""
        if len(self.positions) < 2 or len(self.timestamps) < 2:
            return (0.0, 0.0)

        # Use last up to 5 samples for stability.
        positions = self.positions[-5:]
        timestamps = self.timestamps[-5:]
        if len(positions) < 2:
            return (0.0, 0.0)

        dt = float(timestamps[-1] - timestamps[0])
        if dt <= 0:
            return (0.0, 0.0)

        dx = float(positions[-1][0] - positions[0][0])
        dy = float(positions[-1][1] - positions[0][1])
        return (dx / dt, dy / dt)

    def predict_center(self, now_ts: float, max_dt: float = 1.0) -> Tuple[int, int]:
        """Predict center position forward in time (helps keep the same ID on missed frames)."""
        if not self.timestamps:
            return self.center

        last_ts = float(self.timestamps[-1])
        dt = float(now_ts - last_ts)
        if dt <= 0:
            return self.center

        dt = min(dt, float(max_dt))
        vx, vy = self.get_velocity()
        px = float(self.center[0]) + vx * dt
        py = float(self.center[1]) + vy * dt
        return (int(round(px)), int(round(py)))
    
    def get_direction(self) -> Tuple[float, float]:
        """Get movement direction vector"""
        if len(self.positions) < 2:
            return (0, 0)
        
        start = np.array(self.positions[0])
        end = np.array(self.positions[-1])
        direction = end - start
        
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
    """Detects and tracks vehicles."""
    
    def __init__(self):
        self.bg_subtractor = self._create_bg_subtractor()

        open_k = tuple(getattr(config, "MOTION_MASK_OPEN_KERNEL", (3, 3)))
        close_k = tuple(getattr(config, "MOTION_MASK_CLOSE_KERNEL", (7, 7)))
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, open_k)
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, close_k)
        
        self.vehicles: Dict[int, Vehicle] = {}
        
        self.frame_count = 0
        self.background_ready = False
        self.learning_frames = 60  # ~2 seconds at 30fps

        self.use_color_segmentation = bool(
            getattr(config, "ENABLE_COLOR_FILTERING", False)
            and getattr(config, "USE_COLOR_SEGMENTATION", True)
        )

        self._last_color_mask: Optional[np.ndarray] = None

        self.had_detections_this_frame: bool = False
        self.had_confirmed_updates_this_frame: bool = False

        self._intersection_roi_mask: Optional[np.ndarray] = None
        self._intersection_roi_shape: Optional[Tuple[int, int]] = None

    def _create_bg_subtractor(self):
        history = int(getattr(config, "BACKGROUND_HISTORY", 120) or 120)
        var_threshold = int(getattr(config, "BACKGROUND_THRESHOLD", 30) or 30)
        detect_shadows = bool(getattr(config, "DETECT_SHADOWS", False))
        return cv2.createBackgroundSubtractorMOG2(
            history=max(1, history),
            varThreshold=max(1, var_threshold),
            detectShadows=detect_shadows,
        )

    def _reset_background_learning(self, verbose: bool = False):
        # Reinitialize the background model and warmup counters.
        self.bg_subtractor = self._create_bg_subtractor()
        self.frame_count = 0
        self.background_ready = False

        # Clear cached masks tied to previous frames/background.
        self._last_color_mask = None
        self._intersection_roi_mask = None
        self._intersection_roi_shape = None

        # Re-read segmentation toggle (config may have hot-reloaded).
        self.use_color_segmentation = bool(
            getattr(config, "ENABLE_COLOR_FILTERING", False)
            and getattr(config, "USE_COLOR_SEGMENTATION", True)
        )

        if verbose:
            print("[VehicleDetector] Background learning reset (rebuilding model)")

    def _get_intersection_roi_mask(self, frame: np.ndarray) -> Optional[np.ndarray]:
        if not bool(getattr(config, "REQUIRE_INTERSECTION_ROI_FOR_DETECTION", False)):
            return None

        h, w = frame.shape[:2]
        shape = (h, w)
        if self._intersection_roi_mask is not None and self._intersection_roi_shape == shape:
            return self._intersection_roi_mask

        mask = np.zeros((h, w), dtype=np.uint8)

        polys = []
        try:
            for lane_data in getattr(config, "LANES", {}).values():
                polys.append(np.array(lane_data["region"], dtype=np.int32))
        except Exception:
            polys = []

        if hasattr(config, "INTERSECTION_CENTER"):
            try:
                polys.append(np.array(config.INTERSECTION_CENTER, dtype=np.int32))
            except Exception:
                pass

        for p in polys:
            if p.size != 0:
                cv2.fillPoly(mask, [p], 255)

        dilate_px = int(getattr(config, "INTERSECTION_ROI_DILATE_PX", 0) or 0)
        if dilate_px > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * dilate_px + 1, 2 * dilate_px + 1))
            mask = cv2.dilate(mask, k)

        self._intersection_roi_mask = mask
        self._intersection_roi_shape = shape
        return mask

    @staticmethod
    def _any_point_in_mask(mask: np.ndarray, pts: List[Tuple[int, int]]) -> bool:
        if mask is None or mask.size == 0:
            return True
        h, w = mask.shape[:2]
        for (x, y) in pts:
            xi = int(x)
            yi = int(y)
            if 0 <= xi < w and 0 <= yi < h and mask[yi, xi] != 0:
                return True
        return False

    def _split_contour_if_needed(self, mask: np.ndarray, contour: np.ndarray) -> List[np.ndarray]:
        """Try to split a merged blob into multiple contours.

        Used for bumper-to-bumper toy cars that get connected by segmentation/morphology.
        Works for both color-mask and motion-mask paths.
        """
        if not bool(getattr(config, "ENABLE_BLOB_SPLITTING", True)):
            return [contour]

        try:
            contour_area = float(cv2.contourArea(contour))
        except Exception:
            return [contour]

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return [contour]

        # Only attempt split for clearly elongated or large shapes (common when two cars merge).
        aspect = max(w, h) / float(max(1, min(w, h)))
        max_aspect = float(getattr(config, "MAX_VEHICLE_ASPECT_RATIO", 2.8))
        min_area = float(getattr(config, "BLOB_SPLIT_MIN_AREA", 0.0) or 0.0)
        should_split = (aspect > max_aspect * 1.05) or (min_area > 0 and contour_area >= min_area)
        if not should_split:
            return [contour]

        roi = mask[y : y + h, x : x + w]
        if roi.size == 0:
            return [contour]

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        # Prefer watershed splitting (more reliable for touching objects) before erosion fallback.
        try:
            do_ws = bool(getattr(config, "BLOB_SPLIT_USE_WATERSHED", True))
        except Exception:
            do_ws = True

        min_piece_area = float(getattr(config, "MIN_VEHICLE_AREA", 0)) * 0.6

        if do_ws:
            try:
                roi_bin = (roi > 0).astype(np.uint8) * 255
                dist = cv2.distanceTransform(roi_bin, cv2.DIST_L2, 5)
                dmax = float(dist.max()) if dist is not None else 0.0
                if dmax > 0:
                    fg_ratio = float(getattr(config, "BLOB_SPLIT_DT_THRESH_RATIO", 0.5) or 0.5)
                    fg_ratio = max(0.2, min(0.9, fg_ratio))
                    _, sure_fg = cv2.threshold(dist, fg_ratio * dmax, 255, 0)
                    sure_fg = sure_fg.astype(np.uint8)
                    sure_bg = cv2.dilate(roi_bin, kernel, iterations=2)
                    unknown = cv2.subtract(sure_bg, sure_fg)

                    cc_count, markers = cv2.connectedComponents(sure_fg)
                    if cc_count >= 3:  # background + at least 2 objects
                        markers = markers + 1
                        markers[unknown == 255] = 0
                        ws_img = cv2.cvtColor(roi_bin, cv2.COLOR_GRAY2BGR)
                        markers = cv2.watershed(ws_img, markers)

                        mapped: List[np.ndarray] = []
                        for label in range(2, int(markers.max()) + 1):
                            region = (markers == label).astype(np.uint8) * 255
                            if cv2.countNonZero(region) == 0:
                                continue
                            sub_contours, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            for sc in sub_contours:
                                a = float(cv2.contourArea(sc))
                                if a < min_piece_area:
                                    continue
                                sc2 = sc.copy()
                                sc2[:, 0, 0] += x
                                sc2[:, 0, 1] += y
                                mapped.append(sc2)

                        if len(mapped) >= 2:
                            return mapped
            except Exception:
                pass

        max_erosions = int(getattr(config, "BLOB_SPLIT_MAX_EROSIONS", 4))
        work = roi.copy()

        for _ in range(max_erosions):
            work = cv2.erode(work, kernel, iterations=1)
            if cv2.countNonZero(work) == 0:
                break

            sub_contours, _ = cv2.findContours(work, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            good: List[np.ndarray] = []
            for sc in sub_contours:
                a = float(cv2.contourArea(sc))
                if a < min_piece_area:
                    continue
                good.append(sc)

            if len(good) >= 2:
                mapped: List[np.ndarray] = []
                for sc in good:
                    sc2 = sc.copy()
                    sc2[:, 0, 0] += x
                    sc2[:, 0, 1] += y
                    mapped.append(sc2)
                return mapped

        return [contour]
        
    def detect_vehicles(self, frame: np.ndarray) -> List[Vehicle]:
        def _bbox_iou(b1: Tuple[int, int, int, int], b2: Tuple[int, int, int, int]) -> float:
            x1, y1, w1, h1 = b1
            x2, y2, w2, h2 = b2
            ax1, ay1, ax2, ay2 = x1, y1, x1 + w1, y1 + h1
            bx1, by1, bx2, by2 = x2, y2, x2 + w2, y2 + h2
            ix1, iy1 = max(ax1, bx1), max(ay1, by1)
            ix2, iy2 = min(ax2, bx2), min(ay2, by2)
            iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
            inter = float(iw * ih)
            if inter <= 0:
                return 0.0
            union = float(w1 * h1 + w2 * h2) - inter
            return inter / union if union > 0 else 0.0

        if self.use_color_segmentation:
            color_dets = self._detect_by_color(frame)

            need_reacquire = any(
                getattr(v, "confirmed", False) and int(getattr(v, "lost_frames", 0)) > 0
                for v in self.vehicles.values()
            )
            want_motion = need_reacquire or (len(color_dets) == 0)

            motion_dets: List[dict] = []
            if want_motion:
                motion_dets = self._detect_by_motion(frame)
                for d in motion_dets:
                    d["source"] = "motion"
                    d.setdefault("mask_ratio", 1.0)

            for d in color_dets:
                d["source"] = "color"

            if motion_dets:
                filtered_motion = []
                for md in motion_dets:
                    if not any(_bbox_iou(md["bbox"], cd["bbox"]) >= 0.25 for cd in color_dets):
                        filtered_motion.append(md)
                detections = color_dets + filtered_motion
            else:
                detections = color_dets
        else:
            detections = self._detect_by_motion(frame)

        self.had_detections_this_frame = bool(detections)
        self.had_confirmed_updates_this_frame = False
        
        for v in self.vehicles.values():
            v.lost_frames += 1

        # Update tracked vehicles
        self._update_tracking(detections)

        # If a confirmed track wasn't matched this frame (common when cars touch/merge),
        # keep its bbox moving using prediction for a short window. This prevents
        # "frozen" bboxes and avoids premature removal.
        now_ts = time.time()
        predict_for_lost = int(getattr(config, "TRACK_PREDICT_FOR_LOST_FRAMES", 12) or 12)
        for v in self.vehicles.values():
            if not getattr(v, "confirmed", False):
                continue
            lf = int(getattr(v, "lost_frames", 0))
            if lf <= 0 or lf > predict_for_lost:
                continue
            v.apply_prediction_update(now_ts)

        intersection_roi_mask = self._get_intersection_roi_mask(frame)
        if intersection_roi_mask is not None:
            candidate_ttl = int(getattr(config, "CANDIDATE_VEHICLE_LOST_FRAMES", 8))
            confirmed_ttl = int(getattr(config, "VEHICLE_LOST_FRAMES", 45))
            outside_ttl = int(getattr(config, "ROI_OUTSIDE_REMOVE_FRAMES", 30))

            to_remove = []
            for vehicle_id, vehicle in self.vehicles.items():
                x, y, w, h = vehicle.bbox
                pts = [
                    vehicle.center,
                    (x + 2, y + 2),
                    (x + w - 2, y + 2),
                    (x + 2, y + h - 2),
                    (x + w - 2, y + h - 2),
                    (x + w // 2, y + h - 2),
                ]

                inside = self._any_point_in_mask(intersection_roi_mask, pts)
                if inside:
                    vehicle.roi_outside_frames = 0
                else:
                    vehicle.roi_outside_frames = int(getattr(vehicle, "roi_outside_frames", 0)) + 1

                # Don't instantly drop tracks that drift across the ROI edge;
                # require them to be outside for a short grace period.
                if vehicle.roi_outside_frames > outside_ttl:
                    to_remove.append(vehicle_id)
                    continue

                ttl = confirmed_ttl if getattr(vehicle, "confirmed", False) else candidate_ttl
                if vehicle.lost_frames > ttl:
                    to_remove.append(vehicle_id)

            for vehicle_id in to_remove:
                self.vehicles.pop(vehicle_id, None)

            return [v for v in self.vehicles.values() if getattr(v, "confirmed", False)]

        for v in self.vehicles.values():
            if getattr(v, "confirmed", False) and int(getattr(v, "lost_frames", 0)) == 0:
                self.had_confirmed_updates_this_frame = True
                break

        if self.use_color_segmentation and self._last_color_mask is not None:
            presence_min = float(getattr(config, "TRACK_PRESENCE_MIN_RATIO", 0.0) or 0.0)
            if presence_min > 0:
                h_img, w_img = self._last_color_mask.shape[:2]
                for v in self.vehicles.values():
                    if not getattr(v, "confirmed", False):
                        continue
                    if v.lost_frames <= 0:
                        continue
                    x, y, w, h = v.bbox
                    if w <= 0 or h <= 0:
                        continue
                    x0 = max(0, min(w_img - 1, x))
                    y0 = max(0, min(h_img - 1, y))
                    x1 = max(0, min(w_img, x + w))
                    y1 = max(0, min(h_img, y + h))
                    if x1 <= x0 or y1 <= y0:
                        continue
                    roi = self._last_color_mask[y0:y1, x0:x1]
                    ratio = float(cv2.countNonZero(roi)) / float((x1 - x0) * (y1 - y0))
                    if ratio >= presence_min:
                        v.lost_frames = 0

        candidate_ttl = int(getattr(config, "CANDIDATE_VEHICLE_LOST_FRAMES", 8))
        confirmed_ttl = int(getattr(config, "VEHICLE_LOST_FRAMES", 45))
        to_remove = []
        for vehicle_id, vehicle in self.vehicles.items():
            ttl = confirmed_ttl if getattr(vehicle, "confirmed", False) else candidate_ttl
            if vehicle.lost_frames > ttl:
                to_remove.append(vehicle_id)
        
        for vehicle_id in to_remove:
            del self.vehicles[vehicle_id]
        
        return [v for v in self.vehicles.values() if getattr(v, "confirmed", False)]

    def _passes_shape_filters(self, contour: np.ndarray, w: int, h: int, contour_area: float) -> bool:
        """Heuristics to suppress non-car blobs (lane markings, glare, edges)."""
        if w <= 0 or h <= 0:
            return False

        # Base area guard (kept here too so filters remain safe if callers change).
        try:
            min_area = float(getattr(config, "MIN_VEHICLE_AREA", 0) or 0)
            max_area = float(getattr(config, "MAX_VEHICLE_AREA", 0) or 0)
        except Exception:
            min_area, max_area = 0.0, 0.0
        if min_area > 0 and contour_area < min_area:
            return False
        if max_area > 0 and contour_area > max_area:
            return False

        min_w = int(getattr(config, "MIN_VEHICLE_BBOX_WIDTH", 18))
        min_h = int(getattr(config, "MIN_VEHICLE_BBOX_HEIGHT", 18))
        if w < min_w or h < min_h:
            return False

        aspect = max(w, h) / float(min(w, h))
        max_aspect = float(getattr(config, "MAX_VEHICLE_ASPECT_RATIO", 2.8))
        if aspect > max_aspect:
            return False

        # Rotated-rect aspect ratio catches slanted stripe fragments better than axis-aligned.
        try:
            rect = cv2.minAreaRect(contour)
            rw, rh = rect[1]
            if rw > 0 and rh > 0:
                rot_aspect = max(rw, rh) / float(min(rw, rh))
                if rot_aspect > max_aspect:
                    return False
        except Exception:
            # If OpenCV fails for a degenerate contour, let other filters decide.
            pass

        extent = float(contour_area) / float(w * h)
        min_extent = float(getattr(config, "MIN_VEHICLE_EXTENT", 0.25))
        max_extent = float(getattr(config, "MAX_VEHICLE_EXTENT", 0.95))
        if extent < min_extent or extent > max_extent:
            return False

        # Solidity: area / convex hull area.
        min_solidity = float(getattr(config, "MIN_VEHICLE_SOLIDITY", 0.0) or 0.0)
        if min_solidity > 0:
            hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(hull))
            if hull_area > 0:
                solidity = float(contour_area) / hull_area
                if solidity < min_solidity:
                    return False

        # Circularity helps reject skinny/fragmented blobs (markings, glare edges).
        min_circ = float(getattr(config, "MIN_VEHICLE_CIRCULARITY", 0.0) or 0.0)
        if min_circ > 0:
            try:
                perim = float(cv2.arcLength(contour, True))
            except Exception:
                perim = 0.0
            if perim > 0:
                circ = float(4.0 * np.pi * float(contour_area)) / float(perim * perim)
                if circ < min_circ:
                    return False

        # Be stricter on small blobs (they're most often false positives).
        # This keeps real cars (larger blobs) unaffected while suppressing noise.
        strict_area = float(
            getattr(
                config,
                "SMALL_BLOB_STRICT_AREA",
                max(0.0, min_area * float(getattr(config, "SMALL_BLOB_STRICT_AREA_SCALE", 1.6) or 1.6)),
            )
            or 0.0
        )
        if strict_area > 0 and contour_area < strict_area:
            min_extent_small = float(getattr(config, "MIN_VEHICLE_EXTENT_SMALL", min_extent + 0.08) or (min_extent + 0.08))
            min_solidity_small = float(getattr(config, "MIN_VEHICLE_SOLIDITY_SMALL", min_solidity + 0.04) or (min_solidity + 0.04))
            if extent < min_extent_small:
                return False
            if min_solidity_small > 0:
                try:
                    hull = cv2.convexHull(contour)
                    hull_area = float(cv2.contourArea(hull))
                except Exception:
                    hull_area = 0.0
                if hull_area > 0:
                    solidity = float(contour_area) / hull_area
                    if solidity < min_solidity_small:
                        return False

        return True

    @staticmethod
    def _classify_vehicle_area(area: float) -> str:
        """Classify a detection by area using config.VEHICLE_TYPES."""
        try:
            for vtype, (min_area, max_area) in getattr(config, "VEHICLE_TYPES", {}).items():
                if float(min_area) <= float(area) <= float(max_area):
                    return str(vtype)
        except Exception:
            pass
        return "UNKNOWN"

    @staticmethod
    def _is_allowed_vehicle_type(area: float) -> bool:
        """Gate detections by configured allowed vehicle types (if set)."""
        allowed = getattr(config, "ALLOWED_VEHICLE_TYPES", None)
        if not allowed:
            return True
        try:
            allowed_set = {str(x) for x in allowed}
        except Exception:
            allowed_set = {str(allowed)}

        vtype = VehicleDetector._classify_vehicle_area(area)
        return vtype in allowed_set

    def _detect_by_color(self, frame: np.ndarray) -> List[dict]:
        """Detect vehicles using full-frame HSV color segmentation."""
        ranges = getattr(config, "CAR_COLOR_RANGES", [])
        if not ranges:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        combined_mask = None
        for (lower_hsv, upper_hsv) in ranges:
            lower = np.array(lower_hsv, dtype=np.uint8)
            upper = np.array(upper_hsv, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            combined_mask = mask if combined_mask is None else cv2.bitwise_or(combined_mask, mask)

        if combined_mask is None:
            return []

        # Clean up mask (use smaller close to avoid merging lane markings)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, self.kernel_open)
        kernel_close = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            tuple(getattr(config, "COLOR_MASK_CLOSE_KERNEL", (5, 5))),
        )
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel_close)

        # Save for presence checks.
        self._last_color_mask = combined_mask

        if getattr(config, "SHOW_COLOR_MASK", False):
            cv2.imshow("USAD - Color Mask", combined_mask)

        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        intersection_roi_mask = self._get_intersection_roi_mask(frame)

        detections: List[dict] = []
        for contour in contours:
            for c in self._split_contour_if_needed(combined_mask, contour):
                area = cv2.contourArea(c)
                if not (config.MIN_VEHICLE_AREA <= area <= config.MAX_VEHICLE_AREA):
                    continue

                # Optionally filter to only the main (toy car) size category.
                if not self._is_allowed_vehicle_type(area):
                    continue

                x, y, w, h = cv2.boundingRect(c)
                if not self._passes_shape_filters(c, w, h, area):
                    continue

                # Mask fill ratio is used only to decide whether to spawn new tracks.
                mask_roi = combined_mask[y : y + h, x : x + w]
                mask_ratio = 0.0
                if mask_roi.size != 0 and w > 0 and h > 0:
                    mask_ratio = float(cv2.countNonZero(mask_roi)) / float(w * h)

                center = (x + w // 2, y + h // 2)

                pts = [
                    center,
                    (x + 2, y + 2),
                    (x + w - 2, y + 2),
                    (x + 2, y + h - 2),
                    (x + w - 2, y + h - 2),
                    (x + w // 2, y + h - 2),  # bottom-center
                ]

                if intersection_roi_mask is not None and (not self._any_point_in_mask(intersection_roi_mask, pts)):
                    continue

                # Optionally require lane membership at detection-time
                if getattr(config, "REQUIRE_LANE_MEMBERSHIP_FOR_DETECTION", True):
                    tol = float(getattr(config, "LANE_MEMBERSHIP_TOLERANCE_PX", 0.0) or 0.0)
                    is_in_lane = False
                    for lane_data in config.LANES.values():
                        lane_region = np.array(lane_data["region"], dtype=np.int32)
                        if any(cv2.pointPolygonTest(lane_region, p, True) >= -tol for p in pts):
                            is_in_lane = True
                            break

                    if not is_in_lane and hasattr(config, "INTERSECTION_CENTER"):
                        inter = np.array(config.INTERSECTION_CENTER, dtype=np.int32)
                        if any(cv2.pointPolygonTest(inter, p, True) >= -tol for p in pts):
                            is_in_lane = True

                    if not is_in_lane:
                        continue

                detections.append({"center": center, "bbox": (x, y, w, h), "area": area, "mask_ratio": mask_ratio})

        return detections

    def _detect_by_motion(self, frame: np.ndarray) -> List[dict]:
        """Detect vehicles based on motion using background subtraction."""
        # Build a stable background for the first N frames
        self.frame_count += 1
        if not self.background_ready:
            self.bg_subtractor.apply(frame, learningRate=1.0)
            if self.frame_count >= self.learning_frames:
                self.background_ready = True
            return []

        # Use a small learning rate instead of fully freezing.
        # A fully frozen model can fail if cars were present during warmup.
        learning_rate = float(getattr(config, "BG_LEARNING_RATE", 0.001))
        fg_mask = self.bg_subtractor.apply(frame, learningRate=learning_rate)
        _, fg_mask = cv2.threshold(fg_mask, 200, 255, cv2.THRESH_BINARY)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, self.kernel_open)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, self.kernel_close)

        if getattr(config, "SHOW_FG_MASK", False):
            cv2.imshow("USAD - FG Mask", fg_mask)

        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        roi_mask = self._get_intersection_roi_mask(frame)

        # Reject mostly-hollow motion blobs (common with background subtraction noise).
        fg_min_ratio = float(getattr(config, "FG_MASK_MIN_RATIO", 0.12) or 0.12)

        detections: List[dict] = []
        for contour in contours:
            for c in self._split_contour_if_needed(fg_mask, contour):
                area = cv2.contourArea(c)
                if not (config.MIN_VEHICLE_AREA <= area <= config.MAX_VEHICLE_AREA):
                    continue

                # Optionally filter to only the main (toy car) size category.
                if not self._is_allowed_vehicle_type(area):
                    continue

                x, y, w, h = cv2.boundingRect(c)
                if not self._passes_shape_filters(c, w, h, area):
                    continue

                mask_ratio = 0.0
                if fg_min_ratio > 0 and w > 0 and h > 0:
                    roi = fg_mask[y : y + h, x : x + w]
                    if roi.size != 0:
                        mask_ratio = float(cv2.countNonZero(roi)) / float(w * h)
                        if mask_ratio < fg_min_ratio:
                            continue

                center = (x + w // 2, y + h // 2)

                pts = [
                    center,
                    (x + 2, y + 2),
                    (x + w - 2, y + 2),
                    (x + 2, y + h - 2),
                    (x + w - 2, y + h - 2),
                    (x + w // 2, y + h - 2),
                ]

                if roi_mask is not None and (not self._any_point_in_mask(roi_mask, pts)):
                    continue

                if getattr(config, "REQUIRE_LANE_MEMBERSHIP_FOR_DETECTION", True):
                    tol = float(getattr(config, "LANE_MEMBERSHIP_TOLERANCE_PX", 0.0) or 0.0)
                    is_in_lane = False
                    for lane_data in config.LANES.values():
                        lane_region = np.array(lane_data["region"], dtype=np.int32)
                        if any(cv2.pointPolygonTest(lane_region, p, True) >= -tol for p in pts):
                            is_in_lane = True
                            break
                    if not is_in_lane and hasattr(config, "INTERSECTION_CENTER"):
                        inter = np.array(config.INTERSECTION_CENTER, dtype=np.int32)
                        if any(cv2.pointPolygonTest(inter, p, True) >= -tol for p in pts):
                            is_in_lane = True
                    if not is_in_lane:
                        continue

                detections.append({"center": center, "bbox": (x, y, w, h), "area": area, "mask_ratio": mask_ratio})

        return detections
    
    def _update_tracking(self, detections: List[dict]):
        """Update vehicle tracking with new detections"""
        if not detections:
            return

        def _bbox_iou(b1: Tuple[int, int, int, int], b2: Tuple[int, int, int, int]) -> float:
            x1, y1, w1, h1 = b1
            x2, y2, w2, h2 = b2
            ax1, ay1, ax2, ay2 = x1, y1, x1 + w1, y1 + h1
            bx1, by1, bx2, by2 = x2, y2, x2 + w2, y2 + h2
            ix1, iy1 = max(ax1, bx1), max(ay1, by1)
            ix2, iy2 = min(ax2, bx2), min(ay2, by2)
            iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
            inter = float(iw * ih)
            if inter <= 0:
                return 0.0
            union = float(w1 * h1 + w2 * h2) - inter
            return inter / union if union > 0 else 0.0

        frame_ts = time.time()
        max_dist = float(getattr(config, "MAX_TRACKING_DISTANCE", 50))
        min_iou = float(getattr(config, "MIN_TRACKING_IOU", 0.0) or 0.0)
        near_ratio = float(getattr(config, "TRACKING_NEAR_DISTANCE_RATIO", 0.6) or 0.6)
        near_dist = max_dist * max(0.0, min(1.0, near_ratio))

        # Global assignment: prefer higher overlap, then smaller distance.
        pairs = []  # (iou, distance, vehicle_id, det_idx)
        for vehicle_id, vehicle in self.vehicles.items():
            for det_idx, det in enumerate(detections):
                ref_center = vehicle.center
                if int(getattr(vehicle, "lost_frames", 0)) > 0:
                    ref_center = vehicle.predict_center(frame_ts, max_dt=1.0)

                # Allow a larger gate when a track has been missing for a moment.
                try:
                    last_ts = float(vehicle.timestamps[-1]) if getattr(vehicle, "timestamps", None) else frame_ts
                except Exception:
                    last_ts = frame_ts
                dt = max(0.0, min(1.0, frame_ts - last_ts))
                speed = float(vehicle.get_speed())
                effective_max_dist = max_dist + (speed * dt * 1.2) + 10.0

                distance = float(np.linalg.norm(np.array(ref_center) - np.array(det["center"])))
                if distance > effective_max_dist:
                    continue
                iou = _bbox_iou(vehicle.bbox, det["bbox"])
                # If overlap is tiny, only allow match when extremely close.
                if min_iou > 0 and iou < min_iou and distance > near_dist:
                    continue
                pairs.append((iou, distance, vehicle_id, det_idx))

        pairs.sort(key=lambda t: (-t[0], t[1]))

        matched_vehicle_ids = set()
        used_det_idxs = set()

        for iou, distance, vehicle_id, det_idx in pairs:
            if vehicle_id in matched_vehicle_ids or det_idx in used_det_idxs:
                continue
            det = detections[det_idx]
            self.vehicles[vehicle_id].update(det["center"], det["bbox"], det["area"])
            matched_vehicle_ids.add(vehicle_id)
            used_det_idxs.add(det_idx)

        min_ratio = float(getattr(config, "CAR_COLOR_MIN_RATIO", 0.0) or 0.0)
        # Make it harder for small/noisy blobs to spawn brand-new tracks.
        base_min_area = float(getattr(config, "MIN_VEHICLE_AREA", 0) or 0)
        new_area_scale = float(getattr(config, "NEW_TRACK_MIN_AREA_SCALE", 1.10) or 1.10)
        new_track_min_area = float(getattr(config, "NEW_TRACK_MIN_AREA", base_min_area * new_area_scale) or (base_min_area * new_area_scale))
        for det_idx, det in enumerate(detections):
            if det_idx in used_det_idxs:
                continue
            source = det.get("source")

            # When color segmentation is enabled, motion detections are only used to
            # re-acquire existing tracks and must not create new ones (prevents ghosts).
            if self.use_color_segmentation and source == "motion":
                continue

            if new_track_min_area > 0 and float(det.get("area", 0.0)) < new_track_min_area:
                continue

            if source == "color":
                if min_ratio > 0 and float(det.get("mask_ratio", 1.0)) < min_ratio:
                    continue
            vehicle = Vehicle(det["center"], det["bbox"], det["area"])
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
            if not getattr(vehicle, "confirmed", False):
                continue
            x, y, w, h = vehicle.bbox
            pts = [
                vehicle.center,
                (x + 2, y + 2),
                (x + w - 2, y + 2),
                (x + 2, y + h - 2),
                (x + w - 2, y + h - 2),
                (x + w // 2, y + h - 2),
            ]

            if any(cv2.pointPolygonTest(lane_region, p, False) >= 0 for p in pts):
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
            x, y, w, h = map(int, vehicle.bbox)
            
            color = config.COLOR_VEHICLE
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            
            cv2.circle(frame, tuple(map(int, vehicle.center)), 4, color, -1)
            
            if config.SHOW_VEHICLE_IDS:
                label = f"ID:{vehicle.id} {vehicle.vehicle_type}"
                if vehicle.license_plate:
                    label += f" {vehicle.license_plate}"
                
                cv2.putText(frame, label, (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            if len(vehicle.positions) > 1:
                points = np.array(vehicle.positions, dtype=np.int32)
                cv2.polylines(frame, [points], False, color, 1)
        
        return frame
    
    def reset(self, reset_background: bool = False, verbose: bool = False):
        """Reset detector state.

        By default this clears tracks/IDs only.
        When reset_background=True, it also rebuilds the background subtractor and
        restarts the background learning warmup.
        """
        self.vehicles.clear()
        Vehicle._next_id = 1

        self.had_detections_this_frame = False
        self.had_confirmed_updates_this_frame = False

        if reset_background:
            self._reset_background_learning(verbose=verbose)
