"""Vehicle detection and tracking."""

from collections import deque

import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional
import time
import config

class Vehicle:
    """Represents a tracked vehicle."""
    
    _next_id = 1
    
    def __init__(self, center: Tuple[int, int], bbox: Tuple[int, int, int, int], area: float):
        self.id = Vehicle._next_id
        Vehicle._next_id += 1

        # Normalize center to integer pixel coordinates.
        try:
            cx, cy = center
        except Exception:
            cx, cy = 0, 0
        self.center = (int(round(float(cx))), int(round(float(cy))))

        try:
            x, y, w, h = bbox
        except Exception:
            x, y, w, h = 0, 0, 0, 0
        self.bbox = (
            int(round(float(x))),
            int(round(float(y))),
            int(round(float(w))),
            int(round(float(h))),
        )  # (x, y, w, h)
        self.area = area

        # Last observed (non-predicted) measurement.
        self._last_observed_center = tuple(self.center)
        self._last_observed_bbox = tuple(self.bbox)
        self._last_observed_ts = float(time.time())

        # Frames without supporting segmentation evidence.
        self._no_presence_frames = 0

        self._smooth_center = np.array(self.center, dtype=np.float32)
        self._smooth_bbox = np.array(self.bbox, dtype=np.float32)

        # Short rolling history to reduce flicker.
        self._bbox_history: deque = deque()
        self._center_history: deque = deque()
        
        # Store int centers to avoid tiny float jitter.
        self.positions = [tuple(self.center)]
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

        # Frames spent outside intersection ROI.
        self.roi_outside_frames = 0
        
    def _classify_vehicle(self, area: float) -> str:
        """Classify vehicle by size"""
        for vtype, (min_area, max_area) in config.VEHICLE_TYPES.items():
            if min_area <= area <= max_area:
                return vtype
        return "UNKNOWN"
    
    def update(
        self,
        center: Tuple[int, int],
        bbox: Tuple[int, int, int, int],
        area: float,
        *,
        crowded: bool = False,
        overlapping: bool = False,
    ):
        """Update vehicle position and smoothed tracking state."""
        def _rolling_median(attr_name: str, value: Tuple[int, ...], window: int) -> Tuple[int, ...]:
            window = int(window or 0)
            if window <= 1:
                return value

            hist = getattr(self, attr_name, None)
            if not isinstance(hist, deque) or getattr(hist, "maxlen", None) != window:
                prior = list(hist) if isinstance(hist, deque) else []
                hist = deque(prior, maxlen=window)

            hist.append(tuple(int(v) for v in value))
            setattr(self, attr_name, hist)

            if len(hist) < 3:
                return value
            arr = np.array(list(hist), dtype=np.int32)
            med = np.median(arr, axis=0)
            return tuple(int(round(float(x))) for x in med)

        # Avoid stale smoothing pulling boxes away from detections.
        try:
            old_center = tuple(int(v) for v in getattr(self, "center", (0, 0)))
        except Exception:
            old_center = (0, 0)
        try:
            old_bbox = tuple(int(v) for v in getattr(self, "bbox", (0, 0, 1, 1)))
        except Exception:
            old_bbox = (0, 0, 1, 1)

        # Treat lost as >1 so we only reset after a true miss.
        lost_frames_now = int(getattr(self, "lost_frames", 0) or 0)
        was_lost = lost_frames_now > 1
        teleport_px = float(getattr(config, "TRACK_TELEPORT_RESET_DISTANCE_PX", 0.0) or 0.0)
        if teleport_px <= 0:
            teleport_px = float(getattr(config, "MAX_TRACKING_DISTANCE", 50) or 50) * 2.5

        try:
            jump = float(np.hypot(float(center[0]) - float(old_center[0]), float(center[1]) - float(old_center[1])))
        except Exception:
            jump = 0.0

        # Stationary lock keeps bbox/center stable when car is barely moving.
        enter_px = float(getattr(config, "TRACK_STATIONARY_ENTER_PX", 2.0) or 2.0)
        exit_px = float(getattr(config, "TRACK_STATIONARY_EXIT_PX", 6.0) or 6.0)
        enter_px = max(0.0, min(25.0, enter_px))
        exit_px = max(enter_px + 0.5, min(60.0, exit_px))
        locked = bool(getattr(self, "_stationary_locked", False))

        # Reset lock on true loss or teleport.
        if was_lost or jump > teleport_px:
            locked = False
        else:
            if locked:
                if jump > exit_px:
                    locked = False
            else:
                if jump <= enter_px:
                    locked = True
        setattr(self, "_stationary_locked", bool(locked))

        if was_lost or jump > teleport_px:
            # Reset smoothing/histories to current detection.
            try:
                self._smooth_center = np.array((float(center[0]), float(center[1])), dtype=np.float32)
                self._smooth_bbox = np.array(tuple(float(v) for v in bbox), dtype=np.float32)
            except Exception:
                pass

        # If stationary, freeze bbox/center and treat as observed.
        if locked and (not was_lost):
            now_ts = time.time()
            self.center = (int(old_center[0]), int(old_center[1]))
            self.bbox = tuple(int(v) for v in old_bbox)
            self.area = area
            self.vehicle_type = self._classify_vehicle(area)

            try:
                self._smooth_center = np.array(self.center, dtype=np.float32)
                self._smooth_bbox = np.array(self.bbox, dtype=np.float32)
            except Exception:
                pass

            # Keep history aligned with frozen center/bbox.
            try:
                if isinstance(getattr(self, "_center_history", None), deque):
                    self._center_history.append(tuple(int(v) for v in self.center))
                if isinstance(getattr(self, "_bbox_history", None), deque):
                    self._bbox_history.append(tuple(int(v) for v in self.bbox))
            except Exception:
                pass

            self.positions.append(tuple(int(v) for v in self.center))
            self.timestamps.append(now_ts)
            if len(self.positions) > 30:
                self.positions.pop(0)
                self.timestamps.pop(0)

            self.lost_frames = 0
            setattr(self, "_predicted_only", False)

            # Update last observed measurement and ghost counters.
            try:
                self._last_observed_center = tuple(int(v) for v in self.center)
                self._last_observed_bbox = tuple(int(v) for v in self.bbox)
                self._last_observed_ts = float(now_ts)
                self._no_presence_frames = 0
            except Exception:
                pass

            self.seen_frames += 1
            confirm_frames = int(getattr(config, "MIN_TRACK_CONFIRM_FRAMES", 3))
            if self.seen_frames >= max(1, confirm_frames):
                self.confirmed = True

            # Update stopped state using recent positions.
            if len(self.positions) >= 2:
                distance = np.linalg.norm(np.array(self.center) - np.array(self.positions[-2]))
                if distance < config.STOPPED_DISTANCE_THRESHOLD:
                    if not self.is_stopped:
                        self.stopped_time = now_ts
                        self.is_stopped = True
                else:
                    self.is_stopped = False
                    self.last_moved_time = now_ts

            return

        # Normalize raw detection inputs.
        try:
            rcx, rcy = center
        except Exception:
            rcx, rcy = 0, 0
        raw_center = (int(round(float(rcx))), int(round(float(rcy))))

        try:
            rx, ry, rw, rh = bbox
        except Exception:
            rx, ry, rw, rh = 0, 0, 1, 1
        raw_bbox = (
            int(round(float(rx))),
            int(round(float(ry))),
            max(1, int(round(float(rw)))),
            max(1, int(round(float(rh)))),
        )

        base_alpha = float(getattr(config, "TRACK_SMOOTHING_ALPHA", 0.0) or 0.0)
        base_alpha = max(0.0, min(0.95, base_alpha))

        # Allow separate tuning for bbox smoothing; default to same as TRACK_SMOOTHING_ALPHA.
        bbox_alpha = float(getattr(config, "TRACK_BBOX_SMOOTHING_ALPHA", base_alpha) or base_alpha)
        bbox_alpha = max(0.0, min(0.95, bbox_alpha))

        try:
            seen_frames_now = int(getattr(self, "seen_frames", 1) or 1)
        except Exception:
            seen_frames_now = 1
        startup_frames = int(getattr(config, "TRACK_STARTUP_STABILIZE_FRAMES", 6) or 0)
        startup_frames = max(0, min(30, startup_frames))
        if startup_frames > 0 and seen_frames_now <= startup_frames and (not was_lost):
            startup_min_alpha = float(getattr(config, "TRACK_STARTUP_MIN_ALPHA", 0.80) or 0.80)
            startup_min_alpha = max(0.0, min(0.95, startup_min_alpha))
            base_alpha = max(base_alpha, startup_min_alpha)
            bbox_alpha = max(bbox_alpha, startup_min_alpha)

        now_ts = time.time()
        try:
            collision_until = float(getattr(self, "_collision_stability_until", 0.0) or 0.0)
        except Exception:
            collision_until = 0.0
        in_collision_stability = now_ts < collision_until

        stability_boost = bool(crowded) or bool(in_collision_stability) or bool(overlapping)
        overlap_boost = bool(overlapping) or bool(in_collision_stability)
        if stability_boost:
            alpha_mult = float(getattr(config, "TRACK_STABILITY_ALPHA_MULT", 1.35) or 1.35)
            alpha_mult = max(1.0, min(3.0, alpha_mult))
            base_alpha = min(0.95, base_alpha * alpha_mult)
            bbox_alpha = min(0.95, bbox_alpha * alpha_mult)

        if overlap_boost:
            # Extra damping specifically for touching/overlapping cars.
            extra_mult = float(getattr(config, "TRACK_OVERLAP_ALPHA_MULT", 1.20) or 1.20)
            extra_mult = max(1.0, min(3.0, extra_mult))
            base_alpha = min(0.95, base_alpha * extra_mult)
            bbox_alpha = min(0.95, bbox_alpha * extra_mult)

        c = np.array(raw_center, dtype=np.float32)
        b = np.array(raw_bbox, dtype=np.float32)

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

        b_alpha = bbox_alpha
        if delta >= dynamic_px:
            b_alpha = min(b_alpha, fast_alpha_cap)

        if alpha > 0:
            self._smooth_center = alpha * self._smooth_center + (1 - alpha) * c
            if b_alpha > 0:
                self._smooth_bbox = b_alpha * self._smooth_bbox + (1 - b_alpha) * b
            else:
                self._smooth_bbox = b
        else:
            self._smooth_center = c
            self._smooth_bbox = b

        self.center = (int(self._smooth_center[0]), int(self._smooth_center[1]))
        sb = self._smooth_bbox
        new_bbox = (int(sb[0]), int(sb[1]), int(sb[2]), int(sb[3]))

        # This prevents "flying away" caused by stale smoothing or a brief mismatch.
        max_det_dev = int(getattr(config, "TRACK_BBOX_MAX_DEVIATION_FROM_DET_PX", 14) or 0)
        max_det_dev = max(0, min(200, max_det_dev))
        if stability_boost and max_det_dev > 0:
            dev_mult = float(getattr(config, "TRACK_STABILITY_DET_DEV_MULT", 1.6) or 1.6)
            dev_mult = max(1.0, min(4.0, dev_mult))
            max_det_dev = int(max_det_dev * dev_mult)
            max_det_dev = max(0, min(200, max_det_dev))

        if overlap_boost and (not was_lost) and max_det_dev > 0:
            max_overlap_dev = int(getattr(config, "TRACK_OVERLAP_MAX_DET_DEV_PX", 28) or 28)
            max_overlap_dev = max(0, min(200, max_overlap_dev))
            if max_overlap_dev > 0:
                max_det_dev = min(max_det_dev, max_overlap_dev)
        if max_det_dev > 0:
            try:
                cx, cy = map(int, self.center)
                cx = int(max(raw_center[0] - max_det_dev, min(raw_center[0] + max_det_dev, cx)))
                cy = int(max(raw_center[1] - max_det_dev, min(raw_center[1] + max_det_dev, cy)))
                self.center = (cx, cy)
                # Keep smoothing state aligned so subsequent updates don't snap.
                try:
                    self._smooth_center = np.array(self.center, dtype=np.float32)
                except Exception:
                    pass
            except Exception:
                pass
        if max_det_dev > 0:
            dx, dy, dw, dh = raw_bbox
            nx, ny, nw, nh = map(int, new_bbox)
            nx = int(max(dx - max_det_dev, min(dx + max_det_dev, nx)))
            ny = int(max(dy - max_det_dev, min(dy + max_det_dev, ny)))
            nw = int(max(1, max(dw - max_det_dev, min(dw + max_det_dev, nw))))
            nh = int(max(1, max(dh - max_det_dev, min(dh + max_det_dev, nh))))
            new_bbox = (nx, ny, nw, nh)

        # Ensure the bbox still contains the (stabilized) center.
        try:
            cx, cy = map(int, self.center)
            nx, ny, nw, nh = map(int, new_bbox)
            if nw > 0 and nh > 0:
                left, top, right, bottom = nx, ny, nx + nw, ny + nh
                if cx < left:
                    nx = cx
                elif cx > right:
                    nx = cx - nw
                if cy < top:
                    ny = cy
                elif cy > bottom:
                    ny = cy - nh
                new_bbox = (int(nx), int(ny), int(max(1, nw)), int(max(1, nh)))
        except Exception:
            pass
        if overlap_boost and (not was_lost):
            lock_size = bool(getattr(config, "TRACK_OVERLAP_LOCK_SIZE", True))
            center_box = bool(getattr(config, "TRACK_OVERLAP_CENTER_BOX", True))
            if lock_size:
                try:
                    ow, oh = int(old_bbox[2]), int(old_bbox[3])
                    nx, ny, nw, nh = map(int, new_bbox)
                    nw = max(1, ow)
                    nh = max(1, oh)
                    new_bbox = (nx, ny, nw, nh)
                except Exception:
                    pass
            if center_box:
                try:
                    cx, cy = map(int, self.center)
                    nx, ny, nw, nh = map(int, new_bbox)
                    nx = int(round(cx - (nw / 2.0)))
                    ny = int(round(cy - (nh / 2.0)))
                    new_bbox = (nx, ny, nw, nh)
                except Exception:
                    pass
        
        # Cap bbox movement per frame to dampen jitter from segmentation noise.
        max_shift = float(getattr(config, "TRACK_BBOX_MAX_SHIFT_PER_FRAME", 10))
        if stability_boost and max_shift > 0:
            shift_mult = float(getattr(config, "TRACK_STABILITY_MAX_SHIFT_MULT", 0.75) or 0.75)
            shift_mult = max(0.2, min(1.0, shift_mult))
            max_shift = max(2.0, max_shift * shift_mult)
        if (not was_lost) and max_shift > 0 and len(getattr(self, "positions", [])) > 0:
            dx = new_bbox[0] - old_bbox[0]
            dy = new_bbox[1] - old_bbox[1]
            
            # Clamp movement magnitude to max_shift pixels
            if abs(dx) > max_shift:
                dx = max_shift if dx > 0 else -max_shift
            if abs(dy) > max_shift:
                dy = max_shift if dy > 0 else -max_shift
            
            new_bbox = (old_bbox[0] + dx, old_bbox[1] + dy, new_bbox[2], new_bbox[3])

        # Cap bbox size changes too (w/h jitter is a common source of flicker).
        max_size_delta = int(getattr(config, "TRACK_BBOX_MAX_SIZE_DELTA_PER_FRAME", 8) or 0)
        if stability_boost and max_size_delta > 0:
            size_mult = float(getattr(config, "TRACK_STABILITY_MAX_SIZE_DELTA_MULT", 0.75) or 0.75)
            size_mult = max(0.2, min(1.0, size_mult))
            max_size_delta = max(2, int(round(float(max_size_delta) * size_mult)))
        if max_size_delta > 0:
            ow, oh = int(old_bbox[2]), int(old_bbox[3])
            nw, nh = int(new_bbox[2]), int(new_bbox[3])
            dw = nw - ow
            dh = nh - oh
            if abs(dw) > max_size_delta:
                dw = max_size_delta if dw > 0 else -max_size_delta
            if abs(dh) > max_size_delta:
                dh = max_size_delta if dh > 0 else -max_size_delta
            new_bbox = (int(new_bbox[0]), int(new_bbox[1]), int(ow + dw), int(oh + dh))

        # Deadband: ignore tiny jitter (prevents constant 1-2px tweaking).
        dead_xy = int(getattr(config, "TRACK_BBOX_DEADBAND_PX", 2) or 0)
        dead_wh = int(getattr(config, "TRACK_BBOX_SIZE_DEADBAND_PX", 2) or 0)
        if startup_frames > 0 and seen_frames_now <= startup_frames and (not was_lost):
            dead_xy = max(dead_xy, int(getattr(config, "TRACK_STARTUP_DEADBAND_PX", 3) or 3))
            dead_wh = max(dead_wh, int(getattr(config, "TRACK_STARTUP_SIZE_DEADBAND_PX", 3) or 3))
        if overlap_boost:
            dead_xy = max(dead_xy, int(getattr(config, "TRACK_OVERLAP_DEADBAND_PX", 4) or 4))
            dead_wh = max(dead_wh, int(getattr(config, "TRACK_OVERLAP_SIZE_DEADBAND_PX", 4) or 4))
        if dead_xy > 0:
            nx, ny, nw, nh = map(int, new_bbox)
            ox, oy, ow, oh = map(int, old_bbox)
            if abs(nx - ox) <= dead_xy:
                nx = ox
            if abs(ny - oy) <= dead_xy:
                ny = oy
            new_bbox = (nx, ny, nw, nh)
        if dead_wh > 0:
            nx, ny, nw, nh = map(int, new_bbox)
            ox, oy, ow, oh = map(int, old_bbox)
            if abs(nw - ow) <= dead_wh:
                nw = ow
            if abs(nh - oh) <= dead_wh:
                nh = oh
            new_bbox = (nx, ny, nw, nh)

        bbox_win = int(getattr(config, "TRACK_BBOX_MEDIAN_WINDOW", 3) or 0)
        center_win = int(getattr(config, "TRACK_CENTER_MEDIAN_WINDOW", 3) or 0)
        bbox_win = max(0, min(9, bbox_win))
        center_win = max(0, min(9, center_win))
        if startup_frames > 0 and seen_frames_now <= startup_frames and (not was_lost):
            bbox_win = max(bbox_win, int(getattr(config, "TRACK_STARTUP_BBOX_MEDIAN_WINDOW", 5) or 5))
            center_win = max(center_win, int(getattr(config, "TRACK_STARTUP_CENTER_MEDIAN_WINDOW", 5) or 5))
        if stability_boost:
            bbox_win = max(bbox_win, int(getattr(config, "TRACK_STABILITY_BBOX_MEDIAN_WINDOW", 3) or 3))
            center_win = max(center_win, int(getattr(config, "TRACK_STABILITY_CENTER_MEDIAN_WINDOW", 3) or 3))
        if overlap_boost:
            bbox_win = max(bbox_win, int(getattr(config, "TRACK_OVERLAP_BBOX_MEDIAN_WINDOW", 5) or 5))
            center_win = max(center_win, int(getattr(config, "TRACK_OVERLAP_CENTER_MEDIAN_WINDOW", 5) or 5))
        try:
            new_center = (int(self.center[0]), int(self.center[1]))
            new_center = _rolling_median("_center_history", new_center, center_win)
            self.center = (int(new_center[0]), int(new_center[1]))
        except Exception:
            pass

        try:
            new_bbox = tuple(int(v) for v in new_bbox)
            new_bbox = _rolling_median("_bbox_history", new_bbox, bbox_win)
            new_bbox = tuple(int(v) for v in new_bbox)
        except Exception:
            pass

        # Ensure sane dimensions.
        x, y, w, h = map(int, new_bbox)
        w = max(1, w)
        h = max(1, h)
        new_bbox = (x, y, w, h)
        
        self.bbox = new_bbox
        self.area = area

        self.vehicle_type = self._classify_vehicle(area)
        
        # Store stabilized center for downstream speed/velocity prediction.
        self.positions.append(tuple(int(v) for v in self.center))
        self.timestamps.append(now_ts)
        
        # Keep only recent history
        if len(self.positions) > 30:
            self.positions.pop(0)
            self.timestamps.pop(0)
        
        self.lost_frames = 0

        # This frame is an observed update (not a prediction).
        setattr(self, "_predicted_only", False)

        # Record last observed measurement for drift-free prediction.
        try:
            self._last_observed_center = tuple(int(v) for v in self.center)
            self._last_observed_bbox = tuple(int(v) for v in self.bbox)
            self._last_observed_ts = float(now_ts)
            self._no_presence_frames = 0
        except Exception:
            pass

        self.seen_frames += 1
        confirm_frames = int(getattr(config, "MIN_TRACK_CONFIRM_FRAMES", 3))
        if self.seen_frames >= max(1, confirm_frames):
            self.confirmed = True
        
        # Check if vehicle is stopped
        if len(self.positions) >= 2:
            distance = np.linalg.norm(np.array(self.center) - np.array(self.positions[-2]))
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
        try:
            last_obs = getattr(self, "_last_observed_center", None)
            if last_obs is None:
                last_obs = tuple(self.center)
            lx, ly = map(float, last_obs)
            # Reuse the teleport reset distance as a conservative total drift budget.
            teleport_px = float(getattr(config, "TRACK_TELEPORT_RESET_DISTANCE_PX", 0.0) or 0.0)
            if teleport_px <= 0:
                teleport_px = float(getattr(config, "MAX_TRACKING_DISTANCE", 50) or 50) * 2.5
            teleport_px = max(10.0, min(400.0, teleport_px))

            nx = float(self.center[0] + dx)
            ny = float(self.center[1] + dy)
            dd = float(np.hypot(nx - lx, ny - ly))
            if dd > teleport_px:
                if dd > 0:
                    scale = teleport_px / dd
                    nx = lx + (nx - lx) * scale
                    ny = ly + (ny - ly) * scale
                dx = int(round(nx - float(self.center[0])))
                dy = int(round(ny - float(self.center[1])))
        except Exception:
            pass

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

    @staticmethod
    def _overlap_over_min_area(b1: Tuple[int, int, int, int], b2: Tuple[int, int, int, int]) -> float:
        """Overlap ratio normalized by the smaller bbox area.

        Useful to identify "same object twice" duplicates even when IoU is modest.
        """
        try:
            x1, y1, w1, h1 = map(float, b1)
            x2, y2, w2, h2 = map(float, b2)
        except Exception:
            return 0.0

        if w1 <= 0 or h1 <= 0 or w2 <= 0 or h2 <= 0:
            return 0.0

        ax1, ay1, ax2, ay2 = x1, y1, x1 + w1, y1 + h1
        bx1, by1, bx2, by2 = x2, y2, x2 + w2, y2 + h2
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = float(iw * ih)
        if inter <= 0:
            return 0.0

        a1 = float(w1 * h1)
        a2 = float(w2 * h2)
        denom = float(max(1.0, min(a1, a2)))
        return inter / denom

    def _suppress_duplicate_tracks(self):
        """Remove co-located duplicate tracks (same physical car detected twice).

        This directly stabilizes:
        - on-screen vehicle count (len(vehicles))
        - lane counts used for congestion / adaptive green timing
        """
        if not self.vehicles:
            return

        # Prefer dedicated tracking thresholds, else fall back to collision duplicate threshold.
        overlap_max = float(getattr(config, "TRACK_DUPLICATE_OVERLAP_MAX", 0.0) or 0.0)
        if overlap_max <= 0:
            overlap_max = float(getattr(config, "COLLISION_DUPLICATE_OVERLAP_MAX", 0.40) or 0.40)
        overlap_max = max(0.05, min(0.95, overlap_max))

        center_frac = float(getattr(config, "TRACK_DUPLICATE_CENTER_DIST_FRAC", 0.35) or 0.35)
        center_frac = max(0.10, min(0.90, center_frac))

        # Only consider tracks that were observed this frame; lost/predicted tracks are handled by TTL logic.
        active = [
            v
            for v in self.vehicles.values()
            if bool(getattr(v, "confirmed", False)) and int(getattr(v, "lost_frames", 0) or 0) == 0
        ]
        if len(active) < 2:
            return

        to_remove: set[int] = set()
        for i in range(len(active)):
            v1 = active[i]
            id1 = int(getattr(v1, "id", 0) or 0)
            if id1 in to_remove:
                continue
            for j in range(i + 1, len(active)):
                v2 = active[j]
                id2 = int(getattr(v2, "id", 0) or 0)
                if id2 in to_remove:
                    continue

                ov = float(self._overlap_over_min_area(v1.bbox, v2.bbox))
                if ov < overlap_max:
                    continue

                try:
                    c1 = np.array(v1.center, dtype=np.float32)
                    c2 = np.array(v2.center, dtype=np.float32)
                    center_dist = float(np.linalg.norm(c1 - c2))
                except Exception:
                    center_dist = 1e9

                try:
                    w1, h1 = int(v1.bbox[2]), int(v1.bbox[3])
                    w2, h2 = int(v2.bbox[2]), int(v2.bbox[3])
                    scale = float(max(1, min(w1, h1, w2, h2)))
                except Exception:
                    scale = 1.0

                # If centers are extremely close relative to bbox size, it's almost surely a duplicate.
                if center_dist > (center_frac * scale):
                    continue

                # Keep the more "established" track (more seen frames); break ties by lower ID.
                s1 = int(getattr(v1, "seen_frames", 0) or 0)
                s2 = int(getattr(v2, "seen_frames", 0) or 0)
                if (s1 > s2) or (s1 == s2 and id1 < id2):
                    to_remove.add(id2)
                else:
                    to_remove.add(id1)
                    break

        for vid in to_remove:
            self.vehicles.pop(int(vid), None)

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
                    base_ratio = float(getattr(config, "BLOB_SPLIT_DT_THRESH_RATIO", 0.5) or 0.5)
                    base_ratio = max(0.2, min(0.9, base_ratio))
                    ratio_candidates = [
                        base_ratio,
                        base_ratio + 0.08,
                        base_ratio + 0.16,
                        base_ratio - 0.08,
                    ]
                    ratios = []
                    for r in ratio_candidates:
                        rr = max(0.2, min(0.9, float(r)))
                        if rr not in ratios:
                            ratios.append(rr)

                    bg_iters = int(getattr(config, "BLOB_SPLIT_BG_DILATE_ITERS", 2) or 2)
                    bg_iters = max(1, min(5, bg_iters))

                    for fg_ratio in ratios:
                        _, sure_fg = cv2.threshold(dist, fg_ratio * dmax, 255, 0)
                        sure_fg = sure_fg.astype(np.uint8)
                        sure_bg = cv2.dilate(roi_bin, kernel, iterations=bg_iters)
                        unknown = cv2.subtract(sure_bg, sure_fg)

                        cc_count, markers = cv2.connectedComponents(sure_fg)
                        if cc_count < 3:  # background + at least 2 objects
                            continue

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

        # De-duplicate co-located tracks (prevents 1 physical car -> IDs 1 & 2 flicker)
        self._suppress_duplicate_tracks()

        self.had_confirmed_updates_this_frame = any(
            bool(getattr(v, "confirmed", False)) and int(getattr(v, "lost_frames", 0)) == 0
            for v in self.vehicles.values()
        )
        now_ts = time.time()

        # Keep these windows for removal/TTL behavior (even though we do not move boxes).
        predict_for_lost = int(getattr(config, "TRACK_PREDICT_FOR_LOST_FRAMES", 12) or 12)
        collision_predict_for_lost = int(getattr(config, "COLLISION_TRACK_PREDICT_FOR_LOST_FRAMES", 60) or 60)
        collision_predict_for_lost = max(predict_for_lost, min(600, collision_predict_for_lost))

        for v in self.vehicles.values():
            if not getattr(v, "confirmed", False):
                continue
            lf = int(getattr(v, "lost_frames", 0) or 0)
            if lf <= 0:
                continue

            try:
                collision_until = float(getattr(v, "_collision_stability_until", 0.0) or 0.0)
            except Exception:
                collision_until = 0.0
            pred_window = collision_predict_for_lost if (now_ts < collision_until) else predict_for_lost

            if lf > pred_window:
                continue
            v.apply_prediction_update(now_ts)

        force_remove = set()
        presence_ratio_by_id = {}
        if self.use_color_segmentation and self._last_color_mask is not None:
            presence_min = float(getattr(config, "TRACK_PRESENCE_MIN_RATIO", 0.0) or 0.0)

            kill_ratio = max(0.001, min(0.02, presence_min * 0.10))

            # Convert a small time budget (~200ms) into frames using configured FPS.
            fps = float(getattr(config, "CAMERA_FPS", 30) or 30)
            base_kill_frames = int(round(max(3.0, min(12.0, fps * 0.20))))

            h_img, w_img = self._last_color_mask.shape[:2]
            for v in list(self.vehicles.values()):
                if not getattr(v, "confirmed", False):
                    continue
                lf = int(getattr(v, "lost_frames", 0) or 0)
                if lf <= 0:
                    try:
                        v._no_presence_frames = 0
                    except Exception:
                        pass
                    continue

                try:
                    x, y, w, h = v.bbox
                    x = int(round(float(x)))
                    y = int(round(float(y)))
                    w = int(round(float(w)))
                    h = int(round(float(h)))
                except Exception:
                    continue

                if w <= 0 or h <= 0:
                    continue

                x0 = int(max(0, min(w_img - 1, x)))
                y0 = int(max(0, min(h_img - 1, y)))
                x1 = int(max(0, min(w_img, x + w)))
                y1 = int(max(0, min(h_img, y + h)))
                if x1 <= x0 or y1 <= y0:
                    continue

                roi = self._last_color_mask[y0:y1, x0:x1]
                denom = float(max(1, (x1 - x0) * (y1 - y0)))
                ratio = float(cv2.countNonZero(roi)) / denom
                try:
                    presence_ratio_by_id[int(getattr(v, "id", 0) or 0)] = float(ratio)
                except Exception:
                    pass
                if presence_min > 0 and ratio >= presence_min:
                    try:
                        v._no_presence_frames = 0
                    except Exception:
                        pass
                    continue
                if ratio <= kill_ratio:
                    try:
                        n = int(getattr(v, "_no_presence_frames", 0) or 0) + 1
                        v._no_presence_frames = n
                    except Exception:
                        n = 0

                    # Be slightly more tolerant while collision stability is active.
                    try:
                        collision_until = float(getattr(v, "_collision_stability_until", 0.0) or 0.0)
                    except Exception:
                        collision_until = 0.0
                    limit = int(base_kill_frames)
                    if not bool(getattr(v, "_predicted_only", False)):
                        limit = int(max(limit, min(30, round(float(limit) * 2.0))))
                    if now_ts < collision_until:
                        limit = int(max(limit, min(30, round(float(limit) * 1.5))))

                    # Avoid removing on the very first missed frame.
                    if lf >= 2 and n >= limit:
                        force_remove.add(int(v.id))
                else:
                    try:
                        v._no_presence_frames = 0
                    except Exception:
                        pass

        # Removal thresholds
        candidate_ttl = int(getattr(config, "CANDIDATE_VEHICLE_LOST_FRAMES", 8))
        confirmed_ttl = int(getattr(config, "VEHICLE_LOST_FRAMES", 45))
        outside_ttl = int(getattr(config, "ROI_OUTSIDE_REMOVE_FRAMES", 30))

        # Time-based removal keeps behavior consistent even when FPS dips.
        lost_remove_seconds = float(getattr(config, "TRACK_LOST_REMOVE_SECONDS", 0.0) or 0.0)
        collision_grace_seconds = float(getattr(config, "COLLISION_TRACK_GRACE_SECONDS", 2.5) or 2.5)
        collision_grace_seconds = max(0.0, min(30.0, collision_grace_seconds))
        roi_outside_remove_seconds = float(getattr(config, "ROI_OUTSIDE_REMOVE_SECONDS", 0.0) or 0.0)

        predict_for_lost = int(getattr(config, "TRACK_PREDICT_FOR_LOST_FRAMES", 12) or 12)

        intersection_roi_mask = self._get_intersection_roi_mask(frame)

        to_remove = []
        for vehicle_id, vehicle in self.vehicles.items():
            if vehicle_id in force_remove:
                to_remove.append(vehicle_id)
                continue
            # Intersection ROI enforcement (optional)
            if intersection_roi_mask is not None:
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
                    if hasattr(vehicle, "_roi_outside_since_ts"):
                        delattr(vehicle, "_roi_outside_since_ts")
                else:
                    vehicle.roi_outside_frames = int(getattr(vehicle, "roi_outside_frames", 0)) + 1
                    if not hasattr(vehicle, "_roi_outside_since_ts"):
                        setattr(vehicle, "_roi_outside_since_ts", now_ts)

                # Drop tracks quickly when they remain outside the ROI.
                if roi_outside_remove_seconds > 0:
                    t0 = float(getattr(vehicle, "_roi_outside_since_ts", now_ts) or now_ts)
                    if (now_ts - t0) >= roi_outside_remove_seconds:
                        to_remove.append(vehicle_id)
                        continue

                # Frame-count fallback (for backwards compatibility)
                if vehicle.roi_outside_frames > outside_ttl:
                    to_remove.append(vehicle_id)
                    continue

            # Lost-track removal
            ttl = confirmed_ttl if getattr(vehicle, "confirmed", False) else candidate_ttl

            # During collision stability, keep tracks alive longer (prevents alert flicker).
            try:
                collision_until = float(getattr(vehicle, "_collision_stability_until", 0.0) or 0.0)
            except Exception:
                collision_until = 0.0
            if collision_grace_seconds > 0 and (now_ts < collision_until):
                if lost_remove_seconds > 0:
                    lost_remove_seconds_eff = max(lost_remove_seconds, collision_grace_seconds)
                else:
                    lost_remove_seconds_eff = 0.0
                ttl = max(ttl, int(round(float(collision_predict_for_lost))))
            else:
                lost_remove_seconds_eff = lost_remove_seconds

            if lost_remove_seconds_eff > 0 and int(getattr(vehicle, "lost_frames", 0)) > 0:
                # If we're actively predicting this confirmed track, keep it alive.
                if (
                    bool(getattr(vehicle, "confirmed", False))
                    and bool(getattr(vehicle, "_predicted_only", False))
                    and int(getattr(vehicle, "lost_frames", 0)) <= predict_for_lost
                ):
                    pass
                else:
                    try:
                        last_seen = float(vehicle.timestamps[-1]) if getattr(vehicle, "timestamps", None) else now_ts
                    except Exception:
                        last_seen = now_ts
                    if (now_ts - last_seen) >= lost_remove_seconds_eff:
                        to_remove.append(vehicle_id)
                        continue

            if vehicle.lost_frames > ttl:
                to_remove.append(vehicle_id)

        for vehicle_id in to_remove:
            self.vehicles.pop(vehicle_id, None)
        visible: List[Vehicle] = []

        # Keep the hold window tiny to satisfy "instant clear" when a vehicle truly disappears.
        fps = float(getattr(config, "CAMERA_FPS", 30) or 30)
        hold_frames = int(getattr(config, "COLLISION_TRACK_DISPLAY_HOLD_FRAMES", 0) or 0)
        if hold_frames <= 0:
            hold_frames = int(round(max(2.0, min(5.0, fps * 0.12))))  # ~120ms

        presence_min = float(getattr(config, "TRACK_PRESENCE_MIN_RATIO", 0.0) or 0.0)
        for v in self.vehicles.values():
            if not bool(getattr(v, "confirmed", False)):
                continue
            lf = int(getattr(v, "lost_frames", 0) or 0)
            if lf == 0:
                visible.append(v)
                continue

            # Only consider hold during active collision stabilization.
            try:
                collision_until = float(getattr(v, "_collision_stability_until", 0.0) or 0.0)
            except Exception:
                collision_until = 0.0
            if now_ts >= collision_until:
                continue

            if lf > hold_frames:
                continue

            if presence_min > 0:
                ratio = float(presence_ratio_by_id.get(int(getattr(v, "id", 0) or 0), 0.0) or 0.0)
                if ratio < presence_min:
                    continue

            # Freeze: show last observed box/center without motion.
            try:
                v.center = tuple(int(x) for x in getattr(v, "_last_observed_center", v.center))
                v.bbox = tuple(int(x) for x in getattr(v, "_last_observed_bbox", v.bbox))
            except Exception:
                pass
            visible.append(v)

        return visible

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

        color_open_k = tuple(getattr(config, "COLOR_MASK_OPEN_KERNEL", (3, 3)))
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, color_open_k)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel_open)

        color_close_k = tuple(getattr(config, "COLOR_MASK_CLOSE_KERNEL", (1, 1)))
        if len(color_close_k) == 2 and int(color_close_k[0]) > 1 and int(color_close_k[1]) > 1:
            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, color_close_k)
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

        pred_dt = float(getattr(config, "TRACK_PREDICT_MAX_DT", 0.25) or 0.25)
        pred_dt = max(0.05, min(0.5, pred_dt))

        lost_match_scale = float(getattr(config, "TRACK_LOST_MATCH_MAX_DIST_SCALE", 0.75) or 0.75)
        lost_match_scale = max(0.25, min(1.25, lost_match_scale))

        # Geometry gating to prevent matching a track to a different physical car.
        min_area_ratio = float(getattr(config, "TRACK_MATCH_MIN_AREA_RATIO", 0.45) or 0.45)
        max_area_ratio = float(getattr(config, "TRACK_MATCH_MAX_AREA_RATIO", 2.40) or 2.40)
        min_area_ratio = max(0.05, min(1.0, min_area_ratio))
        max_area_ratio = max(1.0, min(10.0, max_area_ratio))
        max_aspect_ratio_change = float(getattr(config, "TRACK_MATCH_MAX_ASPECT_RATIO_CHANGE", 1.85) or 1.85)
        max_aspect_ratio_change = max(1.05, min(6.0, max_aspect_ratio_change))

        strong_iou = float(getattr(config, "TRACK_MATCH_STRONG_IOU", 0.25) or 0.25)
        strong_iou = max(0.05, min(0.95, strong_iou))

        # Precompute "crowded" flags for detections.
        crowd_dist_px = float(getattr(config, "TRACK_CROWD_DISTANCE_PX", 75.0) or 75.0)
        crowd_iou = float(getattr(config, "TRACK_CROWD_IOU", 0.05) or 0.05)
        overlap_iou = float(getattr(config, "TRACK_OVERLAP_IOU", 0.08) or 0.08)
        crowd_dist_px = max(0.0, min(500.0, crowd_dist_px))
        crowd_iou = max(0.0, min(0.95, crowd_iou))
        overlap_iou = max(0.0, min(0.95, overlap_iou))
        det_crowded = [False for _ in detections]
        det_overlapping = [False for _ in detections]
        if len(detections) >= 2 and (crowd_dist_px > 0 or crowd_iou > 0):
            for i in range(len(detections)):
                ci = detections[i].get("center")
                bi = detections[i].get("bbox")
                best_d = 1e9
                best_i = 0.0
                for j in range(len(detections)):
                    if i == j:
                        continue
                    cj = detections[j].get("center")
                    bj = detections[j].get("bbox")
                    try:
                        d = float(np.linalg.norm(np.array(ci) - np.array(cj)))
                    except Exception:
                        d = 1e9
                    if d < best_d:
                        best_d = d
                    try:
                        ov = _bbox_iou(bi, bj)
                    except Exception:
                        ov = 0.0
                    if ov > best_i:
                        best_i = ov
                det_crowded[i] = (crowd_dist_px > 0 and best_d <= crowd_dist_px) or (crowd_iou > 0 and best_i >= crowd_iou)
                det_overlapping[i] = (overlap_iou > 0 and best_i >= overlap_iou)

        # Global assignment: prefer higher overlap, then smaller distance.
        pairs = []  # (iou, distance, vehicle_id, det_idx)

        min_iou_crowd_match = float(getattr(config, "TRACK_MATCH_MIN_IOU_WHEN_CROWDED", 0.02) or 0.02)
        min_iou_overlap_match = float(getattr(config, "TRACK_MATCH_MIN_IOU_WHEN_OVERLAP", 0.06) or 0.06)
        min_iou_crowd_match = max(0.0, min(0.5, min_iou_crowd_match))
        min_iou_overlap_match = max(0.0, min(0.8, min_iou_overlap_match))
        no_iou_max_dist = float(getattr(config, "TRACK_MATCH_MAX_DIST_WITHOUT_IOU_PX", 10.0) or 10.0)
        no_iou_max_dist = max(0.0, min(80.0, no_iou_max_dist))

        for vehicle_id, vehicle in self.vehicles.items():
            for det_idx, det in enumerate(detections):
                lost = int(getattr(vehicle, "lost_frames", 0) or 0) > 1

                ref_center = vehicle.center
                if lost:
                    ref_center = vehicle.predict_center(frame_ts, max_dt=pred_dt)

                # Match gate: be conservative to avoid ID swaps / teleporting.
                try:
                    last_ts = float(vehicle.timestamps[-1]) if getattr(vehicle, "timestamps", None) else frame_ts
                except Exception:
                    last_ts = frame_ts
                dt = max(0.0, min(1.0, frame_ts - last_ts))
                speed = float(vehicle.get_speed())
                effective_max_dist = max_dist + (speed * dt * 0.8)
                effective_max_dist = min(effective_max_dist, max_dist * 1.5)
                if lost:
                    # When truly lost, require a much closer reacquire.
                    effective_max_dist = max(12.0, min(effective_max_dist, near_dist * lost_match_scale))

                distance = float(np.linalg.norm(np.array(ref_center) - np.array(det["center"])))
                if distance > effective_max_dist:
                    continue
                iou = _bbox_iou(vehicle.bbox, det["bbox"])

                is_crowded = bool(det_crowded[det_idx]) if det_idx < len(det_crowded) else False
                is_overlap = bool(det_overlapping[det_idx]) if det_idx < len(det_overlapping) else False

                if is_overlap and (min_iou_overlap_match > 0) and iou < min_iou_overlap_match and distance > no_iou_max_dist:
                    continue
                if (not is_overlap) and is_crowded and (min_iou_crowd_match > 0) and iou < min_iou_crowd_match and distance > no_iou_max_dist:
                    continue

                # Geometry plausibility gate (unless overlap is strong).
                try:
                    vx, vy, vw, vh = map(float, vehicle.bbox)
                    dx, dy, dw, dh = map(float, det["bbox"])
                    v_area = max(1.0, vw * vh)
                    d_area = max(1.0, dw * dh)
                    area_ratio = d_area / v_area
                    v_aspect = max(vw, vh) / max(1.0, min(vw, vh))
                    d_aspect = max(dw, dh) / max(1.0, min(dw, dh))
                    aspect_change = max(v_aspect, d_aspect) / max(1.0, min(v_aspect, d_aspect))
                except Exception:
                    area_ratio = 1.0
                    aspect_change = 1.0

                if iou < strong_iou:
                    if area_ratio < min_area_ratio or area_ratio > max_area_ratio:
                        continue
                    if aspect_change > max_aspect_ratio_change:
                        continue

                # If overlap is tiny, only allow match when extremely close.
                if min_iou > 0 and iou < min_iou and distance > near_dist:
                    continue

                if lost and min_iou > 0 and iou < min_iou and distance > (near_dist * 0.5):
                    # Extra strictness on reacquire to prevent snapping to another car.
                    continue
                pairs.append((iou, distance, vehicle_id, det_idx))

        pairs.sort(key=lambda t: (-t[0], t[1]))

        matched_vehicle_ids = set()
        used_det_idxs = set()

        for iou, distance, vehicle_id, det_idx in pairs:
            if vehicle_id in matched_vehicle_ids or det_idx in used_det_idxs:
                continue
            det = detections[det_idx]
            self.vehicles[vehicle_id].update(
                det["center"],
                det["bbox"],
                det["area"],
                crowded=bool(det_crowded[det_idx]) if det_idx < len(det_crowded) else False,
                overlapping=bool(det_overlapping[det_idx]) if det_idx < len(det_overlapping) else False,
            )
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
            # Only count vehicles observed in the current frame.
            if int(getattr(vehicle, "lost_frames", 0) or 0) != 0:
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

    def get_vehicles_in_intersection(self) -> List[Vehicle]:
        """Get vehicles in the middle intersection zone."""
        if not hasattr(config, "INTERSECTION_CENTER"):
            return []

        try:
            inter_region = np.array(config.INTERSECTION_CENTER, dtype=np.int32)
        except Exception:
            return []

        vehicles_in_zone: List[Vehicle] = []
        for vehicle in self.vehicles.values():
            if not getattr(vehicle, "confirmed", False):
                continue
            # Only count vehicles observed in the current frame.
            if int(getattr(vehicle, "lost_frames", 0) or 0) != 0:
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

            if any(cv2.pointPolygonTest(inter_region, p, False) >= 0 for p in pts):
                vehicles_in_zone.append(vehicle)

        return vehicles_in_zone
    
    def get_vehicle_count_by_lane(self, *, include_intersection: bool = False) -> Dict[str, int]:
        """Get vehicle count for each lane.

        Args:
            include_intersection: When True, also includes an "INTERSECTION" entry
                for the center-zone count.
        """
        counts = {lane: 0 for lane in config.LANES.keys()}

        for lane_key in config.LANES.keys():
            counts[lane_key] = len(self.get_vehicles_in_lane(lane_key))

        if include_intersection:
            counts["INTERSECTION"] = len(self.get_vehicles_in_intersection())

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
