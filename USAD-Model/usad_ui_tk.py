"""USAD Tkinter UI (Draft)

Two screens:
1) With the cameras
   - Camera tiles (click to fullscreen)
   - Overlay: USAD wordmark, Arduino status, FPS status
   - Counts: vehicles / violations / emergency calls
   - Control keys
   - Stoplights status (draft placeholder)
   - Accident detected alert overlay
   - Violation detected alert overlay

2) Log records
   - Overview dashboard totals
   - Per-lane records (North/East/West/South)
   - Accidents recorded graph (by hour)
   - Violations recorded graph (by hour)
   - Tabs: Accidents logs / Violations logs / Per lane records

Run:
  cd USAD-Model
  python usad_ui_tk.py

Notes:
- Reads CSV logs from either `USAD-Model/logs/` or `USAD/logs/`.
- Camera feed uses OpenCV if available; if the camera can't be opened, tiles show placeholders.
"""

from __future__ import annotations

import base64
import csv
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Tuple

try:
    import tkinter as tk
    from tkinter import ttk
except Exception as exc:  # pragma: no cover
    raise RuntimeError("Tkinter is required (it should ship with Python on Windows).") from exc

try:
    import cv2  # type: ignore
except Exception:
    cv2 = None  # type: ignore

try:
    import config
except Exception:
    config = None  # type: ignore


# Optional: import the existing real-time detectors.
try:
    from vehicle_detector import VehicleDetector, Vehicle
    from accident_detector import AccidentDetector, Accident
    from violation_detector import ViolationDetector, Violation
    from license_plate_detector import LicensePlateDetector
    from event_logger import EventLogger
    from emergency_notifier import EmergencyNotifier
except Exception:
    VehicleDetector = None  # type: ignore
    Vehicle = None  # type: ignore
    AccidentDetector = None  # type: ignore
    Accident = None  # type: ignore
    ViolationDetector = None  # type: ignore
    Violation = None  # type: ignore
    LicensePlateDetector = None  # type: ignore
    EventLogger = None  # type: ignore
    EmergencyNotifier = None  # type: ignore


def _dt_parts(ts: float) -> Tuple[str, str]:
    try:
        dt = datetime.fromtimestamp(float(ts))
        return dt.strftime("%Y-%m-%d"), dt.strftime("%H:%M:%S")
    except Exception:
        return "—", "—"


def _lane_key_to_display(lane_key: str) -> str:
    # lane_display_name expects LANE1..LANE4; accidents can also be INTERSECTION.
    lane_key = (lane_key or "").strip()
    if not lane_key:
        return "—"
    if lane_key.upper() == "INTERSECTION":
        return "Intersection"
    return lane_display_name(lane_key)


class RealtimeDetectorEngine:
    """Runs the existing detection pipeline continuously in a background thread.

    Produces:
    - latest annotated frame (BGR) and a small preview PNG for fast Tk display
    - live metrics (vehicle/violation/emergency counts)
    - signal state per lane (for stoplight status)
    - last confirmed accident + last violation for the sidebar overlays
    """

    def __init__(
        self,
        *,
        camera_source: int | str,
        base_size: Tuple[int, int],
        preview_size: Tuple[int, int] = (640, 360),
    ):
        self._camera_source = camera_source
        self._base_w, self._base_h = base_size
        self._preview_w, self._preview_h = preview_size

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._status: str = "Starting"
        self._last_error: Optional[str] = None

        self._preview_png: Optional[bytes] = None
        self._preview_w_actual: int = 0
        self._preview_h_actual: int = 0
        self._last_full_bgr = None  # numpy ndarray (OpenCV)

        self._vehicle_count: int = 0
        self._violation_count: int = 0
        self._emergency_calls: int = 0
        self._signals: Dict[str, str] = {}
        self._lane_counts: Dict[str, int] = {}

        self._last_confirmed_accident = None
        self._last_violation = None

        self._fps: float = 0.0
        self._fps_t0 = time.time()
        self._fps_frames = 0

        # Pipeline objects (created inside the thread).
        self._vehicle_detector = None
        self._accident_detector = None
        self._violation_detector = None
        self._license_plate_detector = None
        self._event_logger = None
        self._emergency_notifier = None

        # Simple signal simulation state (matches main.py behavior enough for UI/violations).
        self._software_auto_mode = bool(getattr(config, "AUTO_MODE_DEFAULT", True)) if config is not None else True
        self._current_active_lane: Optional[str] = None
        self._current_phase: str = "GREEN"
        self._phase_start_time: float = time.time()
        self._lane_green_duration: float = float(getattr(config, "GREEN_TIME", 30)) if config is not None else 30.0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="USAD-RealtimeEngine", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def get_status(self) -> str:
        with self._lock:
            return self._status

    def get_latest_preview_png(self) -> Tuple[Optional[bytes], int, int]:
        with self._lock:
            return self._preview_png, self._preview_w_actual, self._preview_h_actual

    def get_latest_full_bgr(self):
        with self._lock:
            return self._last_full_bgr

    def get_metrics(self) -> Tuple[int, int, int, float]:
        with self._lock:
            return self._vehicle_count, self._violation_count, self._emergency_calls, float(self._fps)

    def get_signals(self) -> Dict[str, str]:
        with self._lock:
            return dict(self._signals)

    def get_last_events(self):
        with self._lock:
            return self._last_confirmed_accident, self._last_violation

    def _lane_order(self) -> List[str]:
        if config is None or not hasattr(config, "LANES"):
            return ["LANE1", "LANE2", "LANE3", "LANE4"]
        try:
            return list(getattr(config, "LANES").keys())
        except Exception:
            return ["LANE1", "LANE2", "LANE3", "LANE4"]

    def _get_next_lane(self, lane_key: str) -> str:
        order = self._lane_order()
        if not order:
            return lane_key
        if lane_key not in order:
            return order[0]
        idx = order.index(lane_key)
        return order[(idx + 1) % len(order)]

    def _apply_signal_states(self, active_lane: str, phase: str) -> None:
        if self._violation_detector is None:
            return
        for lane in self._lane_order():
            if lane == active_lane:
                self._violation_detector.set_traffic_signal(lane, phase)
            else:
                self._violation_detector.set_traffic_signal(lane, "RED")

    def _update_signal_cycle(self) -> None:
        if self._violation_detector is None:
            return

        # Simulate signals in software (same default as main.py when Arduino isn't used).
        now = time.time()
        order = self._lane_order()
        if not order:
            return

        if self._current_active_lane is None:
            self._current_active_lane = order[0]
            self._current_phase = "GREEN"
            self._phase_start_time = now
            self._lane_green_duration = float(getattr(config, "GREEN_TIME", 30)) if config is not None else 30.0
            self._apply_signal_states(self._current_active_lane, "GREEN")
            return

        if not self._software_auto_mode:
            self._current_phase = "GREEN"
            self._apply_signal_states(self._current_active_lane, "GREEN")
            return

        if self._current_phase == "GREEN":
            if (now - self._phase_start_time) >= float(self._lane_green_duration):
                self._current_phase = "YELLOW"
                self._phase_start_time = now
                self._apply_signal_states(self._current_active_lane, "YELLOW")
        elif self._current_phase == "YELLOW":
            yellow = float(getattr(config, "YELLOW_TIME", 3)) if config is not None else 3.0
            if (now - self._phase_start_time) >= yellow:
                self._current_active_lane = self._get_next_lane(self._current_active_lane)
                self._current_phase = "GREEN"
                self._phase_start_time = now
                self._lane_green_duration = float(getattr(config, "GREEN_TIME", 30)) if config is not None else 30.0
                self._apply_signal_states(self._current_active_lane, "GREEN")

    def _draw_interface(self, frame_bgr, vehicles, accidents) -> None:
        if cv2 is None or config is None:
            return

        if bool(getattr(config, "SHOW_LANE_REGIONS", True)):
            for lane_key, lane_data in getattr(config, "LANES", {}).items():
                try:
                    region = lane_data["region"]
                    stop_line = lane_data["stop_line"]
                    name = str(lane_data.get("name", lane_key))
                except Exception:
                    continue

                try:
                    import numpy as np  # local import for thread safety / optional dependency
                except Exception:
                    np = None  # type: ignore
                if np is not None:
                    try:
                        reg = np.array(region, dtype=np.int32)
                        color = getattr(config, f"COLOR_{lane_key}", None)
                        if color is None:
                            # Fallback to COLOR_LANE1..4 mapping
                            color = getattr(config, {
                                "LANE1": "COLOR_LANE1",
                                "LANE2": "COLOR_LANE2",
                                "LANE3": "COLOR_LANE3",
                                "LANE4": "COLOR_LANE4",
                            }.get(str(lane_key).upper(), "COLOR_VEHICLE"), (255, 255, 255))

                        cv2.polylines(frame_bgr, [reg], True, color, 2)

                        center_x = int(np.mean([p[0] for p in reg]))
                        center_y = int(np.mean([p[1] for p in reg]))
                        cv2.putText(
                            frame_bgr,
                            name,
                            (center_x - 30, center_y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            color,
                            2,
                        )

                        if isinstance(stop_line, (list, tuple)) and len(stop_line) >= 2:
                            cv2.line(frame_bgr, tuple(stop_line[0]), tuple(stop_line[1]), (0, 255, 255), 3)
                    except Exception:
                        pass

        if hasattr(config, "INTERSECTION_CENTER"):
            try:
                import numpy as np
                intersection = np.array(getattr(config, "INTERSECTION_CENTER"), dtype=np.int32)
                cv2.polylines(frame_bgr, [intersection], True, (255, 0, 255), 2)
            except Exception:
                pass

        try:
            if self._vehicle_detector is not None:
                self._vehicle_detector.draw_vehicles(frame_bgr, vehicles)
        except Exception:
            pass
        try:
            if self._accident_detector is not None:
                self._accident_detector.draw_accidents(frame_bgr)
        except Exception:
            pass
        try:
            if bool(getattr(config, "SHOW_VIOLATIONS", True)) and self._violation_detector is not None:
                self._violation_detector.draw_violations(frame_bgr, recent_only=True)
        except Exception:
            pass

    def _run(self) -> None:
        if cv2 is None:
            with self._lock:
                self._status = "OpenCV not available"
            return

        if VehicleDetector is None or AccidentDetector is None or ViolationDetector is None:
            with self._lock:
                self._status = "Detectors not importable"
            return

        # Create pipeline objects.
        try:
            self._vehicle_detector = VehicleDetector()
            self._accident_detector = AccidentDetector()
            self._violation_detector = ViolationDetector()
            self._license_plate_detector = LicensePlateDetector() if LicensePlateDetector is not None else None
            self._event_logger = EventLogger() if EventLogger is not None else None
            self._emergency_notifier = EmergencyNotifier() if EmergencyNotifier is not None else None
        except Exception as exc:
            with self._lock:
                self._status = f"Init failed: {exc}"
            return

        # Open camera.
        source = self._camera_source
        backends = [
            ("DSHOW", getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY)),
            ("MSMF", getattr(cv2, "CAP_MSMF", cv2.CAP_ANY)),
            ("ANY", cv2.CAP_ANY),
        ]

        cap = None
        for name, backend in backends:
            try:
                c = cv2.VideoCapture(source, backend)
                if c is not None and c.isOpened():
                    cap = c
                    with self._lock:
                        self._status = f"Camera open ({name})"
                    break
            except Exception:
                continue

        if cap is None or not cap.isOpened():
            with self._lock:
                self._status = f"Could not open camera source: {source}"
            return

        try:
            if config is not None:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(getattr(config, "CAMERA_WIDTH", self._base_w)))
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(getattr(config, "CAMERA_HEIGHT", self._base_h)))
                cap.set(cv2.CAP_PROP_FPS, float(getattr(config, "CAMERA_FPS", 30)))
        except Exception:
            pass

        # Warm up.
        for _i in range(8):
            if self._stop.is_set():
                break
            try:
                ok, _ = cap.read()
                if not ok:
                    time.sleep(0.05)
            except Exception:
                time.sleep(0.05)

        # Main loop.
        while not self._stop.is_set():
            ok, frame_bgr = cap.read()
            if not ok or frame_bgr is None:
                time.sleep(0.02)
                continue

            vehicles = []
            accidents = []
            try:
                vehicles = self._vehicle_detector.detect_vehicles(frame_bgr)
            except Exception:
                vehicles = []

            try:
                self._update_signal_cycle()
            except Exception:
                pass

            try:
                lane_counts = self._vehicle_detector.get_vehicle_count_by_lane()
            except Exception:
                lane_counts = {}

            try:
                accidents = self._accident_detector.detect_accidents(vehicles, frame=frame_bgr)
                confirmed = self._accident_detector.get_confirmed_accidents()
            except Exception:
                accidents = []
                confirmed = []

            # Emergency notify + log confirmed accidents.
            try:
                if self._emergency_notifier is not None and self._event_logger is not None:
                    for acc in confirmed:
                        if not getattr(acc, "notified", False):
                            notified = bool(self._emergency_notifier.notify_accident(acc))
                            if notified:
                                self._event_logger.log_accident(acc, notified=True)
            except Exception:
                pass

            # Violations + log.
            try:
                new_violations = self._violation_detector.detect_violations(vehicles)
                if self._event_logger is not None:
                    for vio in new_violations:
                        self._event_logger.log_violation(vio)
            except Exception:
                new_violations = []

            # License plate.
            try:
                if config is not None and bool(getattr(config, "ENABLE_LICENSE_PLATE_DETECTION", False)):
                    if self._license_plate_detector is not None:
                        for v in vehicles:
                            if getattr(v, "license_plate", None):
                                continue
                            res = self._license_plate_detector.detect_license_plate(frame_bgr, v.bbox)
                            if res:
                                plate_text, confidence, _plate_bbox = res
                                v.license_plate = plate_text
                                v.license_plate_confidence = confidence
            except Exception:
                pass

            # Draw the same interface visuals your pipeline already provides.
            try:
                self._draw_interface(frame_bgr, vehicles, accidents)
            except Exception:
                pass

            # FPS calc.
            self._fps_frames += 1
            now = time.time()
            if (now - self._fps_t0) >= 1.0:
                self._fps = self._fps_frames / max(0.001, (now - self._fps_t0))
                self._fps_frames = 0
                self._fps_t0 = now

            # Build preview PNG.
            try:
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                full_rgb = frame_rgb
                # Preview for tiles.
                preview_rgb = full_rgb
                try:
                    preview_rgb = cv2.resize(full_rgb, (int(self._preview_w), int(self._preview_h)))
                except Exception:
                    pass
                ok2, buf = cv2.imencode(".png", preview_rgb)
                preview_png = bytes(buf) if ok2 else None
            except Exception:
                preview_png = None
                full_rgb = None

            # Update shared state.
            try:
                vehicle_count = 0
                try:
                    vehicle_count = len({int(v.id) for v in vehicles})
                except Exception:
                    vehicle_count = len(vehicles)

                violation_count = 0
                try:
                    violation_count = len(getattr(self._violation_detector, "violations", []))
                except Exception:
                    violation_count = 0

                emergency_calls = 0
                try:
                    emergency_calls = sum(1 for a in confirmed if bool(getattr(a, "notified", False)))
                except Exception:
                    emergency_calls = 0

                last_confirmed = None
                if confirmed:
                    try:
                        last_confirmed = max(confirmed, key=lambda a: float(getattr(a, "detected_time", 0.0)))
                    except Exception:
                        last_confirmed = confirmed[-1]

                last_vio = None
                try:
                    all_v = list(getattr(self._violation_detector, "violations", []))
                    if all_v:
                        last_vio = max(all_v, key=lambda v: float(getattr(v, "timestamp", 0.0)))
                except Exception:
                    last_vio = None

                signals = {}
                try:
                    signals = dict(getattr(self._violation_detector, "current_signals", {}))
                except Exception:
                    signals = {}

                with self._lock:
                    self._preview_png = preview_png
                    self._preview_w_actual = int(self._preview_w)
                    self._preview_h_actual = int(self._preview_h)
                    try:
                        self._last_full_bgr = frame_bgr.copy()
                    except Exception:
                        self._last_full_bgr = frame_bgr

                    self._vehicle_count = int(vehicle_count)
                    self._violation_count = int(violation_count)
                    self._emergency_calls = int(emergency_calls)
                    self._signals = signals
                    self._lane_counts = dict(lane_counts)
                    self._last_confirmed_accident = last_confirmed
                    self._last_violation = last_vio
                    self._status = "Running"
            except Exception:
                pass

        try:
            cap.release()
        except Exception:
            pass
        with self._lock:
            self._status = "Stopped"


LANE_NAME_BY_KEY_FALLBACK = {
    "LANE1": "North",
    "LANE2": "South",
    "LANE3": "East",
    "LANE4": "West",
}


def lane_display_name(lane_key: str) -> str:
    lane_key = (lane_key or "").strip()
    if not lane_key:
        return "—"

    if config is not None and hasattr(config, "LANES"):
        lane_cfg = getattr(config, "LANES", {}).get(lane_key)
        if isinstance(lane_cfg, dict) and lane_cfg.get("name"):
            return str(lane_cfg["name"]).strip()

    return LANE_NAME_BY_KEY_FALLBACK.get(lane_key, lane_key)


def parse_hour_from_time_str(value: str) -> Optional[int]:
    if not value:
        return None
    try:
        parts = value.strip().split(":")
        hour = int(parts[0])
        if 0 <= hour <= 23:
            return hour
    except Exception:
        return None
    return None


def detect_logs_dir() -> Path:
    here = Path(__file__).resolve().parent
    candidates = [
        here / "logs",
        here.parent / "logs",
    ]

    for c in candidates:
        if (c / "accidents.csv").exists() and (c / "violations.csv").exists() and (c / "traffic_events.csv").exists():
            return c

    # Best-effort fallback (even if logs don't exist yet).
    return candidates[0]


def read_csv_dicts(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []

    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        return [
            {k: (v if v is not None else "") for k, v in row.items()}
            for row in reader
        ]


def unique_nonempty(values: Iterable[str]) -> int:
    seen = set()
    for v in values:
        v = (v or "").strip()
        if v:
            seen.add(v)
    return len(seen)


def counts_by_hour(rows: List[Dict[str, str]], time_field: str) -> Dict[int, int]:
    out: Dict[int, int] = {}
    for r in rows:
        hour = parse_hour_from_time_str(r.get(time_field, ""))
        if hour is None:
            continue
        out[hour] = out.get(hour, 0) + 1
    return out


def last_row(rows: List[Dict[str, str]]) -> Optional[Dict[str, str]]:
    return rows[-1] if rows else None


@dataclass
class CameraOverlay:
    kind: str  # "accident" | "violation"
    x: int
    y: int
    label: str


@dataclass
class LanePolyline:
    points: List[Tuple[int, int]]
    color: str
    width: int
    dash: Optional[Tuple[int, int]] = None


@dataclass
class LaneLabel:
    x: int
    y: int
    text: str
    color: str


def _parse_points(value: object) -> List[Tuple[int, int]]:
    if not isinstance(value, (list, tuple)):
        return []

    out: List[Tuple[int, int]] = []
    for item in value:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            continue
        try:
            x = int(float(item[0]))
            y = int(float(item[1]))
        except Exception:
            continue
        out.append((x, y))
    return out


def _bgr_to_hex(value: object, default: str) -> str:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return default
    try:
        b = int(value[0])
        g = int(value[1])
        r = int(value[2])
    except Exception:
        return default
    r = max(0, min(255, r))
    g = max(0, min(255, g))
    b = max(0, min(255, b))
    return f"#{r:02x}{g:02x}{b:02x}"


def _lane_color_hex(lane_key: str) -> str:
    if config is None:
        return "white"

    color_attr = {
        "LANE1": "COLOR_LANE1",
        "LANE2": "COLOR_LANE2",
        "LANE3": "COLOR_LANE3",
        "LANE4": "COLOR_LANE4",
    }.get((lane_key or "").strip().upper())

    if not color_attr:
        return "white"

    return _bgr_to_hex(getattr(config, color_attr, None), "white")


def lane_polylines_from_config() -> List[LanePolyline]:
    """Returns guide polylines in base camera coordinates (e.g. 1280x720)."""
    if config is None or not hasattr(config, "LANES"):
        return []

    if hasattr(config, "SHOW_LANE_REGIONS") and not bool(getattr(config, "SHOW_LANE_REGIONS")):
        return []

    lanes = getattr(config, "LANES", {})
    if not isinstance(lanes, dict):
        return []

    polylines: List[LanePolyline] = []
    for lane_key, lane_cfg in lanes.items():
        if not isinstance(lane_cfg, dict):
            continue

        lane_color = _lane_color_hex(str(lane_key))

        region = _parse_points(lane_cfg.get("region"))
        if len(region) >= 3:
            # Close the polygon.
            region_closed = list(region) + [region[0]]
            polylines.append(LanePolyline(points=region_closed, color=lane_color, width=2))

        stop_line = _parse_points(lane_cfg.get("stop_line"))
        if len(stop_line) >= 2:
            # Match main.py: cv2.line(..., (0,255,255), 3)
            polylines.append(LanePolyline(points=stop_line[:2], color=_bgr_to_hex((0, 255, 255), "yellow"), width=3))

    # Match main.py: intersection outline magenta (255,0,255) in BGR
    if hasattr(config, "INTERSECTION_CENTER"):
        intersection = _parse_points(getattr(config, "INTERSECTION_CENTER"))
        if len(intersection) >= 3:
            intersection_closed = list(intersection) + [intersection[0]]
            polylines.append(
                LanePolyline(
                    points=intersection_closed,
                    color=_bgr_to_hex((255, 0, 255), "magenta"),
                    width=2,
                    dash=(2, 2),
                )
            )

    return polylines


def lane_labels_from_config() -> List[LaneLabel]:
    if config is None or not hasattr(config, "LANES"):
        return []

    if hasattr(config, "SHOW_LANE_REGIONS") and not bool(getattr(config, "SHOW_LANE_REGIONS")):
        return []

    lanes = getattr(config, "LANES", {})
    if not isinstance(lanes, dict):
        return []

    labels: List[LaneLabel] = []
    for lane_key, lane_cfg in lanes.items():
        if not isinstance(lane_cfg, dict):
            continue
        name = str(lane_cfg.get("name") or lane_key).strip()
        region = _parse_points(lane_cfg.get("region"))
        if len(region) < 3:
            continue

        cx = int(sum(p[0] for p in region) / len(region))
        cy = int(sum(p[1] for p in region) / len(region))
        labels.append(LaneLabel(x=cx, y=cy, text=name, color=_lane_color_hex(str(lane_key))))

    return labels


@dataclass
class LogsSnapshot:
    accidents: List[Dict[str, str]]
    violations: List[Dict[str, str]]
    traffic_events: List[Dict[str, str]]


def load_logs() -> Tuple[Path, LogsSnapshot]:
    logs_dir = detect_logs_dir()
    accidents = read_csv_dicts(logs_dir / "accidents.csv")
    violations = read_csv_dicts(logs_dir / "violations.csv")
    traffic_events = read_csv_dicts(logs_dir / "traffic_events.csv")
    return logs_dir, LogsSnapshot(accidents=accidents, violations=violations, traffic_events=traffic_events)


class BarChart(ttk.Frame):
    def __init__(self, master: tk.Misc, title: str, width: int = 900, height: int = 220):
        super().__init__(master)
        self._title = title
        self._canvas = tk.Canvas(self, width=width, height=height, highlightthickness=1)
        self._canvas.pack(fill="both", expand=True)

    def set_data(self, counts: Dict[int, int]) -> None:
        c = self._canvas
        c.delete("all")

        width = int(c.winfo_reqwidth())
        height = int(c.winfo_reqheight())

        pad_left, pad_right, pad_top, pad_bottom = 40, 10, 28, 28
        plot_w = max(1, width - pad_left - pad_right)
        plot_h = max(1, height - pad_top - pad_bottom)

        c.create_text(10, 14, anchor="w", text=self._title)

        # Axes
        x0, y0 = pad_left, pad_top
        x1, y1 = pad_left, pad_top + plot_h
        c.create_line(x0, y0, x1, y1)
        c.create_line(x1, y1, x1 + plot_w, y1)

        max_val = max(1, *(counts.get(h, 0) for h in range(24)))
        bar_w = plot_w / 24.0

        for h in range(24):
            val = counts.get(h, 0)
            bar_h = int((val / max_val) * (plot_h - 10))
            bx0 = int(pad_left + h * bar_w + 2)
            bx1 = int(pad_left + (h + 1) * bar_w - 2)
            by1 = y1
            by0 = y1 - bar_h

            c.create_rectangle(bx0, by0, bx1, by1, outline="black")
            if h % 3 == 0:
                c.create_text(int(pad_left + h * bar_w + 6), y1 + 14, anchor="w", text=str(h), font=("Segoe UI", 8))

        c.create_text(10, pad_top + 4, anchor="w", text=str(max_val), font=("Segoe UI", 8))


class CameraFeed:
    def __init__(self, source: int | str):
        self._source = source
        self._cap = None
        self._listeners: List[Callable[[Optional[bytes], int, int], None]] = []
        self._last_frame_png: Optional[bytes] = None
        self._last_full_bgr = None
        self._w = 0
        self._h = 0

        # Optional external engine (real-time detection). If set, CameraFeed won't own the camera.
        self._external_engine: Optional[RealtimeDetectorEngine] = None

    def add_listener(self, listener: Callable[[Optional[bytes], int, int], None]) -> None:
        self._listeners.append(listener)

    def attach_external_engine(self, engine: RealtimeDetectorEngine) -> None:
        self._external_engine = engine

    def start(self) -> None:
        # External engine owns the camera.
        if self._external_engine is not None:
            self._notify(None, 0, 0)
            return

        if cv2 is None:
            self._notify(None, 0, 0)
            return

        self._cap = cv2.VideoCapture(self._source)
        if config is not None:
            try:
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, getattr(config, "CAMERA_WIDTH", 1280))
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, getattr(config, "CAMERA_HEIGHT", 720))
                self._cap.set(cv2.CAP_PROP_FPS, getattr(config, "CAMERA_FPS", 30))
            except Exception:
                pass

        if not self._cap.isOpened():
            self._notify(None, 0, 0)
            return

    def stop(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def tick(self) -> None:
        # If an external engine is attached, just forward its latest preview.
        if self._external_engine is not None:
            png_bytes, w, h = self._external_engine.get_latest_preview_png()
            self._last_frame_png = png_bytes
            self._w = int(w)
            self._h = int(h)
            self._last_full_bgr = self._external_engine.get_latest_full_bgr()
            self._notify(png_bytes, int(w), int(h))
            return

        if self._cap is None:
            return

        ok, frame = self._cap.read()
        if not ok or frame is None:
            self._notify(None, 0, 0)
            return

        # Downscale for UI responsiveness
        try:
            frame = cv2.resize(frame, (640, 360))
        except Exception:
            pass

        # Convert BGR -> RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._h, self._w = frame.shape[:2]

        ok2, buf = cv2.imencode(".png", frame)
        if not ok2:
            self._notify(None, 0, 0)
            return

        self._last_frame_png = bytes(buf)
        self._notify(self._last_frame_png, self._w, self._h)

    def _notify(self, png_bytes: Optional[bytes], w: int, h: int) -> None:
        for cb in list(self._listeners):
            cb(png_bytes, w, h)


class CameraTile(ttk.Frame):
    def __init__(
        self,
        master: tk.Misc,
        title: str,
        feed: CameraFeed,
        on_fullscreen: Callable[[], None],
        get_overlays: Callable[[], List[CameraOverlay]],
        base_size: Tuple[int, int],
        show_lane_guides: Callable[[], bool],
    ):
        super().__init__(master)
        self._title = title
        self._on_fullscreen = on_fullscreen
        self._get_overlays = get_overlays
        self._base_w, self._base_h = base_size
        self._show_lane_guides = show_lane_guides
        self._img = None
        self._image_item = None

        self._canvas = tk.Canvas(self, highlightthickness=1)
        self._canvas.pack(fill="both", expand=True)
        self._canvas.bind("<Button-1>", lambda _e: self._on_fullscreen())

        feed.add_listener(self._on_frame)

    def _draw_overlays(self, frame_w: int, frame_h: int) -> None:
        c = self._canvas
        c.delete("overlay")

        overlays = self._get_overlays()
        if not overlays:
            return

        base_w = max(1, int(self._base_w or 1280))
        base_h = max(1, int(self._base_h or 720))
        sx = frame_w / base_w
        sy = frame_h / base_h

        for ov in overlays:
            x = int(ov.x * sx)
            y = int(ov.y * sy)
            x = max(0, min(frame_w - 1, x))
            y = max(0, min(frame_h - 1, y))

            if ov.kind == "accident":
                color = "red"
            else:
                color = "orange"

            r = 10
            c.create_oval(x - r, y - r, x + r, y + r, outline=color, width=3, tags=("overlay",))
            c.create_line(x - 14, y, x + 14, y, fill=color, width=2, tags=("overlay",))
            c.create_line(x, y - 14, x, y + 14, fill=color, width=2, tags=("overlay",))

            # Label box near marker
            text_id = c.create_text(
                x + 16,
                y - 16,
                anchor="nw",
                text=ov.label,
                fill="black",
                font=("Segoe UI", 9, "bold"),
                tags=("overlay",),
            )
            bbox = c.bbox(text_id)
            if bbox:
                pad = 3
                rect_id = c.create_rectangle(
                    bbox[0] - pad,
                    bbox[1] - pad,
                    bbox[2] + pad,
                    bbox[3] + pad,
                    outline=color,
                    fill="white",
                    width=2,
                    tags=("overlay",),
                )
                c.tag_raise(text_id, rect_id)

    def _draw_lane_polylines(self, frame_w: int, frame_h: int) -> None:
        c = self._canvas
        c.delete("lane")

        polylines = lane_polylines_from_config()
        labels = lane_labels_from_config()
        if not polylines and not labels:
            return

        base_w = max(1, int(self._base_w or 1280))
        base_h = max(1, int(self._base_h or 720))
        sx = frame_w / base_w
        sy = frame_h / base_h

        for pl in polylines:
            if len(pl.points) < 2:
                continue
            coords: List[int] = []
            for x0, y0 in pl.points:
                x = int(x0 * sx)
                y = int(y0 * sy)
                x = max(0, min(frame_w - 1, x))
                y = max(0, min(frame_h - 1, y))
                coords.extend([x, y])

            if len(coords) >= 4:
                c.create_line(
                    *coords,
                    fill=pl.color,
                    width=pl.width,
                    dash=pl.dash,
                    tags=("lane",),
                )

        for lab in labels:
            x = int(lab.x * sx)
            y = int(lab.y * sy)
            x = max(0, min(frame_w - 1, x))
            y = max(0, min(frame_h - 1, y))
            c.create_text(
                x,
                y,
                anchor="center",
                text=lab.text,
                fill=lab.color,
                font=("Segoe UI", 10, "bold"),
                tags=("lane",),
            )

    def _on_frame(self, png_bytes: Optional[bytes], frame_w: int, frame_h: int) -> None:
        c = self._canvas

        if png_bytes is None:
            c.delete("all")
            c.create_text(10, 10, anchor="nw", text=f"{self._title}\n(No camera)")
            return

        data = base64.b64encode(png_bytes).decode("ascii")
        img = tk.PhotoImage(data=data)
        self._img = img

        if self._image_item is None:
            c.delete("all")
            self._image_item = c.create_image(0, 0, anchor="nw", image=img)
        else:
            c.itemconfigure(self._image_item, image=img)

        # Ensure canvas scroll region matches frame
        c.config(width=frame_w, height=frame_h)
        c.configure(scrollregion=(0, 0, frame_w, frame_h))

        # If the frame is already annotated by the real-time engine, skip duplicating lane guides.
        if self._show_lane_guides():
            self._draw_lane_polylines(frame_w, frame_h)
        self._draw_overlays(frame_w, frame_h)


class FullscreenViewer(tk.Toplevel):
    def __init__(
        self,
        master: tk.Misc,
        title: str,
        feed: CameraFeed,
        get_overlays: Callable[[], List[CameraOverlay]],
        base_size: Tuple[int, int],
        show_lane_guides: Callable[[], bool],
    ):
        super().__init__(master)
        self.title(title)
        self._img = None

        self._get_overlays = get_overlays
        self._base_w, self._base_h = base_size
        self._feed = feed
        self._show_lane_guides = show_lane_guides

        self._img_off_x = 0
        self._img_off_y = 0
        self._img_w = 1
        self._img_h = 1

        self.configure(bg="black")
        self._canvas = tk.Canvas(self, bg="black", highlightthickness=0)
        self._canvas.pack(fill="both", expand=True)
        self._image_item = None

        self.attributes("-fullscreen", True)
        self.bind("<Escape>", lambda _e: self.destroy())
        self.bind("<Button-1>", lambda _e: self.destroy())

        feed.add_listener(self._on_frame)

    def _on_frame(self, png_bytes: Optional[bytes], _w: int, _h: int) -> None:
        # Prefer rendering from the full-size frame and scale it to fill the screen.
        bgr = getattr(self._feed, "_last_full_bgr", None)
        if cv2 is not None and bgr is not None:
            try:
                # Fit-to-screen while keeping aspect ratio.
                c = self._canvas
                c.update_idletasks()
                target_w = max(1, int(c.winfo_width()))
                target_h = max(1, int(c.winfo_height()))

                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                fh, fw = rgb.shape[:2]
                scale = min(target_w / max(1, fw), target_h / max(1, fh))
                new_w = max(1, int(fw * scale))
                new_h = max(1, int(fh * scale))
                resized = cv2.resize(rgb, (new_w, new_h))

                ok2, buf = cv2.imencode(".png", resized)
                if ok2:
                    data = base64.b64encode(bytes(buf)).decode("ascii")
                    img = tk.PhotoImage(data=data)
                    self._img = img

                    if self._image_item is None:
                        c.delete("all")
                        self._image_item = c.create_image(0, 0, anchor="nw", image=img)
                    else:
                        c.itemconfigure(self._image_item, image=img)

                    # Center the image.
                    off_x = int((target_w - new_w) / 2)
                    off_y = int((target_h - new_h) / 2)
                    c.coords(self._image_item, off_x, off_y)
                    c.configure(scrollregion=(0, 0, target_w, target_h))

                    self._img_off_x = off_x
                    self._img_off_y = off_y
                    self._img_w = new_w
                    self._img_h = new_h

                    # Overlays are drawn in the resized image coordinate space.
                    if self._show_lane_guides():
                        self._draw_lane_polylines(new_w, new_h)
                    self._draw_overlays(new_w, new_h)
                    return
            except Exception:
                pass

        # Fallback to the preview PNG.
        if png_bytes is None:
            return
        data = base64.b64encode(png_bytes).decode("ascii")
        img = tk.PhotoImage(data=data)
        self._img = img
        c = self._canvas
        if self._image_item is None:
            c.delete("all")
            self._image_item = c.create_image(0, 0, anchor="nw", image=img)
        else:
            c.itemconfigure(self._image_item, image=img)
        frame_w = img.width()
        frame_h = img.height()
        self._img_off_x = 0
        self._img_off_y = 0
        self._img_w = frame_w
        self._img_h = frame_h
        c.configure(scrollregion=(0, 0, frame_w, frame_h))
        if self._show_lane_guides():
            self._draw_lane_polylines(frame_w, frame_h)
        self._draw_overlays(frame_w, frame_h)

    def _draw_overlays(self, frame_w: int, frame_h: int) -> None:
        c = self._canvas
        c.delete("overlay")

        overlays = self._get_overlays()
        if not overlays:
            return

        base_w = max(1, int(self._base_w or 1280))
        base_h = max(1, int(self._base_h or 720))
        sx = frame_w / base_w
        sy = frame_h / base_h

        off_x = int(getattr(self, "_img_off_x", 0) or 0)
        off_y = int(getattr(self, "_img_off_y", 0) or 0)

        for ov in overlays:
            x = int(ov.x * sx)
            y = int(ov.y * sy)
            x = max(0, min(frame_w - 1, x))
            y = max(0, min(frame_h - 1, y))
            x += off_x
            y += off_y

            if ov.kind == "accident":
                color = "red"
            else:
                color = "orange"

            r = 14
            c.create_oval(x - r, y - r, x + r, y + r, outline=color, width=4, tags=("overlay",))
            c.create_line(x - 18, y, x + 18, y, fill=color, width=3, tags=("overlay",))
            c.create_line(x, y - 18, x, y + 18, fill=color, width=3, tags=("overlay",))

            text_id = c.create_text(
                x + 22,
                y - 22,
                anchor="nw",
                text=ov.label,
                fill="white",
                font=("Segoe UI", 12, "bold"),
                tags=("overlay",),
            )
            bbox = c.bbox(text_id)
            if bbox:
                pad = 6
                rect_id = c.create_rectangle(
                    bbox[0] - pad,
                    bbox[1] - pad,
                    bbox[2] + pad,
                    bbox[3] + pad,
                    outline=color,
                    fill="black",
                    width=3,
                    tags=("overlay",),
                )
                c.tag_raise(text_id, rect_id)

    def _draw_lane_polylines(self, frame_w: int, frame_h: int) -> None:
        c = self._canvas
        c.delete("lane")

        polylines = lane_polylines_from_config()
        labels = lane_labels_from_config()
        if not polylines and not labels:
            return

        base_w = max(1, int(self._base_w or 1280))
        base_h = max(1, int(self._base_h or 720))
        sx = frame_w / base_w
        sy = frame_h / base_h

        off_x = int(getattr(self, "_img_off_x", 0) or 0)
        off_y = int(getattr(self, "_img_off_y", 0) or 0)

        for pl in polylines:
            if len(pl.points) < 2:
                continue
            coords: List[int] = []
            for x0, y0 in pl.points:
                x = int(x0 * sx)
                y = int(y0 * sy)
                x = max(0, min(frame_w - 1, x))
                y = max(0, min(frame_h - 1, y))
                coords.extend([x + off_x, y + off_y])

            if len(coords) >= 4:
                c.create_line(
                    *coords,
                    fill=pl.color,
                    width=pl.width + 1,
                    dash=pl.dash,
                    tags=("lane",),
                )

        for lab in labels:
            x = int(lab.x * sx)
            y = int(lab.y * sy)
            x = max(0, min(frame_w - 1, x))
            y = max(0, min(frame_h - 1, y))
            c.create_text(
                x + off_x,
                y + off_y,
                anchor="center",
                text=lab.text,
                fill=lab.color,
                font=("Segoe UI", 16, "bold"),
                tags=("lane",),
            )


class USADTkApp(ttk.Frame):
    def __init__(self, master: tk.Tk):
        super().__init__(master)
        self.master = master

        master.title("USAD UI (Tkinter Draft)")
        master.geometry("1200x720")

        self._route = tk.StringVar(value="cameras")
        self._logs_dir: Optional[Path] = None
        self._snapshot = LogsSnapshot(accidents=[], violations=[], traffic_events=[])

        self._base_cam_w = 1280
        self._base_cam_h = 720
        if config is not None:
            self._base_cam_w = int(getattr(config, "CAMERA_WIDTH", 1280) or 1280)
            self._base_cam_h = int(getattr(config, "CAMERA_HEIGHT", 720) or 720)

        self._camera_overlays: List[CameraOverlay] = []
        self._show_accident_marker = True
        self._show_violation_marker = True

        self._last_accident_id_seen: Optional[str] = None
        self._last_violation_id_seen: Optional[str] = None

        self._engine: Optional[RealtimeDetectorEngine] = None

        # Camera feed (single source shared by both tiles)
        cam_source = 0
        if config is not None and hasattr(config, "CAMERA_SOURCE"):
            cam_source = getattr(config, "CAMERA_SOURCE")

        self._feed = CameraFeed(cam_source)

        # If we can import the existing detection modules, run them live and feed annotated
        # frames into Tk. Otherwise fall back to raw camera preview + Tk overlays.
        if cv2 is not None and VehicleDetector is not None and AccidentDetector is not None and ViolationDetector is not None:
            try:
                self._engine = RealtimeDetectorEngine(
                    camera_source=cam_source,
                    base_size=(self._base_cam_w, self._base_cam_h),
                    preview_size=(640, 360),
                )
                self._engine.start()
                self._feed.attach_external_engine(self._engine)
            except Exception:
                self._engine = None

        self._feed.start()

        try:
            self.master.protocol("WM_DELETE_WINDOW", self._on_close)
        except Exception:
            pass

        self._build_header()
        self._build_body()
        self.pack(fill="both", expand=True)

        self._wire_routing()
        self._refresh_logs()

        self._tick_camera()

    def _on_close(self) -> None:
        try:
            if self._engine is not None:
                self._engine.stop()
        except Exception:
            pass
        try:
            self.master.destroy()
        except Exception:
            pass

    def _build_header(self) -> None:
        header = ttk.Frame(self)
        header.pack(fill="x", padx=10, pady=10)

        left = ttk.Frame(header)
        left.pack(side="left", fill="x", expand=True)

        ttk.Label(left, text="USAD", font=("Segoe UI", 16, "bold")).pack(side="left")

        nav = ttk.Frame(left)
        nav.pack(side="left", padx=12)

        self._btn_cameras = ttk.Button(nav, text="Screen 1: Cameras", command=lambda: self._route.set("cameras"))
        self._btn_logs = ttk.Button(nav, text="Screen 2: Log records", command=lambda: self._route.set("logs"))
        self._btn_cameras.pack(side="left", padx=4)
        self._btn_logs.pack(side="left", padx=4)

        right = ttk.Frame(header)
        right.pack(side="right")

        self._arduino_status = tk.StringVar(value="—")
        self._fps_status = tk.StringVar(value="—")

        ttk.Label(right, text="Arduino:").grid(row=0, column=0, sticky="e")
        ttk.Label(right, textvariable=self._arduino_status).grid(row=0, column=1, sticky="w", padx=(4, 12))
        ttk.Label(right, text="FPS:").grid(row=0, column=2, sticky="e")
        ttk.Label(right, textvariable=self._fps_status).grid(row=0, column=3, sticky="w", padx=(4, 0))

        # Static status values from config
        if config is not None:
            fps = getattr(config, "CAMERA_FPS", "—")
            self._fps_status.set(str(fps))

            if getattr(config, "SIMULATE_SIGNALS_WHEN_NO_ARDUINO", False):
                self._arduino_status.set("Simulated")
            else:
                port = getattr(config, "ARDUINO_PORT", "—")
                self._arduino_status.set(str(port))

    def _build_body(self) -> None:
        self._content = ttk.Frame(self)
        self._content.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self._screen_cameras = ttk.Frame(self._content)
        self._screen_logs = ttk.Frame(self._content)

        for s in (self._screen_cameras, self._screen_logs):
            s.place(relx=0, rely=0, relwidth=1, relheight=1)

        self._build_screen_cameras(self._screen_cameras)
        self._build_screen_logs(self._screen_logs)

    def _build_screen_cameras(self, root: ttk.Frame) -> None:
        grid = ttk.Frame(root)
        grid.pack(fill="both", expand=True)
        grid.columnconfigure(0, weight=3)
        grid.columnconfigure(1, weight=1)
        grid.rowconfigure(0, weight=1)

        left = ttk.Frame(grid)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        right = ttk.Frame(grid)
        right.grid(row=0, column=1, sticky="nsew")

        # Cameras panel
        cam_panel = ttk.Labelframe(left, text="Dashboard with the cameras")
        cam_panel.pack(fill="both", expand=True)

        tiles = ttk.Frame(cam_panel)
        tiles.pack(fill="x", padx=8, pady=8)
        tiles.columnconfigure(0, weight=1)
        tiles.columnconfigure(1, weight=1)

        def get_overlays() -> List[CameraOverlay]:
            return list(self._camera_overlays)

        def open_fullscreen():
            FullscreenViewer(
                self.master,
                "USAD Camera",
                self._feed,
                get_overlays=get_overlays,
                base_size=(self._base_cam_w, self._base_cam_h),
                show_lane_guides=lambda: self._engine is None,
            )

        tile1 = CameraTile(
            tiles,
            "Camera 1 (click for full screen)",
            self._feed,
            on_fullscreen=open_fullscreen,
            get_overlays=get_overlays,
            base_size=(self._base_cam_w, self._base_cam_h),
            show_lane_guides=lambda: self._engine is None,
        )
        tile2 = CameraTile(
            tiles,
            "Camera 2 (click for full screen)",
            self._feed,
            on_fullscreen=open_fullscreen,
            get_overlays=get_overlays,
            base_size=(self._base_cam_w, self._base_cam_h),
            show_lane_guides=lambda: self._engine is None,
        )
        tile1.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        tile2.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        # Overlay metrics
        metrics = ttk.Frame(cam_panel)
        metrics.pack(fill="x", padx=8, pady=(0, 8))
        metrics.columnconfigure((0, 1, 2), weight=1)

        self._vehicles_detected = tk.StringVar(value="—")
        self._violations_detected = tk.StringVar(value="—")
        self._emergency_calls_detected = tk.StringVar(value="—")

        self._metric(metrics, 0, "No. of vehicles detected", self._vehicles_detected)
        self._metric(metrics, 1, "No. of violations detected", self._violations_detected)
        self._metric(metrics, 2, "No. of emergency calls detected", self._emergency_calls_detected)

        # Stoplights status
        lights = ttk.Labelframe(cam_panel, text="Stoplights status")
        lights.pack(fill="x", padx=8, pady=(0, 8))

        self._light_vars = {
            "North": tk.StringVar(value="—"),
            "South": tk.StringVar(value="—"),
            "East": tk.StringVar(value="—"),
            "West": tk.StringVar(value="—"),
        }
        row = 0
        for name in ("North", "South", "East", "West"):
            ttk.Label(lights, text=f"{name}:").grid(row=row, column=0, sticky="w", padx=8, pady=2)
            ttk.Label(lights, textvariable=self._light_vars[name]).grid(row=row, column=1, sticky="w", padx=8, pady=2)
            row += 1

        # Control keys
        keys = ttk.Labelframe(cam_panel, text="Control keys")
        keys.pack(fill="x", padx=8, pady=(0, 8))

        key_lines = [
            "Q / ESC  Quit",
            "R        Reset system",
            "A        Auto cycling mode",
            "1-4      Activate lanes",
            "S        Print statistics / generate report",
        ]
        for i, line in enumerate(key_lines):
            ttk.Label(keys, text=line).grid(row=i, column=0, sticky="w", padx=8, pady=1)

        # Right overlays
        self._accident_overlay = self._build_accident_overlay(right)
        self._violation_overlay = self._build_violation_overlay(right)

        btn_refresh = ttk.Button(right, text="Refresh logs", command=self._refresh_logs)
        btn_refresh.pack(fill="x", pady=(10, 0))

    def _build_accident_overlay(self, parent: ttk.Frame) -> ttk.Labelframe:
        box = ttk.Labelframe(parent, text="Accident Detected Alert Overlay")
        box.pack(fill="x", pady=(0, 10))

        self._acc_fields = {
            "ID": tk.StringVar(value="—"),
            "Date of Detection": tk.StringVar(value="—"),
            "Time of Detection": tk.StringVar(value="—"),
            "Detected lane": tk.StringVar(value="—"),
            "Report Status": tk.StringVar(value="—"),
        }

        r = 0
        for label, var in self._acc_fields.items():
            ttk.Label(box, text=f"{label}:").grid(row=r, column=0, sticky="w", padx=8, pady=2)
            ttk.Label(box, textvariable=var).grid(row=r, column=1, sticky="w", padx=8, pady=2)
            r += 1

        def dismiss() -> None:
            self._show_accident_marker = False
            self._recompute_camera_overlays()
            box.pack_forget()

        ttk.Button(box, text="Dismiss", command=dismiss).grid(row=r, column=0, columnspan=2, sticky="ew", padx=8, pady=6)
        return box

    def _build_violation_overlay(self, parent: ttk.Frame) -> ttk.Labelframe:
        box = ttk.Labelframe(parent, text="Violation Detected Alert Overlay")
        box.pack(fill="x")

        self._vio_fields = {
            "ID": tk.StringVar(value="—"),
            "Car Plate Number": tk.StringVar(value="—"),
            "Violation": tk.StringVar(value="—"),
            "Date of Detection": tk.StringVar(value="—"),
            "Time of Detection": tk.StringVar(value="—"),
            "Detected lane": tk.StringVar(value="—"),
            "Report Status": tk.StringVar(value="—"),
        }

        r = 0
        for label, var in self._vio_fields.items():
            ttk.Label(box, text=f"{label}:").grid(row=r, column=0, sticky="w", padx=8, pady=2)
            ttk.Label(box, textvariable=var).grid(row=r, column=1, sticky="w", padx=8, pady=2)
            r += 1

        def dismiss() -> None:
            self._show_violation_marker = False
            self._recompute_camera_overlays()
            box.pack_forget()

        ttk.Button(box, text="Dismiss", command=dismiss).grid(row=r, column=0, columnspan=2, sticky="ew", padx=8, pady=6)
        return box

    def _build_screen_logs(self, root: ttk.Frame) -> None:
        top = ttk.Labelframe(root, text="Overview Dashboard")
        top.pack(fill="both", expand=True)

        summary = ttk.Frame(top)
        summary.pack(fill="x", padx=8, pady=8)
        summary.columnconfigure((0, 1, 2), weight=1)

        self._total_accidents = tk.StringVar(value="—")
        self._total_violations = tk.StringVar(value="—")
        self._total_incidents = tk.StringVar(value="—")

        self._metric(summary, 0, "Total Accidents Recorded", self._total_accidents)
        self._metric(summary, 1, "Total Violations Recorded", self._total_violations)
        self._metric(summary, 2, "Total Incidents", self._total_incidents)

        lanes = ttk.Labelframe(top, text="Per Lane Records")
        lanes.pack(fill="x", padx=8, pady=(0, 8))

        self._lane_vals = {
            "North": {"Congestion": tk.StringVar(value="—"), "Accidents": tk.StringVar(value="—"), "Intersection": tk.StringVar(value="—")},
            "East": {"Congestion": tk.StringVar(value="—"), "Accidents": tk.StringVar(value="—"), "Intersection": tk.StringVar(value="—")},
            "West": {"Congestion": tk.StringVar(value="—"), "Accidents": tk.StringVar(value="—"), "Intersection": tk.StringVar(value="—")},
            "South": {"Congestion": tk.StringVar(value="—"), "Accidents": tk.StringVar(value="—"), "Intersection": tk.StringVar(value="—")},
        }

        # A simple 4-column grid of lane cards
        card_grid = ttk.Frame(lanes)
        card_grid.pack(fill="x", padx=8, pady=8)
        for i, lane in enumerate(("North", "East", "West", "South")):
            card = ttk.Labelframe(card_grid, text=f"{lane} Lane")
            card.grid(row=0, column=i, sticky="nsew", padx=4)
            for r, field in enumerate(("Congestion", "Accidents", "Intersection")):
                ttk.Label(card, text=f"{field}:").grid(row=r, column=0, sticky="w", padx=6, pady=2)
                ttk.Label(card, textvariable=self._lane_vals[lane][field]).grid(row=r, column=1, sticky="w", padx=6, pady=2)

        graphs = ttk.Frame(top)
        graphs.pack(fill="x", padx=8, pady=(0, 8))
        graphs.columnconfigure((0, 1), weight=1)

        self._chart_acc = BarChart(graphs, "Accidents Recorded Graph (by hour)")
        self._chart_vio = BarChart(graphs, "Violations Recorded Graph (by hour)")
        self._chart_acc.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self._chart_vio.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        # Tabs
        tabs = ttk.Notebook(top)
        tabs.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        tab_acc = ttk.Frame(tabs)
        tab_vio = ttk.Frame(tabs)
        tab_lane = ttk.Frame(tabs)

        tabs.add(tab_acc, text="Accidents Logs")
        tabs.add(tab_vio, text="Violations Logs")
        tabs.add(tab_lane, text="Per Lane Records")

        self._acc_tree = self._build_accidents_table(tab_acc)
        self._vio_tree = self._build_violations_table(tab_vio)

        # Lane records tab
        lane_table = ttk.Treeview(tab_lane, columns=("lane", "congestion", "accidents", "intersection"), show="headings", height=6)
        for col, title in (
            ("lane", "Lane"),
            ("congestion", "Congestion"),
            ("accidents", "Accidents"),
            ("intersection", "Intersection"),
        ):
            lane_table.heading(col, text=title)
            lane_table.column(col, width=180 if col == "lane" else 120, anchor="w")
        lane_table.pack(fill="both", expand=True)
        self._lane_tree = lane_table

    def _build_accidents_table(self, parent: ttk.Frame) -> ttk.Treeview:
        cols = ("id", "plate", "date", "time", "lane", "status")
        tree = ttk.Treeview(parent, columns=cols, show="headings")

        tree.heading("id", text="ID")
        tree.heading("plate", text="Car Plate Number")
        tree.heading("date", text="Date of Detection")
        tree.heading("time", text="Time of Detection")
        tree.heading("lane", text="Detected lane (North, West, East, South)")
        tree.heading("status", text="Report Status")

        tree.column("id", width=80, anchor="w")
        tree.column("plate", width=140, anchor="w")
        tree.column("date", width=120, anchor="w")
        tree.column("time", width=120, anchor="w")
        tree.column("lane", width=260, anchor="w")
        tree.column("status", width=120, anchor="w")

        vsb = ttk.Scrollbar(parent, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=vsb.set)
        tree.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        return tree

    def _build_violations_table(self, parent: ttk.Frame) -> ttk.Treeview:
        cols = ("id", "plate", "violation", "date", "time", "lane", "status")
        tree = ttk.Treeview(parent, columns=cols, show="headings")

        tree.heading("id", text="ID")
        tree.heading("plate", text="Car Plate Number")
        tree.heading("violation", text="Violation")
        tree.heading("date", text="Date of Detection")
        tree.heading("time", text="Time of Detection")
        tree.heading("lane", text="Detected lane (North, West, East, South)")
        tree.heading("status", text="Report Status")

        tree.column("id", width=80, anchor="w")
        tree.column("plate", width=140, anchor="w")
        tree.column("violation", width=160, anchor="w")
        tree.column("date", width=120, anchor="w")
        tree.column("time", width=120, anchor="w")
        tree.column("lane", width=260, anchor="w")
        tree.column("status", width=120, anchor="w")

        vsb = ttk.Scrollbar(parent, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=vsb.set)
        tree.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        return tree

    def _metric(self, parent: ttk.Frame, col: int, label: str, var: tk.StringVar) -> None:
        box = ttk.Labelframe(parent, text=label)
        box.grid(row=0, column=col, sticky="nsew", padx=4)
        ttk.Label(box, textvariable=var, font=("Segoe UI", 14, "bold")).pack(anchor="w", padx=8, pady=8)

    def _wire_routing(self) -> None:
        def on_route(*_args: object) -> None:
            if self._route.get() == "logs":
                self._screen_logs.lift()
            else:
                self._screen_cameras.lift()

        self._route.trace_add("write", on_route)
        on_route()

    def _refresh_logs(self) -> None:
        logs_dir, snapshot = load_logs()
        self._logs_dir = logs_dir
        self._snapshot = snapshot

        # When real-time engine is running, Screen 1 is driven by live state.
        # Keep Screen 2 driven by CSV snapshots.
        if self._engine is None:
            # Screen 1 numbers
            vehicles = unique_nonempty(e.get("vehicle_id", "") for e in snapshot.traffic_events)
            violations = len(snapshot.violations)
            emergency_calls = sum(1 for a in snapshot.accidents if a.get("emergency_notified", "").strip().lower() == "yes")

            self._vehicles_detected.set(str(vehicles))
            self._violations_detected.set(str(violations))
            self._emergency_calls_detected.set(str(emergency_calls))

            # Overlays (latest)
            a = last_row(snapshot.accidents)
            if a:
                acc_id = (a.get("accident_id", "") or "").strip() or None
                if acc_id is not None and acc_id != self._last_accident_id_seen:
                    self._last_accident_id_seen = acc_id
                    self._show_accident_marker = True
                    if self._accident_overlay is not None and not self._accident_overlay.winfo_ismapped():
                        self._accident_overlay.pack(fill="x", pady=(0, 10))

                self._acc_fields["ID"].set(a.get("accident_id", "—") or "—")
                self._acc_fields["Date of Detection"].set(a.get("date", "—") or "—")
                self._acc_fields["Time of Detection"].set(a.get("time", "—") or "—")
                self._acc_fields["Detected lane"].set(lane_display_name(a.get("lane", "")))
                reported = "Reported" if a.get("emergency_notified", "").strip().lower() == "yes" else "Unreported"
                self._acc_fields["Report Status"].set(reported)

            v = last_row(snapshot.violations)
            if v:
                vio_id = (v.get("violation_id", "") or "").strip() or None
                if vio_id is not None and vio_id != self._last_violation_id_seen:
                    self._last_violation_id_seen = vio_id
                    self._show_violation_marker = True
                    if self._violation_overlay is not None and not self._violation_overlay.winfo_ismapped():
                        self._violation_overlay.pack(fill="x")

                self._vio_fields["ID"].set(v.get("violation_id", "—") or "—")
                self._vio_fields["Car Plate Number"].set(v.get("license_plate", "N/A") or "N/A")
                self._vio_fields["Violation"].set(v.get("violation_type", "—") or "—")
                self._vio_fields["Date of Detection"].set(v.get("date", "—") or "—")
                self._vio_fields["Time of Detection"].set(v.get("time", "—") or "—")
                self._vio_fields["Detected lane"].set(lane_display_name(v.get("lane", "")))
                self._vio_fields["Report Status"].set("Recorded")

            # Recompute camera overlays at the event locations
            self._recompute_camera_overlays()

        # Screen 2 totals
        self._total_accidents.set(str(len(snapshot.accidents)))
        self._total_violations.set(str(len(snapshot.violations)))
        self._total_incidents.set(str(len(snapshot.accidents) + len(snapshot.violations)))

        # Per lane records (draft congestion proxy)
        congestion: Dict[str, int] = {"North": 0, "South": 0, "East": 0, "West": 0}
        for ev in snapshot.traffic_events:
            name = lane_display_name(ev.get("lane", ""))
            if name in congestion:
                congestion[name] += 1

        accidents_count: Dict[str, int] = {"North": 0, "South": 0, "East": 0, "West": 0}
        for acc in snapshot.accidents:
            name = lane_display_name(acc.get("lane", ""))
            if name in accidents_count:
                accidents_count[name] += 1

        intersection_events = sum(1 for ev in snapshot.traffic_events if (ev.get("lane", "").strip().upper() == "INTERSECTION"))

        for lane in ("North", "East", "West", "South"):
            self._lane_vals[lane]["Congestion"].set(str(congestion.get(lane, 0)))
            self._lane_vals[lane]["Accidents"].set(str(accidents_count.get(lane, 0)))
            self._lane_vals[lane]["Intersection"].set(str(intersection_events))

        # Charts
        self._chart_acc.set_data(counts_by_hour(snapshot.accidents, "time"))
        self._chart_vio.set_data(counts_by_hour(snapshot.violations, "time"))

        # Tables
        self._reload_tables(snapshot)

    def _recompute_camera_overlays(self) -> None:
        overlays: List[CameraOverlay] = []

        a = last_row(self._snapshot.accidents)
        if self._show_accident_marker and a:
            try:
                x = int(float(a.get("location_x", "0") or 0))
                y = int(float(a.get("location_y", "0") or 0))
                acc_id = a.get("accident_id", "") or "—"
                overlays.append(CameraOverlay(kind="accident", x=x, y=y, label=f"ACCIDENT #{acc_id}"))
            except Exception:
                pass

        v = last_row(self._snapshot.violations)
        if self._show_violation_marker and v:
            try:
                x = int(float(v.get("location_x", "0") or 0))
                y = int(float(v.get("location_y", "0") or 0))
                vio_id = v.get("violation_id", "") or "—"
                vio_type = v.get("violation_type", "") or "VIOLATION"
                overlays.append(CameraOverlay(kind="violation", x=x, y=y, label=f"{vio_type} #{vio_id}"))
            except Exception:
                pass

        self._camera_overlays = overlays

    def _reload_tables(self, snapshot: LogsSnapshot) -> None:
        # Accidents table
        self._acc_tree.delete(*self._acc_tree.get_children())
        for row in snapshot.accidents:
            report = "Reported" if row.get("emergency_notified", "").strip().lower() == "yes" else "Unreported"
            self._acc_tree.insert(
                "",
                "end",
                values=(
                    row.get("accident_id", ""),
                    "N/A",  # accidents.csv has no license plate
                    row.get("date", ""),
                    row.get("time", ""),
                    lane_display_name(row.get("lane", "")),
                    report,
                ),
            )

        # Violations table
        self._vio_tree.delete(*self._vio_tree.get_children())
        for row in snapshot.violations:
            self._vio_tree.insert(
                "",
                "end",
                values=(
                    row.get("violation_id", ""),
                    row.get("license_plate", "N/A") or "N/A",
                    row.get("violation_type", ""),
                    row.get("date", ""),
                    row.get("time", ""),
                    lane_display_name(row.get("lane", "")),
                    "Recorded",
                ),
            )

        # Per lane records tab table
        self._lane_tree.delete(*self._lane_tree.get_children())
        for lane in ("North", "East", "West", "South"):
            self._lane_tree.insert(
                "",
                "end",
                values=(
                    f"{lane} Lane",
                    self._lane_vals[lane]["Congestion"].get(),
                    self._lane_vals[lane]["Accidents"].get(),
                    self._lane_vals[lane]["Intersection"].get(),
                ),
            )

    def _tick_camera(self) -> None:
        try:
            self._feed.tick()
        except Exception:
            pass

        # Real-time UI updates (Screen 1)
        if self._engine is not None:
            try:
                vehicles, violations, emergency_calls, fps = self._engine.get_metrics()
                self._vehicles_detected.set(str(vehicles))
                self._violations_detected.set(str(violations))
                self._emergency_calls_detected.set(str(emergency_calls))
                self._fps_status.set(f"{fps:.1f}")

                # Stoplights from live signals
                signals = self._engine.get_signals()
                for lane_key, state in signals.items():
                    lane_name = lane_display_name(lane_key)
                    if lane_name in self._light_vars:
                        self._light_vars[lane_name].set(str(state))

                # Live overlays
                acc, vio = self._engine.get_last_events()
                overlays: List[CameraOverlay] = []

                if acc is not None and bool(getattr(acc, "confirmed", False)):
                    acc_id = str(getattr(acc, "id", ""))
                    if acc_id and acc_id != self._last_accident_id_seen:
                        self._last_accident_id_seen = acc_id
                        self._show_accident_marker = True
                        if self._accident_overlay is not None and not self._accident_overlay.winfo_ismapped():
                            self._accident_overlay.pack(fill="x", pady=(0, 10))

                    ts = float(getattr(acc, "confirmed_time", 0.0) or getattr(acc, "detected_time", 0.0) or 0.0)
                    d, t = _dt_parts(ts)
                    self._acc_fields["ID"].set(acc_id or "—")
                    self._acc_fields["Date of Detection"].set(d)
                    self._acc_fields["Time of Detection"].set(t)
                    self._acc_fields["Detected lane"].set(_lane_key_to_display(getattr(acc, "lane", "")))
                    self._acc_fields["Report Status"].set("Reported" if bool(getattr(acc, "notified", False)) else "Unreported")

                    if self._show_accident_marker:
                        loc = getattr(acc, "location", None)
                        if isinstance(loc, tuple) and len(loc) >= 2:
                            overlays.append(CameraOverlay(kind="accident", x=int(loc[0]), y=int(loc[1]), label=f"ACCIDENT #{acc_id}"))

                if vio is not None:
                    vio_id = str(getattr(vio, "id", ""))
                    if vio_id and vio_id != self._last_violation_id_seen:
                        self._last_violation_id_seen = vio_id
                        self._show_violation_marker = True
                        if self._violation_overlay is not None and not self._violation_overlay.winfo_ismapped():
                            self._violation_overlay.pack(fill="x")

                    ts = float(getattr(vio, "timestamp", 0.0) or 0.0)
                    d, t = _dt_parts(ts)
                    self._vio_fields["ID"].set(vio_id or "—")
                    plate = getattr(vio, "license_plate", None) or "N/A"
                    self._vio_fields["Car Plate Number"].set(str(plate))
                    self._vio_fields["Violation"].set(str(getattr(vio, "type", "—") or "—"))
                    self._vio_fields["Date of Detection"].set(d)
                    self._vio_fields["Time of Detection"].set(t)
                    self._vio_fields["Detected lane"].set(_lane_key_to_display(getattr(vio, "lane", "")))
                    self._vio_fields["Report Status"].set("Recorded")

                    if self._show_violation_marker:
                        loc = getattr(vio, "location", None)
                        if isinstance(loc, tuple) and len(loc) >= 2:
                            vio_type = str(getattr(vio, "type", "VIOLATION") or "VIOLATION")
                            overlays.append(CameraOverlay(kind="violation", x=int(loc[0]), y=int(loc[1]), label=f"{vio_type} #{vio_id}"))

                self._camera_overlays = overlays
            except Exception:
                pass

        # Approx ~15fps
        self.after(66, self._tick_camera)


def run() -> None:
    root = tk.Tk()

    # Use ttk themed widgets
    try:
        style = ttk.Style(root)
        # Keep default Windows theme if available
        if "vista" in style.theme_names():
            style.theme_use("vista")
    except Exception:
        pass

    USADTkApp(root)
    root.mainloop()


if __name__ == "__main__":
    run()
