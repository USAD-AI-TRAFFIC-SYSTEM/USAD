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
import time
from dataclasses import dataclass
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
        self._w = 0
        self._h = 0

    def add_listener(self, listener: Callable[[Optional[bytes], int, int], None]) -> None:
        self._listeners.append(listener)

    def start(self) -> None:
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
    ):
        super().__init__(master)
        self._title = title
        self._on_fullscreen = on_fullscreen
        self._get_overlays = get_overlays
        self._base_w, self._base_h = base_size
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

        self._draw_overlays(frame_w, frame_h)


class FullscreenViewer(tk.Toplevel):
    def __init__(
        self,
        master: tk.Misc,
        title: str,
        feed: CameraFeed,
        get_overlays: Callable[[], List[CameraOverlay]],
        base_size: Tuple[int, int],
    ):
        super().__init__(master)
        self.title(title)
        self._img = None

        self._get_overlays = get_overlays
        self._base_w, self._base_h = base_size

        self.configure(bg="black")
        self._canvas = tk.Canvas(self, bg="black", highlightthickness=0)
        self._canvas.pack(fill="both", expand=True)
        self._image_item = None

        self.attributes("-fullscreen", True)
        self.bind("<Escape>", lambda _e: self.destroy())
        self.bind("<Button-1>", lambda _e: self.destroy())

        feed.add_listener(self._on_frame)

    def _on_frame(self, png_bytes: Optional[bytes], _w: int, _h: int) -> None:
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
        c.configure(scrollregion=(0, 0, frame_w, frame_h))

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

        for ov in overlays:
            x = int(ov.x * sx)
            y = int(ov.y * sy)
            x = max(0, min(frame_w - 1, x))
            y = max(0, min(frame_h - 1, y))

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

        # Camera feed (single source shared by both tiles)
        cam_source = 0
        if config is not None and hasattr(config, "CAMERA_SOURCE"):
            cam_source = getattr(config, "CAMERA_SOURCE")

        self._feed = CameraFeed(cam_source)
        self._feed.start()

        self._build_header()
        self._build_body()
        self.pack(fill="both", expand=True)

        self._wire_routing()
        self._refresh_logs()

        self._tick_camera()

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
            )

        tile1 = CameraTile(
            tiles,
            "Camera 1 (click for full screen)",
            self._feed,
            on_fullscreen=open_fullscreen,
            get_overlays=get_overlays,
            base_size=(self._base_cam_w, self._base_cam_h),
        )
        tile2 = CameraTile(
            tiles,
            "Camera 2 (click for full screen)",
            self._feed,
            on_fullscreen=open_fullscreen,
            get_overlays=get_overlays,
            base_size=(self._base_cam_w, self._base_cam_h),
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
