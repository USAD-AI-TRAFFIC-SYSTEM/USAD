"""USAD Local FastAPI Bridge Server.

Thin API layer between the unchanged USAD engine and the modern React frontend.
Runs entirely on localhost — no internet required.

Endpoints:
  GET  /api/video/feed          MJPEG live stream
  WS   /ws/telemetry            30 Hz JSON telemetry
  POST /api/control/{action}    Engine commands
  GET  /api/logs/{log_type}     CSV log data
"""

import asyncio
import csv
import json
import os
import signal
import sys
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

# ---------------------------------------------------------------------------
# Ensure USAD-Model is on sys.path so we can import engine modules
# ---------------------------------------------------------------------------
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from main import USAD  # noqa: E402
import config  # noqa: E402

# ---------------------------------------------------------------------------
# Global engine singleton
# ---------------------------------------------------------------------------
usad: Optional[USAD] = None

# Latest processed frame (shared between grabber thread and streaming endpoint)
_frame_lock = threading.Lock()
_latest_jpeg: Optional[bytes] = None
_frame_event = threading.Event()

# Shutdown flag
_shutdown_event = asyncio.Event()


# ---------------------------------------------------------------------------
# Background frame-grabber thread
# ---------------------------------------------------------------------------
def _frame_loop():
    """Continuously grab, process, and JPEG-encode frames."""
    global _latest_jpeg
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, 85]

    while not _shutdown_event.is_set():
        if usad is None or usad.cap is None:
            time.sleep(0.05)
            continue

        ret, frame = usad.cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue

        # Process through unchanged engine pipeline
        processed = usad.process_frame(frame)

        # FPS bookkeeping (mirrors app.py logic)
        usad.frame_count += 1
        elapsed = time.time() - usad.start_time
        if elapsed > 1.0:
            usad.fps = usad.frame_count / elapsed
            usad.frame_count = 0
            usad.start_time = time.time()

        usad.check_config_reload()

        # Encode to JPEG
        ok, buf = cv2.imencode(".jpg", processed, encode_params)
        if ok:
            with _frame_lock:
                _latest_jpeg = buf.tobytes()
            _frame_event.set()


# ---------------------------------------------------------------------------
# Telemetry snapshot helper
# ---------------------------------------------------------------------------
def _build_telemetry() -> dict:
    """Read current engine state into a JSON-serialisable dict."""
    if usad is None:
        return {"status": "initialising"}

    lane_counts = usad.vehicle_detector.get_vehicle_count_by_lane()
    lane_counts_int = {k: int(v) for k, v in lane_counts.items()}

    now = time.time()
    remaining = 0.0
    if usad.current_active_lane:
        if usad.current_phase == "GREEN":
            remaining = max(0.0, float(usad.lane_green_duration) - (now - usad.phase_start_time))
        elif usad.current_phase == "YELLOW":
            remaining = max(0.0, float(getattr(config, "YELLOW_TIME", 3)) - (now - usad.phase_start_time))

    lane_signals = {}
    lane_states = {}
    adaptive_durations = {}
    for lk in config.LANES.keys():
        lane_signals[lk] = usad.violation_detector.current_signals.get(lk, "RED")
        count = int(lane_counts.get(lk, 0))
        lane_states[lk] = usad._classify_lane_congestion(count)
        adaptive_durations[lk] = usad._compute_green_duration(lk, lane_counts_int)

    # Detected plates
    plates = []
    for text, (bbox, conf, ts) in list(getattr(usad, "_detected_plates", {}).items()):
        if (now - ts) < 3.0:
            plates.append({"text": text, "confidence": round(conf, 3)})

    violation_stats = usad.violation_detector.get_statistics()

    return {
        "fps": round(usad.fps, 1),
        "camera_source": config.CAMERA_SOURCE,
        "arduino_connected": usad._is_arduino_connected(),
        "software_auto_mode": usad.software_auto_mode,
        "current_active_lane": usad.current_active_lane,
        "current_phase": usad.current_phase,
        "phase_time_remaining": round(remaining, 1),
        "total_vehicles": sum(lane_counts_int.values()),
        "lane_counts": lane_counts_int,
        "lane_states": lane_states,
        "lane_signals": lane_signals,
        "adaptive_green_durations": adaptive_durations,
        "active_accidents_count": len(usad.accident_detector.get_confirmed_accidents()),
        "active_stopped_cars_count": len(usad.accident_detector.get_stopped_vehicle_ids()),
        "total_violations_session": violation_stats.get("total_violations", 0),
        "emergency_notifications_count": usad.emergency_notifier.get_notification_count(),
        "detected_license_plates": plates,
    }


# ---------------------------------------------------------------------------
# Lifespan (startup / shutdown)
# ---------------------------------------------------------------------------
@asynccontextmanager
async def lifespan(app: FastAPI):
    global usad

    # --- Startup ---
    usad = USAD()
    usad.show_cv_panel = False  # React renders the HUD — skip OpenCV overlay

    if not usad.initialize_camera():
        print("[server] FATAL: Camera initialization failed")
        sys.exit(1)

    usad.initialize_arduino()

    # Prime signal simulation (same as app.py)
    if not usad._is_arduino_connected() and config.LANES:
        usad.activate_lane(list(config.LANES.keys())[0])

    # Launch background frame grabber
    grabber = threading.Thread(target=_frame_loop, daemon=True, name="FrameGrabber")
    grabber.start()

    print("[server] ✓ USAD engine ready — serving on http://127.0.0.1:8000")
    yield

    # --- Shutdown ---
    _shutdown_event.set()
    try:
        usad.print_statistics()
    except Exception:
        pass
    try:
        if usad.cap:
            usad.cap.release()
    except Exception:
        pass
    try:
        if usad._is_arduino_connected():
            usad.traffic_controller.disconnect()
    except Exception:
        pass
    print("[server] ✓ Shutdown complete")


# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------
app = FastAPI(title="USAD Local Server", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ---------------------------------------------------------------------------
# Video streaming
# ---------------------------------------------------------------------------
async def _mjpeg_generator():
    """Yield MJPEG frames as multipart chunks."""
    while not _shutdown_event.is_set():
        _frame_event.wait(timeout=0.1)
        _frame_event.clear()
        with _frame_lock:
            jpeg = _latest_jpeg
        if jpeg is None:
            continue
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
        )
        await asyncio.sleep(0.016)  # ~60 fps cap


@app.get("/api/video/feed")
async def video_feed():
    return StreamingResponse(
        _mjpeg_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


# ---------------------------------------------------------------------------
# WebSocket telemetry
# ---------------------------------------------------------------------------
@app.websocket("/ws/telemetry")
async def telemetry_ws(ws: WebSocket):
    await ws.accept()
    try:
        while not _shutdown_event.is_set():
            payload = _build_telemetry()
            await ws.send_json(payload)
            await asyncio.sleep(0.033)  # ~30 Hz
    except WebSocketDisconnect:
        pass
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Control endpoints
# ---------------------------------------------------------------------------
@app.post("/api/control/auto")
async def control_auto():
    if usad._is_arduino_connected():
        usad.traffic_controller.set_auto_mode()
    else:
        usad.software_auto_mode = True
    return {"ok": True, "action": "auto"}


@app.post("/api/control/lane/{lane_key}")
async def control_lane(lane_key: str):
    lane_key = lane_key.upper()
    if lane_key not in config.LANES:
        return JSONResponse({"ok": False, "error": f"Unknown lane {lane_key}"}, 400)
    usad.software_auto_mode = False
    usad.activate_lane(lane_key)
    return {"ok": True, "action": f"activate_{lane_key}"}


@app.post("/api/control/reset")
async def control_reset():
    usad.vehicle_detector.reset()
    usad.accident_detector.reset()
    usad.violation_detector.reset()
    usad.emergency_notifier.reset()
    return {"ok": True, "action": "reset"}


@app.post("/api/control/reset-bg")
async def control_reset_bg():
    try:
        usad.vehicle_detector.reset_background()
    except AttributeError:
        usad.vehicle_detector.reset(reset_background=True, verbose=True)
    return {"ok": True, "action": "reset_bg"}


@app.post("/api/control/cycle-camera")
async def control_cycle_camera():
    usad.cycle_camera()
    return {"ok": True, "action": "cycle_camera", "new_source": config.CAMERA_SOURCE}


@app.post("/api/control/shutdown")
async def control_shutdown():
    usad.print_statistics()
    _shutdown_event.set()
    # Give time for response to be sent, then kill the process
    asyncio.get_event_loop().call_later(0.5, lambda: os.kill(os.getpid(), signal.SIGTERM))
    return {"ok": True, "action": "shutdown"}


# ---------------------------------------------------------------------------
# Log / analytics endpoints
# ---------------------------------------------------------------------------
def _read_csv(filename: str) -> list[dict]:
    logs_dir = os.path.join(_THIS_DIR, "logs")
    filepath = os.path.join(logs_dir, filename)
    if not os.path.exists(filepath):
        return []
    with open(filepath, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


@app.get("/api/logs/summary")
async def logs_summary():
    v_analytics = usad.event_logger.get_violation_analytics()
    a_analytics = usad.event_logger.get_accident_analytics()
    return {
        "violations": v_analytics,
        "accidents": a_analytics,
        "emergency_notifications": usad.emergency_notifier.get_notification_count(),
    }


@app.get("/api/logs/violations")
async def logs_violations():
    return _read_csv("violations.csv")


@app.get("/api/logs/accidents")
async def logs_accidents():
    return _read_csv("accidents.csv")


@app.get("/api/logs/traffic")
async def logs_traffic():
    return _read_csv("traffic_events.csv")


# ---------------------------------------------------------------------------
# Serve React static build (production)
# ---------------------------------------------------------------------------
_ui_dist = _THIS_DIR.parent / "USAD-UI" / "dist"
if _ui_dist.is_dir():
    app.mount("/", StaticFiles(directory=str(_ui_dist), html=True), name="frontend")


# ---------------------------------------------------------------------------
# Entry point — run with uvicorn
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "server:app",
        host="127.0.0.1",
        port=8000,
        log_level="info",
        reload=False,
    )
