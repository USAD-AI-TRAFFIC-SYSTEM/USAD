import cv2
import numpy as np
from typing import Optional, Tuple, List, Dict
import re
import time
from collections import defaultdict
import config
import shutil
import pytesseract
import os
import threading
import queue
import logging
from dataclasses import dataclass
from collections import deque

pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"

# Module logger: provides detailed debug output to the terminal for troubleshooting
logger = logging.getLogger(__name__)
try:
    level_name = getattr(config, "LP_DEBUG_LEVEL", "DEBUG")
    level = getattr(logging, level_name)
except Exception:
    level = logging.DEBUG
logger.setLevel(level)
if not logger.handlers:
    ch = logging.StreamHandler()
    ch.setLevel(level)
    fmt = logging.Formatter("%(asctime)s %(levelname)s [%(name)s] %(message)s")
    ch.setFormatter(fmt)
    logger.addHandler(ch)

try:
    import easyocr
    OCR_AVAILABLE = True
except ImportError:
    OCR_AVAILABLE = False
    print("Warning: easyocr not installed. Falling back to template OCR if enabled.")
    logger.warning("easyocr not installed; falling back to template/Tesseract if enabled")

try:
    import pytesseract
    from pytesseract import Output as _TESS_OUTPUT
    from PIL import Image
    TESS_AVAILABLE = True
except ImportError:
    TESS_AVAILABLE = False

logger.info(f"OCR availability: easyocr={OCR_AVAILABLE}, tesseract_support={TESS_AVAILABLE}")


_PLATE_RE = re.compile(r"^[A-Z]{3}[0-9]{3}$")
_PLATE_SEARCH_RE = re.compile(r"[A-Z]{3}[0-9]{3}")


def _is_permissive_alnum_mode() -> bool:
    """Testing mode: accept any alphanumeric OCR text."""
    return bool(getattr(config, "LP_TEST_READ_ALL_ALNUM", False))


def _extract_alnum_text(text: str) -> Optional[str]:
    """Extract all uppercase alphanumeric tokens for testing mode.

    Returns a space-separated string of tokens in reading order.
    """
    raw = (text or "").upper().strip()
    if not raw:
        return None

    parts = re.findall(r"[A-Z0-9]+", raw)
    if not parts:
        return None
    return " ".join(parts)


def _extract_plate_from_text(text: str) -> Optional[str]:
    """Extract an AAA999 plate from an OCR string.

    EasyOCR sometimes returns extra characters or splits the plate into chunks.
    We keep the final output strict (AAA999) but allow substring extraction.
    """
    if _is_permissive_alnum_mode():
        return _extract_alnum_text(text)

    raw = (text or "").upper().strip()
    if not raw:
        return None

    cleaned = re.sub(r"[^A-Z0-9]", "", raw)
    if not cleaned:
        return None

    # Try direct normalization first.
    norm = _normalize_plate_text(cleaned)
    if _is_valid_plate(norm):
        return norm

    # Try searching within cleaned/coerced strings for AAA999.
    coerced = _coerce_ocr_confusions(cleaned)
    for s in (coerced, cleaned):
        m = _PLATE_SEARCH_RE.search(s)
        if not m:
            continue
        cand = _normalize_plate_text(m.group(0))
        if _is_valid_plate(cand):
            return cand
    return None


def _coerce_ocr_confusions(text: str) -> str:
    """Fix common OCR confusions while keeping the strict AAA999 format.

    Only applies simple, position-aware substitutions for the expected format:
    - First 3 chars are letters
    - Last 3 chars are digits
    """
    t = (text or "").upper().strip()
    t = re.sub(r"[^A-Z0-9]", "", t)
    if len(t) != 6:
        return t

    digit_to_letter = {
        "0": "O",
        "1": "I",
        "2": "Z",
        "5": "S",
        "8": "B",
    }
    letter_to_digit = {
        "O": "0",
        "I": "1",
        "L": "1",
        "Z": "2",
        "S": "5",
        "B": "8",
    }

    chars = list(t)
    for i in range(3):
        if chars[i].isdigit():
            chars[i] = digit_to_letter.get(chars[i], chars[i])
    for i in range(3, 6):
        if chars[i].isalpha():
            chars[i] = letter_to_digit.get(chars[i], chars[i])
    return "".join(chars)


def _normalize_plate_text(text: str) -> str:
    if _is_permissive_alnum_mode():
        raw = (text or "").upper().strip()
        parts = re.findall(r"[A-Z0-9]+", raw)
        return " ".join(parts)
    text = (text or "").upper().strip()
    text = re.sub(r"[^A-Z0-9]", "", text)
    # Apply conservative OCR confusion fixes (e.g., O<->0, I/L<->1)
    text2 = _coerce_ocr_confusions(text)
    if _is_valid_plate(text2):
        return text2
    return text


def _is_valid_plate(text: str) -> bool:
    if _is_permissive_alnum_mode():
        parts = re.findall(r"[A-Z0-9]+", (text or "").upper())
        return len(parts) > 0
    return bool(_PLATE_RE.match(text or ""))


class PlateTemporalTracker:
    """Aggregates plate OCR observations per vehicle ID to avoid flicker.

    Uses a weighted majority vote (score = sum(confidence)) plus a minimum
    observation count before confirming a plate for a track.
    """

    def __init__(self):
        # vehicle_id -> plate -> {"score": float, "count": int, "last_ts": float}
        self._votes: Dict[int, Dict[str, dict]] = defaultdict(dict)
        self._confirmed: Dict[int, str] = {}
        self._last_seen_ts: Dict[int, float] = {}

    def confirmed_plate(self, vehicle_id: int) -> Optional[str]:
        try:
            return self._confirmed.get(int(vehicle_id))
        except Exception:
            return None

    def force_confirm(self, vehicle_id: int, plate_text: str):
        """Force-confirm a plate for a track (used when OCR is highly confident)."""
        try:
            vid = int(vehicle_id)
        except Exception:
            return
        plate = _normalize_plate_text(plate_text)
        if not _is_valid_plate(plate):
            return
        self._confirmed[vid] = plate

    def update(self, vehicle_id: int, plate_text: str, confidence: float, ts: Optional[float] = None) -> Optional[str]:
        try:
            vid = int(vehicle_id)
        except Exception:
            return None

        logger.debug(f"update_plate_for_vehicle start vid={vehicle_id} frame_index={frame_index} ocr_available={self.ocr_available}")

        plate = _normalize_plate_text(plate_text)
        if not _is_valid_plate(plate):
            return None

        now = float(ts if ts is not None else time.time())
        self._last_seen_ts[vid] = now

        # Clamp confidence into 0..100 (EasyOCR is 0..1, our wrapper returns 0..100)
        try:
            conf = float(confidence)
        except Exception:
            conf = 0.0
        if conf <= 1.0:
            conf = conf * 100.0
        conf = max(0.0, min(100.0, conf))

        vmap = self._votes[vid]
        entry = vmap.get(plate)
        if entry is None:
            entry = {"score": 0.0, "count": 0, "last_ts": now}
            vmap[plate] = entry
        entry["score"] = float(entry.get("score", 0.0) or 0.0) + conf
        entry["count"] = int(entry.get("count", 0) or 0) + 1
        entry["last_ts"] = now

        logger.debug(f"PlateTemporalTracker.update vid={vid} plate={plate} conf={conf:.1f} entry_count={entry['count']} entry_score={entry['score']:.1f}")

        min_obs = int(getattr(config, "LP_STABLE_MIN_OBSERVATIONS", 3) or 3)
        min_ratio = float(getattr(config, "LP_STABLE_MIN_RATIO", 0.60) or 0.60)
        min_ratio = max(0.0, min(1.0, min_ratio))
        min_best_score = float(getattr(config, "LP_STABLE_MIN_SCORE", 140.0) or 140.0)

        # Pick best by weighted score.
        best_plate = None
        best_score = -1.0
        best_count = 0
        total_score = 0.0
        for p, e in vmap.items():
            s = float(e.get("score", 0.0) or 0.0)
            c = int(e.get("count", 0) or 0)
            total_score += s
            if s > best_score:
                best_plate = p
                best_score = s
                best_count = c

        if not best_plate:
            return None

        ratio = (best_score / total_score) if total_score > 1e-6 else 0.0
        if best_count >= max(1, min_obs) and best_score >= min_best_score and ratio >= min_ratio:
            prior = self._confirmed.get(vid)
            self._confirmed[vid] = best_plate
            if prior != best_plate:
                logger.info(f"PlateTemporalTracker.confirmed vid={vid} plate={best_plate} score={best_score:.1f} count={best_count} ratio={ratio:.2f}")
                return best_plate
            else:
                logger.debug(f"PlateTemporalTracker.confirmation already known vid={vid} plate={best_plate}")
        return None

    def cleanup(self, active_vehicle_ids: Optional[set] = None, now: Optional[float] = None):
        ttl = float(getattr(config, "LP_TRACK_TTL_SECONDS", 12.0) or 12.0)
        ttl = max(1.0, min(120.0, ttl))
        t = float(now if now is not None else time.time())
        active = set(int(x) for x in active_vehicle_ids) if active_vehicle_ids else None

        for vid in list(self._votes.keys()):
            last = float(self._last_seen_ts.get(int(vid), 0.0) or 0.0)
            if active is not None and int(vid) not in active:
                stale = True
            else:
                stale = (t - last) > ttl
            if stale:
                logger.debug(f"PlateTemporalTracker.cleanup removing vid={vid}")
                self._votes.pop(int(vid), None)
                self._confirmed.pop(int(vid), None)
                self._last_seen_ts.pop(int(vid), None)


class PlateBBoxTracker:
    """Tracks a stable plate bbox per vehicle ID.

    This keeps the plate ROI consistent across frames without running OCR more often.
    """

    def __init__(self):
        self._bbox_ema: Dict[int, np.ndarray] = {}
        self._last_seen_ts: Dict[int, float] = {}

    def update(self, vehicle_id: int, bbox: Tuple[int, int, int, int], ts: Optional[float] = None) -> Tuple[int, int, int, int]:
        try:
            vid = int(vehicle_id)
            x, y, w, h = [int(round(float(v))) for v in bbox]
        except Exception:
            return bbox

        now = float(ts if ts is not None else time.time())
        self._last_seen_ts[vid] = now

        x = max(0, x)
        y = max(0, y)
        w = max(1, w)
        h = max(1, h)

        alpha = float(getattr(config, "LP_BBOX_SMOOTHING_ALPHA", 0.35) or 0.35)
        alpha = max(0.05, min(0.90, alpha))
        new = np.array([x, y, w, h], dtype=np.float32)

        prior = self._bbox_ema.get(vid)
        if prior is None or prior.shape != (4,):
            self._bbox_ema[vid] = new
        else:
            self._bbox_ema[vid] = (alpha * new) + ((1.0 - alpha) * prior)

        out = self._bbox_ema[vid]
        bbox_out = (int(round(float(out[0]))), int(round(float(out[1]))), int(round(float(out[2]))), int(round(float(out[3]))))
        logger.debug(f"PlateBBoxTracker.update vid={vid} bbox={bbox_out} alpha={alpha}")
        return bbox_out

    def get(self, vehicle_id: int) -> Optional[Tuple[int, int, int, int]]:
        try:
            vid = int(vehicle_id)
        except Exception:
            return None
        arr = self._bbox_ema.get(vid)
        if arr is None:
            return None
        return (int(round(float(arr[0]))), int(round(float(arr[1]))), int(round(float(arr[2]))), int(round(float(arr[3]))))

    def cleanup(self, active_vehicle_ids: Optional[set] = None, now: Optional[float] = None):
        ttl = float(getattr(config, "LP_TRACK_TTL_SECONDS", 12.0) or 12.0)
        ttl = max(1.0, min(120.0, ttl))
        t = float(now if now is not None else time.time())
        active = set(int(x) for x in active_vehicle_ids) if active_vehicle_ids else None

        for vid in list(self._bbox_ema.keys()):
            last = float(self._last_seen_ts.get(int(vid), 0.0) or 0.0)
            if active is not None and int(vid) not in active:
                stale = True
            else:
                stale = (t - last) > ttl
            if stale:
                logger.debug(f"PlateBBoxTracker.cleanup removing vid={vid}")
                self._bbox_ema.pop(int(vid), None)
                self._last_seen_ts.pop(int(vid), None)
@dataclass
class _PlateObsState:
    last_frame_index: int = -1
    consecutive_hits: int = 0
    last_bbox: Optional[Tuple[int, int, int, int]] = None
    last_detect_ts: float = 0.0
    last_submit_frame_index: int = -10**9
    in_flight: bool = False
    cached_plate: Optional[str] = None
    cached_conf: float = 0.0
    cached_ts: float = 0.0


class _AsyncOCRWorker:
    def __init__(self, detector: "LicensePlateDetector"):
        self._detector = detector
        self._queue: "queue.Queue" = queue.Queue(maxsize=int(getattr(config, "LP_OCR_JOB_QUEUE_SIZE", 16) or 16))
        self._thread = threading.Thread(target=self._run, name="PlateOCRWorker", daemon=True)
        self._thread.start()

    def submit(self, job) -> bool:
        try:
            self._queue.put_nowait(job)
            return True
        except Exception:
            return False

    def _run(self):
        logger.debug("AsyncOCRWorker thread started")
        while True:
            job = None
            try:
                job = self._queue.get()
                logger.debug(f"AsyncOCRWorker: fetched job={job[0] if job is not None else None}")
            except Exception:
                logger.exception("AsyncOCRWorker: error getting job")
                continue
            if job is None:
                return
            try:
                self._detector._process_ocr_job(job)
            except Exception:
                # Never crash the worker.
                logger.exception("AsyncOCRWorker: exception while processing job")
                pass
            finally:
                try:
                    self._queue.task_done()
                except Exception:
                    pass


class LicensePlateDetector:
    """Detects and reads license plates from vehicle images"""

    def __init__(self):
        self.easyocr_available = OCR_AVAILABLE
        self.tesseract_available = bool(TESS_AVAILABLE)
        self.reader = None
        self.tracker = PlateTemporalTracker()
        self.bbox_tracker = PlateBBoxTracker()

        # Async OCR state (non-blocking). Never run OCR in the main loop.
        self._obs: Dict[int, _PlateObsState] = {}
        self._lock = threading.Lock()
        self._ocr_results: Dict[int, Tuple[str, float, Tuple[int, int, int, int], float]] = {}
        self._debug_patches: Dict[int, Tuple[np.ndarray, Tuple[int, int, int, int], float]] = {}
        self._ocr_rate_window = deque()  # timestamps of recent OCR submissions
        self._fallback_frame_counter = 0
        self._worker: Optional[_AsyncOCRWorker] = None

        # Template OCR fallback is OpenCV-only and works without EasyOCR/Torch.
        self._template_enabled = bool(getattr(config, "LP_TEMPLATE_OCR_ENABLE", True))
        self._templates = None
        self._template_size = int(getattr(config, "LP_TEMPLATE_OCR_SIZE", 36) or 36)
        self._template_size = max(24, min(72, self._template_size))

        if self.easyocr_available:
            try:
                # Initialize EasyOCR reader for English text
                self.reader = easyocr.Reader(['en'], gpu=False)
            except Exception as e:
                print(f"Failed to initialize EasyOCR: {e}")
                self.reader = None

        if self.reader is None and self._template_enabled:
            try:
                self._templates = self._build_ocr_templates()
            except Exception as e:
                print(f"Failed to initialize template OCR: {e}")
                self._templates = None
                self._template_enabled = False

        # Configure Tesseract path (optional) and availability.
        self._tesseract_cmd = None
        if self.tesseract_available:
            try:
                cmd = getattr(config, "TESSERACT_CMD", None)
                cmd = str(cmd).strip() if cmd else ""
                if cmd:
                    self._tesseract_cmd = cmd
                else:
                    self._tesseract_cmd = shutil.which("tesseract")
                    if not self._tesseract_cmd:
                        # Common Windows install location
                        default_cmd = r"C:\\Program Files\\Tesseract-OCR\\tesseract.exe"
                        if os.path.exists(default_cmd):
                            self._tesseract_cmd = default_cmd
                if self._tesseract_cmd:
                    try:
                        pytesseract.pytesseract.tesseract_cmd = self._tesseract_cmd
                    except Exception:
                        pass
                else:
                    self.tesseract_available = False
            except Exception:
                self.tesseract_available = False

        self.ocr_available = bool(self.reader is not None) or bool(self.tesseract_available) or bool(self._template_enabled and self._templates)

        # Performance: prefer the OpenCV-only template OCR when EasyOCR isn't available.
        # Tesseract can be significantly slower and may block the UI/frame loop.
        prefer_template = bool(getattr(config, "LP_PREFER_TEMPLATE_OCR", True))

        if _is_permissive_alnum_mode() and self.tesseract_available:
            self.ocr_mode = "tesseract"
        elif self.reader is not None:
            self.ocr_mode = "easyocr"
        elif prefer_template and self._template_enabled and self._templates:
            self.ocr_mode = "template"
        elif self.tesseract_available:
            self.ocr_mode = "tesseract"
        elif self._template_enabled and self._templates:
            self.ocr_mode = "template"
        else:
            self.ocr_mode = "disabled"

        logger.info(
            f"LicensePlateDetector init: ocr_available={self.ocr_available} ocr_mode={self.ocr_mode} "
            f"easyocr_reader={'yes' if self.reader is not None else 'no'} tesseract={self.tesseract_available} template_enabled={bool(self._templates)}"
        )

        # Start async worker if enabled.
        async_enable = bool(getattr(config, "LP_ASYNC_OCR_ENABLE", True))
        if async_enable and self.ocr_available:
            try:
                self._worker = _AsyncOCRWorker(self)
            except Exception:
                self._worker = None

    def detect_license_plate(self, frame: np.ndarray, vehicle_bbox: Tuple[int, int, int, int]) -> Optional[Tuple[str, float, Tuple[int, int, int, int]]]:
        """
        Detect and read license plate from vehicle region
        
        Args:
            frame: Input frame
            vehicle_bbox: Vehicle bounding box (x, y, w, h)
            
        Returns:
            Tuple of (plate_text, confidence, plate_bbox) or None
        """
        if not config.ENABLE_LICENSE_PLATE_DETECTION:
            return None

        return self._detect_plate_once(frame, vehicle_bbox)

    def cleanup(self, active_vehicle_ids: Optional[set] = None, now: Optional[float] = None, frame_index: Optional[int] = None):
        """Cleanup plate tracking/OCR state for vehicles that have left."""
        try:
            self.tracker.cleanup(active_vehicle_ids=active_vehicle_ids, now=now)
        except Exception:
            pass
        try:
            self.bbox_tracker.cleanup(active_vehicle_ids=active_vehicle_ids, now=now)
        except Exception:
            pass

        if active_vehicle_ids is None:
            return

        try:
            active = {int(x) for x in active_vehicle_ids}
        except Exception:
            active = None
        if active is None:
            return

        with self._lock:
            for vid in list(self._obs.keys()):
                if int(vid) not in active:
                    self._obs.pop(int(vid), None)
                    self._ocr_results.pop(int(vid), None)
                    self._debug_patches.pop(int(vid), None)

    def get_debug_patches(self) -> List[Tuple[int, np.ndarray, Tuple[int, int, int, int], float]]:
        """Return latest OCR input patch per vehicle for debug visualization."""
        out: List[Tuple[int, np.ndarray, Tuple[int, int, int, int], float]] = []
        with self._lock:
            for vid, payload in self._debug_patches.items():
                try:
                    patch, bbox, ts = payload
                    if patch is None or getattr(patch, "size", 0) == 0:
                        continue
                    out.append((int(vid), patch.copy(), tuple(int(x) for x in bbox), float(ts)))
                except Exception:
                    continue
        out.sort(key=lambda t: int(t[0]))
        return out

    def update_plate_for_vehicle(
        self,
        frame: np.ndarray,
        vehicle_bbox: Tuple[int, int, int, int],
        vehicle_id: int,
        *,
        frame_index: Optional[int] = None,
        now_ts: Optional[float] = None,
    ) -> Optional[Tuple[str, float, Tuple[int, int, int, int]]]:
        """Non-blocking plate pipeline.

        - Detect plate bbox in a small ROI (no OCR).
        - Require N consecutive frame detections before scheduling OCR.
        - Run OCR asynchronously with per-vehicle cadence + caching.
        """
        if not config.ENABLE_LICENSE_PLATE_DETECTION:
            return None
        if not self.ocr_available:
            return None

        try:
            vid = int(vehicle_id)
        except Exception:
            return None

        t_now = float(now_ts if now_ts is not None else time.time())
        if frame_index is None:
            self._fallback_frame_counter += 1
            frame_index = int(self._fallback_frame_counter)

        # Pull any completed OCR result (non-blocking).
        completed = None
        with self._lock:
            completed = self._ocr_results.pop(int(vid), None)
            st = self._obs.get(int(vid))
            if st is None:
                st = _PlateObsState()
                self._obs[int(vid)] = st

            if completed is not None:
                plate_text, conf, bbox, finished_ts = completed
                st.in_flight = False
                st.cached_plate = plate_text
                st.cached_conf = float(conf)
                st.cached_ts = float(finished_ts)

        if completed is not None:
            plate_text, conf, bbox, _finished_ts = completed

            logger.debug(f"update_plate_for_vehicle: OCR completed for vid={vid} plate={plate_text} conf={conf}")

            # If OCR is confident, confirm immediately; otherwise rely on the existing temporal tracker.
            confirm_conf = float(getattr(config, "LP_OCR_CACHE_MIN_CONF", 70.0) or 70.0)
            confirm_conf = max(0.0, min(100.0, confirm_conf))
            if float(conf) >= confirm_conf:
                try:
                    self.tracker.force_confirm(int(vid), str(plate_text))
                except Exception:
                    pass
            else:
                try:
                    self.tracker.update(int(vid), str(plate_text), float(conf), ts=t_now)
                except Exception:
                    pass

            # Update bbox smoothing from the OCR job bbox.
            try:
                bbox = self.bbox_tracker.update(int(vid), tuple(int(x) for x in bbox), ts=t_now)
            except Exception:
                pass

        # If already confirmed/cached, return without scheduling more OCR.
        confirmed = self.tracker.confirmed_plate(int(vid))
        with self._lock:
            st = self._obs.get(int(vid))
            cached_plate = (st.cached_plate if st else None)
            cached_conf = float(st.cached_conf if st else 0.0)
            cached_bbox = (st.last_bbox if st else None)

        if confirmed:
            bbox_out = None
            try:
                bbox_out = self.bbox_tracker.get(int(vid)) or cached_bbox
            except Exception:
                bbox_out = cached_bbox
            return (str(confirmed), float(cached_conf or 0.0), bbox_out or (0, 0, 1, 1))

        confirm_conf = float(getattr(config, "LP_OCR_CACHE_MIN_CONF", 70.0) or 70.0)
        confirm_conf = max(0.0, min(100.0, confirm_conf))
        if cached_plate and float(cached_conf) >= confirm_conf:
            bbox_out = None
            try:
                bbox_out = self.bbox_tracker.get(int(vid)) or cached_bbox
            except Exception:
                bbox_out = cached_bbox
            return (str(cached_plate), float(cached_conf), bbox_out or (0, 0, 1, 1))

        # Detect plate candidate bbox (NO OCR) in small ROIs.
        candidate = self._detect_plate_candidate_bbox_and_patch(frame, vehicle_bbox)
        if not candidate:
            # Reset consecutive streak when we miss.
            with self._lock:
                st = self._obs.get(int(vid))
                if st is not None:
                    st.consecutive_hits = 0
                    st.last_frame_index = int(frame_index)
            logger.debug(f"update_plate_for_vehicle: no plate candidate found vid={vid}")
            return None

        plate_bbox, plate_patch = candidate

        # Keep the exact patch that will be sent to OCR for live debugging/tuning.
        with self._lock:
            try:
                self._debug_patches[int(vid)] = (
                    plate_patch.copy(),
                    tuple(int(x) for x in plate_bbox),
                    float(t_now),
                )
            except Exception:
                pass

        # Update consecutive detection streak.
        with self._lock:
            st = self._obs.get(int(vid))
            if st is None:
                st = _PlateObsState()
                self._obs[int(vid)] = st

            if int(frame_index) == int(st.last_frame_index) + 1:
                st.consecutive_hits = int(st.consecutive_hits) + 1
            else:
                st.consecutive_hits = 1
            st.last_frame_index = int(frame_index)
            st.last_bbox = tuple(int(x) for x in plate_bbox)
            st.last_detect_ts = float(t_now)

        # Smooth bbox for display.
        try:
            self.bbox_tracker.update(int(vid), tuple(int(x) for x in plate_bbox), ts=t_now)
        except Exception:
            pass

        # Schedule OCR only after N consecutive detections and at a controlled cadence.
        required = int(getattr(config, "LP_OCR_CONSECUTIVE_DETECTIONS", 3) or 3)
        required = max(1, min(10, required))
        ocr_every_n = int(getattr(config, "LP_OCR_EVERY_N_FRAMES", 7) or 7)
        ocr_every_n = max(1, min(60, ocr_every_n))

        # Bootstrap mode: for a new/unconfirmed track, run OCR sooner so text appears
        # on live feed quickly. Once cached/confirmed, normal cadence applies.
        with self._lock:
            st_boot = self._obs.get(int(vid))
            has_cached = bool(st_boot and st_boot.cached_plate)
        if (not confirmed) and (not has_cached):
            required = min(required, 1)
            ocr_every_n = min(ocr_every_n, 2)

        with self._lock:
            st = self._obs.get(int(vid))
            if st is None:
                return None
            if st.in_flight:
                return None
            if int(st.consecutive_hits) < int(required):
                return None
            if (int(frame_index) - int(st.last_submit_frame_index)) < int(ocr_every_n):
                return None

        # Global OCR rate limiting (calls/sec).
        if not self._can_submit_ocr(t_now):
            return None

        if self._worker is None:
            return None

        # Submit async OCR job.
        submitted = False
        with self._lock:
            st = self._obs.get(int(vid))
            if st is None:
                return None
            st.in_flight = True
            st.last_submit_frame_index = int(frame_index)
            submitted = self._worker.submit((int(vid), plate_patch, tuple(int(x) for x in plate_bbox), float(t_now)))
            if not submitted:
                st.in_flight = False
            logger.debug(f"update_plate_for_vehicle: submitted OCR job vid={vid} submitted={submitted}")

        return None

    def recognize_plate_for_vehicle(
        self,
        frame: np.ndarray,
        vehicle_bbox: Tuple[int, int, int, int],
        vehicle_id: int,
        *,
        now_ts: Optional[float] = None,
    ) -> Optional[Tuple[str, float, Tuple[int, int, int, int]]]:
        """Recognize a plate for a specific vehicle ID using temporal aggregation.

        Returns a confirmed/stable plate only when the tracker considers it reliable.
        """
        # Backwards-compatible API: now non-blocking.
        return self.update_plate_for_vehicle(
            frame,
            vehicle_bbox,
            int(vehicle_id),
            frame_index=None,
            now_ts=now_ts,
        )

    def _can_submit_ocr(self, now_ts: float) -> bool:
        max_per_sec = float(getattr(config, "LP_OCR_MAX_CALLS_PER_SECOND", 2.0) or 2.0)
        max_per_sec = max(0.0, min(30.0, max_per_sec))
        if max_per_sec <= 0.0:
            return False

        with self._lock:
            t = float(now_ts)
            while self._ocr_rate_window and (t - float(self._ocr_rate_window[0])) > 1.0:
                self._ocr_rate_window.popleft()
            if len(self._ocr_rate_window) >= int(max_per_sec):
                return False
            self._ocr_rate_window.append(t)
            return True

    def _process_ocr_job(self, job):
        """Worker thread: run OCR on a cropped/warped plate patch and publish the result."""
        try:
            vid, patch, bbox, submit_ts = job
            vid = int(vid)
            bbox = tuple(int(x) for x in bbox)
            t0 = time.time()
        except Exception:
            return

        logger.debug(f"_process_ocr_job start vid={vid} submit_ts={submit_ts}")

        plate_text = None
        conf = 0.0
        try:
            plate_text, conf = self._run_ocr_on_plate_patch(patch)
        except Exception:
            plate_text, conf = (None, 0.0)

        elapsed = float(time.time() - t0)
        timeout_s = float(getattr(config, "LP_OCR_TIMEOUT_SECONDS", 0.60) or 0.60)
        timeout_s = max(0.05, min(5.0, timeout_s))
        if elapsed > timeout_s:
            plate_text = None
            conf = 0.0

        logger.debug(f"_process_ocr_job done vid={vid} elapsed={elapsed:.3f}s plate={plate_text} conf={conf}")

        plate_text = _normalize_plate_text(plate_text or "")
        if not _is_valid_plate(plate_text):
            plate_text = None
            conf = 0.0

        with self._lock:
            st = self._obs.get(int(vid))
            if st is not None:
                st.in_flight = False

            if plate_text:
                self._ocr_results[int(vid)] = (str(plate_text), float(conf), bbox, float(time.time()))
                logger.debug(f"_process_ocr_job stored result vid={vid} plate={plate_text} conf={conf}")

    def _run_ocr_on_plate_patch(self, patch: np.ndarray) -> Tuple[Optional[str], float]:
        """Preprocess and OCR a plate patch. Never called on full frame."""
        if patch is None or getattr(patch, "size", 0) == 0:
            return None, 0.0

        processed_list = self._preprocess_plate_for_ocr(patch)
        best_plate = None
        best_conf = 0.0
        for img in processed_list:
            text, confidence = self._perform_ocr(img)
            cand = _normalize_plate_text(text or "")
            if not _is_valid_plate(cand):
                continue
            if float(confidence) > float(best_conf):
                best_plate = cand
                best_conf = float(confidence)

        min_conf = float(getattr(config, "LP_MIN_OCR_CONFIDENCE", 35.0) or 35.0)
        min_conf = max(0.0, min(100.0, min_conf))
        if (best_plate is None) or (float(best_conf) < float(min_conf)):
            return None, 0.0
        return best_plate, float(best_conf)

    def _preprocess_plate_for_ocr(self, plate_patch: np.ndarray) -> List[np.ndarray]:
        """Preprocessing per spec: grayscale, upscale, denoise, adaptive threshold."""
        if plate_patch is None or getattr(plate_patch, "size", 0) == 0:
            return []

        if len(plate_patch.shape) == 3:
            gray = cv2.cvtColor(plate_patch, cv2.COLOR_BGR2GRAY)
        else:
            gray = plate_patch.copy()

        # Upscale 2x or 3x.
        scale = float(getattr(config, "LP_OCR_UPSCALE", 2.0) or 2.0)
        if scale < 1.0:
            scale = 1.0
        if scale > 4.0:
            scale = 4.0
        if scale != 1.0:
            try:
                gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            except Exception:
                pass

        # Noise reduction.
        try:
            gray = cv2.bilateralFilter(gray, 7, 25, 25)
        except Exception:
            try:
                gray = cv2.GaussianBlur(gray, (3, 3), 0)
            except Exception:
                pass

        # Adaptive threshold + Otsu (two variants).
        try:
            block = int(getattr(config, "LP_ADAPTIVE_BLOCK", 21) or 21)
            if block % 2 == 0:
                block += 1
            block = max(9, min(51, block))
            c_val = int(getattr(config, "LP_ADAPTIVE_C", 5) or 5)
            adapt = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, block, c_val)
        except Exception:
            adapt = None

        try:
            _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        except Exception:
            otsu = None

        out: List[np.ndarray] = [gray]
        if otsu is not None:
            out.append(otsu)
        if adapt is not None:
            out.append(adapt)
        return out

    def _detect_plate_candidate_bbox_and_patch(
        self,
        frame: np.ndarray,
        vehicle_bbox: Tuple[int, int, int, int],
    ) -> Optional[Tuple[Tuple[int, int, int, int], np.ndarray]]:
        """Detect a plate candidate bbox + patch WITHOUT OCR.

        Returns (global_bbox, rectified_patch) or None.
        """
        if frame is None or getattr(frame, "size", 0) == 0:
            return None

        try:
            x, y, w, h = [int(round(float(v))) for v in vehicle_bbox]
        except Exception:
            return None
        if w <= 1 or h <= 1:
            return None

        def _scan(roi_bgr: np.ndarray, ox: int, oy: int) -> Optional[Tuple[Tuple[int, int, int, int], np.ndarray]]:
            if roi_bgr is None or roi_bgr.size == 0:
                return None
            cands = self._find_plate_candidate_patches(roi_bgr)
            if not cands:
                return None
            patch, bbox = cands[0]
            try:
                bx, by, bw, bh = [int(v) for v in bbox]
            except Exception:
                return None
            gb = (int(ox + bx), int(oy + by), int(bw), int(bh))
            logger.debug(f"_detect_plate_candidate_bbox_and_patch: found candidate global_bbox={gb}")
            return (gb, patch)

        # 1) Prefer placeholder ROI above/roof.
        try:
            ph = self.placeholder_plate_bbox(frame.shape, (x, y, w, h))
        except Exception:
            ph = None
        if ph is not None:
            try:
                px, py, pw, phh = [int(v) for v in ph]
                roi = frame[py : py + phh, px : px + pw]
                found = _scan(roi, px, py)
                if found:
                    return found
            except Exception:
                pass

        # 2) Fallback: top portion of vehicle ROI.
        padding = int(getattr(config, "LP_VEHICLE_PADDING_PX", 10) or 10)
        padding = max(0, min(50, padding))

        above_extra_base = int(getattr(config, "LP_ABOVE_VEHICLE_EXTRA_PX", 25) or 25)
        above_extra_ratio = float(getattr(config, "LP_ABOVE_VEHICLE_EXTRA_RATIO", 0.85) or 0.85)
        above_extra_ratio = max(0.0, min(1.5, above_extra_ratio))
        above_extra_dyn = int(round(float(h) * above_extra_ratio))
        above_extra = int(max(above_extra_base, above_extra_dyn))
        above_extra = max(0, min(120, above_extra))

        x1 = max(0, x - padding)
        y1 = max(0, y - padding - above_extra)
        x2 = min(frame.shape[1], x + w + padding)
        y2 = min(frame.shape[0], y + h + padding)
        roi = frame[y1:y2, x1:x2]
        if roi is None or roi.size == 0:
            return None

        roof_y_max_ratio = float(getattr(config, "LP_ROOF_Y_MAX_RATIO", 0.45) or 0.45)
        roof_y_max_ratio = max(0.15, min(1.0, roof_y_max_ratio))
        rx_margin_ratio = float(getattr(config, "LP_ROOF_X_MARGIN_RATIO", 0.05) or 0.05)
        rx_margin_ratio = max(0.0, min(0.30, rx_margin_ratio))

        rh = int(roi.shape[0])
        rw = int(roi.shape[1])
        roof_y2 = max(1, int(round(rh * roof_y_max_ratio)))
        mx = int(round(rw * rx_margin_ratio))
        roof = roi[0:roof_y2, mx: max(mx + 1, rw - mx)]
        found = _scan(roof, int(x1 + mx), int(y1))
        if found:
            return found

        # 3) Final fallback: scan the full vehicle ROI.
        found = _scan(roi, int(x1), int(y1))
        if found:
            return found

        # 4) Contour miss fallback: still run OCR on a bounded roof/placeholder ROI.
        # This keeps live detection working when edges/white masks fail due to blur,
        # glare, or low contrast in the camera feed.
        return self._fallback_plate_roi_patch(frame, (x, y, w, h), placeholder_bbox=ph)

    def _fallback_plate_roi_patch(
        self,
        frame: np.ndarray,
        vehicle_bbox: Tuple[int, int, int, int],
        *,
        placeholder_bbox: Optional[Tuple[int, int, int, int]] = None,
    ) -> Optional[Tuple[Tuple[int, int, int, int], np.ndarray]]:
        """Return a bounded fallback patch when contour candidate extraction fails."""
        if frame is None or getattr(frame, "size", 0) == 0:
            return None

        bbox = placeholder_bbox
        if bbox is None:
            try:
                bbox = self.placeholder_plate_bbox(frame.shape, vehicle_bbox)
            except Exception:
                bbox = None
        if bbox is None:
            # Final fallback: derive a conservative roof strip directly from the
            # vehicle bbox so OCR can still run even when placeholder estimation fails.
            try:
                x, y, w, h = [int(round(float(v))) for v in vehicle_bbox]
                if w > 1 and h > 1:
                    H, W = int(frame.shape[0]), int(frame.shape[1])
                    x_margin_ratio = float(getattr(config, "LP_ROOF_X_MARGIN_RATIO", 0.05) or 0.05)
                    x_margin_ratio = max(0.0, min(0.30, x_margin_ratio))
                    roof_ratio = float(getattr(config, "LP_ROOF_Y_MAX_RATIO", 0.45) or 0.45)
                    roof_ratio = max(0.15, min(1.0, roof_ratio))

                    mx = int(round(w * x_margin_ratio))
                    bx = int(max(0, min(W - 1, x + mx)))
                    bw = int(max(1, min(W - bx, w - (2 * mx))))

                    above = int(max(0, int(round(h * 0.25))))
                    by = int(max(0, min(H - 1, y - above)))
                    bh = int(max(1, min(H - by, int(round(h * roof_ratio)) + above)))

                    if bw > 1 and bh > 1:
                        bbox = (bx, by, bw, bh)
            except Exception:
                bbox = None
        if bbox is None:
            return None

        try:
            bx, by, bw, bh = [int(round(float(v))) for v in bbox]
        except Exception:
            return None

        H, W = int(frame.shape[0]), int(frame.shape[1])
        bx = max(0, min(W - 1, bx))
        by = max(0, min(H - 1, by))
        bw = max(1, min(W - bx, bw))
        bh = max(1, min(H - by, bh))

        # Keep OCR patch bounded for stable latency in async worker.
        max_w = int(getattr(config, "LP_FALLBACK_ROI_MAX_WIDTH", 260) or 260)
        max_h = int(getattr(config, "LP_FALLBACK_ROI_MAX_HEIGHT", 120) or 120)
        max_w = max(80, min(480, max_w))
        max_h = max(36, min(240, max_h))

        if bw > max_w:
            cx = bx + (bw // 2)
            bw = max_w
            bx = max(0, min(W - bw, cx - (bw // 2)))
        if bh > max_h:
            cy = by + (bh // 2)
            bh = max_h
            by = max(0, min(H - bh, cy - (bh // 2)))

        patch = frame[by : by + bh, bx : bx + bw]
        if patch is None or patch.size == 0:
            return None

        return ((int(bx), int(by), int(bw), int(bh)), patch)

    def placeholder_plate_bbox(
        self,
        frame_shape: Tuple[int, int, int],
        vehicle_bbox: Tuple[int, int, int, int],
    ) -> Optional[Tuple[int, int, int, int]]:
        """Compute a cheap placeholder bbox for where a roof-mounted plate likely is.

        This does NOT run OCR; it's used to keep a consistent ROI across frames.
        """
        try:
            h_img, w_img = int(frame_shape[0]), int(frame_shape[1])
        except Exception:
            return None

        try:
            x, y, w, h = [int(round(float(v))) for v in vehicle_bbox]
        except Exception:
            return None
        if w <= 1 or h <= 1:
            return None

        # Plates in this project can be placed slightly ABOVE the detected vehicle bbox.
        # Make the above-search height-aware so it still works when the vehicle bbox is
        # a bit low (e.g., detector focuses on the car body but plate sits on top).
        above_extra_base = int(getattr(config, "LP_ABOVE_VEHICLE_EXTRA_PX", 25) or 25)
        above_extra_ratio = float(getattr(config, "LP_ABOVE_VEHICLE_EXTRA_RATIO", 0.85) or 0.85)
        above_extra_ratio = max(0.0, min(1.5, above_extra_ratio))
        above_extra_dyn = int(round(float(h) * above_extra_ratio))
        above_extra = int(max(above_extra_base, above_extra_dyn))
        above_extra = max(0, min(120, above_extra))
        roof_y_max_ratio = float(getattr(config, "LP_ROOF_Y_MAX_RATIO", 0.45) or 0.45)
        roof_y_max_ratio = max(0.15, min(1.0, roof_y_max_ratio))
        rx_margin_ratio = float(getattr(config, "LP_ROOF_X_MARGIN_RATIO", 0.05) or 0.05)
        rx_margin_ratio = max(0.0, min(0.30, rx_margin_ratio))

        mx = int(round(w * rx_margin_ratio))
        ph = int(round(h * roof_y_max_ratio)) + int(above_extra)
        px = int(x + mx)
        py = int(y - above_extra)
        pw = int(max(1, w - 2 * mx))
        ph = int(max(1, ph))

        # Clamp within frame
        px = int(max(0, min(w_img - 1, px)))
        py = int(max(0, min(h_img - 1, py)))
        px2 = int(max(0, min(w_img, px + pw)))
        py2 = int(max(0, min(h_img, py + ph)))
        if px2 <= px or py2 <= py:
            return None
        return (px, py, int(px2 - px), int(py2 - py))

    def _detect_plate_once(
        self,
        frame: np.ndarray,
        vehicle_bbox: Tuple[int, int, int, int],
    ) -> Optional[Tuple[str, float, Tuple[int, int, int, int]]]:
        """Run a single OCR attempt (no temporal logic)."""
        if not self.ocr_available:
            return None

        def _scan_roi(
            roi_bgr: np.ndarray,
            *,
            roi_offset_x: int,
            roi_offset_y: int,
        ) -> Optional[Tuple[str, float, Tuple[int, int, int, int]]]:
            """Scan a given ROI for a plate and return global bbox."""
            if roi_bgr is None or roi_bgr.size == 0:
                return None

            candidates = self._find_plate_candidate_patches(roi_bgr)
            if not candidates:
                candidates = [(roi_bgr, (0, 0, int(roi_bgr.shape[1]), int(roi_bgr.shape[0])))]

            best_text = None
            best_conf = 0.0
            best_bbox = None

            max_candidates = int(getattr(config, "LP_MAX_CANDIDATES_PER_VEHICLE", 3) or 3)
            max_candidates = max(1, min(10, max_candidates))

            for patch, bbox in candidates[:max_candidates]:
                variants = self._preprocess_plate_variants(patch)

                # Tesseract can be slow; limit the number of variants we attempt.
                # Template OCR is cheap, EasyOCR is moderate.
                if self.reader is None and self.tesseract_available and (not (self._template_enabled and self._templates)):
                    max_variants = int(getattr(config, "LP_TESSERACT_MAX_VARIANTS", 2) or 2)
                    max_variants = max(1, min(5, max_variants))
                    variants = variants[:max_variants]

                for plate_processed in variants:
                    text, confidence = self._perform_ocr(plate_processed)
                    text = _normalize_plate_text(text or "")
                    if not _is_valid_plate(text):
                        continue
                    if float(confidence) > best_conf:
                        best_text = text
                        best_conf = float(confidence)
                        best_bbox = bbox

            if not best_text or best_bbox is None:
                return None

            px, py, pw, ph = [int(v) for v in best_bbox]
            global_bbox = (int(roi_offset_x + px), int(roi_offset_y + py), int(pw), int(ph))
            return (best_text, float(best_conf), global_bbox)

        # Bounding boxes may be floats from detectors; convert safely to ints for slicing
        x, y, w, h = [int(round(float(v))) for v in vehicle_bbox]
        if w <= 1 or h <= 1:
            return None

        # Pass 0: scan a dedicated "above/roof" placeholder ROI (global coords).
        # This is the most reliable path for roof/above-mounted plates.
        try:
            ph_bbox = self.placeholder_plate_bbox(frame.shape, (x, y, w, h))
        except Exception:
            ph_bbox = None
        if ph_bbox is not None:
            try:
                px, py, pw, ph = [int(v) for v in ph_bbox]
                roi = frame[py : py + ph, px : px + pw]
                found = _scan_roi(roi, roi_offset_x=int(px), roi_offset_y=int(py))
                if found:
                    return found
            except Exception:
                pass

        padding = int(getattr(config, "LP_VEHICLE_PADDING_PX", 10) or 10)
        padding = max(0, min(50, padding))

        # Keep consistent with placeholder_plate_bbox() and allow above-mounted plates.
        above_extra_base = int(getattr(config, "LP_ABOVE_VEHICLE_EXTRA_PX", 25) or 25)
        above_extra_ratio = float(getattr(config, "LP_ABOVE_VEHICLE_EXTRA_RATIO", 0.85) or 0.85)
        above_extra_ratio = max(0.0, min(1.5, above_extra_ratio))
        above_extra_dyn = int(round(float(h) * above_extra_ratio))
        above_extra = int(max(above_extra_base, above_extra_dyn))
        above_extra = max(0, min(120, above_extra))

        x1 = max(0, x - padding)
        # Allow searching slightly ABOVE the vehicle bbox for roof/above-mounted plates.
        y1 = max(0, y - padding - above_extra)
        x2 = min(frame.shape[1], x + w + padding)
        y2 = min(frame.shape[0], y + h + padding)

        vehicle_roi = frame[y1:y2, x1:x2]
        if vehicle_roi.size == 0:
            return None

        # Pass 1 (fast): roof-mounted plate constraint — prioritize the top portion.
        roof_y_max_ratio = float(getattr(config, "LP_ROOF_Y_MAX_RATIO", 0.45) or 0.45)
        roof_y_max_ratio = max(0.15, min(1.0, roof_y_max_ratio))
        rx_margin_ratio = float(getattr(config, "LP_ROOF_X_MARGIN_RATIO", 0.05) or 0.05)
        rx_margin_ratio = max(0.0, min(0.30, rx_margin_ratio))

        rh = vehicle_roi.shape[0]
        rw = vehicle_roi.shape[1]
        roof_y2 = max(1, int(round(rh * roof_y_max_ratio)))
        mx = int(round(rw * rx_margin_ratio))
        roof = vehicle_roi[0:roof_y2, mx: max(mx + 1, rw - mx)]

        if roof is not None and roof.size > 0:
            found = _scan_roi(roof, roi_offset_x=int(x1 + mx), roi_offset_y=int(y1))
            if found:
                return found

        # Pass 2 (fallback): scan the full vehicle ROI.
        # This handles cases where the plate sits mid-body in the crop (or when the
        # detector bbox doesn't tightly align with the roof).
        return _scan_roi(vehicle_roi, roi_offset_x=int(x1), roi_offset_y=int(y1))
    
    def _find_plate_candidate_patches(self, image: np.ndarray) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """Find potential plate regions and return (rectified_patch, bbox) pairs.

        Uses rotated rectangles (minAreaRect) to tolerate sideways/tilted plates.
        """
        # Strategy 1 (preferred for this project): explicit white-plate mask.
        # Roof-mounted toy-car plates are usually white paper, and edge-based
        # contours can produce large false positives that would otherwise dominate.
        try:
            white_first = self._find_white_plate_candidates(image) or []
        except Exception:
            white_first = []

        # Strategy 2: edge/contour-based rotated rectangles.
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 9, 25, 25)

        edges = cv2.Canny(gray, 30, 200)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates: List[Tuple[float, np.ndarray, Tuple[int, int, int, int]]] = []

        for contour in contours:
            if contour is None or len(contour) < 4:
                continue

            rect = cv2.minAreaRect(contour)
            (_cx, _cy), (rw, rh), _angle = rect
            rw = float(rw)
            rh = float(rh)
            if rw <= 1.0 or rh <= 1.0:
                continue

            # Normalize so rw >= rh for aspect filtering.
            w = max(rw, rh)
            h = min(rw, rh)

            if not (float(getattr(config, "LP_MIN_WIDTH", 40)) <= w <= float(getattr(config, "LP_MAX_WIDTH", 250))):
                continue
            if not (float(getattr(config, "LP_MIN_HEIGHT", 12)) <= h <= float(getattr(config, "LP_MAX_HEIGHT", 80))):
                continue

            aspect_ratio = w / max(1.0, h)
            if not (float(getattr(config, "LP_ASPECT_RATIO_MIN", 1.8)) <= aspect_ratio <= float(getattr(config, "LP_ASPECT_RATIO_MAX", 8.0))):
                continue

            patch, bbox = self._warp_rect(image, rect)
            if patch is None or patch.size == 0:
                continue

            area = float(w * h)
            candidates.append((area, patch, bbox))

        # Prefer larger edge candidates first (more OCR signal)
        candidates.sort(key=lambda t: t[0], reverse=True)
        edge_out = [(p, b) for _, p, b in candidates]

        # Merge: keep white candidates FIRST, then edge candidates.
        merged = []
        seen = set()
        for patch, bbox in list(white_first) + list(edge_out):
            try:
                bx, by, bw, bh = [int(v) for v in bbox]
                key = (int(bx), int(by), int(bw), int(bh))
            except Exception:
                continue
            if key in seen:
                continue
            seen.add(key)
            merged.append((patch, (int(bx), int(by), int(bw), int(bh))))

        max_all = int(getattr(config, "LP_MAX_CANDIDATES_TOTAL", 8) or 8)
        max_all = max(1, min(25, max_all))
        return merged[:max_all]

    def _find_white_plate_candidates(self, image: np.ndarray) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """Find candidate plate patches by thresholding for white regions.

        This targets the common setup in this project: a white printed paper plate
        stuck on top of a toy car.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        s_max = int(getattr(config, "LP_WHITE_S_MAX", 70) or 70)
        v_min = int(getattr(config, "LP_WHITE_V_MIN", 160) or 160)
        s_max = max(0, min(255, s_max))
        v_min = max(0, min(255, v_min))

        lower = np.array([0, 0, v_min], dtype=np.uint8)
        upper = np.array([179, s_max, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        # Morphology to consolidate the paper region.
        k_close = int(getattr(config, "LP_WHITE_CLOSE_KERNEL", 7) or 7)
        k_open = int(getattr(config, "LP_WHITE_OPEN_KERNEL", 3) or 3)
        k_close = max(1, min(31, k_close))
        k_open = max(1, min(15, k_open))

        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k_close, k_close))
        open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k_open, k_open))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []

        min_area = float(getattr(config, "LP_WHITE_MIN_AREA", 200.0) or 200.0)
        min_area = max(10.0, min(20000.0, min_area))

        candidates: List[Tuple[float, np.ndarray, Tuple[int, int, int, int]]] = []
        for c in contours:
            if c is None or len(c) < 3:
                continue
            area = float(cv2.contourArea(c))
            if area < min_area:
                continue

            # Use rotated rectangles so tilted/sideways plates are rectified.
            try:
                rect = cv2.minAreaRect(c)
                (cx, cy), (rw, rh), angle = rect
                rw = float(rw)
                rh = float(rh)
                if rw <= 1.0 or rh <= 1.0:
                    continue

                ww = float(max(rw, rh))
                hh = float(min(rw, rh))
                if not (float(getattr(config, "LP_MIN_WIDTH", 40)) <= ww <= float(getattr(config, "LP_MAX_WIDTH", 250))):
                    continue
                if not (float(getattr(config, "LP_MIN_HEIGHT", 12)) <= hh <= float(getattr(config, "LP_MAX_HEIGHT", 80))):
                    continue

                aspect_ratio = ww / max(1.0, hh)
                if not (
                    float(getattr(config, "LP_ASPECT_RATIO_MIN", 1.8))
                    <= aspect_ratio
                    <= float(getattr(config, "LP_ASPECT_RATIO_MAX", 8.0))
                ):
                    continue

                patch, bbox = self._warp_rect(image, rect)
                if patch is None or patch.size == 0:
                    continue
                candidates.append((float(ww * hh), patch, bbox))
            except Exception:
                continue

        candidates.sort(key=lambda t: t[0], reverse=True)
        return [(p, b) for _, p, b in candidates]

    def _warp_rect(
        self, image: np.ndarray, rect
    ) -> Tuple[Optional[np.ndarray], Tuple[int, int, int, int]]:
        """Warp a rotated rect into an upright patch."""
        box = cv2.boxPoints(rect)
        box = np.array(box, dtype=np.float32)

        x, y, w, h = cv2.boundingRect(box.astype(np.int32))
        if w <= 1 or h <= 1:
            return None, (0, 0, 0, 0)

        # Order points: tl, tr, br, bl
        s = box.sum(axis=1)
        diff = np.diff(box, axis=1)
        tl = box[np.argmin(s)]
        br = box[np.argmax(s)]
        tr = box[np.argmin(diff)]
        bl = box[np.argmax(diff)]
        pts = np.array([tl, tr, br, bl], dtype=np.float32)

        widthA = np.linalg.norm(br - bl)
        widthB = np.linalg.norm(tr - tl)
        heightA = np.linalg.norm(tr - br)
        heightB = np.linalg.norm(tl - bl)
        maxW = int(round(max(widthA, widthB)))
        maxH = int(round(max(heightA, heightB)))
        if maxW <= 1 or maxH <= 1:
            return None, (x, y, w, h)

        # Enforce plate orientation as wide
        if maxH > maxW:
            maxW, maxH = maxH, maxW

        dst = np.array(
            [[0, 0], [maxW - 1, 0], [maxW - 1, maxH - 1], [0, maxH - 1]],
            dtype=np.float32,
        )

        M = cv2.getPerspectiveTransform(pts, dst)
        warped = cv2.warpPerspective(image, M, (maxW, maxH))
        return warped, (x, y, w, h)
    
    def _preprocess_plate_variants(self, plate_image: np.ndarray) -> List[np.ndarray]:
        """Generate a small set of preprocessed variants for OCR.

        White paper plates often work best with contrast enhancement and either
        normal or inverted binarization depending on lighting.
        """
        if plate_image is None or plate_image.size == 0:
            return []

        if len(plate_image.shape) == 3:
            gray = cv2.cvtColor(plate_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = plate_image.copy()

        # Upscale to give OCR more pixels.
        height, width = gray.shape[:2]
        min_h = int(getattr(config, "LP_OCR_MIN_HEIGHT", 64) or 64)
        min_h = max(32, min(200, min_h))
        if height > 0 and height < min_h:
            scale = float(min_h) / float(height)
            gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)

        # Contrast enhancement.
        try:
            clahe_clip = float(getattr(config, "LP_CLAHE_CLIP", 2.0) or 2.0)
            clahe_clip = max(0.5, min(6.0, clahe_clip))
            clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(8, 8))
            gray_c = clahe.apply(gray)
        except Exception:
            gray_c = gray

        # Threshold variants.
        _, otsu = cv2.threshold(gray_c, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        try:
            block = int(getattr(config, "LP_ADAPTIVE_BLOCK", 21) or 21)
            if block % 2 == 0:
                block += 1
            block = max(9, min(51, block))
            c_val = int(getattr(config, "LP_ADAPTIVE_C", 5) or 5)
            adaptive = cv2.adaptiveThreshold(gray_c, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, block, c_val)
        except Exception:
            adaptive = otsu

        inv_otsu = cv2.bitwise_not(otsu)
        inv_adapt = cv2.bitwise_not(adaptive)

        # Close small gaps so characters are more legible.
        k = int(getattr(config, "LP_BIN_CLOSE_KERNEL", 3) or 3)
        k = max(1, min(9, k))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        otsu = cv2.morphologyEx(otsu, cv2.MORPH_CLOSE, kernel)
        adaptive = cv2.morphologyEx(adaptive, cv2.MORPH_CLOSE, kernel)
        inv_otsu = cv2.morphologyEx(inv_otsu, cv2.MORPH_CLOSE, kernel)
        inv_adapt = cv2.morphologyEx(inv_adapt, cv2.MORPH_CLOSE, kernel)

        # Order: keep grayscale first (EasyOCR sometimes prefers it), then binaries.
        return [gray_c, otsu, adaptive, inv_otsu, inv_adapt]
    
    def _perform_ocr(self, image: np.ndarray) -> Tuple[Optional[str], float]:
        """Perform OCR on preprocessed plate image using EasyOCR"""
        if not self.ocr_available:
            return None, 0.0

        # When EasyOCR isn't available, prefer template OCR first (fast, OpenCV-only),
        # then fall back to Tesseract (slower) if enabled/available.
        if self.reader is None:
            tmpl_plate: Optional[str] = None
            tmpl_conf: float = 0.0
            prefer_template = bool(getattr(config, "LP_PREFER_TEMPLATE_OCR", True)) and (not _is_permissive_alnum_mode())
            if prefer_template and self._template_enabled and self._templates:
                try:
                    plate, conf = self._perform_template_ocr(image)
                except Exception:
                    plate, conf = (None, 0.0)
                if plate:
                    # Template OCR is fast but can be over-confident on some glyphs.
                    # Only accept it immediately when it's very confident; otherwise
                    # fall through to Tesseract (async + rate-limited elsewhere).
                    min_accept = float(getattr(config, "LP_TEMPLATE_ACCEPT_MIN_CONF", 97.0) or 97.0)
                    min_accept = max(0.0, min(100.0, min_accept))
                    tmpl_plate, tmpl_conf = (str(plate), float(conf))
                    if (not self.tesseract_available) or float(conf) >= float(min_accept):
                        return str(plate), float(conf)

            if self.tesseract_available:
                try:
                    t_plate, t_conf = self._perform_tesseract_ocr(image)
                    if t_plate:
                        return t_plate, float(t_conf)
                except Exception:
                    pass

            # If Tesseract failed/unavailable, fall back to template result (if any).
            if prefer_template and self._template_enabled and self._templates and tmpl_plate:
                return str(tmpl_plate), float(tmpl_conf)

            if (not prefer_template) and self._template_enabled and self._templates:
                try:
                    return self._perform_template_ocr(image)
                except Exception:
                    return None, 0.0

            return None, 0.0
        
        try:
            # EasyOCR returns list of (bbox, text, confidence)
            rotation_info = getattr(config, "LP_OCR_ROTATIONS", [0, 90, 180, 270])
            allowlist = getattr(config, "LP_OCR_ALLOWLIST", "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")
            # rotation_info improves sideways/rotated reads for roof-mounted plates.
            results = self.reader.readtext(
                image,
                allowlist=allowlist,
                rotation_info=rotation_info,
                paragraph=False,
            )
            
            if not results:
                logger.debug("_perform_ocr: EasyOCR returned no results")
                return None, 0.0
            logger.debug(f"_perform_ocr: EasyOCR returned {len(results)} results")

            # Evaluate single-token and small multi-token combinations.
            best_plate: Optional[str] = None
            best_conf: float = 0.0

            tokens: List[Tuple[str, float]] = []
            for (_bbox, text, conf) in results:
                t = (text or "").strip()
                if not t:
                    continue
                try:
                    c = float(conf)
                except Exception:
                    c = 0.0
                tokens.append((t, c))

            logger.debug(f"_perform_ocr: EasyOCR tokens: {tokens}")

            if not tokens:
                return None, 0.0

            # 1) Single token candidates.
            for t, c in tokens:
                cand = _extract_plate_from_text(t)
                if cand and c > best_conf:
                    best_plate = cand
                    best_conf = float(c)

            # 2) Combine up to 3 adjacent tokens (common: "ABC" + "123").
            max_join = 3
            for i in range(len(tokens)):
                joined = ""
                confs: List[float] = []
                for j in range(i, min(len(tokens), i + max_join)):
                    joined += tokens[j][0]
                    confs.append(float(tokens[j][1]))
                    cand = _extract_plate_from_text(joined)
                    if not cand:
                        continue
                    c = (sum(confs) / max(1, len(confs)))
                    if c > best_conf:
                        best_plate = cand
                        best_conf = float(c)

            if not best_plate:
                logger.debug("_perform_ocr: no best_plate found from EasyOCR tokens")
                return None, 0.0

            logger.debug(f"_perform_ocr: selected best_plate={best_plate} best_conf={best_conf}")
            return best_plate, float(best_conf * 100.0)
            
        except Exception as e:
            print(f"OCR error: {e}")
            return None, 0.0

    def _perform_tesseract_ocr(self, image: np.ndarray) -> Tuple[Optional[str], float]:
        """OCR using system-level Tesseract via pytesseract.

        Returns (plate, confidence_percent). Confidence is estimated from word-level
        confidences when available; otherwise 0.
        """
        if not (self.tesseract_available and TESS_AVAILABLE):
            return None, 0.0

        if image is None or getattr(image, "size", 0) == 0:
            return None, 0.0

        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        rotations = getattr(config, "LP_TESSERACT_ROTATIONS", None)
        if rotations is None:
            rotations = getattr(config, "LP_OCR_ROTATIONS", [0])
        allowlist = getattr(config, "LP_OCR_ALLOWLIST", "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")
        base_cfg = str(getattr(config, "TESSERACT_CONFIG", "--psm 8 --oem 3") or "--psm 8 --oem 3")
        # Ensure whitelist is applied even if config doesn't include it.
        if "tessedit_char_whitelist" not in base_cfg:
            base_cfg = f"{base_cfg} -c tessedit_char_whitelist={allowlist}"

        cfg_candidates: List[str] = [base_cfg]
        extra_psms = getattr(config, "LP_TESSERACT_EXTRA_PSMS", [7, 6])
        try:
            extra_psms = list(extra_psms) if extra_psms is not None else []
        except Exception:
            extra_psms = [7, 6]
        for psm in extra_psms:
            try:
                p = int(psm)
            except Exception:
                continue
            if p < 3 or p > 13:
                continue
            token = f"--psm {p}"
            if token in base_cfg:
                continue
            cfg_candidates.append(f"--psm {p} --oem 3 -c tessedit_char_whitelist={allowlist}")

        best_plate: Optional[str] = None
        best_conf: float = 0.0

        for cfg in cfg_candidates:
            for r in (rotations or [0]):
                try:
                    angle = int(r)
                except Exception:
                    angle = 0

                g = gray
                if angle % 360 == 90:
                    g = cv2.rotate(gray, cv2.ROTATE_90_CLOCKWISE)
                elif angle % 360 == 180:
                    g = cv2.rotate(gray, cv2.ROTATE_180)
                elif angle % 360 == 270:
                    g = cv2.rotate(gray, cv2.ROTATE_90_COUNTERCLOCKWISE)

                # Slightly boost contrast for Tesseract.
                g = cv2.GaussianBlur(g, (3, 3), 0)
                g = cv2.normalize(g, None, 0, 255, cv2.NORM_MINMAX)

                pil_img = Image.fromarray(g)

                # Use image_to_data for confidence. Tesseract conf is -1..100.
                try:
                    data = pytesseract.image_to_data(pil_img, config=cfg, output_type=_TESS_OUTPUT.DICT)
                    words = data.get("text", []) or []
                    confs = data.get("conf", []) or []
                except Exception:
                    words = []
                    confs = []

                joined = "".join(str(w or "") for w in words)
                plate = _extract_plate_from_text(joined)
                if not plate:
                    # Fallback to plain string in case data output is sparse.
                    try:
                        raw = pytesseract.image_to_string(pil_img, config=cfg)
                    except Exception:
                        raw = ""
                    plate = _extract_plate_from_text(raw)
                    if not plate:
                        continue

                # Estimate confidence as mean of valid word confidences.
                valid = []
                for c in confs:
                    try:
                        v = float(c)
                    except Exception:
                        continue
                    if v >= 0.0:
                        valid.append(v)
                est = float(sum(valid) / len(valid)) if valid else 0.0
                est = max(0.0, min(100.0, est))

                if est > best_conf:
                    best_plate = plate
                    best_conf = est

        return best_plate, float(best_conf)

    def _build_ocr_templates(self) -> Dict[str, List[np.ndarray]]:
        """Build glyph templates for A-Z and 0-9 using a few Hershey fonts."""
        size = int(self._template_size)
        fonts = [
            cv2.FONT_HERSHEY_SIMPLEX,
            cv2.FONT_HERSHEY_DUPLEX,
            cv2.FONT_HERSHEY_COMPLEX,
        ]
        scales = [0.9, 1.0, 1.1]
        thicknesses = [2, 3]

        chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
        templates: Dict[str, List[np.ndarray]] = {c: [] for c in chars}

        for ch in chars:
            for font in fonts:
                for scale in scales:
                    for thick in thicknesses:
                        img = np.zeros((size, size), dtype=np.uint8)
                        (tw, th), baseline = cv2.getTextSize(ch, font, float(scale), int(thick))
                        x = max(0, (size - tw) // 2)
                        y = max(th, (size + th) // 2)
                        cv2.putText(img, ch, (x, y), font, float(scale), 255, int(thick), cv2.LINE_AA)
                        _, bw = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY)
                        templates[ch].append(bw)

        return templates

    def _perform_template_ocr(self, image: np.ndarray) -> Tuple[Optional[str], float]:
        """Template-based OCR specialized for AAA999 roof plates.

        This is a lightweight fallback intended for the toy-car setup where the
        plate has high contrast and a fixed length/format.
        """
        if not (self._template_enabled and self._templates):
            return None, 0.0

        if image is None or getattr(image, "size", 0) == 0:
            return None, 0.0

        def _run(gray_img: np.ndarray) -> Tuple[Optional[str], float]:
            # Normalize and binarize.
            g = cv2.GaussianBlur(gray_img, (3, 3), 0)
            _, bw = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            # Ensure characters are white on black.
            white_ratio = float(np.mean(bw > 127))
            if white_ratio > 0.55:
                char_mask = cv2.bitwise_not(bw)
            else:
                char_mask = bw

            # Clean small speckles.
            k = int(getattr(config, "LP_TEMPLATE_OCR_OPEN_KERNEL", 2) or 2)
            k = max(1, min(5, k))
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
            char_mask = cv2.morphologyEx(char_mask, cv2.MORPH_OPEN, kernel)

            H, W = char_mask.shape[:2]
            if H < 8 or W < 8:
                return None, 0.0

            contours, _ = cv2.findContours(char_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None, 0.0

            boxes: List[Tuple[int, int, int, int, float]] = []
            for c in contours:
                x, y, w, h = cv2.boundingRect(c)
                area = float(w * h)

                # Character-like size constraints (relative to patch size).
                if h < int(0.35 * H) or h > int(0.98 * H):
                    continue
                if w < int(0.03 * W) or w > int(0.40 * W):
                    continue
                if area < float(max(20, int(0.002 * H * W))):
                    continue
                boxes.append((x, y, w, h, area))

            if len(boxes) < 6:
                return None, 0.0

            boxes.sort(key=lambda t: int(t[0]))

            # If we have more than 6 candidates, pick the best 6-length window.
            if len(boxes) > 6:
                best_i = 0
                best_score = -1e9
                for i in range(0, len(boxes) - 6 + 1):
                    win = boxes[i : i + 6]
                    sum_area = sum(float(b[4]) for b in win)
                    gaps = []
                    for k2 in range(5):
                        gaps.append(max(0, int(win[k2 + 1][0]) - int(win[k2][0] + win[k2][2])))
                    gap_pen = float(np.std(gaps)) if gaps else 0.0
                    score = float(sum_area) - (15.0 * gap_pen)
                    if score > best_score:
                        best_score = score
                        best_i = i
                boxes = boxes[best_i : best_i + 6]

            size = int(self._template_size)
            min_char_score = float(getattr(config, "LP_TEMPLATE_MIN_CHAR_SCORE", 0.45) or 0.45)
            min_char_score = max(0.0, min(0.99, min_char_score))

            letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            digits = "0123456789"

            plate_chars: List[str] = []
            plate_scores: List[float] = []
            for idx, (x, y, w, h, _area) in enumerate(boxes[:6]):
                pad = int(getattr(config, "LP_TEMPLATE_OCR_CHAR_PAD", 2) or 2)
                pad = max(0, min(8, pad))
                x1 = max(0, x - pad)
                y1 = max(0, y - pad)
                x2 = min(W, x + w + pad)
                y2 = min(H, y + h + pad)
                roi = char_mask[y1:y2, x1:x2]
                if roi.size == 0:
                    return None, 0.0

                # Normalize to square.
                rh, rw = roi.shape[:2]
                side = int(max(rh, rw))
                canvas = np.zeros((side, side), dtype=np.uint8)
                ox = (side - rw) // 2
                oy = (side - rh) // 2
                canvas[oy : oy + rh, ox : ox + rw] = roi
                glyph = cv2.resize(canvas, (size, size), interpolation=cv2.INTER_AREA)

                allow = letters if idx < 3 else digits
                best_ch = None
                best = -1.0
                for ch in allow:
                    for tmpl in self._templates.get(ch, []):
                        s = float(cv2.matchTemplate(glyph, tmpl, cv2.TM_CCOEFF_NORMED)[0][0])
                        if s > best:
                            best = s
                            best_ch = ch

                if best_ch is None:
                    return None, 0.0
                if best < min_char_score:
                    return None, 0.0

                plate_chars.append(best_ch)
                plate_scores.append(float(best))

            plate = "".join(plate_chars)
            plate = _normalize_plate_text(plate)
            if not _is_valid_plate(plate):
                return None, 0.0

            avg = float(sum(plate_scores) / max(1, len(plate_scores)))
            return plate, float(max(0.0, min(1.0, avg)) * 100.0)

        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        rotations = getattr(config, "LP_TEMPLATE_OCR_ROTATIONS", [0, 90, 180, 270])
        best_plate = None
        best_conf = 0.0
        for r in rotations or [0]:
            try:
                angle = int(r)
            except Exception:
                angle = 0

            g = gray
            if angle % 360 == 90:
                g = cv2.rotate(gray, cv2.ROTATE_90_CLOCKWISE)
            elif angle % 360 == 180:
                g = cv2.rotate(gray, cv2.ROTATE_180)
            elif angle % 360 == 270:
                g = cv2.rotate(gray, cv2.ROTATE_90_COUNTERCLOCKWISE)

            plate, conf = _run(g)
            if plate and float(conf) > float(best_conf):
                best_plate = plate
                best_conf = float(conf)

        return best_plate, float(best_conf)
    
    def draw_license_plate(self, frame: np.ndarray, plate_text: str, 
                          plate_bbox: Tuple[int, int, int, int], confidence: float) -> np.ndarray:
        """Draw license plate detection on frame"""
        x, y, w, h = map(int, plate_bbox)
        
        box_color = tuple(getattr(config, "COLOR_LICENSE_PLATE_BBOX", (0, 255, 0)))
        text_color = tuple(getattr(config, "COLOR_LICENSE_PLATE", (0, 255, 255)))
        cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
        
        label = f"{plate_text} ({confidence:.1f}%)"
        cv2.putText(frame, label, (x, y - 10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
        
        return frame


# Test function
if __name__ == "__main__":
    print("License Plate Detector Test")
    print(f"EasyOCR available: {OCR_AVAILABLE}")
    
    if OCR_AVAILABLE:
        detector = LicensePlateDetector()
        print("OK License plate detector initialized")
    else:
        print("Install easyocr: pip install easyocr")
