"""License plate detection and OCR using EasyOCR with an ultra-fast zero-delay worker."""

import os
import cv2
import numpy as np
from typing import Optional, Tuple, List, Dict
import re
import threading
import time
import config

try:
    import torch
    # Set PyTorch to utilize all available CPU cores for maximum speed
    torch.set_num_threads(os.cpu_count() or 4)
except Exception:
    pass

try:
    import easyocr
    OCR_AVAILABLE = True
except ImportError:
    OCR_AVAILABLE = False
    print("Warning: easyocr not installed. License plate OCR will be disabled.")


def _extract_plate_from_text(raw_text: str) -> Optional[str]:
    """Extract and sanitize plate alphanumeric text."""
    if not raw_text:
        return None
    cleaned = re.sub(r'[^A-Za-z0-9]', '', raw_text).upper()
    if not cleaned:
        return None
    
    # Correct character confusion for AAA999 pattern if length is 6
    if len(cleaned) == 6:
        corrected = []
        char_to_num = {'O': '0', 'Q': '0', 'D': '0', 'I': '1', 'L': '1', 'Z': '2', 'S': '5', 'B': '8', 'G': '6', 'T': '7'}
        num_to_char = {'0': 'O', '1': 'I', '2': 'Z', '5': 'S', '8': 'B'}
        for idx, char in enumerate(cleaned):
            if idx < 3:
                corrected.append(num_to_char.get(char, char) if char.isdigit() else char)
            else:
                corrected.append(char_to_num.get(char, char) if char.isalpha() else char)
        cleaned = "".join(corrected)

    if 2 <= len(cleaned) <= 12:
        return cleaned
    return None


class LicensePlateDetector:
    """Detect and read license plates instantly with zero queue delay."""

    def __init__(self):
        self.ocr_available = OCR_AVAILABLE
        self.ocr_mode = "easyocr" if OCR_AVAILABLE else "none"
        self.reader = None
        if self.ocr_available:
            try:
                # Initialize EasyOCR reader for English text (CPU mode)
                self.reader = easyocr.Reader(['en'], gpu=False)
                print("[LicensePlateDetector] EasyOCR initialized with CPU multi-threading")
            except Exception as e:
                print(f"Failed to initialize EasyOCR: {e}")
                self.ocr_available = False
                self.ocr_mode = "none"

        # Zero-lag latest-frame state (no multi-item queue backlog)
        self._latest_job = None
        self._job_event = threading.Event()
        self._results: Dict[int, Optional[List[Tuple[str, float, Tuple[int, int, int, int]]]]] = {}
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        if self.ocr_available:
            self._worker = threading.Thread(
                target=self._ocr_worker,
                name="Instant-OCR-Worker",
                daemon=True,
            )
            self._worker.start()
            print("[LicensePlateDetector] Instant async OCR worker thread started")

    # ── Public async API ──────────────────────────────────────────────────

    def submit_async(
        self,
        vehicle_id: int,
        frame: np.ndarray,
        vehicle_bbox: Tuple[int, int, int, int],
    ) -> bool:
        """Queue the latest frame for OCR. Always overwrites older pending frames for ZERO lag."""
        if not self.ocr_available or frame is None or frame.size == 0:
            return False

        try:
            x, y, w, h = [int(round(v)) for v in vehicle_bbox]
            padding = int(getattr(config, "LP_VEHICLE_PADDING_PX", 10))
            x1 = max(0, x - padding)
            y1 = max(0, y - padding)
            x2 = min(frame.shape[1], x + w + padding)
            y2 = min(frame.shape[0], y + h + padding)
            roi = frame[y1:y2, x1:x2]
            if roi.size == 0:
                return False
            roi = roi.copy()
        except Exception:
            return False

        # Atomically replace latest job so worker ALWAYS processes freshest frame
        with self._lock:
            self._latest_job = (vehicle_id, roi, (x1, y1, x, y, w, h))
        self._job_event.set()
        return True

    def get_result(
        self, vehicle_id: int
    ) -> Optional[List[Tuple[str, float, Tuple[int, int, int, int]]]]:
        """Return and consume the latest OCR results for this vehicle/camera."""
        with self._lock:
            return self._results.pop(vehicle_id, None)

    def shutdown(self):
        """Signal the worker thread to stop (on app exit)."""
        self._stop_event.set()
        self._job_event.set()

    # ── Background worker ─────────────────────────────────────────────────

    def _ocr_worker(self):
        """Background loop: always consume newest frame with zero latency."""
        while not self._stop_event.is_set():
            self._job_event.wait(timeout=0.05)
            self._job_event.clear()
            if self._stop_event.is_set():
                break

            job = None
            with self._lock:
                if self._latest_job is not None:
                    job = self._latest_job
                    self._latest_job = None

            if job is None:
                continue

            vehicle_id, roi, bbox_info = job
            x1, y1, _, _, _, _ = bbox_info

            try:
                result = self._run_full_ocr(roi, x1, y1)
            except Exception as e:
                print(f"[OCR Worker Exception] {e}", flush=True)
                result = None

            with self._lock:
                self._results[vehicle_id] = result

    def _run_full_ocr(
        self,
        vehicle_roi: np.ndarray,
        origin_x: int,
        origin_y: int,
    ) -> List[Tuple[str, float, Tuple[int, int, int, int]]]:
        """Ultra-fast plate localization + recognition pipeline."""
        if not self.ocr_available or self.reader is None:
            return []

        h, w = vehicle_roi.shape[:2]
        if h < 10 or w < 10:
            return []

        valid_plates = []
        seen_texts = set()

        # Step 1: Ultra-fast OpenCV Plate Candidate Localization (<2ms)
        candidates = self._find_plate_candidates(vehicle_roi)

        # Step 2: High-speed OCR on localized candidate crops
        for cx, cy, cw, ch in candidates:
            # Crop candidate with slight padding
            pad_x = max(2, int(cw * 0.05))
            pad_y = max(2, int(ch * 0.08))
            px1 = max(0, cx - pad_x)
            py1 = max(0, cy - pad_y)
            px2 = min(w, cx + cw + pad_x)
            py2 = min(h, cy + ch + pad_y)
            plate_crop = vehicle_roi[py1:py2, px1:px2]
            if plate_crop.size == 0:
                continue

            # Run OCR on small candidate patch with optimized parameters
            text, conf, relative_bbox = self._run_ocr_on_candidate(plate_crop)
            if text and text not in seen_texts:
                seen_texts.add(text)
                # Map bounding box back to global frame coordinates
                if relative_bbox is not None:
                    rx, ry, rw, rh = relative_bbox
                    global_bbox = (origin_x + px1 + rx, origin_y + py1 + ry, rw, rh)
                else:
                    global_bbox = (origin_x + px1, origin_y + py1, px2 - px1, py2 - py1)
                
                print(f"[LPR INSTANT] Plate: '{text}' (Conf: {conf:.1f}%) Box: {global_bbox}", flush=True)
                valid_plates.append((text, conf, global_bbox))

        # If candidates yielded plates, return them immediately!
        if valid_plates:
            return valid_plates

        # Step 3: Fast Downscaled Scan Fallback (if no candidates were found)
        # Instead of 5.2s on 1280x720, downscale to max 640 for ~0.35s scan
        scale = 1.0
        max_dim = max(w, h)
        if max_dim > 640:
            scale = 640.0 / max_dim
            target_w = max(32, int(w * scale))
            target_h = max(32, int(h * scale))
            scan_img = cv2.resize(vehicle_roi, (target_w, target_h), interpolation=cv2.INTER_AREA)
        else:
            scan_img = vehicle_roi

        try:
            results = self.reader.readtext(
                scan_img,
                canvas_size=640,
                mag_ratio=1.0,
                text_threshold=0.4,
                link_threshold=0.3,
                low_text=0.3,
                allowlist=getattr(config, "LP_OCR_ALLOWLIST", "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"),
            )
            for (bbox, text, conf) in results:
                cleaned = _extract_plate_from_text(text)
                if cleaned and cleaned not in seen_texts:
                    seen_texts.add(cleaned)
                    conf_pct = float(conf) * 100.0 if conf <= 1.0 else float(conf)
                    if conf_pct >= getattr(config, "LP_MIN_OCR_CONFIDENCE", 30.0):
                        # Map bbox back to original image
                        bx = int(min(pt[0] for pt in bbox) / scale)
                        by = int(min(pt[1] for pt in bbox) / scale)
                        bw = int((max(pt[0] for pt in bbox) - min(pt[0] for pt in bbox)) / scale)
                        bh = int((max(pt[1] for pt in bbox) - min(pt[1] for pt in bbox)) / scale)
                        global_bbox = (origin_x + bx, origin_y + by, bw, bh)
                        print(f"[LPR FAST SCAN] Plate: '{cleaned}' (Conf: {conf_pct:.1f}%)", flush=True)
                        valid_plates.append((cleaned, conf_pct, global_bbox))
        except Exception as e:
            print(f"[LPR Fallback Error] {e}", flush=True)

        return valid_plates

    def _run_ocr_on_candidate(
        self, plate_crop: np.ndarray
    ) -> Tuple[Optional[str], float, Optional[Tuple[int, int, int, int]]]:
        """Run fast OCR on a candidate crop."""
        if not self.ocr_available or self.reader is None or plate_crop.size == 0:
            return None, 0.0, None

        ch, cw = plate_crop.shape[:2]
        # Ensure minimum height for character recognition
        if ch < 32:
            scale_y = 32.0 / max(1, ch)
            scale_x = max(1.0, scale_y)
            plate_crop = cv2.resize(plate_crop, (int(cw * scale_x), 32), interpolation=cv2.INTER_CUBIC)

        try:
            results = self.reader.readtext(
                plate_crop,
                canvas_size=256,
                mag_ratio=1.0,
                min_size=4,
                text_threshold=0.35,
                low_text=0.3,
                allowlist=getattr(config, "LP_OCR_ALLOWLIST", "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"),
            )
            if not results:
                return None, 0.0, None

            best_text = None
            best_conf = 0.0
            best_bbox = None

            for (bbox, raw_text, conf) in results:
                cleaned = _extract_plate_from_text(raw_text)
                if cleaned:
                    conf_pct = float(conf) * 100.0 if conf <= 1.0 else float(conf)
                    if conf_pct > best_conf:
                        best_conf = conf_pct
                        best_text = cleaned
                        bx = int(min(pt[0] for pt in bbox))
                        by = int(min(pt[1] for pt in bbox))
                        bw = int(max(pt[0] for pt in bbox) - bx)
                        bh = int(max(pt[1] for pt in bbox) - by)
                        best_bbox = (bx, by, bw, bh)

            if best_text and best_conf >= getattr(config, "LP_MIN_OCR_CONFIDENCE", 30.0):
                return best_text, best_conf, best_bbox

            return None, 0.0, None
        except Exception:
            return None, 0.0, None

    def _run_ocr_on_plate_patch(self, patch: np.ndarray) -> Tuple[Optional[str], float]:
        """Synchronous OCR test probe helper for smoke tests."""
        if not self.ocr_available or self.reader is None or patch is None or patch.size == 0:
            return None, 0.0

        # Test patch directly and with rotations if needed
        rotations = getattr(config, "LP_OCR_ROTATIONS", [0])
        for rot in rotations:
            cur = patch
            if rot == 90:
                cur = cv2.rotate(patch, cv2.ROTATE_90_CLOCKWISE)
            elif rot == 180:
                cur = cv2.rotate(patch, cv2.ROTATE_180)
            elif rot == 270:
                cur = cv2.rotate(patch, cv2.ROTATE_90_COUNTERCLOCKWISE)

            try:
                results = self.reader.readtext(
                    cur,
                    canvas_size=320,
                    mag_ratio=1.0,
                    allowlist=getattr(config, "LP_OCR_ALLOWLIST", "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"),
                )
                for (_, raw_text, conf) in results:
                    cleaned = _extract_plate_from_text(raw_text)
                    if cleaned:
                        conf_pct = float(conf) * 100.0 if conf <= 1.0 else float(conf)
                        return cleaned, conf_pct
            except Exception:
                pass

        return None, 0.0

    # ── Ultra-Fast Candidate Localization ─────────────────────────────────

    def _find_plate_candidates(self, image: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """Find rectangular high-contrast plate candidates in < 2ms."""
        h, w = image.shape[:2]
        if h < 10 or w < 10:
            return []

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image

        candidates = []

        # 1. Morphological BlackHat / Gradient (detects dark text on light plate)
        rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (13, 5))
        blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, rect_kernel)

        grad_x = cv2.Sobel(blackhat, cv2.CV_32F, 1, 0, ksize=-1)
        grad_x = np.absolute(grad_x)
        min_val, max_val = np.min(grad_x), np.max(grad_x)
        if max_val > min_val:
            grad_x = (255 * ((grad_x - min_val) / (max_val - min_val))).astype("uint8")
        else:
            grad_x = grad_x.astype("uint8")

        grad_x = cv2.GaussianBlur(grad_x, (5, 5), 0)
        _, thresh = cv2.threshold(grad_x, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, close_kernel)
        thresh = cv2.erode(thresh, None, iterations=1)
        thresh = cv2.dilate(thresh, None, iterations=2)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            x, y, cw, ch = cv2.boundingRect(c)
            if ch == 0:
                continue
            aspect = cw / float(ch)
            area = cw * ch
            min_w = getattr(config, "LP_MIN_WIDTH", 20)
            max_w = getattr(config, "LP_MAX_WIDTH", 1200)
            min_h = getattr(config, "LP_MIN_HEIGHT", 8)
            max_h = getattr(config, "LP_MAX_HEIGHT", 500)
            min_ar = getattr(config, "LP_ASPECT_RATIO_MIN", 1.2)
            max_ar = getattr(config, "LP_ASPECT_RATIO_MAX", 9.0)

            if min_ar <= aspect <= max_ar and min_w <= cw <= max_w and min_h <= ch <= max_h and area >= 200:
                # Score: prefer aspect ratios close to 2.8 - 4.5 and larger prominent areas
                score = area * (1.0 - min(abs(aspect - 3.2) / 4.0, 0.8))
                candidates.append((x, y, cw, ch, score))

        # 2. Edge / Canny Contour Search (captures clean rectangular boundaries)
        edges = cv2.Canny(gray, 40, 180)
        edges = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 3)), iterations=1)
        contours_edge, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours_edge:
            x, y, cw, ch = cv2.boundingRect(c)
            if ch == 0:
                continue
            aspect = cw / float(ch)
            area = cw * ch
            min_ar = getattr(config, "LP_ASPECT_RATIO_MIN", 1.2)
            max_ar = getattr(config, "LP_ASPECT_RATIO_MAX", 9.0)
            if min_ar <= aspect <= max_ar and 25 <= cw <= (w * 0.98) and 10 <= ch <= (h * 0.8) and area >= 300:
                # Avoid near-duplicates
                if not any(abs(x - cx) < 20 and abs(y - cy) < 15 for cx, cy, _, _, _ in candidates):
                    score = area * (1.0 - min(abs(aspect - 3.2) / 4.0, 0.8))
                    candidates.append((x, y, cw, ch, score))

        # Sort by plate fitness score descending (most likely candidate first)
        candidates.sort(key=lambda item: item[4], reverse=True)
        return [(x, y, cw, ch) for x, y, cw, ch, _ in candidates[:3]]

    # ── Drawing and Visuals ───────────────────────────────────────────────

    def draw_license_plate(
        self,
        frame: np.ndarray,
        plate_text: str,
        plate_bbox: Tuple[int, int, int, int],
        confidence: float,
    ) -> np.ndarray:
        """Draw modern, high-contrast license plate detection overlay."""
        try:
            x, y, w, h = map(int, plate_bbox)
            ih, iw = frame.shape[:2]
            x = max(0, min(x, iw - 1))
            y = max(0, min(y, ih - 1))
            w = max(10, min(w, iw - x))
            h = max(8, min(h, ih - y))

            # Vivid cyan/lime green accent
            color_border = (0, 255, 180)
            color_accent = (0, 200, 255)
            color_bg = (15, 23, 42)

            # Draw outer glowing box
            cv2.rectangle(frame, (x, y), (x + w, y + h), color_border, 2)

            # Draw corner brackets for tech HUD look
            corner_len = max(6, min(18, int(min(w, h) * 0.25)))
            # Top-left
            cv2.line(frame, (x, y), (x + corner_len, y), color_accent, 3)
            cv2.line(frame, (x, y), (x, y + corner_len), color_accent, 3)
            # Top-right
            cv2.line(frame, (x + w, y), (x + w - corner_len, y), color_accent, 3)
            cv2.line(frame, (x + w, y), (x + w, y + corner_len), color_accent, 3)
            # Bottom-left
            cv2.line(frame, (x, y + h), (x + corner_len, y + h), color_accent, 3)
            cv2.line(frame, (x, y + h), (x, y + h - corner_len), color_accent, 3)
            # Bottom-right
            cv2.line(frame, (x + w, y + h), (x + w - corner_len, y + h), color_accent, 3)
            cv2.line(frame, (x + w, y + h), (x + w, y + h - corner_len), color_accent, 3)

            # Label badge
            label = f"{plate_text} [{confidence:.0f}%]"
            font = cv2.FONT_HERSHEY_DUPLEX
            font_scale = 0.55
            thickness = 1
            (lw, lh), baseline = cv2.getTextSize(label, font, font_scale, thickness)

            badge_y = max(lh + 6, y - 6)
            badge_x = x

            # Background pill for badge
            cv2.rectangle(
                frame,
                (badge_x, badge_y - lh - 6),
                (badge_x + lw + 12, badge_y + 4),
                color_bg,
                -1,
            )
            cv2.rectangle(
                frame,
                (badge_x, badge_y - lh - 6),
                (badge_x + lw + 12, badge_y + 4),
                color_border,
                1,
            )
            # Text
            cv2.putText(
                frame,
                label,
                (badge_x + 6, badge_y - 2),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )
        except Exception:
            pass

        return frame


if __name__ == "__main__":
    print("License Plate Detector Test")
    print(f"EasyOCR available: {OCR_AVAILABLE}")

    if OCR_AVAILABLE:
        detector = LicensePlateDetector()
        print("✓ License plate detector initialized (instant zero-delay worker running)")
    else:
        print("✗ Install easyocr: pip install easyocr")
