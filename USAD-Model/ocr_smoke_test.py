import os
import sys
from typing import List, Tuple

import cv2
import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import config
from license_plate_detector import LicensePlateDetector, _extract_plate_from_text


def _make_plate_patch(text: str, width: int = 300, height: int = 110, angle: int = 0) -> np.ndarray:
    img = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.rectangle(img, (2, 2), (width - 3, height - 3), (0, 0, 0), 2)

    font = cv2.FONT_HERSHEY_DUPLEX
    scale = 1.7
    thickness = 3
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    x = max(6, (width - tw) // 2)
    y = max(th + 6, (height + th) // 2 - baseline)
    cv2.putText(img, text, (x, y), font, scale, (0, 0, 0), thickness, cv2.LINE_AA)

    a = int(angle) % 360
    if a == 90:
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    elif a == 180:
        img = cv2.rotate(img, cv2.ROTATE_180)
    elif a == 270:
        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img


def run_smoke_test() -> int:
    print("[SMOKE] License Plate OCR smoke test")
    detector = LicensePlateDetector()

    print(f"[SMOKE] OCR available: {bool(getattr(detector, 'ocr_available', False))}")
    print(f"[SMOKE] OCR mode: {str(getattr(detector, 'ocr_mode', 'unknown'))}")
    print(f"[SMOKE] permissive mode: {bool(getattr(config, 'LP_TEST_READ_ALL_ALNUM', False))}")

    if not bool(getattr(detector, "ocr_available", False)):
        print("[SMOKE] FAIL: OCR engine is unavailable")
        return 1

    parser_probe = "ab-12 xy_3456 z9"
    parsed = _extract_plate_from_text(parser_probe)
    print(f"[SMOKE] parser probe: {parser_probe!r} -> {parsed!r}")
    if not parsed:
        print("[SMOKE] FAIL: alnum parser returned empty output")
        return 1

    probes: List[Tuple[str, int]] = [
        ("ABC123", 0),
        ("ABC123", 90),
        ("AB12XY", 0),
        ("Z9X8Q7", 270),
    ]

    ok = 0
    for idx, (text, angle) in enumerate(probes, start=1):
        patch = _make_plate_patch(text, angle=angle)
        plate, conf = detector._run_ocr_on_plate_patch(patch)
        status = "OK" if plate else "MISS"
        print(f"[SMOKE] probe#{idx} text={text!r} angle={angle} -> {plate!r} conf={float(conf):.1f} [{status}]")
        if plate:
            ok += 1

    if ok <= 0:
        print("[SMOKE] FAIL: OCR did not recognize any synthetic probe")
        return 1

    print(f"[SMOKE] PASS: {ok}/{len(probes)} probes recognized")
    return 0


if __name__ == "__main__":
    raise SystemExit(run_smoke_test())
