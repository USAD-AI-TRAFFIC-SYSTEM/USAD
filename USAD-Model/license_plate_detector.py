"""License plate detection + OCR via EasyOCR."""

import cv2
import numpy as np
from typing import Optional, Tuple, List
import re
import config

try:
    import easyocr
    OCR_AVAILABLE = True
except ImportError:
    OCR_AVAILABLE = False
    print("Warning: easyocr not installed. License plate OCR will be disabled.")


class LicensePlateDetector:
    """Detects and reads license plates from vehicle images"""
    
    def __init__(self):
        self.ocr_available = OCR_AVAILABLE
        self.reader = None
        if self.ocr_available:
            try:
                # Initialize EasyOCR reader for English text
                self.reader = easyocr.Reader(['en'], gpu=False)
            except Exception as e:
                print(f"Failed to initialize EasyOCR: {e}")
                self.ocr_available = False
        
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
        
        # Bounding boxes may be floats from detectors; convert safely to ints for slicing
        x, y, w, h = [int(round(v)) for v in vehicle_bbox]
        
        # Extract vehicle region with some padding
        padding = 10
        x1 = max(0, x - padding)
        y1 = max(0, y - padding)
        x2 = min(frame.shape[1], x + w + padding)
        y2 = min(frame.shape[0], y + h + padding)
        
        vehicle_roi = frame[y1:y2, x1:x2]
        
        if vehicle_roi.size == 0:
            return None
        
        # Find license plate candidate regions
        plate_candidates = self._find_plate_candidates(vehicle_roi)
        
        if not plate_candidates:
            return None
        
        # Try OCR on each candidate
        best_result = None
        best_confidence = 0
        
        for candidate_bbox in plate_candidates:
            px, py, pw, ph = candidate_bbox
            plate_roi = vehicle_roi[py:py+ph, px:px+pw]
            
            # Preprocess plate image
            plate_processed = self._preprocess_plate(plate_roi)
            
            # Perform OCR
            if self.ocr_available:
                text, confidence = self._perform_ocr(plate_processed)
                
                if text and confidence > best_confidence:
                    # Convert to frame coordinates
                    global_bbox = (x1 + px, y1 + py, pw, ph)
                    best_result = (text, confidence, global_bbox)
                    best_confidence = confidence
        
        return best_result
    
    def _find_plate_candidates(self, image: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """Find potential license plate regions using contour detection"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        gray = cv2.bilateralFilter(gray, 11, 17, 17)
        
        edges = cv2.Canny(gray, 30, 200)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        candidates = []
        
        for contour in contours:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
            
            if len(approx) >= 4:
                x, y, w, h = cv2.boundingRect(contour)
                
                if not (config.LP_MIN_WIDTH <= w <= config.LP_MAX_WIDTH):
                    continue
                if not (config.LP_MIN_HEIGHT <= h <= config.LP_MAX_HEIGHT):
                    continue
                
                aspect_ratio = w / float(h)
                if not (config.LP_ASPECT_RATIO_MIN <= aspect_ratio <= config.LP_ASPECT_RATIO_MAX):
                    continue
                
                candidates.append((x, y, w, h))
        
        return candidates
    
    def _preprocess_plate(self, plate_image: np.ndarray) -> np.ndarray:
        """Preprocess license plate image for better OCR"""
        if len(plate_image.shape) == 3:
            gray = cv2.cvtColor(plate_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = plate_image
        
        height, width = gray.shape
        if height < 50:
            scale = 50 / height
            gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        
        return thresh
    
    def _perform_ocr(self, image: np.ndarray) -> Tuple[Optional[str], float]:
        """Perform OCR on preprocessed plate image using EasyOCR"""
        if not self.ocr_available or self.reader is None:
            return None, 0.0
        
        try:
            # EasyOCR returns list of (bbox, text, confidence)
            results = self.reader.readtext(image)
            
            if not results:
                return None, 0.0
            
            texts = []
            confidences = []
            
            for (bbox, text, conf) in results:
                if text.strip():
                    texts.append(text.strip())
                    confidences.append(conf)
            
            if not texts:
                return None, 0.0
            
            full_text = ''.join(texts)
            
            # Clean text: keep only alphanumeric characters
            full_text = re.sub(r'[^A-Z0-9]', '', full_text.upper())
            
            avg_confidence = sum(confidences) / len(confidences) if confidences else 0
            avg_confidence_percent = avg_confidence * 100
            
            if len(full_text) >= 3:
                return full_text, avg_confidence_percent
            
            return None, 0.0
            
        except Exception as e:
            print(f"OCR error: {e}")
            return None, 0.0
    
    def draw_license_plate(self, frame: np.ndarray, plate_text: str, 
                          plate_bbox: Tuple[int, int, int, int], confidence: float) -> np.ndarray:
        """Draw license plate detection on frame"""
        x, y, w, h = map(int, plate_bbox)
        
        color = config.COLOR_LICENSE_PLATE
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        
        label = f"{plate_text} ({confidence:.1f}%)"
        cv2.putText(frame, label, (x, y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        return frame


# Test function
if __name__ == "__main__":
    print("License Plate Detector Test")
    print(f"EasyOCR available: {OCR_AVAILABLE}")
    
    if OCR_AVAILABLE:
        detector = LicensePlateDetector()
        print("✓ License plate detector initialized")
    else:
        print("✗ Install easyocr: pip install easyocr")
