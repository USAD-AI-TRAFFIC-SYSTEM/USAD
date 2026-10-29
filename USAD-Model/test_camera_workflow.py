"""Focused tests for camera freshness and source validation."""

import unittest
from unittest import mock
import time

import cv2
import numpy as np

import config
import main
from vehicle_detector import Vehicle, VehicleDetector


class FakeCapture:
    def __init__(self, readable):
        self.readable = readable
        self.released = False
        self.properties = {}

    def isOpened(self):
        return True

    def set(self, prop, value):
        self.properties[prop] = value
        return True

    def get(self, prop):
        if prop == main.cv2.CAP_PROP_FRAME_WIDTH:
            return 1280
        if prop == main.cv2.CAP_PROP_FRAME_HEIGHT:
            return 720
        if prop == main.cv2.CAP_PROP_FPS:
            return 30
        return 0

    def read(self):
        if not self.readable:
            return False, None
        return True, np.zeros((720, 1280, 3), dtype=np.uint8)

    def release(self):
        self.released = True


class CameraWorkflowTests(unittest.TestCase):
    def setUp(self):
        timing = {
            "GREEN_TIME": 15,
            "YELLOW_TIME": 4,
            "EMPTY_LANE_CHECK_SECONDS": 5,
            "EMPTY_GAP_CHECK_FROM_REMAINING": 10,
            "EMPTY_GAP_SECONDS": 3,
        }
        for name, value in timing.items():
            patcher = mock.patch.object(config, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)

    @staticmethod
    def _signal_controller(active_lane="LANE1", phase="GREEN", start=100.0, duration=10.0):
        usad = main.USAD.__new__(main.USAD)
        usad.current_active_lane = active_lane
        usad.current_phase = phase
        usad.phase_start_time = start
        usad.lane_green_duration = duration
        usad.software_auto_mode = True
        usad.arduino_connected = False
        usad._car_seen_this_green = False
        usad._is_arduino_connected = mock.Mock(return_value=False)
        usad._apply_signal_states = mock.Mock()
        usad._sync_arduino = mock.Mock()
        return usad

    def test_latest_frame_is_consumed_once_per_sequence(self):
        grabber = main.LatestFrameGrabber(None)
        grabber._latest = np.zeros((1, 1, 3), dtype=np.uint8)
        grabber._latest_ts = 123.0
        grabber._sequence = 7

        ok, _frame, timestamp, sequence = grabber.get_latest_after(6)
        self.assertTrue(ok)
        self.assertEqual(timestamp, 123.0)
        self.assertEqual(sequence, 7)

        ok, frame, _timestamp, sequence = grabber.get_latest_after(7)
        self.assertFalse(ok)
        self.assertIsNone(frame)
        self.assertEqual(sequence, 7)

    def test_initialization_rejects_open_but_unreadable_camera(self):
        captures = []

        def make_capture(_source, _backend):
            cap = FakeCapture(readable=len(captures) >= 3)
            captures.append(cap)
            return cap

        usad = main.USAD.__new__(main.USAD)
        usad.cap = None

        with (
            mock.patch.object(main.cv2, "VideoCapture", side_effect=make_capture),
            mock.patch.object(main.time, "sleep"),
            mock.patch.object(config, "CAMERA_SOURCE", 1),
        ):
            self.assertTrue(usad.initialize_camera())
            self.assertEqual(config.CAMERA_SOURCE, 0)

        self.assertTrue(all(cap.released for cap in captures[:3]))
        self.assertFalse(captures[3].released)

    def test_camera_roles_are_explicit(self):
        original = dict(config.CAMERA_ASSIGNMENTS)
        try:
            config.set_camera_assignments(1, 2)
            self.assertEqual(config.get_camera_role(1), "vehicle_detection")
            self.assertEqual(config.get_camera_role(2), "license_plate")
            self.assertEqual(config.get_camera_role(99), "vehicle_detection")
        finally:
            config.set_camera_assignments(
                original["vehicle_detection"],
                original["license_plate"],
            )

    def test_camera_assignments_rebuild_cycle_and_role_maps(self):
        original = dict(config.CAMERA_ASSIGNMENTS)
        try:
            config.set_camera_assignments(4, 7)
            self.assertEqual(config.CAMERA_SOURCES, [4, 7])
            self.assertEqual(config.get_camera_role(4), "vehicle_detection")
            self.assertEqual(config.get_camera_role(7), "license_plate")
        finally:
            config.set_camera_assignments(
                original["vehicle_detection"],
                original["license_plate"],
            )

    def test_grainy_small_horizontal_and_vertical_cars_are_segmented(self):
        frame = np.full((240, 320, 3), 35, dtype=np.uint8)
        cv2.rectangle(frame, (25, 75), (124, 124), (180, 35, 20), -1)
        cv2.rectangle(frame, (205, 65), (264, 169), (15, 25, 190), -1)
        # Dark windows break up each colored surface before morphological closing.
        cv2.rectangle(frame, (58, 84), (91, 115), (25, 25, 25), -1)
        # Window reaches the side so the outline is not a perfect rectangle
        # (real cars are not; near-perfect rectangles are rejected by extent).
        cv2.rectangle(frame, (215, 100), (264, 134), (25, 25, 25), -1)
        rng = np.random.default_rng(913)
        grain = rng.normal(0, 11, frame.shape).astype(np.int16)
        frame = np.clip(frame.astype(np.int16) + grain, 0, 255).astype(np.uint8)

        with mock.patch.object(config, "REQUIRE_INTERSECTION_ROI_FOR_DETECTION", False):
            detections = VehicleDetector()._detect_by_color(frame)

        self.assertGreaterEqual(len(detections), 2)
        boxes = [d["bbox"] for d in detections]
        self.assertTrue(any(w > h for _x, _y, w, h in boxes))
        self.assertTrue(any(h > w for _x, _y, w, h in boxes))

    def test_color_supported_motion_can_create_a_track(self):
        detector = VehicleDetector()
        detection = {
            "source": "motion",
            "center": (50, 50),
            "bbox": (38, 42, 24, 16),
            "area": 600.0,
            "mask_ratio": 0.5,
            "color_ratio": config.MOTION_NEW_TRACK_MIN_COLOR_RATIO + 0.01,
        }
        detector._update_tracking([detection])
        self.assertEqual(len(detector.vehicles), 1)

    def test_weakly_colored_motion_cannot_create_a_track(self):
        detector = VehicleDetector()
        detection = {
            "source": "motion",
            "center": (50, 50),
            "bbox": (20, 25, 60, 50),
            "area": 1200.0,
            "mask_ratio": 0.8,
            "color_ratio": config.MOTION_NEW_TRACK_MIN_COLOR_RATIO / 4,
        }
        detector._update_tracking([detection])
        self.assertEqual(len(detector.vehicles), 0)

    def test_incidental_red_light_and_blue_line_are_not_cars(self):
        frame = np.full((240, 320, 3), 35, dtype=np.uint8)
        cv2.rectangle(frame, (80, 80), (87, 87), (0, 0, 220), -1)
        cv2.line(frame, (120, 40), (124, 200), (220, 30, 10), 3)
        with mock.patch.object(config, "REQUIRE_INTERSECTION_ROI_FOR_DETECTION", False):
            detections = VehicleDetector()._detect_by_color(frame)
        self.assertEqual(detections, [])

    def test_roi_requires_center_or_substantial_bbox_overlap(self):
        mask = np.zeros((200, 240), dtype=np.uint8)
        mask[50:150, 100:200] = 255
        self.assertFalse(
            VehicleDetector._bbox_supported_by_roi(mask, (10, 70, 100, 60), (60, 100))
        )
        self.assertTrue(
            VehicleDetector._bbox_supported_by_roi(mask, (60, 70, 100, 60), (90, 100))
        )

    def test_hand_is_not_detected_as_a_red_car(self):
        def detect(hsv_color):
            frame = np.full((240, 320, 3), 35, dtype=np.uint8)
            bgr = cv2.cvtColor(np.uint8([[hsv_color]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()
            cv2.rectangle(frame, (40, 60), (139, 109), bgr, -1)
            cv2.rectangle(frame, (70, 60), (100, 72), (35, 35, 35), -1)
            with mock.patch.object(config, "REQUIRE_INTERSECTION_ROI_FOR_DETECTION", False):
                return VehicleDetector()._detect_by_color(frame)

        # Medians measured on the live setup: skin vs the red toy car.
        self.assertEqual(detect((5, 113, 110)), [])
        self.assertEqual(len(detect((176, 253, 134))), 1)

    def test_traffic_light_zone_is_never_detected_as_a_car(self):
        frame = np.full((240, 320, 3), 35, dtype=np.uint8)
        # Car-sized red blob, e.g. a lit light plus its glow.
        cv2.rectangle(frame, (40, 60), (139, 109), (20, 20, 210), -1)
        # Notch so the outline is not a perfect rectangle (those are rejected as non-cars).
        cv2.rectangle(frame, (70, 60), (100, 72), (35, 35, 35), -1)

        with mock.patch.object(config, "REQUIRE_INTERSECTION_ROI_FOR_DETECTION", False):
            with mock.patch.object(config, "TRAFFIC_LIGHT_ZONES", []):
                self.assertEqual(len(VehicleDetector()._detect_by_color(frame)), 1)
            with mock.patch.object(config, "TRAFFIC_LIGHT_ZONES", [(30, 50, 120, 70)]):
                self.assertEqual(VehicleDetector()._detect_by_color(frame), [])

    def test_roi_mask_follows_lane_recalibration(self):
        detector = VehicleDetector()
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        original = config.LANES["LANE1"]["region"]
        try:
            before = detector._get_intersection_roi_mask(frame)
            self.assertEqual(before[5, 5], 0)
            config.LANES["LANE1"]["region"] = [(0, 0), (60, 0), (60, 60), (0, 60)]
            after = detector._get_intersection_roi_mask(frame)
            self.assertNotEqual(after[5, 5], 0)
        finally:
            config.LANES["LANE1"]["region"] = original

    def test_recent_color_supported_track_is_held_through_one_dropout(self):
        vehicle = Vehicle((50, 50), (38, 42, 24, 16), 600.0)
        vehicle.confirmed = True
        vehicle.lost_frames = 1
        vehicle._last_observed_ts = time.time()
        vehicle._last_presence_ratio = config.TRACK_DISPLAY_HOLD_MIN_PRESENCE_RATIO + 0.01
        self.assertTrue(VehicleDetector._is_track_displayable(vehicle))

    def test_live_vehicle_overlay_includes_bbox_width_and_height(self):
        detector = VehicleDetector()
        vehicle = Vehicle((50, 50), (20, 30, 64, 28), 1200.0)
        vehicle.confirmed = True
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        with mock.patch.object(cv2, "putText") as put_text:
            detector.draw_vehicles(frame, [vehicle])
        labels = [call.args[1] for call in put_text.call_args_list]
        self.assertIn("BOX W:64px H:28px", labels)

    def test_car_size_constraints(self):
        detector = VehicleDetector()
        def rectangle(w, h):
            return np.array([[[0, 0]], [[w, 0]], [[w, h]], [[0, h]]], dtype=np.int32)

        with (
            mock.patch.object(config, "MAX_VEHICLE_EXTENT", 1.0),
            mock.patch.object(config, "CAR_LENGTH_RANGE_PX", (80, 140)),
            mock.patch.object(config, "CAR_WIDTH_RANGE_PX", (45, 80)),
            mock.patch.object(config, "CAR_SIZE_TOLERANCE", 0.25),
        ):
            self.assertTrue(detector._passes_shape_filters(rectangle(100, 50), 100, 50, 5000.0))
            self.assertTrue(detector._passes_shape_filters(rectangle(60, 105), 60, 105, 6300.0))
            # Within tolerance: the same car seen from a slightly higher or lower camera.
            self.assertTrue(detector._passes_shape_filters(rectangle(70, 38), 70, 38, 2660.0))
            self.assertTrue(detector._passes_shape_filters(rectangle(165, 90), 165, 90, 14850.0))
            # Outside tolerance.
            self.assertFalse(detector._passes_shape_filters(rectangle(55, 30), 55, 30, 1650.0))
            self.assertFalse(detector._passes_shape_filters(rectangle(180, 100), 180, 100, 18000.0))

    def test_default_area_limits_are_unchanged(self):
        from vehicle_detector import vehicle_area_bounds
        self.assertEqual(vehicle_area_bounds(), (config.MIN_VEHICLE_AREA, config.MAX_VEHICLE_AREA))

    def test_calibrated_large_car_is_not_blocked_by_area_limit(self):
        detector = VehicleDetector()
        contour = np.array([[[0, 0]], [[200, 0]], [[200, 120]], [[0, 120]]], dtype=np.int32)
        with (
            mock.patch.object(config, "MAX_VEHICLE_EXTENT", 1.0),
            mock.patch.object(config, "CAR_LENGTH_RANGE_PX", (150, 210)),
            mock.patch.object(config, "CAR_WIDTH_RANGE_PX", (90, 125)),
        ):
            # 24000 px^2 is above MAX_VEHICLE_AREA (20000) but matches the calibrated size.
            self.assertTrue(detector._passes_shape_filters(contour, 200, 120, 24000.0))
            self.assertTrue(detector._is_allowed_vehicle_type(24000.0))

    def test_diagonal_car_passes_size_constraints(self):
        detector = VehicleDetector()
        # A 110x55 car rotated 45 degrees: its axis-aligned box is ~117x117.
        contour = cv2.boxPoints(((100.0, 100.0), (110.0, 55.0), 45.0))
        contour = np.round(contour).astype(np.int32).reshape(-1, 1, 2)
        _x, _y, w, h = cv2.boundingRect(contour)
        area = float(cv2.contourArea(contour))
        self.assertGreater(min(w, h), 100)
        self.assertTrue(detector._passes_shape_filters(contour, w, h, area))

    def _run_signal(self, usad, t, counts):
        with mock.patch.object(main.time, "time", return_value=t):
            usad.update_traffic_control(counts, [])
            usad.update_signal_cycle(counts)

    @staticmethod
    def _counts(**cars):
        counts = {lane: 0 for lane in config.LANES}
        counts.update(cars)
        return counts

    def _timed_controller(self):
        """A 15 s green that started at t=100 (countdown 15 at 100, 10 at 105, 0 at 115)."""
        return self._signal_controller(active_lane="LANE1", duration=15.0)

    def test_empty_lane_green_is_cut_after_the_check_window(self):
        usad = self._timed_controller()
        empty = self._counts()

        self._run_signal(usad, 104.9, empty)
        self.assertEqual(usad.current_phase, "GREEN")

        # 15 ... 10 and still no car: cut, without counting down to 0.
        self._run_signal(usad, 105.0, empty)
        self.assertEqual(usad.current_active_lane, "LANE1")
        self.assertEqual(usad.current_phase, "YELLOW")

        # Yellow is kept, then the normal order continues.
        self._run_signal(usad, 109.0, empty)
        self.assertEqual(usad.current_active_lane, "LANE2")
        self.assertEqual(usad.current_phase, "GREEN")
        self.assertEqual(usad.lane_green_duration, float(config.GREEN_TIME))

    def test_occupied_lane_keeps_the_full_green(self):
        usad = self._timed_controller()
        for t in (100.5, 105.0, 110.0, 114.9):
            self._run_signal(usad, t, self._counts(LANE1=1))
            self.assertEqual(usad.current_phase, "GREEN")
        self._run_signal(usad, 115.0, self._counts(LANE1=1))
        self.assertEqual(usad.current_phase, "YELLOW")

    def test_lane_that_empties_late_is_cut_after_3s_empty(self):
        usad = self._timed_controller()
        self._run_signal(usad, 100.5, self._counts(LANE1=1))
        self._run_signal(usad, 107.0, self._counts(LANE1=1))
        self._run_signal(usad, 107.5, self._counts())  # car leaves at 7.5s left
        self._run_signal(usad, 110.4, self._counts())
        self.assertEqual(usad.current_phase, "GREEN")
        self._run_signal(usad, 110.5, self._counts())  # empty for 3s
        self.assertEqual(usad.current_phase, "YELLOW")

    def test_empty_time_before_10s_left_counts(self):
        usad = self._timed_controller()
        self._run_signal(usad, 100.5, self._counts(LANE1=1))
        self._run_signal(usad, 103.0, self._counts())  # car leaves at 12s left
        # At 10s left the lane has been empty 2s: the 3s check has started
        # but is not met yet.
        self._run_signal(usad, 105.0, self._counts())
        self.assertEqual(usad.current_phase, "GREEN")
        self._run_signal(usad, 106.0, self._counts())  # empty 3s: cut at 9s left
        self.assertEqual(usad.current_phase, "YELLOW")

    def test_car_arriving_resets_the_empty_count(self):
        usad = self._timed_controller()
        self._run_signal(usad, 100.5, self._counts(LANE1=1))
        self._run_signal(usad, 106.0, self._counts())
        self._run_signal(usad, 108.0, self._counts(LANE1=1))  # a car shows up
        self._run_signal(usad, 108.5, self._counts())
        self._run_signal(usad, 111.0, self._counts())
        self.assertEqual(usad.current_phase, "GREEN")
        self._run_signal(usad, 111.5, self._counts())  # 3s since 108.5
        self.assertEqual(usad.current_phase, "YELLOW")

    def test_each_new_green_gets_a_fresh_check_window(self):
        usad = self._timed_controller()
        for t in (101.0, 110.0, 114.9):
            self._run_signal(usad, t, self._counts(LANE1=1))
        self._run_signal(usad, 115.0, self._counts(LANE1=1))
        self._run_signal(usad, 119.0, self._counts())  # South goes green
        self.assertEqual(usad.current_active_lane, "LANE2")
        self._run_signal(usad, 123.9, self._counts())
        self.assertEqual(usad.current_phase, "GREEN")
        self._run_signal(usad, 124.0, self._counts())  # South empty for its first 5s
        self.assertEqual(usad.current_phase, "YELLOW")

    def test_congestion_does_not_extend_or_reorder_greens(self):
        usad = self._timed_controller()
        busy = self._counts(LANE1=3, LANE3=4)

        self._run_signal(usad, 101.0, busy)
        self.assertEqual(usad.lane_green_duration, 15.0)  # not extended
        self._run_signal(usad, 115.0, busy)
        self.assertEqual(usad.current_phase, "YELLOW")

        # Normal order: South next, even though East is congested.
        self._run_signal(usad, 119.0, busy)
        self.assertEqual(usad.current_active_lane, "LANE2")
        self.assertEqual(usad.lane_green_duration, float(config.GREEN_TIME))

    def test_manual_green_is_cut_when_lane_empties(self):
        usad = self._timed_controller()
        usad.activate_lane("LANE2")
        usad.software_auto_mode = False
        usad.phase_start_time = 100.0
        self._run_signal(usad, 101.0, self._counts(LANE2=1))  # car in South
        self._run_signal(usad, 103.0, self._counts())  # car crossed
        self._run_signal(usad, 105.9, self._counts())
        self.assertEqual(usad.current_phase, "GREEN")
        self._run_signal(usad, 106.0, self._counts())  # empty for 3s
        self.assertEqual(usad.current_phase, "YELLOW")
        self.assertEqual(usad.current_active_lane, "LANE2")

    def test_manual_lane_override_cannot_stick_at_zero(self):
        usad = self._signal_controller(active_lane="LANE3", duration=10.0)
        usad.software_auto_mode = False
        cleared = {lane: 0 for lane in config.LANES}

        with mock.patch.object(main.time, "time", return_value=110.0):
            usad.update_signal_cycle(cleared)
        self.assertEqual(usad.current_active_lane, "LANE3")
        self.assertEqual(usad.current_phase, "YELLOW")
        self.assertFalse(usad.software_auto_mode)

        with mock.patch.object(main.time, "time", return_value=114.0):
            usad.update_signal_cycle(cleared)
        self.assertEqual(usad.current_active_lane, "LANE4")
        self.assertEqual(usad.current_phase, "GREEN")
        self.assertTrue(usad.software_auto_mode)

    def test_auto_mode_resumes_cycle_with_arduino_connected(self):
        usad = self._signal_controller()
        usad.software_auto_mode = False
        usad._is_arduino_connected = mock.Mock(return_value=True)
        usad.traffic_controller = mock.Mock()
        usad.enable_auto_mode()
        self.assertTrue(usad.software_auto_mode)
        usad.traffic_controller.set_auto_mode.assert_called_once()

    def test_congestion_display_uses_one_threshold(self):
        usad = self._signal_controller()
        with mock.patch.object(config, "CONGESTION_THRESHOLD", 3):
            self.assertEqual(usad._classify_lane_congestion(0), "EMPTY")
            self.assertEqual(usad._classify_lane_congestion(2), "NON-CONGESTED")
            self.assertEqual(usad._classify_lane_congestion(3), "CONGESTED")

    def test_arduino_init_skips_connect_without_a_port(self):
        usad = main.USAD.__new__(main.USAD)
        usad.traffic_controller = mock.Mock()
        with mock.patch.object(config, "ARDUINO_PORT", None):
            self.assertFalse(usad.initialize_arduino())
        usad.traffic_controller.connect.assert_not_called()

    def test_activate_lane_resets_stale_green_duration(self):
        usad = self._signal_controller(active_lane="LANE1", duration=0.0)
        usad.lane_green_start_time = None
        with mock.patch.object(main.time, "time", return_value=200.0):
            usad.activate_lane("LANE3")
        self.assertEqual(usad.current_active_lane, "LANE3")
        self.assertEqual(usad.current_phase, "GREEN")
        self.assertEqual(usad.phase_start_time, 200.0)
        self.assertEqual(usad.lane_green_duration, float(config.GREEN_TIME))


if __name__ == "__main__":
    unittest.main()
