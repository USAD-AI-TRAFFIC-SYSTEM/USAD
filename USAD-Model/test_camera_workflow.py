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
    @staticmethod
    def _signal_controller(active_lane="LANE1", phase="GREEN", start=100.0, duration=10.0):
        usad = main.USAD.__new__(main.USAD)
        usad.current_active_lane = active_lane
        usad.current_phase = phase
        usad.phase_start_time = start
        usad.lane_green_duration = duration
        usad.software_auto_mode = True
        usad.arduino_connected = False
        usad._pending_congested_lane = None
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
        cv2.rectangle(frame, (215, 100), (254, 134), (25, 25, 25), -1)
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

    def test_orientation_specific_box_constraints(self):
        detector = VehicleDetector()
        def rectangle(w, h):
            return np.array([[[0, 0]], [[w, 0]], [[w, h]], [[0, h]]], dtype=np.int32)

        with mock.patch.object(config, "MAX_VEHICLE_EXTENT", 1.0):
            self.assertTrue(detector._passes_shape_filters(rectangle(100, 50), 100, 50, 5000.0))
            self.assertTrue(detector._passes_shape_filters(rectangle(60, 105), 60, 105, 6300.0))
            self.assertFalse(detector._passes_shape_filters(rectangle(79, 50), 79, 50, 3950.0))
            self.assertFalse(detector._passes_shape_filters(rectangle(100, 61), 100, 61, 6100.0))
            self.assertFalse(detector._passes_shape_filters(rectangle(47, 105), 47, 105, 4935.0))
            self.assertFalse(detector._passes_shape_filters(rectangle(60, 121), 60, 121, 7260.0))

    def test_congestion_waits_for_green_and_yellow_before_handoff(self):
        usad = self._signal_controller()
        counts = {"LANE1": 0, "LANE2": 0, "LANE3": 3, "LANE4": 0}

        usad.update_traffic_control(counts, [])
        self.assertEqual(usad._pending_congested_lane, "LANE3")
        self.assertEqual(usad.current_active_lane, "LANE1")
        self.assertEqual(usad.current_phase, "GREEN")
        self.assertEqual(usad.lane_green_duration, 10.0)

        with mock.patch.object(main.time, "time", return_value=109.9):
            usad.update_signal_cycle(counts)
        self.assertEqual(usad.current_phase, "GREEN")

        with mock.patch.object(main.time, "time", return_value=110.0):
            usad.update_signal_cycle(counts)
        self.assertEqual(usad.current_active_lane, "LANE1")
        self.assertEqual(usad.current_phase, "YELLOW")

        with mock.patch.object(main.time, "time", return_value=114.0):
            usad.update_signal_cycle(counts)
        self.assertEqual(usad.current_active_lane, "LANE3")
        self.assertEqual(usad.current_phase, "GREEN")
        self.assertEqual(usad.lane_green_duration, 20)
        self.assertIsNone(usad._pending_congested_lane)

    def test_cleared_congestion_does_not_shorten_active_green(self):
        usad = self._signal_controller(active_lane="LANE3", duration=20.0)
        cleared = {lane: 0 for lane in config.LANES}

        with mock.patch.object(main.time, "time", return_value=105.0):
            usad.update_traffic_control(cleared, [])
            usad.update_signal_cycle(cleared)
        self.assertEqual(usad.current_active_lane, "LANE3")
        self.assertEqual(usad.current_phase, "GREEN")
        self.assertEqual(usad.lane_green_duration, 20.0)

        with mock.patch.object(main.time, "time", return_value=120.0):
            usad.update_signal_cycle(cleared)
        self.assertEqual(usad.current_active_lane, "LANE3")
        self.assertEqual(usad.current_phase, "YELLOW")

        with mock.patch.object(main.time, "time", return_value=124.0):
            usad.update_signal_cycle(cleared)
        self.assertEqual(usad.current_active_lane, "LANE4")
        self.assertEqual(usad.current_phase, "GREEN")
        self.assertEqual(usad.lane_green_duration, config.MIN_GREEN_TIME)

    def test_active_lane_congestion_repeats_after_yellow_then_exits_cleanly(self):
        usad = self._signal_controller(active_lane="LANE3", duration=10.0)
        congested = {lane: 0 for lane in config.LANES}
        congested["LANE3"] = 3

        with mock.patch.object(main.time, "time", return_value=105.0):
            usad.update_traffic_control(congested, [])
            usad.update_signal_cycle(congested)
        self.assertEqual(usad._pending_congested_lane, "LANE3")
        self.assertEqual(usad.current_active_lane, "LANE3")
        self.assertEqual(usad.current_phase, "GREEN")
        self.assertEqual(usad.lane_green_duration, 10.0)

        with mock.patch.object(main.time, "time", return_value=110.0):
            usad.update_signal_cycle(congested)
        self.assertEqual(usad.current_active_lane, "LANE3")
        self.assertEqual(usad.current_phase, "YELLOW")

        with mock.patch.object(main.time, "time", return_value=114.0):
            usad.update_signal_cycle(congested)
        self.assertEqual(usad.current_active_lane, "LANE3")
        self.assertEqual(usad.current_phase, "GREEN")
        self.assertEqual(usad.lane_green_duration, 20)

        # Clearing congestion does not truncate the newly granted 20 seconds.
        cleared = {lane: 0 for lane in config.LANES}
        with mock.patch.object(main.time, "time", return_value=120.0):
            usad.update_traffic_control(cleared, [])
            usad.update_signal_cycle(cleared)
        self.assertEqual(usad.current_active_lane, "LANE3")
        self.assertEqual(usad.current_phase, "GREEN")
        self.assertEqual(usad.lane_green_duration, 20)

        with mock.patch.object(main.time, "time", return_value=134.0):
            usad.update_signal_cycle(cleared)
        self.assertEqual(usad.current_active_lane, "LANE3")
        self.assertEqual(usad.current_phase, "YELLOW")

        with mock.patch.object(main.time, "time", return_value=138.0):
            usad.update_signal_cycle(cleared)
        self.assertEqual(usad.current_active_lane, "LANE4")
        self.assertEqual(usad.current_phase, "GREEN")

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
