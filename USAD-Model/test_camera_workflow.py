"""Focused tests for camera freshness and source validation."""

import unittest
from unittest import mock

import numpy as np

import config
import main


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
        self.assertEqual(config.get_camera_role(1), "vehicle_detection")
        self.assertEqual(config.get_camera_role(2), "license_plate")
        self.assertEqual(config.get_camera_role(99), "vehicle_detection")

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


if __name__ == "__main__":
    unittest.main()
