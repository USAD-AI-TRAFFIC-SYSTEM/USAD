"""Regression tests for analytics paths used by packaged builds."""

import csv
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import config
import server
from event_logger import EventLogger


class ServerAnalyticsTests(unittest.TestCase):
    def test_read_csv_uses_configured_runtime_log_directory(self):
        with tempfile.TemporaryDirectory() as runtime_dir, tempfile.TemporaryDirectory() as bundle_dir:
            log_path = Path(runtime_dir) / "violations.csv"
            with log_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=["lane", "violation_type"])
                writer.writeheader()
                writer.writerow({"lane": "LANE3", "violation_type": "RED_LIGHT_VIOLATION"})

            # Simulate PyInstaller: server source lives in a temporary bundle,
            # while writable analytics data lives beside the executable.
            with (
                mock.patch.object(config, "LOG_DIRECTORY", runtime_dir),
                mock.patch.object(server, "_THIS_DIR", Path(bundle_dir)),
            ):
                rows = server._read_csv("violations.csv")

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["lane"], "LANE3")

    def test_read_csv_returns_empty_list_for_missing_log(self):
        with tempfile.TemporaryDirectory() as runtime_dir:
            with mock.patch.object(config, "LOG_DIRECTORY", runtime_dir):
                self.assertEqual(server._read_csv("accidents.csv"), [])

    def test_event_logger_preserves_rows_across_restart(self):
        with tempfile.TemporaryDirectory() as runtime_dir:
            with mock.patch.object(config, "LOG_DIRECTORY", runtime_dir):
                logger = EventLogger()
                with open(logger.violation_log_path, "a", newline="", encoding="utf-8") as stream:
                    csv.writer(stream).writerow([
                        "2026-09-13T14:00:00", "2026-09-13", "14:00:00", "14",
                        "RED_LIGHT_VIOLATION", "1", "7", "MEDIUM", "N/A",
                        "LANE3", "RED", "0", "100", "200",
                    ])

                EventLogger()  # Simulate closing and reopening the packaged app.
                with open(logger.violation_log_path, newline="", encoding="utf-8") as stream:
                    rows = list(csv.DictReader(stream))

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["lane"], "LANE3")


if __name__ == "__main__":
    unittest.main()
