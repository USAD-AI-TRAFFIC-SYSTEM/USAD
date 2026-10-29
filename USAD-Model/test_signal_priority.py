"""Congestion priority in the signal cycle (no camera needed)."""

import unittest
from unittest import mock

import config
import main


def make_usad():
    usad = main.USAD.__new__(main.USAD)
    usad.software_auto_mode = True
    usad.current_active_lane = None
    usad.current_phase = None
    usad.phase_start_time = 0.0
    usad.lane_green_duration = 15.0
    usad._car_seen_this_green = False
    usad._lane_empty_since = None
    usad._lane_last_green = {}
    usad.arduino_connected = False
    usad._is_arduino_connected = lambda: False
    usad._apply_signal_states = lambda lane, phase: None
    usad._sync_arduino = lambda lane, phase: None
    return usad


def counts(**kw):
    base = {"LANE1": 0, "LANE2": 0, "LANE3": 0, "LANE4": 0}
    base.update(kw)
    return base


class SignalPriorityTest(unittest.TestCase):
    def run_cycle(self, usad, lane_counts, start, seconds, step=0.5):
        t = start
        with mock.patch.object(main.time, "time") as clock:
            while t < start + seconds:
                clock.return_value = t
                usad.update_signal_cycle(lane_counts)
                t += step
        return t

    def test_all_clear_keeps_normal_order(self):
        usad = make_usad()
        self.assertEqual(usad._get_next_lane("LANE1", counts()), "LANE2")
        self.assertEqual(usad._get_next_lane("LANE3", counts(LANE1=1)), "LANE4")

    def test_congested_lane_goes_next_not_east(self):
        usad = make_usad()
        self.assertEqual(usad._get_next_lane("LANE1", counts(LANE4=3)), "LANE4")

    def test_most_congested_first(self):
        usad = make_usad()
        self.assertEqual(usad._get_next_lane("LANE1", counts(LANE2=2, LANE4=4)), "LANE4")

    def test_active_green_cut_for_congested_lane(self):
        usad = make_usad()
        with mock.patch.object(main.time, "time", return_value=100.0):
            usad._begin_green("LANE1", 100.0)
        # North has a car (so no empty cut); West is congested.
        self.run_cycle(usad, counts(LANE1=1, LANE4=3), 100.0, 4.0)
        self.assertEqual(usad.current_phase, "YELLOW")
        self.run_cycle(usad, counts(LANE1=1, LANE4=3), 104.0, 4.5)
        self.assertEqual((usad.current_active_lane, usad.current_phase), ("LANE4", "GREEN"))

    def test_no_cut_before_min_green(self):
        usad = make_usad()
        with mock.patch.object(main.time, "time", return_value=100.0):
            usad._begin_green("LANE1", 100.0)
        self.run_cycle(usad, counts(LANE1=1, LANE4=3), 100.0, 2.5)
        self.assertEqual(usad.current_phase, "GREEN")

    def test_congested_active_lane_finishes(self):
        usad = make_usad()
        with mock.patch.object(main.time, "time", return_value=100.0):
            usad._begin_green("LANE1", 100.0)
        self.run_cycle(usad, counts(LANE1=3, LANE4=3), 100.0, 10.0)
        self.assertEqual((usad.current_active_lane, usad.current_phase), ("LANE1", "GREEN"))

    def test_manual_selection_not_preempted(self):
        usad = make_usad()
        usad.software_auto_mode = False
        with mock.patch.object(main.time, "time", return_value=100.0):
            usad._begin_green("LANE1", 100.0)
        self.run_cycle(usad, counts(LANE1=1, LANE4=3), 100.0, 10.0)
        self.assertEqual(usad.current_phase, "GREEN")

    def test_manual_green_cut_empty_then_goes_to_congested_lane(self):
        usad = make_usad()
        usad.software_auto_mode = False
        with mock.patch.object(main.time, "time", return_value=100.0):
            usad._begin_green("LANE1", 100.0)
        west_busy = counts(LANE4=3)
        t = self.run_cycle(usad, west_busy, 100.0, 10.5)  # North empty: 5s check + 5s more
        self.assertEqual(usad.current_phase, "YELLOW")
        self.run_cycle(usad, west_busy, t, 4.5)
        self.assertEqual((usad.current_active_lane, usad.current_phase), ("LANE4", "GREEN"))

    def test_congested_green_ends_when_congestion_clears(self):
        usad = make_usad()
        usad.current_active_lane = "LANE1"
        with mock.patch.object(main.time, "time", return_value=100.0):
            usad._begin_green("LANE4", 100.0, counts(LANE4=3))
        t = self.run_cycle(usad, counts(LANE4=3), 100.0, 6.0)
        self.assertEqual(usad.current_phase, "GREEN")  # still congested
        t = self.run_cycle(usad, counts(LANE4=1), t, 1.5)
        self.assertEqual(usad.current_phase, "GREEN")  # clear for under 2s
        self.run_cycle(usad, counts(LANE4=1), t, 1.5)
        self.assertEqual(usad.current_phase, "YELLOW")

    def test_flicker_does_not_cut_congested_green(self):
        usad = make_usad()
        with mock.patch.object(main.time, "time", return_value=100.0):
            usad._begin_green("LANE4", 100.0, counts(LANE4=3))
        t = self.run_cycle(usad, counts(LANE4=3), 100.0, 5.0)
        t = self.run_cycle(usad, counts(LANE4=1), t, 1.5)
        t = self.run_cycle(usad, counts(LANE4=3), t, 1.0)
        t = self.run_cycle(usad, counts(LANE4=1), t, 1.5)
        self.assertEqual(usad.current_phase, "GREEN")

    def test_ordinary_green_not_cut_by_congestion_rule(self):
        usad = make_usad()
        with mock.patch.object(main.time, "time", return_value=100.0):
            usad._begin_green("LANE1", 100.0, counts(LANE1=1))
        self.run_cycle(usad, counts(LANE1=1), 100.0, 8.0)
        self.assertEqual(usad.current_phase, "GREEN")

    def test_empty_lane_starts_at_15_then_cut_to_5_more(self):
        usad = make_usad()
        with mock.patch.object(main.time, "time", return_value=100.0):
            usad._begin_green("LANE3", 100.0)
        self.assertEqual(usad.lane_green_duration, 15.0)
        t = self.run_cycle(usad, counts(), 100.0, 4.5)
        self.assertEqual(usad.lane_green_duration, 15.0)  # still within the check window
        t = self.run_cycle(usad, counts(), t, 1.0)  # 5s passed with no car
        self.assertEqual(usad.lane_green_duration, 10.0)  # 5s elapsed + 5s left
        self.assertEqual(usad.current_phase, "GREEN")
        t = self.run_cycle(usad, counts(), t, 4.4)
        self.assertEqual(usad.current_phase, "GREEN")
        self.run_cycle(usad, counts(), t, 1.0)
        self.assertEqual(usad.current_phase, "YELLOW")

    def test_car_returning_after_cut_restores_full_green(self):
        usad = make_usad()
        with mock.patch.object(main.time, "time", return_value=100.0):
            usad._begin_green("LANE3", 100.0)
        t = self.run_cycle(usad, counts(), 100.0, 6.0)
        self.assertEqual(usad.lane_green_duration, 10.0)
        self.run_cycle(usad, counts(LANE3=1), t, 5.0)
        self.assertEqual(usad.lane_green_duration, 15.0)
        self.assertEqual(usad.current_phase, "GREEN")

    def test_car_in_check_window_extends_to_full_green(self):
        usad = make_usad()
        with mock.patch.object(main.time, "time", return_value=100.0):
            usad._begin_green("LANE3", 100.0)
        t = self.run_cycle(usad, counts(), 100.0, 3.0)
        t = self.run_cycle(usad, counts(LANE3=1), t, 8.0)
        self.assertEqual(usad.lane_green_duration, 15.0)
        self.assertEqual(usad.current_phase, "GREEN")

    def test_can_be_disabled(self):
        usad = make_usad()
        with mock.patch.object(config, "ENABLE_CONGESTION_PRIORITY", False):
            self.assertEqual(usad._get_next_lane("LANE1", counts(LANE4=3)), "LANE2")


if __name__ == "__main__":
    unittest.main()
