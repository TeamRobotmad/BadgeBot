"""Focused tests for SensorTestMgr display helpers and page rendering."""

# pylint: disable=protected-access

import sys
from types import SimpleNamespace

sys.path.append("../../../")

import sim.run as _sim_run


def test_sensor_display_orders_rgb_first():
    from sim.apps.BadgeBot.sensor_test import SensorTestMgr

    ordered = SensorTestMgr._ordered_display_items({"w": 4, "b": 3, "extra": 5, "g": 2, "r": 1})
    assert ordered == [("r", "1"), ("g", "2"), ("b", "3"), ("w", "4"), ("extra", "5")]


def test_sensor_white_reference_normalises_rgb_channels():
    from sim.apps.BadgeBot.sensor_test import SensorTestMgr

    gains = SensorTestMgr._reference_to_gains(50, 100, 200, 400)
    calibrated = SensorTestMgr._apply_white_reference(50, 100, 150, 200, gains)
    assert calibrated == (1024, 1024, 768, 512)


def test_distance_sensor_raw_page_shows_latest_sample():
    from sim.apps.BadgeBot.sensor_test import SensorTestMgr, _PAGE_RAW

    mgr = object.__new__(SensorTestMgr)
    mgr._display_data = {}
    mgr._page_selected = _PAGE_RAW
    mgr._page_count = 0
    mgr._sample_rate = 0
    mgr._sensor_data = {"dist_mm": "345"}
    mgr._sensor_mgr = SimpleNamespace(type="Distance")

    mgr._update_display_values()

    assert mgr._display_data == {"dist_mm": "345"}
