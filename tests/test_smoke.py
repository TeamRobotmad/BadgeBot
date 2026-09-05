import re
from math import pi, radians
from pathlib import Path

import pytest
from system.hexpansion.config import HexpansionConfig


def _extract_version_from_source(path: Path) -> int:
    content = path.read_text(encoding="utf-8")
    match = re.search(r"^\s*VERSION\s*=\s*(\d+)", content, re.MULTILINE)
    assert match is not None, f"Could not find VERSION in {path}"
    return int(match.group(1))


def test_parse_version_handles_dev_and_git_suffixes():
    from sim.apps.BadgeBot.utils import parse_version

    assert parse_version("v2.2.0") == (2, 2, 0)
    assert parse_version("v2.2.0-dev") == (2, 2, 0)
    assert parse_version("v2.2.0-20-gabcdef") == (2, 2, 0)
    assert parse_version("v2.2.1-rc1") == (2, 2, 1)
    assert parse_version("v2.2.1-rc1+build.7") == (2, 2, 1)


def test_parse_version_works_without_re_findall(monkeypatch):
    import sim.apps.BadgeBot.utils as badge_utils

    class NoFindallModule:
        pass

    monkeypatch.setattr(badge_utils, "re", NoFindallModule())

    assert badge_utils.parse_version("v2.3.4-rc1+build.7") == (2, 3, 4)


def test_ctx_fake_imports_without_wasmtime(monkeypatch):
    import builtins
    import importlib
    import sys

    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "wasmtime":
            raise ModuleNotFoundError("No module named 'wasmtime'")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    sys.modules.pop("sim.fakes.ctx", None)

    mod = importlib.import_module("sim.fakes.ctx")
    assert mod.wasmtime is None
    assert mod._wasm is None
    with pytest.raises(RuntimeError, match="wasmtime"):
        mod._require_wasm()


def test_import_badgebot_app_and_app_export():
    import sim.apps.BadgeBot.app as BadgeBot
    from sim.apps.BadgeBot import BadgeBotApp
    assert BadgeBot.__app_export__ == BadgeBotApp

def test_import_hexdrive_app_and_app_export():
    import sim.apps.BadgeBot.vendor.HexDrive.hexdrive as HexDrive
    from sim.apps.BadgeBot.vendor.HexDrive.hexdrive import HexDriveApp
    assert HexDrive.__app_export__ == HexDriveApp

def test_hexdrive_instance_exposes_version():
    from sim.apps.BadgeBot.vendor.HexDrive.hexdrive import HexDriveApp
    app_instance = HexDriveApp(HexpansionConfig(1))
    assert getattr(app_instance, "VERSION", None) == HexDriveApp.VERSION

def test_badgebot_app_init():
    from sim.apps.BadgeBot import BadgeBotApp
    BadgeBotApp()

def test_hexdrive_app_init(port):
    from sim.apps.BadgeBot.vendor.HexDrive.hexdrive import HexDriveApp
    config = HexpansionConfig(port)
    HexDriveApp(config)

def test_app_versions_match():
    import sim.apps.BadgeBot.app as BadgeBot
    from sim.apps.BadgeBot.vendor.HexDrive.hexdrive import HexDriveApp
    assert BadgeBot.HEXDRIVE_APP_VERSION == HexDriveApp.VERSION

def test_hexdrive2_metadata_matches_vendor_source():
    import sim.apps.BadgeBot.app as BadgeBot
    from sim.apps.BadgeBot import BadgeBotApp

    source_version = _extract_version_from_source(
        Path(__file__).resolve().parents[1] / "vendor" / "HexDrive2" / "hexdrive2.py"
    )
    assert BadgeBot.HEXDRIVE2_APP_VERSION == source_version

    app_instance = BadgeBotApp()
    hexdrive2_entries = [
        ht for ht in app_instance.HEXPANSION_TYPES if ht.name == "HexDrive2"
    ]
    assert hexdrive2_entries, "No HexDrive2 entries found in BadgeBot metadata"
    for entry in hexdrive2_entries:
        assert entry.app_mpy_name == "hexdrive2"
        assert entry.app_mpy_version == BadgeBot.HEXDRIVE2_APP_VERSION


def test_hexdrive_type_pids_consistent():
    """Verify HexDriveType PIDs in hexdrive.py are consistent with HexpansionType PIDs in app.py.

    HexDriveType stores a single PID byte (low byte), while HexpansionType
    stores the full 16-bit PID.  For every HexDrive-flavour HexpansionType
    the low byte of its PID must match exactly one HexDriveType entry, and
    the motor/servo capability counts must agree.
    """
    from sim.apps.BadgeBot import BadgeBotApp
    from sim.apps.BadgeBot.vendor.HexDrive.hexdrive import _HEXDRIVE_TYPES

    app_instance = BadgeBotApp()
    hexdrive_hexpansion_types = [
        ht for ht in app_instance.HEXPANSION_TYPES if ht.name == "HexDrive"
    ]

    # Build a lookup from PID byte -> HexDriveType
    # Also verify that PID bytes are unique within _HEXDRIVE_TYPES
    hd_by_pid = {}
    for hdt in _HEXDRIVE_TYPES:
        assert hdt.pid not in hd_by_pid, (
            f"Duplicate HexDriveType PID byte 0x{hdt.pid:02X}: "
            f"'{hd_by_pid[hdt.pid].name}' and '{hdt.name}'"
        )
        hd_by_pid[hdt.pid] = hdt

    for ht in hexdrive_hexpansion_types:
        pid_byte = ht.pid & 0xFF
        assert pid_byte in hd_by_pid, (
            f"HexpansionType PID 0x{ht.pid:04X} low byte 0x{pid_byte:02X} "
            f"has no matching HexDriveType"
        )
        hdt = hd_by_pid[pid_byte]
        assert ht.motors == hdt.motors, (
            f"Motor count mismatch for PID 0x{pid_byte:02X}: "
            f"HexpansionType={ht.motors}, HexDriveType={hdt.motors}"
        )
        assert ht.servos == hdt.servos, (
            f"Servo count mismatch for PID 0x{pid_byte:02X}: "
            f"HexpansionType={ht.servos}, HexDriveType={hdt.servos}"
        )


def test_new_settings_registered():
    """Verify motor direction and front-face base settings are always registered."""
    from sim.apps.BadgeBot import BadgeBotApp
    app_instance = BadgeBotApp()
    for key in ('mtr1_dir', 'mtr2_dir', 'front_face'):
        assert key in app_instance.settings, f"Missing setting: {key}"


def test_autodrive_settings_need_hexpansion():
    """auto_speed/auto_obstacle are hardware-dependent; not present without a HexDrive."""
    from sim.apps.BadgeBot import BadgeBotApp
    app_instance = BadgeBotApp()
    # Without a HexDrive, auto-drive settings are NOT registered
    for key in ('auto_speed', 'auto_obstacle'):
        assert key not in app_instance.settings, (
            f"Setting '{key}' should not be registered without a HexDrive"
        )


def test_autodrive_decide_state_and_turn_transition():
    """Auto-drive should pause to display the scan before entering the turn state."""
    import types
    import sim.apps.BadgeBot.autodrive as autodrive

    class DummyApp:
        def __init__(self):
            self.sensor_test_mgr = None
            self.bluetooth_mgr = None
            self.acceleration = 20000
            self.max_power = 55000
            self.button_states = {}
            self.settings = {
                "auto_speed": types.SimpleNamespace(v=56000),
                "auto_scan_speed": types.SimpleNamespace(v=14000),
                "auto_obstacle": types.SimpleNamespace(v=250),
            }
            self.refresh = False
            self.hexdrive_apps = []
            self.notification = None

        def enable_motors(self, *args, **kwargs):
            return True

        def set_menu(self, *args, **kwargs):
            return None

        def return_to_menu(self, *args, **kwargs):
            return None

        def draw_message(self, *args, **kwargs):
            return None

    mgr = autodrive.AutoDriveMgr(DummyApp(), logging=False)
    mgr._active = True
    mgr.sub_state = autodrive._AUTO_SUB_DECIDE
    mgr.decide_timer = 10
    mgr.quadrant_candidates = [("front", 0.0, 0.0, True)]
    mgr.turn_dir = 1
    mgr.turn_deg = 90.0
    mgr._app.button_states = {}

    mgr.update(100)

    assert mgr.sub_state == autodrive._AUTO_SUB_TURN


def test_front_face_labels_complete():
    """Verify _FRONT_FACE_LABELS has one entry for each valid front_face value (0-11)."""
    import sim.apps.BadgeBot.app as BadgeBot
    front_face_labels = getattr(BadgeBot, '_FRONT_FACE_LABELS', None)
    assert front_face_labels is not None
    assert len(front_face_labels) == 12


def test_autodrive_scan_theta_uses_front_face_rotation():
    """Scan plot angles should rotate with the configured front face, not a fixed 90° offset."""
    import types
    import sim.apps.BadgeBot.autodrive as autodrive

    app = types.SimpleNamespace(
        sensor_test_mgr=None,
        settings={"front_face": types.SimpleNamespace(v=5)},
    )
    mgr = autodrive.AutoDriveMgr(app, logging=False)

    expected = radians(0.0) - (pi / 2.0) + radians(5 * 30.0)
    assert mgr._scan_angle_to_theta(0.0) == pytest.approx(expected)


def test_menu_items_include_sensor_and_auto():
    """Verify the main menu includes Sensor Test and Auto Drive entries."""
    import sim.apps.BadgeBot.app as BadgeBot
    assert "Sensor Test" in BadgeBot.MAIN_MENU_ITEMS
    assert "Auto Drive" in BadgeBot.MAIN_MENU_ITEMS


def test_remote_autodrive_button_3_maps_and_starts():
    """Bluefruit button 3 should trigger Auto Drive start/stop via the same remote command path."""
    import sim.apps.BadgeBot.app as BadgeBot
    from sim.apps.BadgeBot.bluetooth_mgr import _CONTROL_BUTTON_COMMANDS

    app = BadgeBot.BadgeBotApp()
    app.current_state = BadgeBot.STATE_MENU
    app.num_motors = 2
    seen = {}

    class DummyAutoDriveMgr:
        def start(self):
            seen["start"] = True
            return True

        def stop(self):
            seen["stop"] = True

    app._autodrive_mgr = DummyAutoDriveMgr()

    assert _CONTROL_BUTTON_COMMANDS["3"] == BadgeBot.REMOTE_CMD_AUTO_DRIVE_TOGGLE
    app.post_remote_command(BadgeBot.REMOTE_CMD_AUTO_DRIVE_TOGGLE)
    app._process_remote_commands()

    assert app.current_state == BadgeBot.STATE_AUTODRIVE
    assert seen["start"] is True


def test_sensor_base_interface():
    """Verify SensorBase class has the expected interface."""
    from sim.apps.BadgeBot.sensors.sensor_base import SensorBase
    sensor = SensorBase()
    assert hasattr(sensor, 'begin')
    assert hasattr(sensor, 'read')
    assert hasattr(sensor, 'reset')
    assert hasattr(sensor, 'is_ready')
    assert sensor.is_ready is False


def test_legacy_sensor_registry_is_disabled():
    """HexDrive2 owns sensor polling; the old app-level registry stays empty."""
    from sim.apps.BadgeBot.sensors import ALL_SENSOR_CLASSES
    assert ALL_SENSOR_CLASSES == []
