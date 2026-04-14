import sys

import pytest

# Add badge software to pythonpath
sys.path.append("../../../") 

import sim.run
from system.hexpansion.config import HexpansionConfig


def test_import_badgebot_app_and_app_export():
    import sim.apps.BadgeBot.app as BadgeBot
    from sim.apps.BadgeBot import BadgeBotApp
    assert BadgeBot.__app_export__ == BadgeBotApp

def test_import_hexdrive_app_and_app_export():
    import sim.apps.BadgeBot.hexdrive as HexDrive
    from sim.apps.BadgeBot.hexdrive import HexDriveApp
    assert HexDrive.__app_export__ == HexDriveApp

def test_badgebot_app_init():
    from sim.apps.BadgeBot import BadgeBotApp
    BadgeBotApp()

def test_hexdrive_app_init(port):
    from sim.apps.BadgeBot.hexdrive import HexDriveApp
    config = HexpansionConfig(port)
    HexDriveApp(config)

def test_app_versions_match():
    import sim.apps.BadgeBot.app as BadgeBot
    import sim.apps.BadgeBot.hexdrive as HexDrive
    assert BadgeBot.HEXDRIVE_APP_VERSION == HexDrive.VERSION
    # above test should always pass since BadgeBot.HEXDRIVE_APP_VERSION is imported from HexDrive.VERSION, but this test will at least catch if someone accidentally changes one without the other. 

def test_hexdrive_type_pids_consistent():
    """Verify HexDriveType PIDs in hexdrive.py are consistent with HexpansionType PIDs in app.py.

    HexDriveType stores a single PID byte (low byte), while HexpansionType
    stores the full 16-bit PID.  For every HexDrive-flavour HexpansionType
    the low byte of its PID must match exactly one HexDriveType entry, and
    the motor/servo/stepper capability counts must agree.
    """
    from sim.apps.BadgeBot import BadgeBotApp
    from sim.apps.BadgeBot.hexdrive import _HEXDRIVE_TYPES

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
        assert ht.steppers == hdt.steppers, (
            f"Stepper count mismatch for PID 0x{pid_byte:02X}: "
            f"HexpansionType={ht.steppers}, HexDriveType={hdt.steppers}"
        )


def test_new_states_exist():
    """Verify the new STATE_SENSOR and STATE_AUTODRIVE constants are defined."""
    import sim.apps.BadgeBot.app as BadgeBot
    assert hasattr(BadgeBot, 'STATE_SENSOR')
    assert hasattr(BadgeBot, 'STATE_AUTODRIVE')
    assert BadgeBot.STATE_SENSOR != BadgeBot.STATE_AUTODRIVE


def test_new_settings_registered():
    """Verify motor1_dir, motor2_dir, and front_face base settings are always registered."""
    from sim.apps.BadgeBot import BadgeBotApp
    app_instance = BadgeBotApp()
    for key in ('motor1_dir', 'motor2_dir', 'front_face'):
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


def test_front_face_labels_complete():
    """Verify _FRONT_FACE_LABELS has one entry for each valid front_face value (0-11)."""
    import sim.apps.BadgeBot.app as BadgeBot
    assert hasattr(BadgeBot, '_FRONT_FACE_LABELS')
    assert len(BadgeBot._FRONT_FACE_LABELS) == 12


def test_menu_items_include_sensor_and_auto():
    """Verify the main menu includes Sensor Test and Auto Drive entries."""
    import sim.apps.BadgeBot.app as BadgeBot
    assert "Sensor Test" in BadgeBot.MAIN_MENU_ITEMS
    assert "Auto Drive" in BadgeBot.MAIN_MENU_ITEMS


def test_sensor_base_interface():
    """Verify SensorBase class has the expected interface."""
    from sim.apps.BadgeBot.sensors.sensor_base import SensorBase
    sensor = SensorBase()
    assert hasattr(sensor, 'begin')
    assert hasattr(sensor, 'read')
    assert hasattr(sensor, 'reset')
    assert hasattr(sensor, 'is_ready')
    assert sensor.is_ready is False


def test_all_sensor_classes_populated():
    """Verify ALL_SENSOR_CLASSES contains the expected sensor drivers."""
    from sim.apps.BadgeBot.sensors import ALL_SENSOR_CLASSES
    assert len(ALL_SENSOR_CLASSES) >= 5
    names = {cls.NAME for cls in ALL_SENSOR_CLASSES}
    assert 'VL53L0X' in names or 'VL6180X' in names  # at least one ToF sensor
    assert 'TCS3472' in names or 'TCS3430' in names  # at least one color sensor
    assert 'OPT4048' in names  # OPT4048 tristimulus sensor
