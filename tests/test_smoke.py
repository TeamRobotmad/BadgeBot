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

@pytest.fixture
def port():
    return 1

def test_app_versions_match():
    import sim.apps.BadgeBot.app as BadgeBot
    import sim.apps.BadgeBot.hexdrive as HexDrive
    assert BadgeBot.CURRENT_HEXDRIVE_APP_VERSION == HexDrive.HEXDRIVE_APP_VERSION


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
    hd_by_pid = {hdt.pid: hdt for hdt in _HEXDRIVE_TYPES}

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
