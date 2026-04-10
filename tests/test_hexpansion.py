"""Tests for fake-hexpansion infrastructure and hardware-dependent behaviour.

These tests exercise the BadgeBotApp with various fake hexpansion
configurations to verify that settings and menu items are correctly
gated by the detected hardware capabilities.
"""
import pytest


# =====================================================================
#  Baseline: NO hexpansion
# =====================================================================

class TestNoHexpansion:
    """Tests with no fake hexpansion – only base settings should exist."""

    def test_base_settings_present(self, badgebot_app):
        """Base settings are always registered in __init__."""
        for key in ('brightness', 'logging', 'fwd_dir', 'front_face'):
            assert key in badgebot_app.settings, f"Missing base setting: {key}"

    def test_motor_settings_absent_without_hexpansion(self, badgebot_app):
        """Motor-dependent settings must not exist without a HexDrive."""
        for key in ('acceleration', 'max_power', 'drive_step_ms', 'turn_step_ms'):
            assert key not in badgebot_app.settings, (
                f"Setting '{key}' should not be registered without a HexDrive"
            )

    def test_servo_settings_absent_without_hexpansion(self, badgebot_app):
        """Servo-dependent settings must not exist without a HexDrive."""
        for key in ('servo_step', 'servo_range', 'servo_period'):
            assert key not in badgebot_app.settings, (
                f"Setting '{key}' should not be registered without a HexDrive"
            )

    def test_stepper_settings_absent_without_hexpansion(self, badgebot_app):
        """Stepper-dependent settings must not exist without a HexDrive."""
        assert 'step_max_pos' not in badgebot_app.settings

    def test_autodrive_settings_absent_without_hexpansion(self, badgebot_app):
        """Auto-drive settings must not exist without a HexDrive."""
        for key in ('auto_speed', 'auto_obstacle'):
            assert key not in badgebot_app.settings, (
                f"Setting '{key}' should not be registered without a HexDrive"
            )

    def test_hardware_counts_zero(self, badgebot_app):
        """Without any hexpansion, all hardware counts must be zero."""
        assert badgebot_app.num_motors == 0
        assert badgebot_app.num_servos == 0
        assert badgebot_app.num_steppers == 0


# =====================================================================
#  2-Motor HexDrive (PID 0xCBCA)
# =====================================================================

class TestTwoMotorHexDrive:
    """Tests with a fake 2-Motor HexDrive."""

    @pytest.fixture
    def hexdrive_pid(self):
        return 0xCBCA

    def test_hardware_counts(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        assert app.num_motors == 2
        assert app.num_servos == 0
        assert app.num_steppers == 0

    def test_reaches_menu(self, badgebot_app_with_hexpansion):
        from sim.apps.BadgeBot.app import STATE_MENU
        assert badgebot_app_with_hexpansion.current_state == STATE_MENU

    def test_motor_settings_registered(self, badgebot_app_with_hexpansion):
        s = badgebot_app_with_hexpansion.settings
        for key in ('acceleration', 'max_power', 'drive_step_ms', 'turn_step_ms'):
            assert key in s, f"Missing motor setting: {key}"

    def test_servo_settings_absent(self, badgebot_app_with_hexpansion):
        s = badgebot_app_with_hexpansion.settings
        for key in ('servo_step', 'servo_range', 'servo_period'):
            assert key not in s, f"Setting '{key}' should not exist for 2-Motor"

    def test_stepper_settings_absent(self, badgebot_app_with_hexpansion):
        assert 'step_max_pos' not in badgebot_app_with_hexpansion.settings

    def test_autodrive_settings_registered(self, badgebot_app_with_hexpansion):
        s = badgebot_app_with_hexpansion.settings
        for key in ('auto_speed', 'auto_obstacle'):
            assert key in s, f"Missing auto-drive setting: {key}"

    def test_menu_includes_motor_moves(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Motor Moves" in items

    def test_menu_includes_auto_drive(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Auto Drive" in items

    def test_menu_excludes_servo_test(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Servo Test" not in items

    def test_menu_excludes_stepper_test(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Stepper Test" not in items

    def test_menu_excludes_line_follower_without_hexsense(self, badgebot_app_with_hexpansion):
        """Line Follower needs both motors and line sensors."""
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Line Follower" not in items

    def test_menu_includes_sensor_test(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Sensor Test" in items

    def test_menu_includes_common_items(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        for expected in ("Hexpansions", "Settings", "About", "Exit"):
            assert expected in items, f"Missing common menu item: {expected}"


# =====================================================================
#  4-Servo HexDrive (PID 0xCBCC)
# =====================================================================

class TestFourServoHexDrive:
    """Tests with a fake 4-Servo HexDrive."""

    @pytest.fixture
    def hexdrive_pid(self):
        return 0xCBCC

    def test_hardware_counts(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        assert app.num_motors == 0
        assert app.num_servos == 4
        assert app.num_steppers == 0

    def test_servo_settings_registered(self, badgebot_app_with_hexpansion):
        s = badgebot_app_with_hexpansion.settings
        for key in ('servo_step', 'servo_range', 'servo_period'):
            assert key in s, f"Missing servo setting: {key}"

    def test_motor_settings_absent(self, badgebot_app_with_hexpansion):
        s = badgebot_app_with_hexpansion.settings
        for key in ('acceleration', 'max_power', 'drive_step_ms', 'turn_step_ms'):
            assert key not in s, f"Setting '{key}' should not exist for 4-Servo"

    def test_stepper_settings_absent(self, badgebot_app_with_hexpansion):
        assert 'step_max_pos' not in badgebot_app_with_hexpansion.settings

    def test_autodrive_settings_absent(self, badgebot_app_with_hexpansion):
        """Auto drive requires motors > 1."""
        s = badgebot_app_with_hexpansion.settings
        for key in ('auto_speed', 'auto_obstacle'):
            assert key not in s, f"Setting '{key}' should not exist for 4-Servo"

    def test_menu_includes_servo_test(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Servo Test" in items

    def test_menu_excludes_motor_moves(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Motor Moves" not in items

    def test_menu_excludes_auto_drive(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Auto Drive" not in items

    def test_menu_excludes_stepper_test(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Stepper Test" not in items


# =====================================================================
#  Stepper HexDrive (PID 0xCBCE)
# =====================================================================

class TestStepperHexDrive:
    """Tests with a fake Stepper HexDrive."""

    @pytest.fixture
    def hexdrive_pid(self):
        return 0xCBCE

    def test_hardware_counts(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        assert app.num_motors == 0
        assert app.num_servos == 0
        assert app.num_steppers == 1

    def test_stepper_settings_registered(self, badgebot_app_with_hexpansion):
        assert 'step_max_pos' in badgebot_app_with_hexpansion.settings

    def test_motor_settings_absent(self, badgebot_app_with_hexpansion):
        s = badgebot_app_with_hexpansion.settings
        for key in ('acceleration', 'max_power', 'drive_step_ms', 'turn_step_ms'):
            assert key not in s, f"Setting '{key}' should not exist for Stepper"

    def test_servo_settings_absent(self, badgebot_app_with_hexpansion):
        s = badgebot_app_with_hexpansion.settings
        for key in ('servo_step', 'servo_range', 'servo_period'):
            assert key not in s, f"Setting '{key}' should not exist for Stepper"

    def test_menu_includes_stepper_test(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Stepper Test" in items

    def test_menu_excludes_motor_moves(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Motor Moves" not in items

    def test_menu_excludes_servo_test(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Servo Test" not in items


# =====================================================================
#  1 Motor 2 Servo HexDrive (PID 0xCBCD)
# =====================================================================

class TestOneMotorTwoServoHexDrive:
    """Tests with a fake 1-Motor-2-Servo HexDrive."""

    @pytest.fixture
    def hexdrive_pid(self):
        return 0xCBCD

    def test_hardware_counts(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        assert app.num_motors == 1
        assert app.num_servos == 2
        assert app.num_steppers == 0

    def test_servo_settings_registered(self, badgebot_app_with_hexpansion):
        s = badgebot_app_with_hexpansion.settings
        for key in ('servo_step', 'servo_range', 'servo_period'):
            assert key in s, f"Missing servo setting: {key}"

    def test_motor_moves_settings_absent(self, badgebot_app_with_hexpansion):
        """Motor Moves requires num_motors > 1; 1-Motor variant has only 1."""
        s = badgebot_app_with_hexpansion.settings
        for key in ('acceleration', 'max_power', 'drive_step_ms', 'turn_step_ms'):
            assert key not in s, f"Setting '{key}' should not exist for 1-Motor"

    def test_menu_includes_servo_test(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Servo Test" in items

    def test_menu_excludes_motor_moves(self, badgebot_app_with_hexpansion):
        """Motor Moves requires > 1 motor."""
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Motor Moves" not in items

    def test_menu_excludes_auto_drive(self, badgebot_app_with_hexpansion):
        """Auto Drive requires > 1 motor."""
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Auto Drive" not in items


# =====================================================================
#  Generic "Unknown" HexDrive (PID 0xCBCB) - has everything
# =====================================================================

class TestFullHexDrive:
    """Tests with the full-capability HexDrive (2 motors, 4 servos, 1 stepper)."""

    @pytest.fixture
    def hexdrive_pid(self):
        return 0xCBCB

    def test_hardware_counts(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        assert app.num_motors == 2
        assert app.num_servos == 4
        assert app.num_steppers == 1

    def test_all_hardware_settings_registered(self, badgebot_app_with_hexpansion):
        s = badgebot_app_with_hexpansion.settings
        expected = (
            # base
            'brightness', 'logging', 'fwd_dir', 'front_face',
            # motor moves
            'acceleration', 'max_power', 'drive_step_ms', 'turn_step_ms',
            # servo test
            'servo_step', 'servo_range', 'servo_period',
            # stepper test
            'step_max_pos',
            # auto drive
            'auto_speed', 'auto_obstacle',
        )
        for key in expected:
            assert key in s, f"Missing setting: {key}"

    def test_menu_includes_motor_servo_stepper(self, badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        app.set_menu("main")
        items = [item for item in app.menu.menu_items]
        assert "Motor Moves" in items
        assert "Servo Test" in items
        assert "Stepper Test" in items
        assert "Auto Drive" in items
