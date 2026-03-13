# PID AutoTune UI Manager for BadgeBot
#
# Manages the PID Auto Tune user interface and state machine
# (STATE_AUTOTUNE).  The core tuning algorithm lives in autotune.py;
# this module handles the UI, button interactions, countdown integration,
# and display rendering.
#
# When the user presses CONFIRM ("C") to start tuning, the app now
# transitions to STATE_COUNTDOWN first (reusing the shared countdown
# mechanism) before entering STATE_AUTOTUNE with the tuner running.
# This gives the user time to move their hand away from the robot.
#
# Public interface (called by the main app):
#   __init__(app)           – wire up to LineFollowerApp
#   start()                 – enter autotune mode from menu
#   begin_tuning()          – actually start the tuner (called after countdown)
#   update(delta)           – per-tick state machine update
#   draw(ctx)               – render autotune UI
#   background_update(delta)– called from the fast background loop

from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification

from .autotune import PIDAutoTuner, compute_error, METHOD_ZIEGLER_NICHOLS
from .line_follow import (LineSensors, SENSOR_CTRL_PINS, SENSOR_SIGNAL_PINS,
                          SENSOR_NAMES, FOLLOWER_SENSOR_SCAN_PERIOD,
                          DEFAULT_UPDATE_PERIOD, create_line_sensors)


class AutotuneMgr:
    """Manages the PID Auto Tune UI and state machine.

    Parameters
    ----------
    app : LineFollowerApp
        Reference to the main application instance.
    """

    def __init__(self, app):
        self.app = app

    # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self):
        """Enter PID Auto Tune mode from the main menu."""
        app = self.app
        app.set_menu(None)
        app.button_states.clear()
        app._animation_counter = 0
        if app._line_sensors is None:
            app._line_sensors = create_line_sensors(app)
        if app._line_sensors is None:
            # Line sensors are not available; inform the user and abort autotune.
            Notification(app, "Line sensors not available")
            return
        app._line_sensors.enable()
        if app.hexdrive_app is not None:
            app.hexdrive_app.set_logging(False)
            if not app.hexdrive_app.set_power(True):
                print("Failed to enable HexDrive power")
        else:
            print("No HexDrive App")
        app._autotuner = None  # Reset - user will press CONFIRM to start
        app._sample_time = 0
        app._sample_count = 0
        from .linefollower import STATE_AUTOTUNE
        app.current_state = STATE_AUTOTUNE
        app._update_period = FOLLOWER_SENSOR_SCAN_PERIOD
        app._refresh = True
        print("AUTOTUNE: Entered PID Auto Tune mode")

    # ------------------------------------------------------------------
    # Begin tuning (called after countdown completes)
    # ------------------------------------------------------------------

    def begin_tuning(self):
        """Create and start a new auto-tuner instance.

        Called when the countdown finishes (after the user pressed CONFIRM).
        """
        app = self.app
        relay_amp = app._settings['max_power'].v // 4
        base_power = -app._settings['max_power'].v // 2
        app._autotuner = PIDAutoTuner(
            relay_amplitude=relay_amp,
            base_power=base_power,
            hysteresis=0.05,
            target_cycles=12,
            method=METHOD_ZIEGLER_NICHOLS,
            logging=app._settings['logging'].v
        )
        app._autotuner.start()
        print(f"AUTOTUNE: Starting with relay_amp={relay_amp} base_power={base_power}")
        from .linefollower import STATE_AUTOTUNE
        app.current_state = STATE_AUTOTUNE
        app._refresh = True

    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta):
        """Handle STATE_AUTOTUNE.  Returns True if handled."""
        app = self.app
        from .linefollower import (STATE_AUTOTUNE, STATE_MENU, STATE_COUNTDOWN,
            DEFAULT_UPDATE_PERIOD)
        if app.current_state != STATE_AUTOTUNE:
            return False

        app._sample_time += delta
        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if app.hexdrive_app is not None:
                app.hexdrive_app.set_motors((0, 0))
                app.hexdrive_app.set_power(False)
            app._line_sensors.disable()
            app._autotuner = None
            app._update_period = DEFAULT_UPDATE_PERIOD
            app.current_state = STATE_MENU
            print("AUTOTUNE: Cancelled by user")
            return True
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            if app._autotuner is None or not app._autotuner.is_running:
                # Instead of starting immediately, go through the countdown
                app._countdown_next_state = STATE_AUTOTUNE
                app.run_countdown_elapsed_ms = 0
                app.current_state = STATE_COUNTDOWN
                app._refresh = True
        if app._autotuner is not None:
            app._refresh = True
        return True

    # ------------------------------------------------------------------
    # Background update (called from the fast loop)
    # ------------------------------------------------------------------

    def background_update(self, delta):
        """PID auto-tune relay feedback control during STATE_AUTOTUNE.
        Returns motor output tuple, or None if not active."""
        app = self.app
        from .linefollower import STATE_AUTOTUNE
        if app.current_state != STATE_AUTOTUNE:
            return None
        if app._autotuner is not None and app._autotuner.is_running:
            app._line_sensors.read()
            left_raw = app._line_sensors.raw_value(0)
            right_raw = app._line_sensors.raw_value(1)
            error = compute_error(left_raw, right_raw)
            output = app._autotuner.update(error, delta)
            if app._autotuner.is_running:
                return output
            else:
                app._refresh = True
                if app._autotuner.is_complete:
                    gains = app._autotuner.get_gains()
                    if gains is not None:
                        app._settings['pid_kp'].v = gains[0]
                        app._settings['pid_ki'].v = gains[1]
                        app._settings['pid_kd'].v = gains[2]
                        app._settings['pid_kp'].persist()
                        app._settings['pid_ki'].persist()
                        app._settings['pid_kd'].persist()
                        print(f"AUTOTUNE: Gains saved to settings: Kp={gains[0]:.4f} Ki={gains[1]:.6f} Kd={gains[2]:.4f}")
                    app.notification = Notification(" Tuning   Complete")
                return (0, 0)
        return None

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx):
        """Render PID Auto Tune UI.  Returns True if handled."""
        app = self.app
        from .linefollower import STATE_AUTOTUNE
        if app.current_state != STATE_AUTOTUNE:
            return False

        ctx.save()
        if app._autotuner is None:
            app.draw_message(ctx,
                ["PID Auto Tune", "Place on line", "Press C to start"],
                [(1, 1, 1), (1, 1, 0), (0, 1, 0)], label_font_size)
            button_labels(ctx, confirm_label="Start", cancel_label="Exit")
        elif app._autotuner.is_running:
            diag = app._autotuner.get_diagnostics()
            status = app._autotuner.get_status_text()
            app.draw_message(ctx,
                ["PID Auto Tune", status,
                 f"Cross: {diag['crossings']}/{diag['target']}",
                 f"T={diag['elapsed']//1000}s"],
                [(1, 1, 1), (1, 1, 0), (0, 1, 1), (0.7, 0.7, 0.7)], label_font_size)
            button_labels(ctx, cancel_label="Stop")
        elif app._autotuner.is_complete:
            diag = app._autotuner.get_diagnostics()
            q = diag['quality']
            q_colour = (0, 1, 0) if q >= 60 else (1, 1, 0) if q >= 30 else (1, 0, 0)
            app.draw_message(ctx,
                ["Tune Complete",
                 f"Q={q:.0f}%",
                 f"Kp={diag['Kp']:.2f}",
                 f"Ki={diag['Ki']:.4f}",
                 f"Kd={diag['Kd']:.2f}"],
                [(0, 1, 0), q_colour, (1, 1, 1), (1, 1, 1), (1, 1, 1)], label_font_size)
            button_labels(ctx, confirm_label="Retry", cancel_label="Exit")
        else:
            app.draw_message(ctx,
                ["Tune Failed", "Check line", "and retry"],
                [(1, 0, 0), (1, 1, 0), (1, 1, 0)], label_font_size)
            button_labels(ctx, confirm_label="Retry", cancel_label="Exit")
        ctx.restore()
        return True
