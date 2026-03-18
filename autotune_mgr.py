# PID AutoTune UI Manager for BadgeBot
#
# Manages the PID Auto Tune user interface and state machine.
# The core tuning algorithm lives in autotune.py;
# this module handles the UI, button interactions, countdown integration,
# and display rendering.
#
# When the user presses CONFIRM ("C") to start tuning, the app now
# transitions to STATE_COUNTDOWN first (reusing the shared countdown
# mechanism) before entering STATE_AUTOTUNE with the tuner running.
# This gives the user time to move their hand away from the robot.
#
# Public interface (called by the main app):
#   __init__(app)           – wire up to BadgeBotApp
#   start()                 – enter autotune mode from menu
#   begin_tuning()          – actually start the tuner (called after countdown)
#   update(delta)           – per-tick state machine update
#   draw(ctx)               – render autotune UI
#   background_update(delta)– called from the fast background loop

from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification

from .autotune import PIDAutoTuner, METHOD_ZIEGLER_NICHOLS
from .line_follow import create_line_sensors
from .app import (STATE_AUTOTUNE, STATE_COUNTDOWN)

AUTOTUNER_UPDATE_PERIOD = 10  # ms between updates while tuning

class AutotuneMgr:
    """Manages the PID Auto Tune UI and state machine.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """

    def __init__(self, app, follower):
        self.app = app
        self.follower = follower
        self.autotuner = None

    # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Enter PID Auto Tune mode from the main menu."""
        app = self.app
        if self.follower.line_sensors is None:
            self.follower.line_sensors = create_line_sensors(app.line_sensors_hexpansion_config, app.num_line_sensors)

        if self.follower.line_sensors is None:
            # Line sensors are not available; inform the user and abort autotune.
            Notification(app, "Line sensors not available")
            return False
        if app.hexdrive_app is not None:
            app.hexdrive_app.set_logging(False)
            if not app.hexdrive_app.set_power(True):
                print("Failed to enable HexDrive power")
        else:
            print("No HexDrive App")
            return False
        #self.follower.line_sensors.enable() # using blocking_read which does not require enabling.
        app.set_menu(None)
        app.button_states.clear()
        self.autotuner = None
        app.update_period = AUTOTUNER_UPDATE_PERIOD
        app.refresh = True
        if app.settings['logging'].v:
            print("AUTOTUNE: Entered PID Auto Tune mode")
        return True

    # ------------------------------------------------------------------
    # Begin tuning (called after countdown completes)
    # ------------------------------------------------------------------

    def begin_tuning(self):
        """Create and start a new auto-tuner instance.

        Called when the countdown finishes (after the user pressed CONFIRM).
        """
        app = self.app
        relay_amp = app.settings['max_power'].v // 4
        base_power = -app.settings['max_power'].v // 2
        self.autotuner = PIDAutoTuner(
            relay_amplitude=relay_amp,
            base_power=base_power,
            hysteresis=50,  # out of 1000
            target_cycles=12,
            method=METHOD_ZIEGLER_NICHOLS,
            logging=app.settings['logging'].v
        )
        self.autotuner.start()
        if app.settings['logging'].v:
            print(f"AUTOTUNE: Starting with relay_amp={relay_amp} base_power={base_power}")
        app.refresh = True

    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta) -> bool:        # pylint: disable=unused-argument
        """Handle Autotune UI.  Returns True if handled."""
        app = self.app
        self.follower.sample_time += delta

        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if app.hexdrive_app is not None:
                app.hexdrive_app.set_motors((0, 0))
                app.hexdrive_app.set_power(False)
            self.follower.line_sensors.disable()
            self.autotuner = None
            app.return_to_menu()
            print("AUTOTUNE: Cancelled by user")
            return True
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            if self.autotuner is None or not self.autotuner.is_running:
                # Instead of starting immediately, go through the countdown
                app.countdown_next_state = STATE_AUTOTUNE
                app.run_countdown_elapsed_ms = 0
                app.current_state = STATE_COUNTDOWN
                app.refresh = True

        if (self.follower.sample_time > 1000):
            sample_count = self.follower.line_sensors.sample_count_and_reset()
            self.follower.sensor_rate = int(((self.follower.sample_time / self.follower.line_sensors.num_sensors) * sample_count) // self.follower.sample_time)
            self.follower.sample_time = 0
            app.refresh = True                
        
        return True


    # ------------------------------------------------------------------
    # Background update (called from the fast loop)
    # ------------------------------------------------------------------

    def background_update(self, delta) -> tuple[int, int] | None:
        """PID auto-tune relay feedback control during STATE_AUTOTUNE.
        Returns motor output tuple, or None if not active."""
        if self.autotuner is not None and self.autotuner.is_running:
            #self.follower.line_sensors.read()
            self.follower.line_sensors.read_blocking()    # wait for sensor reading        
            left_raw  = self.follower.line_sensors.raw_value(0)
            right_raw = self.follower.line_sensors.raw_value(1)
            error = self.follower.compute_error(left_raw, right_raw)
            output = self.autotuner.update(error, delta)
            if self.autotuner.is_running:
                return output
            else:
                # Autotuner has just completed/failed
                self.autotune_complete() # Halt
                return (0, 0)
        return None


    def autotune_complete(self):
        app = self.app
        app.refresh = True
        if self.autotuner.is_complete:
            gains = self.autotuner.get_gains()
            if gains is not None:
                app.settings['pid_kp'].v = int(1000 * gains[0])
                app.settings['pid_ki'].v = int(1000 * gains[1])
                app.settings['pid_kd'].v = int(1000 * gains[2])
                app.settings['pid_kp'].persist()
                app.settings['pid_ki'].persist()
                app.settings['pid_kd'].persist()
                print(f"AUTOTUNE: Gains saved to settings: Kp={gains[0]:.4f} Ki={gains[1]:.6f} Kd={gains[2]:.4f}")
            app.notification = Notification(" Tuning    Complete")


    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx) -> bool:
        """Render PID Auto Tune UI.  Returns True if handled."""
        app = self.app

        ctx.save()
        if self.autotuner is None:
            app.draw_message(ctx,
                ["PID Auto Tune", "Place on line", "Press C to start"],
                [(1, 1, 1), (1, 1, 0), (0, 1, 0)], label_font_size)
            button_labels(ctx, confirm_label="Start", cancel_label="Exit")
        elif self.autotuner.is_running:
            diag = self.autotuner.get_diagnostics()
            status = self.autotuner.get_status_text()
            app.draw_message(ctx,
                ["PID Auto Tune", status,
                 f"Cross: {diag['crossings']}/{diag['target']}",
                 f"T={diag['elapsed']//1000}s",
                 f"Rate: {self.follower.sensor_rate} sps"],
                [(1, 1, 1), (1, 1, 0), (0, 1, 1), (0.7, 0.7, 0.7), (1, 0, 1)], label_font_size)
            button_labels(ctx, cancel_label="Stop")
        elif self.autotuner.is_complete:
            diag = self.autotuner.get_diagnostics()
            q = diag['quality']
            q_colour = (0, 1, 0) if q >= 60 else (1, 1, 0) if q >= 30 else (1, 0, 0)
            app.draw_message(ctx,
                ["Tune Complete",
                 f"Q={q:.0f}%",
                 f"Kp={diag['Kp']:.2f}",
                 f"Ki={diag['Ki']:.4f}",
                 f"Kd={diag['Kd']:.2f}"],
                [(0, 1, 0), q_colour, (1, 1, 1), (1, 1, 1), (1, 1, 1)], label_font_size)
            button_labels(ctx, confirm_label="Retry", cancel_label="Accept")
        else:
            app.draw_message(ctx,
                ["Tune Failed", "Check line", "and retry"],
                [(1, 0, 0), (1, 1, 0), (1, 1, 0)], label_font_size)
            button_labels(ctx, confirm_label="Retry", cancel_label="Exit")
        ctx.restore()
        return True
