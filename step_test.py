# Stepper Tester Module for BadgeBot
#
# Handles the stepper motor tester functionality (STATE_STEPPER).
# Contains the Stepper motor driver class and the StepperMode helper.
#
# Public interface (called by the main app):
#   __init__(app)   – wire up to LineFollowerApp
#   start()         – enter stepper test from menu
#   update(delta)   – per-tick state machine update
#   draw(ctx)       – render stepper tester UI

import time
from math import pi
from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification
try:
    from machine import Timer
except ImportError:
    Timer = None


# Stepper Tester - Defaults
_STEPPER_MAX_SPEED     = 200
_STEPPER_MAX_POSITION  = 3100
_STEPPER_DEFAULT_SPEED = 50
_STEPPER_NUM_PHASES    = 8
_STEPPER_DEFAULT_SPR   = 200
_STEPPER_DEFAULT_STEP  = 1


# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting):
    """Register stepper-test-specific settings in the shared settings dict."""
    s['step_max_pos']  = MySetting(s, _STEPPER_MAX_POSITION, 0, 65535)


class StepperMode:
    OFF = 0
    POSITION = 1
    SPEED = 2
    stepper_modes = ["OFF", "POSITION", "SPEED"]

    def __init__(self, mode=OFF):
        self.mode = mode

    def set(self, mode):
        self.mode = mode

    def inc(self):
        self.mode = (self.mode + 1) % 3

    def __eq__(self, other):
        return self.mode == other

    def __str__(self):
        return self.stepper_modes[self.mode]


# ---- Stepper Motor Class ---------------------------------------------------

class Stepper:
    def __init__(self, container, hexdrive_app, step_size=1,
                 steps_per_rev=_STEPPER_DEFAULT_SPR,
                 speed_sps=_STEPPER_DEFAULT_SPEED,
                 max_sps=_STEPPER_MAX_SPEED,
                 max_pos=_STEPPER_MAX_POSITION, timer_id=0):
        self._container = container
        self._hexdrive_app = hexdrive_app
        self._phase = 0
        self._calibrated = False
        self._timer = Timer(timer_id) if Timer is not None else None
        self._timer_is_running = False
        self._timer_mode = 0
        self._free_run_mode = 0
        self._enabled = False
        self._target_pos = 0
        self._pos = 0
        self._max_sps = int(max_sps)
        self._steps_per_sec = int(speed_sps)
        self._steps_per_rev = int(steps_per_rev)
        self._max_pos = 2 * int(max_pos)
        self._freq = 0
        self._min_period = 0
        self._step_size = int(step_size)
        self._last_step_time = 0
        self.track_target()

    def step_size(self, sz=1):
        if sz < 1:
            sz = 1
        elif sz > 2:
            sz = 2
        self._step_size = int(sz)

    def speed(self, sps):
        if self._free_run_mode == 1 and sps < 0:
            self._free_run_mode = -1
        elif self._free_run_mode == -1 and sps > 0:
            self._free_run_mode = 1
        if sps > self._max_sps:
            sps = self._max_sps
        elif sps < -self._max_sps:
            sps = -self._max_sps
        self._steps_per_sec = int(sps)
        self._update_timer((2 // self._step_size) * abs(self._steps_per_sec))

    def speed_rps(self, rps):
        self.speed(rps * self._steps_per_rev)

    def get_speed(self):
        return self._steps_per_sec

    def target(self, t):
        if self._calibrated and t < 0:
            self._target_pos = 0
        elif self._calibrated and (2 * int(t)) > self._max_pos:
            self._target_pos = self._max_pos
        else:
            self._target_pos = 2 * int(t)

    def target_deg(self, deg):
        self.target(self._steps_per_rev * deg / 360.0)

    def target_rad(self, rad):
        self.target(self._steps_per_rev * rad / (2 * pi))

    def get_pos(self):
        return (self._pos // 2)

    def get_pos_deg(self):
        return self._pos * 180.0 / self._steps_per_rev

    def get_pos_rad(self):
        return self._pos * pi / self._steps_per_rev

    def overwrite_pos(self, p=0):
        self._pos = 2 * int(p)

    def overwrite_pos_deg(self, deg):
        self._pos = deg * self._steps_per_rev / 180.0

    def overwrite_pos_rad(self, rad):
        self._pos = rad * self._steps_per_rev / pi

    def step(self, d=0):
        cur_time = time.ticks_ms()
        if time.ticks_diff(cur_time, self._last_step_time) < self._min_period:
            return
        self._last_step_time = cur_time
        if d > 0:
            self._pos += self._step_size
            self._phase = (self._phase - self._step_size) % _STEPPER_NUM_PHASES
        elif d < 0:
            self._pos -= self._step_size
            self._phase = (self._phase + self._step_size) % _STEPPER_NUM_PHASES
        if self._calibrated and self._pos < 0:
            print("s/w min endstop")
            self._pos = 0
            self.speed(0)
            return
        elif self._calibrated and self._pos > self._max_pos:
            print("s/w max endstop")
            self._pos = self._max_pos
            self.speed(0)
            return
        try:
            self._hexdrive_app.motor_step(self._phase)
        except Exception as e:
            print(f"step phase {self._phase} failed:{e}")

    def _hit_endstop(self):
        print("Endstop - hit")
        if not self._calibrated:
            self._calibrated = True
        self.overwrite_pos(0)
        if self._free_run_mode < 0:
            self.speed(0)
        elif self._free_run_mode == 0 and self._target_pos < self._pos:
            self.speed(0)

    def _timer_callback_fwd(self, t):
        self.step(1)

    def _timer_callback_rev(self, t):
        self.step(-1)

    def _timer_callback(self, t):
        if self._target_pos > self._pos:
            self.step(1)
        elif self._target_pos < self._pos:
            self.step(-1)

    def free_run(self, d=1):
        self._free_run_mode = d
        if d != 0:
            self._update_timer((2 // self._step_size) * abs(self._steps_per_sec))

    def track_target(self):
        self._free_run_mode = 0
        self._update_timer((2 // self._step_size) * abs(self._steps_per_sec))

    def _update_timer(self, freq):
        if self._timer is None:
            return
        if self._timer_is_running and freq != self._freq:
            try:
                self._timer.deinit()
                self._freq = 0
                self._timer_is_running = False
            except Exception as e:
                print(f"update_timer failed:{e}")
        if 0 != freq and (freq != self._freq or self._free_run_mode != self._timer_mode):
            try:
                print(f"Timer: {freq}Hz")
                if self._free_run_mode > 0:
                    self._timer.init(freq=freq, callback=self._timer_callback_fwd)
                elif self._free_run_mode < 0:
                    self._timer.init(freq=freq, callback=self._timer_callback_rev)
                else:
                    self._timer.init(freq=freq, callback=self._timer_callback)
                self._freq = freq
                self._min_period = (1000 // freq) - 1
                self._timer_is_running = True
                self._timer_mode = self._free_run_mode
            except Exception as e:
                print(f"update_timer failed:{e}")
        elif freq == 0:
            print("Timer: 0Hz")

    def stop(self):
        self._update_timer(0)
        try:
            self._hexdrive_app.motor_release()
        except Exception as e:
            print(f"stop failed:{e}")

    def enable(self, e=True):
        self._enabled = e
        try:
            if e:
                if self._free_run_mode != 0:
                    self._update_timer((2 // self._step_size) * abs(self._steps_per_sec))
                self._hexdrive_app.motor_step(self._phase)
            else:
                self._update_timer(0)
                self._hexdrive_app.motor_release()
            self._hexdrive_app.set_power(e)
        except Exception as e:
            print(f"enable failed:{e}")

    def is_enabled(self):
        return self._enabled


# ---- Stepper Tester Manager ------------------------------------------------

class StepperTestMgr:
    """Manages the Stepper Tester functionality.

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
        """Enter stepper test from the main menu."""
        app = self.app
        if app._stepper is None:
            for i in range(4):
                try:
                    app._stepper = Stepper(app, app.hexdrive_app, step_size=1,
                                           timer_id=i,
                                           max_pos=app._settings['step_max_pos'].v)
                    break
                except:
                    pass
        if app._stepper is None:
            app.notification = Notification("No Free Timers")
            return False
        app.set_menu(None)
        app.button_states.clear()
        from .linefollower import STATE_STEPPER
        app.current_state = STATE_STEPPER
        app._refresh = True
        app._auto_repeat_clear()
        app._stepper.enable(True)
        app._time_since_last_input = 0
        return True

    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta):
        """Handle STATE_STEPPER.  Returns True if this module handled the state."""
        app = self.app
        from .linefollower import STATE_STEPPER, STATE_MENU
        if app.current_state != STATE_STEPPER:
            return False

        if app.button_states.get(BUTTON_TYPES["RIGHT"]):
            if app._auto_repeat_check(delta, True):
                if app.stepper_mode == StepperMode.SPEED:
                    speed = app._stepper.get_speed()
                    speed = app._inc(speed, app._auto_repeat_level + 1)
                    if _STEPPER_MAX_SPEED < speed:
                        speed = _STEPPER_MAX_SPEED
                    app._stepper.speed(speed)
                else:
                    if app.stepper_mode != StepperMode.POSITION:
                        app.stepper_mode.set(StepperMode.POSITION)
                        app._stepper.speed(_STEPPER_DEFAULT_SPEED)
                        app._stepper.track_target()
                    pos = app._stepper.get_pos()
                    pos = app._inc(pos, app._auto_repeat_level + 1)
                    app._stepper.target(pos)
                app._refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            if app._auto_repeat_check(delta, True):
                if app.stepper_mode == StepperMode.SPEED:
                    speed = app._stepper.get_speed()
                    speed = app._dec(speed, app._auto_repeat_level + 1)
                    if -_STEPPER_MAX_SPEED > speed:
                        speed = -_STEPPER_MAX_SPEED
                    app._stepper.speed(speed)
                else:
                    if app.stepper_mode != StepperMode.POSITION:
                        app.stepper_mode.set(StepperMode.POSITION)
                        app._stepper.speed(_STEPPER_DEFAULT_SPEED)
                        app._stepper.track_target()
                    pos = app._stepper.get_pos()
                    pos = app._dec(pos, app._auto_repeat_level + 1)
                    app._stepper.target(pos)
                app._refresh = True
        else:
            app._auto_repeat_clear()
            if app.button_states.get(BUTTON_TYPES["CANCEL"]):
                app.button_states.clear()
                if app.hexdrive_app is not None:
                    app._stepper.enable(False)
                app.current_state = STATE_MENU
                return True
            elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
                app.button_states.clear()
                app.stepper_mode.inc()
                if app.stepper_mode == StepperMode.POSITION:
                    app._stepper.speed(_STEPPER_DEFAULT_SPEED)
                    app._stepper.target(app._stepper.get_pos())
                    app._stepper.track_target()
                elif app.stepper_mode == StepperMode.SPEED:
                    app._stepper.speed(0)
                    app._stepper.free_run(1)
                else:
                    app._stepper.stop()
                app._refresh = True
                app.notification = Notification(f"  Stepper:\n {app.stepper_mode}")
                print(f"Stepper:{app.stepper_mode}")
        if app._refresh:
            app._time_since_last_input = 0
        else:
            app._time_since_last_input += delta
            if app._time_since_last_input > app._timeout_period:
                app._stepper.stop()
                app._stepper.speed(0)
                app._stepper.enable(False)
                app.current_state = STATE_MENU
                app.notification = Notification("  Stepper:\n Timeout")
                print("Stepper:Timeout")
            elif app.stepper_mode == StepperMode.SPEED:
                app._refresh = True
        app._time_since_last_update += delta
        if app._time_since_last_update > app._keep_alive_period:
            app._stepper.step()
            app._time_since_last_update = 0
        return True

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx):
        """Render Stepper Tester UI.  Returns True if handled."""
        app = self.app
        from .linefollower import STATE_STEPPER
        if app.current_state != STATE_STEPPER:
            return False

        stepper_text = ["S"] * (1 + app.num_steppers)
        stepper_text_colours = [(0.4, 0.0, 0.0)] * (1 + app.num_steppers)
        stepper_text[0] = "Stepper Test"
        stepper_text_colours[0] = (1, 1, 1)
        if app._stepper is not None:
            i = 0
            if app.stepper_mode == StepperMode.OFF:
                body_colour = (0.2, 0.2, 0.2)
                bar_colour = (0.4, 0.4, 0.4)
            else:
                body_colour = (0.1, 0.1, 0.5)
                bar_colour = (0.1, 0.1, 1.0)
                stepper_text_colours[1] = (0.4, 0.4, 0.0)

            ctx.save()
            ctx.translate(0, (i - (app.num_steppers / 2) + 0.5) * label_font_size)
            background_colour = (0.15, 0.15, 0.15)
            ctx.rgb(*background_colour).rectangle(-100, 1, 200, label_font_size - 2).fill()
            c = 0
            x = 200 * (app._stepper.get_pos() / app._settings['step_max_pos'].v) - 100
            ctx.rgb(*bar_colour).rectangle(x - 2, 1, 5, label_font_size - 2).fill()
            ctx.rgb(*body_colour)
            if x > (c + 4):
                ctx.rectangle(c + 1, 3, x - c - 4, label_font_size - 6).fill()
            elif x < (c - 4):
                ctx.rectangle(x + 4, 3, c - x - 4, label_font_size - 6).fill()
            ctx.rgb(0, 0, 0).move_to(c, 0).line_to(c, label_font_size).stroke()
            ctx.restore()
            if app.stepper_mode == StepperMode.SPEED:
                stepper_text[i + 1] = f"{int(app._stepper.get_speed()):4}/s"
            else:
                stepper_text[i + 1] = "Off" if (app.stepper_mode == StepperMode.OFF) else f"{int(app._stepper.get_pos()):+6} "
        app.draw_message(ctx, stepper_text, stepper_text_colours, label_font_size)
        button_labels(ctx, confirm_label="Mode", cancel_label="Exit", left_label="<--", right_label="-->")
        return True
