# Stepper Tester Module for BadgeBot
#
# Handles the stepper motor tester functionality.
# Contains the Stepper motor driver class and the StepperMode helper.
#
# Public interface (called by the main app):
#   __init__(app)   – wire up to BadgeBotApp
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

from .utils import inc_value, dec_value


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
        self._mode = mode

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        self._mode = mode

    def inc(self):
        self._mode = (self._mode + 1) % 3

    def __eq__(self, other):
        return self._mode == other

    def __str__(self):
        return self.stepper_modes[self._mode]


# ---- Stepper Motor Class ---------------------------------------------------

class Stepper:
    def __init__(self, container, hexdrive_app, logging: bool = False, 
                 step_size=1,
                 steps_per_rev=_STEPPER_DEFAULT_SPR,
                 speed_sps=_STEPPER_DEFAULT_SPEED,
                 max_sps=_STEPPER_MAX_SPEED,
                 max_pos=_STEPPER_MAX_POSITION, timer_id=0):
        self._container = container
        self._hexdrive_app = hexdrive_app
        self._logging: bool = logging
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
        if self._logging:
            print("Stepper initialised")
    

    @property
    def max_pos(self) -> int:
        return self._max_pos


    @max_pos.setter
    def max_pos(self, mp: int):
        """ Set the maximum position for the stepper (in full steps)."""
        self._max_pos = 2 * int(mp)


    @property
    def step_size(self) -> int:
        return self._step_size
    

    @step_size.setter
    def step_size(self, sz: int):
        """ Set the step size (microstepping level) for the stepper.  Valid values are 1 (full step) or 2 (half step)."""
        if sz < 1:
            sz = 1
        elif sz > 2:
            sz = 2
        self._step_size = int(sz)


    @property
    def speed(self) -> int:
        return self._steps_per_sec


    @speed.setter
    def speed(self, sps: int):
        if self._free_run_mode == 1 and sps < 0:
            self._free_run_mode = -1
        elif self._free_run_mode == -1 and sps > 0:
            self._free_run_mode = 1
        if sps > self._max_sps:
            sps = self._max_sps
        elif sps < -self._max_sps:
            sps = -self._max_sps
        self._steps_per_sec = sps
        self._update()


    @property
    def speed_rps(self) -> float:
        return self._steps_per_sec / self._steps_per_rev
    

    @speed_rps.setter
    def speed_rps(self, rps: float):
        self.speed = int(rps * self._steps_per_rev)


    @property
    def target(self) -> int:
        return self._target_pos // 2


    @target.setter
    def target(self, t: int):
        if self._calibrated and t < 0:
            self._target_pos = 0
        elif self._calibrated and (2 * int(t)) > self._max_pos:
            self._target_pos = self._max_pos
        else:
            self._target_pos = 2 * int(t)
        self._update()


    @property
    def target_deg(self) -> float:
        return self._target_pos * 180.0 / self._steps_per_rev


    @target_deg.setter
    def target_deg(self, deg: float):
        self.target = int(self._steps_per_rev * deg / 360.0)


    @property
    def target_rad(self) -> float:
        return self._target_pos * pi / self._steps_per_rev


    @target_rad.setter
    def target_rad(self, rad: float):
        self.target = int(self._steps_per_rev * rad / (2 * pi))


    @property
    def pos(self) -> int:
        """ Get the current position of the stepper in full steps.  Note that the internal position is tracked in half-steps to allow for microstepping, so the returned value is the internal position divided by 2."""
        return (self._pos // 2)


    @pos.setter
    def pos(self, p: int):
        self._pos = 2 * int(p)
        self._update()


    @property
    def pos_deg(self) -> float:
        return self._pos * 180.0 / self._steps_per_rev


    @property
    def pos_rad(self) -> float:
        return self._pos * pi / self._steps_per_rev


    def step(self, d: int = 0):
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
            self.speed = 0
            return
        elif self._calibrated and self._pos > self._max_pos:
            print("s/w max endstop")
            self._pos = self._max_pos
            self.speed = 0
            return
        try:
            self._hexdrive_app.motor_step(self._phase)
        except Exception as e:      # pylint: disable=broad-except
            print(f"step phase {self._phase} failed:{e}")


    def _hit_endstop(self):
        print("Endstop - hit")
        if not self._calibrated:
            self._calibrated = True
        self.pos = 0
        if self._free_run_mode < 0:
            self.speed = 0
        elif self._free_run_mode == 0 and self._target_pos < self._pos:
            self.speed = 0


    def _timer_callback_fwd(self, t):   # pylint: disable=unused-argument
        self.step(1)


    def _timer_callback_rev(self, t):   # pylint: disable=unused-argument
        self.step(-1)


    def _timer_callback(self, t):       # pylint: disable=unused-argument
        if self._target_pos > self._pos:
            self.step(1)
        elif self._target_pos < self._pos:
            self.step(-1)


    def free_run(self, d=1):
        """Run the stepper at the current speed in the given direction, ignoring position feedback."""
        self._free_run_mode = d
        self._update()


    def track_target(self):
        """Run the stepper, using position feedback to maintain the target position."""
        self._free_run_mode = 0
        self._update()


    def _update(self):
        if (self._free_run_mode != 0) or (self._target_pos != self._pos):
            self._update_timer((2 // self._step_size) * abs(self._steps_per_sec))
        else:
            self._update_timer(0)


    def _update_timer(self, freq: int):
        if self._timer is None:
            return
        if self._timer_is_running and freq != self._freq:
            try:
                self._timer.deinit()
                self._freq = 0
                self._timer_is_running = False
            except Exception as e:    # pylint: disable=broad-except
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
            except Exception as e:  # pylint: disable=broad-except
                print(f"update_timer failed:{e}")


    def stop(self):
        '''Stop the motor and disable the coils.'''
        self._update_timer(0)
        try:
            self._hexdrive_app.motor_release()
        except Exception as e:      # pylint: disable=broad-except
            print(f"stop failed:{e}")


    @property
    def enable(self) -> bool:
        '''Return True if the stepper is currently enabled (i.e. not in a power-saving state).'''
        return self._enabled


    @enable.setter
    def enable(self, e: bool = True):
        '''Enable or disable the stepper motor coils.'''
        self._enabled = e
        try:
            if e:
                if self._free_run_mode != 0:
                    self._update_timer((2 // self._step_size) * abs(self._steps_per_sec))
                self._hexdrive_app.motor_step(self._phase)
            else:
                self._update_timer(0)
                self._hexdrive_app.motor_release()
        except Exception as ex:      # pylint: disable=broad-except
            print(f"enable failed {ex}")




# ---- Stepper Tester Manager ------------------------------------------------

class StepperTestMgr:
    """Manages the Stepper Tester functionality.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """

    def __init__(self, app, logging: bool = False):
        self._app = app
        self._logging: bool = logging
        self.stepper = None
        self.stepper_mode = StepperMode()
        self.timeout_period: int = 120000                     # ms (2 minutes - without any user input)       
        self.keep_alive_period: int = 500                     # ms (half the value used in hexdrive.py)
        self._time_since_last_input: int = 0
        self._time_since_last_update: int = 0
        if self._logging:
            print("StepperTestMgr initialised")


    # ------------------------------------------------------------------

    @property
    def logging(self) -> bool:
        """Whether to print debug logs to the console."""
        return self._logging
    
    @logging.setter
    def logging(self, value: bool):
        self._logging = value

            
    # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Enter stepper test from the main menu."""
        app = self._app
        if self.stepper is None:
            # Find the first available timer for the stepper - we need a timer to do the stepping in the background while we wait for user input.
            for i in range(4):
                try:
                    self.stepper = Stepper(app,
                                           app.hexdrive_apps[0] if len(app.hexdrive_apps) > 0 else None,
                                           step_size=1,
                                           timer_id=i,
                                           max_pos=self.step_max_pos)
                    break
                except Exception:   # pylint: disable=broad-except
                    pass
        if self.stepper is None:
            app.notification = Notification("No Free Timers")
            return False
        if len(app.hexdrive_apps) > 0:
            if app.hexdrive_apps[0].initialise() and app.hexdrive_apps[0].set_power(True):   
                app.set_menu(None)
                app.button_states.clear()
                app.refresh = True
                app.auto_repeat_clear()
                self.stepper.enable = True
                self._time_since_last_input = 0
            
                if self._logging:
                    print("Entered Stepper Test mode")
                return True
        return False


    @property
    def step_max_pos(self) -> int:
        """Get the maximum position for the stepper from settings."""
        return self._app.settings['step_max_pos'].v if 'step_max_pos' in self._app.settings else _STEPPER_MAX_POSITION


    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def _mode_position(self):
        if self.stepper_mode != StepperMode.POSITION:
            self.stepper_mode.mode = StepperMode.POSITION
            self.stepper.speed = _STEPPER_DEFAULT_SPEED
            self.stepper.enable = True
            self.stepper.track_target()


    def update(self, delta) -> bool:
        """Handle Stepper UI updates.  Returns True if this module handled the state."""
        app = self._app
        if app.button_states.get(BUTTON_TYPES["RIGHT"]):
            if app.auto_repeat_check(delta, True):
                if self.stepper_mode == StepperMode.SPEED:
                    speed = inc_value(self.stepper.speed, app.auto_repeat_level + 1)
                    if _STEPPER_MAX_SPEED < speed:
                        speed = _STEPPER_MAX_SPEED
                    self.stepper.speed = speed
                else:   # Off or already in Position mode - increase target position
                    self._mode_position()
                    self.stepper.target = inc_value(self.stepper.pos, app.auto_repeat_level + 1)
                app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            if app.auto_repeat_check(delta, True):
                if self.stepper_mode == StepperMode.SPEED:
                    speed = dec_value(self.stepper.speed, app.auto_repeat_level + 1)
                    if -_STEPPER_MAX_SPEED > speed:
                        speed = -_STEPPER_MAX_SPEED
                    self.stepper.speed = speed
                else:   # Off or already in Position mode - decrease target position
                    self._mode_position()
                    self.stepper.target = dec_value(self.stepper.pos, app.auto_repeat_level + 1)
                app.refresh = True
        else:
            app.auto_repeat_clear()
            if app.button_states.get(BUTTON_TYPES["CANCEL"]):
                app.button_states.clear()
                self.stepper.enable = False
                if self._logging:
                    print("Stepper:Back to menu")
                app.return_to_menu()
                return True
            elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):    #"Mode" button
                app.button_states.clear()
                self.stepper_mode.inc()
                if self.stepper_mode == StepperMode.POSITION:
                    self.stepper.speed = 0
                    self.stepper.target = self.stepper.pos
                    self.stepper.speed = _STEPPER_DEFAULT_SPEED
                    self.stepper.enable = True
                    self.stepper.track_target()
                elif self.stepper_mode == StepperMode.SPEED:
                    self.stepper.speed = 0
                    self.stepper.enable = True
                    self.stepper.free_run(1)
                else:
                    # "Off" mode - stop the stepper and disable coils to save power
                    self.stepper.stop()
                    self.stepper.enable = False
                    self.stepper.speed = 0
                app.refresh = True
                app.notification = Notification(f"  Stepper:\n {self.stepper_mode}")
                print(f"Stepper:{self.stepper_mode}")
        if app.refresh:
            self._time_since_last_input = 0
        else:
            self._time_since_last_input += delta
            if self._time_since_last_input > self.timeout_period:
                self.stepper.stop()
                self.stepper.speed = 0
                self.stepper.enable = False
                app.return_to_menu()
                app.notification = Notification("  Stepper:\n Timeout")
                print("Stepper:Timeout")
            elif self.stepper_mode == StepperMode.SPEED:
                app.refresh = True
        if self.stepper_mode != StepperMode.OFF:
            # Keep Alive for HexDrive if we are stationary.
            self._time_since_last_update += delta
            if self._time_since_last_update > self.keep_alive_period:
                self.stepper.step()
                self._time_since_last_update = 0
        return True


    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx) -> bool:
        """Render Stepper Tester UI.  Returns True if handled."""
        app = self._app

        stepper_text = ["S"] * (1 + app.num_steppers)
        stepper_text_colours = [(0.4, 0.0, 0.0)] * (1 + app.num_steppers)
        stepper_text[0] = "Stepper Test"
        stepper_text_colours[0] = (1, 1, 0)
        if self.stepper is not None:
            i = 0
            if self.stepper_mode == StepperMode.OFF:
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
            x = 200 * (self.stepper.pos / self.stepper.max_pos) - 100
            ctx.rgb(*bar_colour).rectangle(x - 2, 1, 5, label_font_size - 2).fill()
            ctx.rgb(*body_colour)
            if x > (c + 4):
                ctx.rectangle(c + 1, 3, x - c - 4, label_font_size - 6).fill()
            elif x < (c - 4):
                ctx.rectangle(x + 4, 3, c - x - 4, label_font_size - 6).fill()
            ctx.rgb(0, 0, 0).move_to(c, 0).line_to(c, label_font_size).stroke()
            ctx.restore()
            if self.stepper_mode == StepperMode.SPEED:
                stepper_text[i + 1] = f"{int(self.stepper.speed):4}/s"
            else:
                stepper_text[i + 1] = "Off" if (self.stepper_mode == StepperMode.OFF) else f"{int(self.stepper.pos):+6} "
        app.draw_message(ctx, stepper_text, stepper_text_colours, label_font_size)
        button_labels(ctx, confirm_label="Mode", cancel_label="Back", left_label="<--", right_label="-->")
        return True
