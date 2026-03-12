# Motor Moves Module for BadgeBot
#
# Handles the "turtle/Logo" style motor-move programming:
#   STATE_HELP          – instruction help screen
#   STATE_RECEIVE_INSTR – capture directional button presses as instructions
#   STATE_COUNTDOWN     – countdown before executing the movement sequence
#   STATE_RUN           – executing the movement sequence
#   STATE_DONE          – sequence complete, offer replay/restart
#
# Also contains the Instruction class and related helper functions.
#
# The countdown state is *shared* with PID AutoTune (and potentially other
# features in the future).  The main app stores which state to transition
# to after the countdown in  app._countdown_next_state.
#
# Public interface (called by the main app):
#   __init__(app)   – wire up to LineFollowerApp
#   update(delta)   – per-tick state machine update
#   draw(ctx)       – render motor-moves-related UI
#   background_update(delta) – called from the fast background loop
#   start()         – enter the motor-moves flow (from menu)
#   reset_robot()   – reset sequence state and return to HELP

from events.input import BUTTON_TYPES, Button, ButtonUpEvent
from system.eventbus import eventbus
from app_components.tokens import label_font_size, twentyfour_pt, button_labels
from app_components.notification import Notification


# Screen positioning for movement sequence text
H_START = -63
V_START = -58

# Timings
_TICK_MS       =  10
_USER_DRIVE_MS =  50
_USER_TURN_MS  =  20
_LONG_PRESS_MS = 750
_RUN_COUNTDOWN_MS = 5000
_POWER_STEP_PER_TICK = 7500
_MAX_POWER = 30000


# ---- Instruction class (moved from linefollower.py) -----------------------

class Instruction:
    def __init__(self, press_type: Button) -> None:
        self._press_type = press_type
        self._duration = 1
        self.power_plan = []

    @property
    def press_type(self) -> Button:
        return self._press_type

    def inc(self):
        self._duration += 1

    def __str__(self):
        return f"{self.press_type.name} {self._duration}"

    def directional_power_tuple(self, power):
        if self._press_type == BUTTON_TYPES["UP"]:
            return (power, power)
        elif self._press_type == BUTTON_TYPES["DOWN"]:
            return (-power, -power)
        elif self._press_type == BUTTON_TYPES["LEFT"]:
            return (-power, power)
        elif self._press_type == BUTTON_TYPES["RIGHT"]:
            return (power, -power)

    def directional_duration(self, mysettings):
        if self._press_type == BUTTON_TYPES["UP"] or self._press_type == BUTTON_TYPES["DOWN"]:
            return (mysettings['drive_step_ms'].v)
        elif self._press_type == BUTTON_TYPES["LEFT"] or self._press_type == BUTTON_TYPES["RIGHT"]:
            return (mysettings['turn_step_ms'].v)

    def make_power_plan(self, mysettings):
        curr_power = 0
        ramp_up = []
        for i in range(1 * (self._duration + 3)):
            ramp_up.append((self.directional_power_tuple(curr_power), _TICK_MS))
            curr_power += mysettings['acceleration'].v
            if curr_power >= mysettings['max_power'].v:
                ramp_up.append((self.directional_power_tuple(mysettings['max_power'].v), _TICK_MS))
                break
        user_power_duration = (self.directional_duration(mysettings) * self._duration) - (2 * (i + 1) * _TICK_MS)
        power_durations = ramp_up.copy()
        if user_power_duration > 0:
            power_durations.append((self.directional_power_tuple(mysettings['max_power'].v), user_power_duration))
        ramp_down = ramp_up.copy()
        ramp_down.reverse()
        power_durations.extend(ramp_down)
        if mysettings['logging'].v:
            print("Power durations:")
            print(power_durations)
        self.power_plan = power_durations


# ---- Motor Moves manager ---------------------------------------------------

class MotorMovesMgr:
    """Manages the Motor Moves (turtle/Logo) programming workflow.

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
        """Enter the Motor Moves flow from the main menu."""
        app = self.app
        app.set_menu(None)
        app.button_states.clear()
        app._animation_counter = 0
        from .linefollower import STATE_HELP
        app.current_state = STATE_HELP
        app._refresh = True

    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta):
        """Handle Motor Moves states.  Returns True if this module handled the state."""
        app = self.app
        from .linefollower import (STATE_HELP, STATE_RECEIVE_INSTR,
            STATE_COUNTDOWN, STATE_RUN, STATE_DONE, STATE_MENU,
            DEFAULT_UPDATE_PERIOD)

        if app.current_state == STATE_HELP:
            self._update_state_help(delta)
            return True
        elif app.current_state == STATE_RECEIVE_INSTR:
            self._update_state_receive_instr(delta)
            return True
        elif app.current_state == STATE_RUN:
            app.clear_leds()
            # Run is primarily managed in the background update
            return True
        elif app.current_state == STATE_DONE:
            self._update_state_done(delta)
            return True
        return False

    # ------------------------------------------------------------------
    # Background update (called from the fast loop)
    # ------------------------------------------------------------------

    def background_update(self, delta):
        """DC Motor control during STATE_RUN."""
        app = self.app
        from .linefollower import STATE_RUN, STATE_DONE, DEFAULT_UPDATE_PERIOD
        if app.current_state == STATE_RUN:
            output = self.get_current_power_level(delta)
            if output is None:
                app.current_state = STATE_DONE
                app._update_period = DEFAULT_UPDATE_PERIOD
            elif app.hexdrive_app is not None:
                app.hexdrive_app.set_motors(output)

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _update_state_help(self, delta):
        app = self.app
        from .linefollower import STATE_MENU, STATE_RECEIVE_INSTR, STATE_LOGO
        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            app.current_state = STATE_MENU
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            app.is_scroll = True
            eventbus.on_async(ButtonUpEvent, app._handle_button_up, app)
            app.current_state = STATE_RECEIVE_INSTR
        else:
            app._animation_counter += delta / 1000
            if app._animation_counter > 10:
                app._animation_counter = 0
                app.current_state = STATE_LOGO

    def _update_state_receive_instr(self, delta):
        app = self.app
        from .linefollower import (STATE_HELP, STATE_COUNTDOWN, STATE_RUN,
            _LONG_PRESS_MS)
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.long_press_delta += delta
            if app.long_press_delta >= _LONG_PRESS_MS:
                if app.power_plan_iter is None:
                    app.current_state = STATE_HELP
                else:
                    self.finalize_instruction()
                    app._countdown_next_state = STATE_RUN
                    app.run_countdown_elapsed_ms = 0
                    app.current_state = STATE_COUNTDOWN
                app.is_scroll = False
                eventbus.remove(ButtonUpEvent, app._handle_button_up, app)
        else:
            app.long_press_delta = 0
            if app.button_states.get(BUTTON_TYPES["CANCEL"]):
                app.button_states.clear()
                app._animation_counter = 0
                app.is_scroll = False
                app.current_state = STATE_HELP
                eventbus.remove(ButtonUpEvent, app._handle_button_up, app)
                return
            if app.is_scroll:
                if app.button_states.get(BUTTON_TYPES["DOWN"]):
                    app.button_states.clear()
                    app.scroll_offset -= 1
                    app._refresh = True
                elif app.button_states.get(BUTTON_TYPES["UP"]):
                    app.button_states.clear()
                    app.scroll_offset += 1
                    app._refresh = True
            elif app.button_states.get(BUTTON_TYPES["RIGHT"]):
                self._handle_instruction_press(BUTTON_TYPES["RIGHT"])
                app.button_states.clear()
                app._set_direction_leds(BUTTON_TYPES["RIGHT"])
                app._refresh = True
            elif app.button_states.get(BUTTON_TYPES["LEFT"]):
                self._handle_instruction_press(BUTTON_TYPES["LEFT"])
                app.button_states.clear()
                app._set_direction_leds(BUTTON_TYPES["LEFT"])
                app._refresh = True
            elif app.button_states.get(BUTTON_TYPES["UP"]):
                self._handle_instruction_press(BUTTON_TYPES["UP"])
                app.button_states.clear()
                app._set_direction_leds(BUTTON_TYPES["UP"])
                app._refresh = True
            elif app.button_states.get(BUTTON_TYPES["DOWN"]):
                self._handle_instruction_press(BUTTON_TYPES["DOWN"])
                app.button_states.clear()
                app._set_direction_leds(BUTTON_TYPES["DOWN"])
                app._refresh = True
            else:
                app._set_direction_leds(app.last_press)

    def _update_state_done(self, delta):
        app = self.app
        from .linefollower import (STATE_HELP, STATE_COUNTDOWN, STATE_RUN)
        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if app.hexdrive_app is not None:
                app.hexdrive_app.set_power(False)
            self.reset_robot()
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            if app.hexdrive_app is not None:
                app.hexdrive_app.set_power(False)
            app.run_countdown_elapsed_ms = 1
            app.current_power_duration = ((0, 0, 0, 0), 0)
            app._countdown_next_state = STATE_RUN
            app.current_state = STATE_COUNTDOWN

    # ------------------------------------------------------------------
    # Instruction helpers
    # ------------------------------------------------------------------

    def _handle_instruction_press(self, press_type):
        app = self.app
        if app.last_press == press_type:
            app.current_instruction.inc()
        else:
            self.finalize_instruction()
            app.current_instruction = Instruction(press_type)
        app.last_press = press_type

    def finalize_instruction(self):
        app = self.app
        if app.current_instruction is not None:
            app.current_instruction.make_power_plan(app._settings)
            app.instructions.append(app.current_instruction)
            if len(app.instructions) >= 5:
                app.scroll_offset -= 1
            app.current_instruction = None

    def reset_robot(self):
        app = self.app
        from .linefollower import STATE_HELP
        app.current_state = STATE_HELP
        app.last_press = BUTTON_TYPES["CONFIRM"]
        app._animation_counter = 0
        app.long_press_delta = 0
        app.is_scroll = False
        app.scroll_offset = 0
        app.run_countdown_elapsed_ms = 0
        app.instructions = []
        app.current_instruction = None
        app.current_power_duration = ((0, 0), 0)
        app.power_plan_iter = iter([])

    def get_current_power_level(self, delta):
        app = self.app
        if delta >= _TICK_MS:
            delta = _TICK_MS - 1
        current_power, current_duration = app.current_power_duration
        updated_duration = current_duration - delta
        if updated_duration <= 0:
            try:
                next_power, next_duration = next(app.power_plan_iter)
            except StopIteration:
                return None
            next_duration += updated_duration
            app.current_power_duration = next_power, next_duration
            return next_power
        else:
            app.current_power_duration = current_power, updated_duration
            return current_power

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx):
        """Render Motor Moves UI states.  Returns True if handled."""
        app = self.app
        from .linefollower import (STATE_HELP, STATE_RECEIVE_INSTR,
            STATE_COUNTDOWN, STATE_RUN, STATE_DONE)

        if app.current_state == STATE_HELP:
            app.draw_message(ctx, ["BadgeBot", "To program:", "Press C", "When finished:", "Long press C"], [(1, 1, 0), (1, 1, 1), (1, 1, 1), (1, 1, 1), (1, 1, 1)], label_font_size)
            return True
        elif app.current_state == STATE_RECEIVE_INSTR:
            self._draw_receive_instr(ctx)
            return True
        elif app.current_state == STATE_RUN:
            current_power, _ = app.current_power_duration
            power_str = str(tuple([int(x / (app._settings['max_power'].v // 100)) for x in current_power]))
            app.draw_message(ctx, ["Running...", power_str], [(1, 1, 1), (1, 1, 0)], label_font_size)
            return True
        elif app.current_state == STATE_DONE:
            app.draw_message(ctx, ["Program", "complete!"], [(0, 1, 0), (0, 1, 0)], label_font_size)
            button_labels(ctx, confirm_label="Replay", cancel_label="Restart")
            return True
        return False

    def _draw_receive_instr(self, ctx):
        app = self.app
        for i_num, instr in enumerate(["START"] + app.instructions + [app.current_instruction, "END"]):
            colour = (1, 1, 1)
            if instr is not None:
                direction = str(instr).split()[0]
                if direction == "UP":
                    instr = "FWD " + str(instr).split()[1]
                    colour = (0, 1, 1)
                elif direction == "DOWN":
                    instr = "REV " + str(instr).split()[1]
                    colour = (1, 0, 1)
                elif direction == "LEFT":
                    colour = (1, 0, 0)
                elif direction == "RIGHT":
                    colour = (0, 1, 0)
                elif direction == "START" or direction == "END":
                    colour = (0.5, 0.5, 0.5)
            ctx.rgb(*colour).move_to(H_START, V_START + label_font_size * (app.scroll_offset + i_num)).text(str(instr))
