# Motor Moves Module for BadgeBot
#
# Handles the "turtle/Logo" style motor-move programming.
# Internally manages its own sub-states (HELP, RECEIVE_INSTR, RUN, DONE).
#
# The countdown state (STATE_COUNTDOWN) is shared with PID AutoTune and
# remains in the main app.  When the countdown finishes it calls
# begin_moves() to kick off execution.
#
# Also contains the Instruction class and related helper functions.
#
# Public interface (called by the main app):
#   __init__(app)            – wire up to BadgeBotApp
#   start()                  – enter the motor-moves flow (from menu)
#   begin_moves()            – start executing (called after countdown)
#   update(delta)            – per-tick state machine update
#   draw(ctx)                – render motor-moves-related UI
#   background_update(delta) – called from the fast background loop;
#                              returns motor output tuple or None
#   reset_robot()            – reset sequence state and return to HELP
#   init_settings(settings)  – register motor-moves specific settings

import asyncio
from events.input import BUTTON_TYPES, Button
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification

from .utils import chain
from .app import (STATE_COUNTDOWN, STATE_MOTOR_MOVES, STATE_LOGO, DEFAULT_BACKGROUND_UPDATE_PERIOD, MOTOR_PWM_FREQ)

# Screen positioning for movement sequence text
H_START = -63
V_START = -58

# Timings
_TICK_MS       =  10
_LONG_PRESS_MS = 750

# Default user timings for drive and turn steps (can be configured in settings)
DEFAULT_POWER_STEP_PER_TICK = 7500
DEFAULT_MAX_POWER = 20000
DEFAULT_USER_DRIVE_MS =  50
DEFAULT_USER_TURN_MS  =  20

# Drive modes for TIME and DISTANCE (acceleration-based)
DRIVE_MODE_TIME = 0
DRIVE_MODE_DISTANCE = 1
DEFAULT_DRIVE_MODE  = DRIVE_MODE_DISTANCE


# Local sub-states (internal to Motor Moves)
_SUB_HELP          = 0
_SUB_RECEIVE_INSTR = 1
_SUB_RUN           = 2
_SUB_DONE          = 3


# ---- Instruction class -----------------------------------------------------

class Instruction:
    def __init__(self, press_type: Button) -> None:
        self._press_type = press_type
        self._duration = 1
        self.power_plan = []

    @property
    def press_type(self) -> Button:
        return self._press_type

    @property
    def duration(self) -> int:
        return self._duration

    def inc(self):
        self._duration += 1


    def __str__(self):
        return f"{self.press_type.name} {self._duration}"


    def directional_power_tuple(self, power) -> tuple[int, int]:
        if self._press_type == BUTTON_TYPES["UP"]:
            return (power, power)
        elif self._press_type == BUTTON_TYPES["DOWN"]:
            return (-power, -power)
        elif self._press_type == BUTTON_TYPES["LEFT"]:
            return (-power, power)
        elif self._press_type == BUTTON_TYPES["RIGHT"]:
            return (power, -power)

    def directional_duration(self, mysettings) -> int:
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


# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting: type):
    """Register motor-moves-specific settings in the shared settings dict."""
    s['acceleration']  = MySetting(s, DEFAULT_POWER_STEP_PER_TICK, 1, 65535)
    s['max_power']     = MySetting(s, DEFAULT_MAX_POWER, 1000, 65535)
    s['drive_step_ms'] = MySetting(s, DEFAULT_USER_DRIVE_MS, 5, 200)
    s['turn_step_ms']  = MySetting(s, DEFAULT_USER_TURN_MS, 5, 200)
    if 'drive_mode' not in s:
        s['drive_mode']    = MySetting(s, DEFAULT_DRIVE_MODE, DRIVE_MODE_TIME, DRIVE_MODE_DISTANCE)

# ---- Motor Moves manager ---------------------------------------------------

class MotorMovesMgr:
    """Manages the Motor Moves (turtle/Logo) programming workflow.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """

    def __init__(self, app, logging: bool = False):
        self._app = app
        self._logging: bool = logging
        self._sub_state = _SUB_HELP
        self._prev_state = _SUB_HELP
        # Motor-moves instance variables (previously on app)
        self.instructions = []
        self.current_instruction = None
        self.current_power_duration = ((0, 0, 0, 0), 0)
        self.power_plan_iter = iter([])
        self.long_press_delta = 0
        self._mc_task = None  # asyncio task for MotorController-based execution    
        if self._logging:
            print("MotorMovesMgr initialised")


    # ------------------------------------------------------------------

    @property
    def logging(self) -> bool:
        """Get or set logging state for this manager."""
        return self._logging
    
    @logging.setter
    def logging(self, value: bool):
        self._logging = value

    @property
    def drive_mode(self):
        return self._app.settings['drive_mode'].v if 'drive_mode' in self._app.settings else DRIVE_MODE_DISTANCE


    # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Enter the Motor Moves flow from the main menu."""
        app = self._app
        if self._logging:
            print("Entered Motor Moves mode")
        app.set_menu(None)
        app.button_states.clear()
        app.animation_counter = 0
        self._sub_state = _SUB_HELP
        app.refresh = True
        return True

    # ------------------------------------------------------------------
    # begin_moves – called after countdown to start execution
    # ------------------------------------------------------------------

    def begin_moves(self):
        """Build the power plan and start running (called after countdown).
        When a MotorController is available, uses it for IMU-aided execution."""
        app = self._app
        if app.motor_controller is not None and self.drive_mode == DRIVE_MODE_DISTANCE:
            # Use the MotorController for gyro-aided execution
            self._mc_task = asyncio.get_event_loop().create_task(
                self._run_instructions_async()
            )
        else:
            # Fallback: old power-plan iterator
            self.power_plan_iter = chain(*(instr.power_plan for instr in self.instructions))
            if app.hexdrive_app is not None:
                if app.hexdrive_app.initialise() and app.hexdrive_app.set_power(True) and app.hexdrive_app.set_freq(MOTOR_PWM_FREQ):
                    pass
                else:
                    if self._logging:
                        print("H:Failed to initialise HexDrive for motor moves")
                    app.notification = Notification("HexDrive Init Failed")
                    self._sub_state = _SUB_DONE
                    return
        self._sub_state = _SUB_RUN
        app.update_period = _TICK_MS
        app.refresh = True


    async def _run_instructions_async(self):
        """Execute the recorded instruction list via MotorController.
        Runs as an asyncio task spawned from begin_moves; the background_update
        loop monitors the task and transitions to _SUB_DONE when it completes."""
        try:
            await self._app.motor_controller.run_instructions(self.instructions)
        except asyncio.CancelledError:
            self._app.motor_controller.stop()
        except Exception as e:      # pylint: disable=broad-exception-caught
            print(f"MotorController run error: {e}")
            self._app.motor_controller.stop()


    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta: int) -> bool:
        """Handle Motor Moves states.  Returns True if this module handled the state."""

        if self._sub_state == _SUB_HELP:
            self._update_state_help(delta)
        elif self._sub_state == _SUB_RECEIVE_INSTR:
            self._update_state_receive_instr(delta)
        elif self._sub_state == _SUB_RUN:
            self._update_state_run(delta)
        elif self._sub_state == _SUB_DONE:
            self._update_state_done(delta)

        if self._sub_state != self._prev_state:
            if self._logging:
                print(f"M:State: {self._prev_state} -> {self._sub_state}")
            self._prev_state = self._sub_state

        return True


    # ------------------------------------------------------------------
    # Background update (called from the fast loop)
    # Returns motor output tuple, or None when sequence is done.
    # ------------------------------------------------------------------

    def background_update(self, delta: int) -> tuple[int, int] | None:
        """DC Motor control during RUN sub-state.  Returns output tuple or None."""
        if self._mc_task is not None:
            # MotorController handles motors via its own async task;
            # we just check whether it has finished.
            if self._mc_task.done():
                self._sub_state = _SUB_DONE
                self._app.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
                self._mc_task = None
            return None  # MotorController manages motors directly
        else:
            # Legacy power-plan path
            output = self._get_current_power_level(delta)
            if output is None and self._sub_state == _SUB_RUN:
                self._sub_state = _SUB_DONE
                self._app.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
            return output


    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _update_state_help(self, delta: int) -> None:
        app = self._app
        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            app.return_to_menu()
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            app.scroll(False)            
            app.scroll_mode_enable(True)
            self._sub_state = _SUB_RECEIVE_INSTR
        else:
            app.animation_counter += delta
            if app.animation_counter > 10000:
                app.animation_counter = 0
                app.current_state = STATE_LOGO


    def _update_state_receive_instr(self, delta: int) -> None:       
        app = self._app

        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.long_press_delta += delta
            if app.long_press_delta >= _LONG_PRESS_MS:
                if self.power_plan_iter is None:
                    self._sub_state = _SUB_HELP
                else:
                    self.finalize_instruction()
                    app.countdown_next_state = STATE_MOTOR_MOVES
                    app.run_countdown_elapsed_ms = 0
                    app.current_state = STATE_COUNTDOWN
                app.scroll_mode_enable(False)
                app.long_press_delta = 0
        else:
            app.long_press_delta = 0
            if app.button_states.get(BUTTON_TYPES["CANCEL"]):
                app.button_states.clear()
                app.animation_counter = 0
                app.scroll_mode_enable(False)
                self._sub_state = _SUB_HELP
                return
            elif app.button_states.get(BUTTON_TYPES["RIGHT"]):
                self._handle_instruction_press(BUTTON_TYPES["RIGHT"])
                app.button_states.clear()
                self.set_direction_leds(BUTTON_TYPES["RIGHT"])
                app.refresh = True
            elif app.button_states.get(BUTTON_TYPES["LEFT"]):
                self._handle_instruction_press(BUTTON_TYPES["LEFT"])
                app.button_states.clear()
                self.set_direction_leds(BUTTON_TYPES["LEFT"])
                app.refresh = True
            elif app.button_states.get(BUTTON_TYPES["UP"]):
                self._handle_instruction_press(BUTTON_TYPES["UP"])
                app.button_states.clear()
                self.set_direction_leds(BUTTON_TYPES["UP"])
                app.refresh = True
            elif app.button_states.get(BUTTON_TYPES["DOWN"]):
                self._handle_instruction_press(BUTTON_TYPES["DOWN"])
                app.button_states.clear()
                self.set_direction_leds(BUTTON_TYPES["DOWN"])
                app.refresh = True
            else:
                self.set_direction_leds(app.last_press)


    def _update_state_run(self, delta: int) -> None:        # pylint: disable=unused-argument
        app = self._app
        app.clear_leds()
        # Run is primarily managed in the background update - but we allow CANCEL here as well to stop immediately
        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if app.hexdrive_app is not None:
                app.hexdrive_app.set_power(False)
            self.reset_robot()


    def _update_state_done(self, delta: int) -> None:        # pylint: disable=unused-argument
        app = self._app
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
            self.current_power_duration = ((0, 0, 0, 0), 0)
            app.countdown_next_state = STATE_MOTOR_MOVES
            app.current_state = STATE_COUNTDOWN


    # ------------------------------------------------------------------
    # Instruction helpers
    # ------------------------------------------------------------------

    def _handle_instruction_press(self, press_type):
        app = self._app
        if app.last_press == press_type:
            self.current_instruction.inc()
        else:
            self.finalize_instruction()
            self.current_instruction = Instruction(press_type)
        app.last_press = press_type


    def finalize_instruction(self):
        """Finalize the current instruction (if any) and add it to the list."""
        app = self._app
        if self.current_instruction is not None:
            self.current_instruction.make_power_plan(app.settings)
            self.instructions.append(self.current_instruction)
            if len(self.instructions) >= 5:
                app.scroll_offset -= 1
            self.current_instruction = None


    def reset_robot(self):
        """Reset sequence state and return to HELP."""
        app = self._app
        if self._mc_task is not None:
            self._mc_task.cancel()
            self._mc_task = None
        if app.motor_controller is not None:
            app.motor_controller.stop()
        self._sub_state = _SUB_HELP
        app.last_press = BUTTON_TYPES["CONFIRM"]
        app.animation_counter = 0
        app.long_press_delta = 0
        app.scroll(False)
        app.run_countdown_elapsed_ms = 0
        self.instructions = []
        self.current_instruction = None
        self.current_power_duration = ((0, 0), 0)
        self.power_plan_iter = iter([])


    def set_direction_leds(self, direction: Button):
        """Utility function to set the LEDs to indicate a direction (up, down, left, right) based on the button input.
        Delegates to the app's front_face-aware LED method so LEDs rotate with the configured forward direction."""
        self._app.set_direction_leds(direction)


    def _get_current_power_level(self, delta: int) -> tuple[int, int] | None:
        if delta >= _TICK_MS:
            delta = _TICK_MS - 1
        current_power, current_duration = self.current_power_duration
        updated_duration = current_duration - delta
        if updated_duration <= 0:
            try:
                next_power, next_duration = next(self.power_plan_iter)
            except StopIteration:
                return None
            next_duration += updated_duration
            self.current_power_duration = next_power, next_duration
            return next_power
        else:
            self.current_power_duration = current_power, updated_duration
            return current_power


    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx) -> bool:
        """Render Motor Moves UI states.  Returns True if handled."""
        app = self._app
        if self._sub_state == _SUB_HELP:
            app.draw_message(ctx, ["BadgeBot", "To program:", "Press C", "When finished:", "Long press C"], [(1, 1, 0), (1, 1, 1), (1, 1, 1), (1, 1, 1), (1, 1, 1)], label_font_size)
        elif self._sub_state == _SUB_RECEIVE_INSTR:
            self._draw_receive_instr(ctx)
        elif self._sub_state == _SUB_RUN:
            current_power, _ = self.current_power_duration
            power_str = str(tuple([int(x / (app.settings['max_power'].v // 100)) for x in current_power]))
            app.draw_message(ctx, ["Running...", power_str], [(1, 1, 1), (1, 1, 0)], label_font_size)
        elif self._sub_state == _SUB_DONE:
            app.draw_message(ctx, ["Program", "complete!"], [(0, 1, 0), (0, 1, 0)], label_font_size)
            button_labels(ctx, confirm_label="Replay", cancel_label="Restart")
        return True


    def _draw_receive_instr(self, ctx):
        app = self._app
        for i_num, instr in enumerate(["START"] + self.instructions + [self.current_instruction, "END"]):
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
