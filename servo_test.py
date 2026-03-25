# Servo Tester Module for BadgeBot
#
# Handles the servo tester functionality.
# Allows the user to control up to 4 servos with position, trim,
# and scanning modes.
#
# Public interface (called by the main app):
#   __init__(app)   – wire up to BadgeBotApp
#   start()         – enter servo test from menu
#   update(delta)   – per-tick state machine update
#   draw(ctx)       – render servo tester UI

import gc
import time
from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification
try:
    from machine import Pin, disable_irq, enable_irq
except ImportError:
    # Simulator fallback
    class Pin:
        OUT = 1
from system.hexpansion.config import HexpansionConfig
from .utils import inc_value, dec_value

# Servo Tester - Defaults
_SERVO_DEFAULT_STEP    = 10         # Default step size for position and trim adjustments in microseconds
_SERVO_DEFAULT_CENTRE  = 1500       # Default servo centre position in microseconds (e.g. 1500 for typical hobby servos)
_SERVO_DEFAULT_RANGE   = 1000       # Default servo range in microseconds
_SERVO_DEFAULT_RATE    = 250        # Default scanning rate in microseconds per second
_SERVO_DEFAULT_PERIOD  = 20         # Default servo period in milliseconds (e.g. 20ms for typical hobby servos)
_SERVO_MAX_RATE        = 10000      # Maximum scanning rate in us/s
_SERVO_MIN_RATE        = 10         # Minimum scanning rate in us/s
_SERVO_MAX_TRIM        = 1000       # Maximum trim in microseconds
_MAX_SERVO_RANGE       = 1400
_LS_SERVO_PIN_INDEXES  = (2, 3, 4)  # HexDrive LS0/LS1 are reserved


# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting):
    """Register servo-test-specific settings in the shared settings dict."""
    s['servo_step']    = MySetting(s, _SERVO_DEFAULT_STEP, 1, 100)
    s['servo_range']   = MySetting(s, _SERVO_DEFAULT_RANGE, 100, _MAX_SERVO_RANGE)
    s['servo_period']  = MySetting(s, _SERVO_DEFAULT_PERIOD, 5, 50)


class ServoMode:
    OFF = 0
    TRIM = 1
    POSITION = 2
    SCANNING = 3
    servo_modes = ["OFF", "TRIM", "POSITION", "SCANNING"]

    def __init__(self, mode=OFF):
        self._mode = mode
    
    @property
    def mode(self):
        return self._mode
    
    @mode.setter
    def mode(self, mode):
        self._mode = mode

    def inc(self):
        self._mode = (self._mode + 1) % 4

    def __eq__(self, other):
        return self._mode == other

    def __str__(self):
        return self.servo_modes[self._mode]


class ServoTestMgr:
    """Manages the Servo Tester functionality.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """

    def __init__(self, app):
        self.app = app
        self.servo               = [None]*4                         # Servo Positions
        self.servo_centre        = [_SERVO_DEFAULT_CENTRE]*4        # Trim Servo Centre
        self.servo_range         = [_SERVO_DEFAULT_RANGE]*4         # Limit Servo Range
        self.servo_rate          = [_SERVO_DEFAULT_RATE]*4          # Servo Rate of Change
        self.servo_mode          = [ServoMode() for _ in range(4)]  # Servo Mode
        self.servo_selected: int = 0        
        self.time_since_last_input: int = 0
        self.timeout_period: int = 300000                     # ms (5 minutes - without any user input)       
        self.keep_alive_period: int = 500                     # ms (half the value used in hexdrive.py)  
        self._ls_pins = []


    def _available_hs_servos(self) -> int:
        return min(4, max(0, self.app.num_servos))


    @property
    def available_servo_count(self) -> int:
        if self.app.hexdrive_port is None:
            return self._available_hs_servos()
        return min(4, self._available_hs_servos() + len(_LS_SERVO_PIN_INDEXES))


    def _is_ls_servo_index(self, index: int) -> bool:
        return index >= self._available_hs_servos()


    def _ls_pin_for_servo_index(self, index: int):
        if not self._is_ls_servo_index(index):
            return None
        ls_idx = index - self._available_hs_servos()
        if 0 <= ls_idx < len(self._ls_pins):
            return self._ls_pins[ls_idx]
        return None


    def _pin_high(self, pin) -> None:
        if pin is None:
            return
        pin.value(1)


    def _pin_low(self, pin) -> None:
        if pin is None:
            return
        pin.value(0)


    def _prepare_ls_pin(self, pin) -> None:
        if pin is None:
            return
        pin.init(mode=Pin.OUT)
        pin.value(0)


    def _clear_all_ls_outputs(self) -> None:
        for pin in self._ls_pins:
            self._pin_low(pin)


    def _effective_pulse_us(self, index: int):
        if self.servo_mode[index] == ServoMode.OFF or self.servo[index] is None:
            return None
        pulse = int(self.servo[index] + self.servo_centre[index])
        if pulse <= 0:
            return None
        return pulse


    def _apply_servo_output(self, index: int) -> None:
        app = self.app
        pulse = self._effective_pulse_us(index)
        if self._is_ls_servo_index(index):
            # LS channels are pulsed in background_update().
            if pulse is None:
                self._pin_low(self._ls_pin_for_servo_index(index))
            return

        if app.hexdrive_app is None:
            return

        if pulse is None:
            app.hexdrive_app.set_servoposition(index, None)
        else:
            app.hexdrive_app.set_servoposition(index, int(self.servo[index]))


    def _pulse_ls_servos(self) -> None:
        active = []
        for i in range(self._available_hs_servos(), self.available_servo_count):
            pin = self._ls_pin_for_servo_index(i)
            pulse_us = self._effective_pulse_us(i)
            if pin is None or pulse_us is None:
                self._pin_low(pin)
                continue
            active.append((pulse_us, pin))

        if not active:
            return

        active.sort(key=lambda item: item[0])

        #print(f"Pulsing LS servos: {[(pulse, str(pin)) for pulse, pin in active]}")
        gc.disable()
        try:
            for _, pin in active:
                self._pin_high(pin)

            irq_state = disable_irq()    
            start = time.ticks_us()
            for pulse_us, pin in active:
                while time.ticks_diff(time.ticks_us(), start) < pulse_us:
                    pass
                enable_irq(irq_state)
                self._pin_low(pin)
                irq_state = disable_irq()
        finally:
            enable_irq(irq_state)
            gc.enable()
        #print("Finished pulsing LS servos")

    # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Enter servo test from the main menu."""
        app = self.app
        if self.reset_servo():
            app.update_period = app.settings['servo_period'].v
            app.set_menu(None)
            app.button_states.clear()
            app.refresh = True
            app.auto_repeat_clear()
            self.time_since_last_input = 0
            if app.settings['logging'].v:
                print("Entered Servo Test mode")
            return True
        return False
    

    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta: int) -> bool:
        """Handle Servo Test UI.  Returns True if this module handled the state."""
        app = self.app

        if app.button_states.get(BUTTON_TYPES["RIGHT"]):
            if app.auto_repeat_check(delta, (self.servo_mode[self.servo_selected] != ServoMode.SCANNING)):
                if self.servo_mode[self.servo_selected] == ServoMode.TRIM:
                    self.servo_centre[self.servo_selected] += app.settings['servo_step'].v
                    if self.servo_centre[self.servo_selected] > (_SERVO_DEFAULT_CENTRE + _SERVO_MAX_TRIM):
                        self.servo_centre[self.servo_selected] = _SERVO_DEFAULT_CENTRE + _SERVO_MAX_TRIM
                    if (not self._is_ls_servo_index(self.servo_selected)) and app.hexdrive_app is not None:
                        if not app.hexdrive_app.set_servocentre(self.servo_centre[self.servo_selected], self.servo_selected):
                            print("H:Failed to set servo centre")
                elif self.servo_mode[self.servo_selected] == ServoMode.SCANNING:
                    if self.servo_rate[self.servo_selected] < 0:
                        negative = True
                        rate = -self.servo_rate[self.servo_selected]
                    else:
                        negative = False
                        rate = self.servo_rate[self.servo_selected]
                    rate = 10 * inc_value(rate//10, app.auto_repeat_level)
                    if _SERVO_MAX_RATE < rate:
                        rate = _SERVO_MAX_RATE
                    if negative:
                        self.servo_rate[self.servo_selected] = -rate
                    else:
                        self.servo_rate[self.servo_selected] = rate
                else:
                    if self.servo[self.servo_selected] is None:
                        self.servo[self.servo_selected] = 0
                    self.servo_mode[self.servo_selected].mode = ServoMode.POSITION
                    self.servo[self.servo_selected] += app.settings['servo_step'].v
                if self.servo[self.servo_selected] is not None:
                    if self.servo_range[self.servo_selected] < (self.servo[self.servo_selected] + (self.servo_centre[self.servo_selected] - _SERVO_DEFAULT_CENTRE)):
                        self.servo[self.servo_selected] = self.servo_range[self.servo_selected] - (self.servo_centre[self.servo_selected] - _SERVO_DEFAULT_CENTRE)
                app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            if app.auto_repeat_check(delta, (self.servo_mode[self.servo_selected] != ServoMode.SCANNING)):
                if self.servo_mode[self.servo_selected] == ServoMode.TRIM:
                    self.servo_centre[self.servo_selected] -= app.settings['servo_step'].v
                    if self.servo_centre[self.servo_selected] < (_SERVO_DEFAULT_CENTRE - _SERVO_MAX_TRIM):
                        self.servo_centre[self.servo_selected] = _SERVO_DEFAULT_CENTRE - _SERVO_MAX_TRIM
                    if (not self._is_ls_servo_index(self.servo_selected)) and app.hexdrive_app is not None:
                        if not app.hexdrive_app.set_servocentre(self.servo_centre[self.servo_selected], self.servo_selected):
                            print("H:Failed to set servo centre")
                elif self.servo_mode[self.servo_selected] == ServoMode.SCANNING:
                    if self.servo_rate[self.servo_selected] < 0:
                        negative = True
                        rate = -self.servo_rate[self.servo_selected]
                    else:
                        negative = False
                        rate = self.servo_rate[self.servo_selected]
                    rate = 10 * dec_value(rate//10, app.auto_repeat_level)
                    if _SERVO_MIN_RATE > rate:
                        rate = _SERVO_MIN_RATE
                    if negative:
                        self.servo_rate[self.servo_selected] = -rate
                    else:
                        self.servo_rate[self.servo_selected] = rate
                else:
                    if self.servo[self.servo_selected] is None:
                        self.servo[self.servo_selected] = 0
                    self.servo_mode[self.servo_selected].mode = ServoMode.POSITION
                    self.servo[self.servo_selected] -= app.settings['servo_step'].v
                if self.servo[self.servo_selected] is not None:
                    if -self.servo_range[self.servo_selected] > (self.servo[self.servo_selected] + (self.servo_centre[self.servo_selected] - _SERVO_DEFAULT_CENTRE)):
                        self.servo[self.servo_selected] = -self.servo_range[self.servo_selected] - (self.servo_centre[self.servo_selected] - _SERVO_DEFAULT_CENTRE)
                app.refresh = True
        else:
            app.auto_repeat_clear()
            if app.button_states.get(BUTTON_TYPES["UP"]):
                app.button_states.clear()
                self.servo_selected = (self.servo_selected - 1) % self.available_servo_count
                app.refresh = True
            elif app.button_states.get(BUTTON_TYPES["DOWN"]):
                app.button_states.clear()
                self.servo_selected = (self.servo_selected + 1) % self.available_servo_count
                app.refresh = True
            elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
                app.button_states.clear()
                if app.hexdrive_app is not None:
                    app.hexdrive_app.set_power(False)
                    app.hexdrive_app.set_servoposition()
                self._clear_all_ls_outputs()
                app.return_to_menu()
                return True
            elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
                app.button_states.clear()
                self.servo_mode[self.servo_selected].inc()
                if self.servo_mode[self.servo_selected] == ServoMode.OFF:
                    if self._is_ls_servo_index(self.servo_selected):
                        self._pin_low(self._ls_pin_for_servo_index(self.servo_selected))
                    elif app.hexdrive_app is not None:
                        app.hexdrive_app.set_servoposition(self.servo_selected, None)
                else:
                    app.refresh = True
                app.notification = Notification(f"  Servo {self.servo_selected}:\n {self.servo_mode[self.servo_selected]}")

        if app.refresh:
            self.time_since_last_input = 0
        else:
            self.time_since_last_input += delta
            if self.time_since_last_input > self.timeout_period:
                if app.hexdrive_app is not None:
                    app.hexdrive_app.set_power(False)
                    app.hexdrive_app.set_servoposition()
                self._clear_all_ls_outputs()
                app.return_to_menu()
                app.notification = Notification("  Servo:\n Timeout")

        app.time_since_last_update += delta
        if app.time_since_last_update > self.keep_alive_period:
            app.time_since_last_update = 0
            app.refresh = True

        for i in range(self.available_servo_count):
            _refresh = app.refresh
            if self.servo_mode[i] == ServoMode.SCANNING:
                if self.servo[i] is None:
                    self.servo[i] = 0
                self.servo[i] = self.servo[i] + ((self.servo_rate[i] * delta) // 1000)
                if self.servo_range[i] < (self.servo[i] + (self.servo_centre[i] - _SERVO_DEFAULT_CENTRE)):
                    self.servo_rate[i] = -self.servo_rate[i]
                    self.servo[i] = self.servo_range[i] - (self.servo_centre[i] - _SERVO_DEFAULT_CENTRE)
                elif -self.servo_range[i] > (self.servo[i] + (self.servo_centre[i] - _SERVO_DEFAULT_CENTRE)):
                    self.servo_rate[i] = -self.servo_rate[i]
                    self.servo[i] = -self.servo_range[i] - (self.servo_centre[i] - _SERVO_DEFAULT_CENTRE)
                _refresh = True
            if _refresh:
                self._apply_servo_output(i)

        return True


    def background_update(self, _delta: int):
        self._pulse_ls_servos()
        return None

        
    # ------------------------------------------------------------------
    # Servo reset
    # ------------------------------------------------------------------

    def reset_servo(self) -> bool:
        app = self.app

        self._ls_pins = []
        if app.hexdrive_port is not None:
            try:
                config = HexpansionConfig(app.hexdrive_port)
                for pin_index in _LS_SERVO_PIN_INDEXES:
                    try:
                        pin = config.ls_pin[pin_index]
                        self._prepare_ls_pin(pin)
                        self._ls_pins.append(pin)
                    except Exception:       # pylint: disable=broad-exception-caught
                        pass
            except Exception:               # pylint: disable=broad-exception-caught
                self._ls_pins = []

        if app.hexdrive_app is not None:
            if app.hexdrive_app.initialise() and app.hexdrive_app.set_power(True) and app.hexdrive_app.set_freq(1000 // app.settings['servo_period'].v):
                for i in range(self.available_servo_count):
                    if not self._is_ls_servo_index(i):
                        app.hexdrive_app.set_servocentre(self.servo_centre[i], i)
                    self.servo_range[i] = app.settings['servo_range'].v
                    if self.servo[i] is not None:
                        if self.servo[i] > self.servo_range[i]:
                            self.servo[i] = self.servo_range[i]
                        elif self.servo[i] < -self.servo_range[i]:
                            self.servo[i] = -self.servo_range[i]
                        if not self._is_ls_servo_index(i):
                            if not app.hexdrive_app.set_servoposition(i, int(self.servo[i])):
                                if app.settings['logging'].v:
                                    print("H:Failed to set servo position")
                self.servo_selected = 0
                app.time_since_last_update = 0
                self.time_since_last_input = 0
                if app.settings['logging'].v:
                    print("H:HexDrive initialised for servo test")
                return True
        if app.settings['logging'].v:
            print("H:Failed to initialise HexDrive for servo test")
        app.notification = Notification("HexDrive Init Failed")
        return False


    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx) -> bool:
        """Render Servo Tester UI.  Returns True if handled."""
        app = self.app

        servo_count = self.available_servo_count
        servo_text = ["S"] * (1 + servo_count)
        servo_text_colours = [(0.4, 0.0, 0.0)] * (1 + servo_count)
        servo_text[0] = "Servo Test"
        servo_text_colours[0] = (1, 1, 1)
        for i in range(servo_count):
            if self.servo[i] is None or self.servo_mode[i] == ServoMode.OFF:
                body_colour = (0.2, 0.2, 0.2)
                bar_colour = (0.4, 0.4, 0.4)
            elif self.servo_mode[i] == ServoMode.SCANNING:
                body_colour = (0.1, 0.5, 0.1)
                bar_colour = (0.1, 1.0, 0.1)
                servo_text_colours[1 + i] = (0.4, 0.0, 0.4)
            else:
                body_colour = (0.1, 0.1, 0.5)
                bar_colour = (0.1, 0.1, 1.0)
                servo_text_colours[1 + i] = (0.4, 0.4, 0.0)

            ctx.save()
            ctx.translate(0, (i - (servo_count / 2) + 0.5) * label_font_size)
            background_colour = (0.1, 0.1, 0.1) if i != self.servo_selected else (0.15, 0.15, 0.15)
            ctx.rgb(*background_colour).rectangle(-100, 1, 200, label_font_size - 2).fill()
            c = 100 * (self.servo_centre[i] - _SERVO_DEFAULT_CENTRE) / self.servo_range[i]
            if self.servo[i] is not None:
                x = 100 * (self.servo[i] + self.servo_centre[i] - _SERVO_DEFAULT_CENTRE) / self.servo_range[i]
                ctx.rgb(*bar_colour).rectangle(x - 2, 1, 5, label_font_size - 2).fill()
                ctx.rgb(*body_colour)
                if x > (c + 4):
                    ctx.rectangle(c + 1, 3, x - c - 4, label_font_size - 6).fill()
                elif x < (c - 4):
                    ctx.rectangle(x + 4, 3, c - x - 4, label_font_size - 6).fill()
            ctx.rgb(0, 0, 0).move_to(c, 0).line_to(c, label_font_size).stroke()
            ctx.restore()
            if self.servo_mode[i] == ServoMode.SCANNING:
                servo_text[i + 1] = f"{int(abs(self.servo_rate[i])):4}\u00B5s/s"
            else:
                servo_text[i + 1] = "Off" if (self.servo[i] is None or self.servo_mode[i] == ServoMode.OFF) else f"{int(self.servo[i]):+5}\u00B5s"
        servo_text_colours[1 + self.servo_selected] = tuple(int(j * 2.5) for j in servo_text_colours[1 + self.servo_selected])
        app.draw_message(ctx, servo_text, servo_text_colours, label_font_size)
        if self.servo_mode[self.servo_selected] == ServoMode.SCANNING:
            button_labels(ctx, up_label="^", down_label="\u25BC", confirm_label="Mode", cancel_label="Exit", left_label="Slower", right_label="Faster")
        elif self.servo_mode[self.servo_selected] == ServoMode.TRIM:
            button_labels(ctx, up_label="^", down_label="\u25BC", confirm_label="Mode", cancel_label="Exit", left_label="Trim-", right_label="+Trim")
        else:
            button_labels(ctx, up_label="^", down_label="\u25BC", confirm_label="Mode", cancel_label="Exit", left_label="<--", right_label="-->")
        return True
