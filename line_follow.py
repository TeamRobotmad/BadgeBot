""" Line Follower Module for BadgeBot """
#
# Handles the line-following functionality.
# Uses the colour sensor (shared with the Sensor Test module, which provides its calibration)
# to follow a coloured line by steering on the detected hue.
#
# Public interface (called by the main app):
#   __init__(app)           – wire up to BadgeBotApp
#   start()                 – enter line follower from menu (False if no colour sensor)
#   update(delta)           – per-tick state machine update
#   draw(ctx)               – render line follower UI
#   background_update(delta)– called from the fast background loop;
#                             returns motor output tuple or None
#   init_settings(settings) – register line-follower specific settings


from events.input import BUTTON_TYPES
from app_components.notification import Notification
from app_components.tokens import label_font_size, button_labels
import micropython

from .app import MOTOR_PWM_FREQ
from .motor_moves import DEFAULT_MAX_POWER, POWER_SCALE_FACTOR, MIN_MAX_POWER, MAX_MAX_POWER

try:
    from micropython import const
except ImportError:
    # CPython / simulator fallback – const() is just an identity function
    # on MicroPython; replicate that so module-level const() calls work.
    const = lambda x: x         #pylint: disable=unnecessary-lambda-assignment


# Line Follower constants
_LINE_SENSOR_UPDATE_PERIOD_MS = const(10)
# For integer Hue values we use 0.1-degree units, so 360 degrees = 3600 units.
_DEFAULT_HUE_NEUTRAL = const(3000)  # Default 'neutral hue' for colour sensor, midway between red and blue (3000 = 300.0 degrees)
_DEFAULT_HUE_MAX = const(900)       # Clamp steering input to this hue-distance from neutral (0.1-degree units)

_HUE_CIRCLE = const(3600)
_HUE_HALF_CIRCLE = const(_HUE_CIRCLE // 2)

# PID Gains for Steering Control (scaled up by 1000 for integer maths)
_FOLLOWER_PID_KP_DEFAULT = const(10000)
_FOLLOWER_PID_KI_DEFAULT = const(0)
_FOLLOWER_PID_KD_DEFAULT = const(10000)
_FOLLOWER_PID_SCALE_FACTOR = const(1000)

# Do not set this too high otherwise there is no scope for one wheel to be given more power than the other to steer. (max is 65536)
_DEFAULT_FOLLOWER_POWER = 30000 // POWER_SCALE_FACTOR
_DEFAULT_STEERING_GAIN = 10 # while this could be taken up by the PID gains, this fixed scale factor allows the PID gains to be smaller values.

# Line Follower Modes
_FOLLOWER_MODE_DIFFERENTIAL = const(0)
_FOLLOWER_MODE_BINARY = const(1)



@micropython.viper
def _signed_hue_delta(hue: int, neutral_hue: int) -> int:
    """Return shortest signed distance hue-neutral in 0.1-degree units.

    Positive means hue is clockwise from neutral, negative anti-clockwise.
    Result is always in [-1800, 1800].
    """
    delta = hue - neutral_hue
    if delta > (_HUE_HALF_CIRCLE):
        delta -= (_HUE_CIRCLE)
    elif delta < -(_HUE_HALF_CIRCLE):
        delta += (_HUE_CIRCLE)
    return delta


@micropython.viper
def _clamp(value: int, lo: int, hi: int) -> int:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting: type):      #pylint: disable=invalid-name
    """Register line-follower-specific settings in the shared settings dict."""
    s['line_power']     = MySetting(s, _DEFAULT_FOLLOWER_POWER, MIN_MAX_POWER, MAX_MAX_POWER)  # Follow power setting
    s['hue_neutral']    = MySetting(s, _DEFAULT_HUE_NEUTRAL, 0, 3600)
    s['hue_max']        = MySetting(s, _DEFAULT_HUE_MAX, 0, 1800)
    s['steering_gain']  = MySetting(s, _DEFAULT_STEERING_GAIN, -100, 100)
    s['pid_kp']         = MySetting(s, _FOLLOWER_PID_KP_DEFAULT, 0, 65536)
    s['pid_ki']         = MySetting(s, _FOLLOWER_PID_KI_DEFAULT, 0, 65535)
    s['pid_kd']         = MySetting(s, _FOLLOWER_PID_KD_DEFAULT, 0, 65535)


# ---- Line Follower Manager -------------------------------------------------

class LineFollowMgr:
    """Manages the Line Follower functionality.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """
    __slots__ = ("_app", "_logging", "sensor_rate", "follower_mode",
                 "line_power", "pid_integral", "pid_previous_error",
                 "kp", "ki", "kd", "max_pwr", "integral_limit", "motor_output",
                 "_last_colour", "_last_colour_hue", "_last_colour_name", "_colour_hexdrive",
                 "_hue_neutral", "_hue_max", "_new_sample", "_refresh_time", "_refresh_interval", "_signed_steering_gain")

    def __init__(self, app, logging: bool = False):
        self._app = app
        self._logging: bool = logging
        self.sensor_rate: int = 0     # sample rate
        self.follower_mode: int = _FOLLOWER_MODE_DIFFERENTIAL   # Default follower mode
        self.line_power: int = _DEFAULT_FOLLOWER_POWER                # Default line follower power
        self.pid_integral: int = 0                              # Accumulated integral term for PID controller
        self.pid_previous_error: int = 0                        # Previous error for derivative term of PID controller
        self.kp: int = _FOLLOWER_PID_KP_DEFAULT
        self.ki: int = _FOLLOWER_PID_KI_DEFAULT
        self.kd: int = _FOLLOWER_PID_KD_DEFAULT
        self.max_pwr: int = DEFAULT_MAX_POWER * POWER_SCALE_FACTOR
        self.integral_limit: int = 0
        self.motor_output = (0,0)
        self._last_colour_hue: int = 0
        self._last_colour: tuple[int, int, int] = (0, 0, 0)
        self._last_colour_name: str = "unknown"
        self._colour_hexdrive = None
        self._hue_neutral: int = _DEFAULT_HUE_NEUTRAL
        self._hue_max: int = _DEFAULT_HUE_MAX
        self._new_sample: bool = False
        self._refresh_time: int = 0
        self._refresh_interval: int = 200
        self._signed_steering_gain: int = -_DEFAULT_STEERING_GAIN       # sign reversed for line-following on the opposite side of the line
        if self._logging:
            print("LineFollowMgr initialised")


    # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Enter line follower from the main menu.
        Requires a colour sensor (calibrated beforehand via Sensor Test) and a HexDrive for the
        motors.  Returns False (with a notification) if either is unavailable."""
        app = self._app

        # A colour sensor is required to follow a coloured line.
        sensor_mgr = app.sensor_test_mgr
        colour_hexdrive = sensor_mgr.active_colour_hexdrive() if sensor_mgr is not None else None
        if colour_hexdrive is None:
            app.notification = Notification("Colour Sensor not available")
            return False

        # A HexDrive is required to drive the motors.
        if len(app.hexdrive_apps) == 0:
            if self._logging:
                print("HexDrive not available; Line Follower requires HexDrive to run")
            app.notification = Notification("HexDrive Init Failed")
            return False

        motor_hexdrive = app.hexdrive_apps[0]
        motor_hexdrive.set_logging(False)
        if not (motor_hexdrive.initialise() and motor_hexdrive.set_power(True) and motor_hexdrive.set_freq(MOTOR_PWM_FREQ)):
            if self._logging:
                print("HexDrive initialisation failed for Line Follower")
            app.notification = Notification("HexDrive Init Failed")
            return False

        # Load any persisted colour calibration, then enable the colour sensor for polling
        # (no events, no interrupts).
        if not sensor_mgr.enable_colour_sensor(colour_hexdrive, events=False, interrupts=False):
            app.notification = Notification("Colour Sensor not available")
            return False
        # Load any persisted colour calibration
        if not sensor_mgr.apply_colour_calibration(colour_hexdrive):
            app.notification = Notification("Please Calibrate Colour Sensor")
            return False
        self._colour_hexdrive = colour_hexdrive

        app.update_period = _LINE_SENSOR_UPDATE_PERIOD_MS
        app.set_menu(None)
        app.button_states.clear()
        app.refresh = True
        app.auto_repeat_clear()

        # Reset controller and reading state
        self.motor_output = (0, 0)
        self.pid_integral = 0
        self.pid_previous_error = 0
        self._last_colour_hue = 0
        self._last_colour = (0, 0, 0)
        self._last_colour_name = "unknown"
        self._new_sample = False
        self._refresh_time = 0

        # Load PID / tuning parameters from settings
        self.kp = app.settings['pid_kp'].v
        self.ki = app.settings['pid_ki'].v
        self.kd = app.settings['pid_kd'].v
        self._hue_neutral = app.settings['hue_neutral'].v
        self._signed_steering_gain = app.settings['steering_gain'].v
        self._hue_max = app.settings['hue_max'].v
        self.max_pwr = POWER_SCALE_FACTOR * (app.settings['max_power'].v if 'max_power' in app.settings else DEFAULT_MAX_POWER)
        self.line_power = POWER_SCALE_FACTOR * (app.settings['line_power'].v if 'line_power' in app.settings else _DEFAULT_FOLLOWER_POWER)
        if self.ki > 0:
            self.integral_limit = self.max_pwr // self.ki
        else:
            self.integral_limit = 0
        if self._logging:
            print("Entered Line Follower mode")
        return True


    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta) -> bool:
        """Handle Line Follower UI.  Returns True if handled."""
        app = self._app

        # We don't want to update display every sample, so we use a refresh timer to limit the update rate.
        self._refresh_time += delta
        if self._refresh_time >= self._refresh_interval:
            if self._new_sample:
                self._new_sample = False
                self._refresh_time = 0
                app.refresh = True

        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if len(app.hexdrive_apps) > 0:
                app.hexdrive_apps[0].set_power(False)
            sensor_mgr = app.sensor_test_mgr
            if sensor_mgr is not None and self._colour_hexdrive is not None:
                sensor_mgr.disable_colour_sensor(self._colour_hexdrive)
            self._colour_hexdrive = None
            app.set_ring_colour(None)
            self.pid_integral = 0
            self.pid_previous_error = 0
            # persist any changes to the line follower settings before returning to the menu
            app.settings['hue_neutral'].persist()
            app.settings['steering_gain'].persist()
            app.return_to_menu()
            return True
        elif app.button_states.get(BUTTON_TYPES["RIGHT"]): # Left/Right provide a shortcut to adjust the neutral hue for line following, without having to go into the settings menu.
            app.button_states.clear()
            app.settings['hue_neutral'].v = app.settings['hue_neutral'].inc(app.settings['hue_neutral'].v, l=1)
            self._hue_neutral = app.settings['hue_neutral'].v
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            app.button_states.clear()
            app.settings['hue_neutral'].v = app.settings['hue_neutral'].dec(app.settings['hue_neutral'].v, l=1)
            self._hue_neutral = app.settings['hue_neutral'].v
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["DOWN"]): # Up/Down provide a shortcut to adjust the steering gain for line following, without having to go into the settings menu.
            app.button_states.clear()
            app.settings['steering_gain'].v = app.settings['steering_gain'].dec(app.settings['steering_gain'].v)
            # we need to skip 0 so that the steering gain is never 0, otherwise the robot will not steer at all.
            if app.settings['steering_gain'].v == 0:
                app.settings['steering_gain'].v = -1
            self._signed_steering_gain = app.settings['steering_gain'].v
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["UP"]):
            app.button_states.clear()
            app.settings['steering_gain'].v = app.settings['steering_gain'].inc(app.settings['steering_gain'].v)
            # we need to skip 0 so that the steering gain is never 0, otherwise the robot will not steer at all.
            if app.settings['steering_gain'].v == 0:
                app.settings['steering_gain'].v = 1
            self._signed_steering_gain = app.settings['steering_gain'].v
            app.refresh = True
        return True


    # ------------------------------------------------------------------
    # Background update (called from the fast loop)
    # ------------------------------------------------------------------

    def background_update(self, delta) -> tuple[int, int] | None:  # pylint: disable=unused-argument
        """Line follower motor control based on the colour sensor hue.
        Returns motor output tuple, or None if not active."""
        sensor_mgr = self._app.sensor_test_mgr
        if sensor_mgr is None or self._colour_hexdrive is None:
            return None

        # Poll the shared colour sensor; read_colour also updates the ring colour on change.
        new_sample, hue, name, _raw = sensor_mgr.read_colour(self._colour_hexdrive)
        if new_sample:
            if self._app.logging:
                print(f"Line Follower: Hue={hue//10}.{hue%10}° Name={name}")
            self._last_colour_hue = hue
            self._last_colour_name = name
            self._last_colour = _raw
            self._new_sample = True

            if self._last_colour_name not in ("White", "Grey", "Black"):
                hue_difference_from_neutral = _signed_hue_delta(self._last_colour_hue, self._hue_neutral)

                if self.follower_mode == _FOLLOWER_MODE_BINARY:
                    # Binary mode: if the hue is within the neutral +/- max range, go straight; otherwise turn left or right.
                    # NOT TESTED – this is a simple example of a non-PID line follower, but it is not as effective as the differential mode.
                    if abs(hue_difference_from_neutral) < self._hue_max:
                        output = (self.line_power, self.line_power)
                    elif hue_difference_from_neutral > 0:
                        output = (self.line_power // 2, self.line_power)
                    else:
                        output = (self.line_power, self.line_power // 2)
                elif self.follower_mode == _FOLLOWER_MODE_DIFFERENTIAL:
                    # Use circular hue distance so values near 0/3600 wrap correctly.
                    # need setting to flip the sign of the steering input to reverse the direction of the turn if needed
                    if abs(hue_difference_from_neutral) < self._hue_max:
                        steering_input = self._signed_steering_gain * hue_difference_from_neutral
                        output = self.compute_differential_output(steering_input)
                    else:
                        output = (0, 0)
                else:
                    output = (0, 0)
            else:
                # Stop if the colour is white, grey, or black (i.e. no line detected)
                output = (0, 0)
            self.motor_output = output
        else:
            output = self.motor_output
        return output


    def compute_differential_output(self, error: int) -> tuple[int, int]:
        """Compute motor output using a full PID controller for differential line following.

        Uses the difference between left and right sensor readings as the error signal,
        and applies proportional, integral, and derivative terms to compute a steering correction.
        Returns a tuple of (left_motor, right_motor) power values, clamped to max_power.
        Uses integer maths for efficiency, scaling down the PID gains and error values to avoid overflow.
        """

        # Proportional term
        p_term = (self.kp * error) // _FOLLOWER_PID_SCALE_FACTOR  # Scale down the error for the proportional term

        # Integral term - accumulate error over time with anti-windup clamping
        if self.ki > 0:
            self.pid_integral += error
            self.pid_integral = max(min(self.pid_integral, self.integral_limit), -self.integral_limit)
            i_term = (self.ki * self.pid_integral) // _FOLLOWER_PID_SCALE_FACTOR
        else:
            i_term = 0

        # Derivative term - rate of change of error
        d_term = (self.kd * (error - self.pid_previous_error)) // _FOLLOWER_PID_SCALE_FACTOR
        self.pid_previous_error = error

        # Combined PID output
        correction = p_term + i_term + d_term

        # Combine correction with base forward power to get output for each motor & limit output to max power
        max_power = self.max_pwr
        output = (_clamp(self.line_power + correction, -max_power, max_power), _clamp(self.line_power - correction, -max_power, max_power))

        if self._logging:
            print(f"PID: err={error} P={p_term} I={i_term} D={d_term} corr={correction} out={output}")

        return output


    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------



    def draw(self, ctx) -> bool:
        """Render Line Follower UI.  Returns True if handled."""

        # draw a box to show the deviation from neutral hue:
        # outer box shows the maximum hue deviation possible, inner box shows the maxiimum hue deviation used, and a line shows the current deviation position
        ctx.rgb(0.5,0.5,0.5).rectangle(-100, (-label_font_size+8)//2, 200, label_font_size - 8).fill()
        #ctx.rgb(1,0,1).rectangle(-max_deviation_x, -label_font_size//2 + 4, 2 * max_deviation_x, label_font_size - 8).fill()

        # draw 'rainbow' like bands to show the hue ranges for the different colours between the minimum and maximum hue deviation, with the neutral hue in the centre
        width = 4
        half_height = label_font_size // 2
        ctx.line_width = width
        max_deviation_x = (100 * self._hue_max) // 1800
        # only need to draw lines every line_width pixels, so we can skip some to reduce the number of lines drawn
        for x in range(-max_deviation_x, max_deviation_x + 1, width):
            pixel_x = x + width // 2
            hue_offset = (x * 1800) // 100
            if self._signed_steering_gain < 0:
                # reverse the hue direction if the steering gain is negative
                hue_offset = -hue_offset
            hue = (self._hue_neutral + hue_offset) % _HUE_CIRCLE
            ctx.rgb(*hue_to_rgb(hue)).move_to(pixel_x, -half_height).line_to(pixel_x, half_height).stroke()

        if self._last_colour is not None:
            ctx.rgb(1,1,1).move_to(-ctx.text_width("Hue")//2, -(3 * label_font_size)//2 ).text("Hue")

            neutral_hue_text = f"{self._hue_neutral//10}°"
            ctx.rgb(1, 1, 0).move_to(-ctx.text_width(neutral_hue_text)//2, -label_font_size//2).text(neutral_hue_text)

            #ctx.move_to(-80,  2 * label_font_size).text(f"Hue:{self._last_colour_hue//10}.{self._last_colour_hue%10}°")
            display_rgb = self._app.sensor_test_mgr.colour_card_rgb(self._last_colour_name) if self._app.sensor_test_mgr is not None else (0.5, 0.5, 0.5)
            ctx.rgb(*display_rgb).move_to(-ctx.text_width(f"{self._last_colour_name}")//2, 2 * label_font_size).text(f"{self._last_colour_name}")

            # Steering Gain
            steering_gain_text = f"Gain: {self._signed_steering_gain}"
            ctx.rgb(1, 1, 0).move_to(-ctx.text_width(steering_gain_text)//2, 3 * label_font_size).text(steering_gain_text)

            deviation = _signed_hue_delta(self._last_colour_hue, self._hue_neutral)
            deviation_x = (100 * deviation) // 1800
            if self._signed_steering_gain < 0:
                # reverse the deviation direction if the steering gain is negative
                deviation_x = -deviation_x

            # current deviation line in white
            ctx.line_width = 4
            ctx.rgb(1,1,1).move_to(deviation_x, -half_height).line_to(deviation_x, half_height).stroke()

        # tick mark for the centre in black
        ctx.line_width = 2
        ctx.rgb(0,0,0).move_to(0, -half_height).line_to(0, 0).stroke()

        button_labels(ctx, cancel_label="Cancel", up_label="+gain", down_label="-gain", left_label="\u25C0", right_label="\u25B6")

        return True

def hue_to_rgb(h: int) -> tuple[float, float, float]:
    """Convert a hue value (0-360 degrees) to an RGB tuple (0-1.0 each).
    This function uses the HSV to RGB conversion algorithm, assuming full saturation and value.
    h values are in units of 0.1degrees (0-3600).
    """
    x = 1 - abs(((h / 600) % 2) - 1)

    if h < 600:
        r1, g1, b1 = 1, x, 0
    elif h < 1200:
        r1, g1, b1 = x, 1, 0
    elif h < 1800:
        r1, g1, b1 = 0, 1, x
    elif h < 2400:
        r1, g1, b1 = 0, x, 1
    elif h < 3000:
        r1, g1, b1 = x, 0, 1
    else:
        r1, g1, b1 = 1, 0, x

    return r1, g1, b1
