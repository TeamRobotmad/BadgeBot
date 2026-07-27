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
from app_components.tokens import label_font_size, small_font_size, button_labels
import micropython

from .app import MOTOR_PWM_FREQ, MOTOR_POWER_SCALE_FACTOR, STATE_FOLLOWER, DEFAULT_ACTIVE_UPDATE_PERIOD

try:
    from micropython import const
except ImportError:
    # CPython / simulator fallback – const() is just an identity function
    # on MicroPython; replicate that so module-level const() calls work.
    const = lambda x: x         #pylint: disable=unnecessary-lambda-assignment


# Line Follower constants
_MAX_TIME_WITHOUT_LINE = const(500)  # milliseconds to continue moving after losing the line before stopping
_DEFAULT_DISPLAY_REFRESH_INTERVAL_MS = const(1000)

# Automatic Stop baesd on Range Sensor
_DEFAULT_MIN_OBSTACLE_DISTANCE = const(100)  # minimum distance in mm to an obstacle before stopping
_MIN_MIN_OBSTACLE_DISTANCE_MM = const(20)       # Minimum allowed value for the minimum range setting
_MIN_MAX_OBSTACLE_DISTANCE_MM = const(500)      # Maximum allowed value for the minimum range setting

# For integer Hue values we use 0.1-degree units, so 360 degrees = 3600 units.
_DEFAULT_MID_HUE = const(300)      # Default 'mid hue' for colour sensor, midway between red and blue (300 = 300.0 degrees)
_DEFAULT_MAX_HUE = const( 70)      # Clamp steering input to this hue-distance from neutral (degree units)
                                   # if the detected Hue is further away than this from the mid hue then we consider it to be off the line and stop moving.

_HUE_CIRCLE = const(3600)
_HUE_HALF_CIRCLE = const(_HUE_CIRCLE // 2)
_HUE_COLOUR_UNKNOWN = const(-1)  # special value for unknown hue (Achromatic colours - so not indicated in UI)
_HUE_SCALE_FACTOR = const(10)  # scale factor for hue values to allow finer adjustment in settings

# PID Gains for Steering Control (scaled up by 1000 for integer maths)
_DEFAULT_FOLLOWER_PID_KP = const( 25)
_DEFAULT_FOLLOWER_PID_KI = const(  0)
_DEFAULT_FOLLOWER_PID_KD = const(200)
_FOLLOWER_PID_SCALE_FACTOR = const(10)        # if you change the number of digits in this scale factor then update the draw formatting

# Do not set this too high otherwise there is no scope for one wheel to be given more power than the other to steer. (max is 65536)
_DEFAULT_FOLLOWER_POWER = const(10) # 20000 // MOTOR_POWER_SCALE_FACTOR   # NB can't wrap inside const()
_MIN_LINE_POWER = const(2)          #  1000 // MOTOR_POWER_SCALE_FACTOR
_MAX_LINE_POWER = const(127)        # 65536 // MOTOR_POWER_SCALE_FACTOR

# Line Follower Modes
_FOLLOWER_MODE_DIFFERENTIAL = const(0)
_FOLLOWER_MODE_BINARY       = const(1)    # not implemented


# Editable fields shown on the Line Follower display.  Left/Right cycle the
# selected field; Up/Down adjust its value via MySetting.inc/dec.
# Each entry: (label, setting_name, l, (rect_x, rect_y, rect_w, rect_h))
#   label        - short text used in the +/- button labels
#   setting_name - key into app.settings for inc/dec and read-back
#   l            - order-of-magnitude step passed to MySetting.inc/dec
#   rect         - absolute grey-highlight rectangle (top-left x, y, width, height)
# The structure is explicitly: (Label, Variable, ID, (X, Y, W, H))
_EDIT_FIELDS: tuple[tuple[str, str, int, tuple[int, int, int, int]], ...] = (
    ("Hue", "mid_hue", 0, (-45, -66, 90, 30)),
    ("Kp",  "pid_kp",  1, (-92, 33, 82, 22)),
    ("Kd",  "pid_kd",  1, (10, 33, 82, 22)),
)


_EDIT_FIELDS_LABEL_INDEX = const(0)        # index into each _EDIT_FIELDS entry for the label
_EDIT_FIELDS_SETTING_INDEX = const(1)      # index into each _EDIT_FIELDS entry for the setting_name
_EDIT_FIELDS_L_INDEX = const(2)            # index into each _EDIT_FIELDS entry for the l value
_EDIT_FIELDS_RECTANGLE_INDEX = const(3)    # index into each _EDIT_FIELDS entry for the rectangle tuple


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
    s['line_power']     = MySetting(s, _DEFAULT_FOLLOWER_POWER, _MIN_LINE_POWER, _MAX_LINE_POWER)  # Follow power setting
    s['mid_hue']        = MySetting(s, _DEFAULT_MID_HUE, 0, 360, wrap=True)  # Mid hue for line colour
    s['max_hue']        = MySetting(s, _DEFAULT_MAX_HUE, 0, 180)
    s['pid_kp']         = MySetting(s, _DEFAULT_FOLLOWER_PID_KP, 0, 65536)
    s['pid_ki']         = MySetting(s, _DEFAULT_FOLLOWER_PID_KI, 0, 65535)
    s['pid_kd']         = MySetting(s, _DEFAULT_FOLLOWER_PID_KD, 0, 65535)
    s['min_range']      = MySetting(s, _DEFAULT_MIN_OBSTACLE_DISTANCE, _MIN_MIN_OBSTACLE_DISTANCE_MM, _MIN_MAX_OBSTACLE_DISTANCE_MM)  # minimum distance in mm to an obstacle before stopping


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
                 "kp", "ki", "kd", "integral_limit", "motor_output",
                 "_last_colour", "_last_colour_hue", "_last_colour_name", "_colour_hexdrive", "_range_hexdrive",
                 "_mid_hue", "_max_hue", "_new_sample", "_display_refresh_time", "_display_refresh_interval", "_signed_steering_gain",
                 "_time_since_line_detected", "_selected_field", "_enable_movement", "_min_obstacle_distance", "_obstacle_detection_count", "_calibration_msg_shown")

    def __init__(self, app, logging: bool = True):
        self._app = app
        self._logging: bool = logging
        self.sensor_rate: int = 0     # sample rate
        self.follower_mode: int = _FOLLOWER_MODE_DIFFERENTIAL   # Default follower mode
        self.line_power: int = _DEFAULT_FOLLOWER_POWER                # Default line follower power
        self.pid_integral: int = 0                              # Accumulated integral term for PID controller
        self.pid_previous_error: int = 0                        # Previous error for derivative term of PID controller
        self.kp: int = _DEFAULT_FOLLOWER_PID_KP
        self.ki: int = _DEFAULT_FOLLOWER_PID_KI
        self.kd: int = _DEFAULT_FOLLOWER_PID_KD
        self.integral_limit: int = 0
        self.motor_output = (0, 0)
        self._last_colour_hue: int = 0
        self._last_colour: tuple[int, int, int] = (0, 0, 0)
        self._last_colour_name: str = "unknown"
        self._colour_hexdrive = None
        self._range_hexdrive = None
        self._mid_hue: int = _DEFAULT_MID_HUE
        self._max_hue: int = _DEFAULT_MAX_HUE
        self._new_sample: bool = False
        self._display_refresh_time: int = 0
        self._display_refresh_interval: int = _DEFAULT_DISPLAY_REFRESH_INTERVAL_MS
        self._signed_steering_gain: int = 1  # sign reversed for line-following on the opposite side of the line
        self._time_since_line_detected: int = _MAX_TIME_WITHOUT_LINE  # time since a line was last detected, in milliseconds
        self._selected_field: int = 0  # index into _EDIT_FIELDS of the field currently being adjusted
        self._enable_movement: bool = False
        self._min_obstacle_distance: int = _DEFAULT_MIN_OBSTACLE_DISTANCE  # minimum distance in mm to an obstacle before stopping
        self._obstacle_detection_count: int = 0
        self._calibration_msg_shown: bool = False  # whether the calibration message has been shown
        if self._logging:
            print("B:LineFollowMgr initialised")


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
                print("B:HexDrive not available; Line Follower requires HexDrive to run")
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
        if sensor_mgr is not None:
            if not sensor_mgr.enable_colour_sensor(colour_hexdrive, period=DEFAULT_ACTIVE_UPDATE_PERIOD):
                app.notification = Notification("Colour Sensor not available")
                return False
            # Load any persisted colour calibration
            if not sensor_mgr.apply_colour_calibration(colour_hexdrive):
                app.notification = Notification("Please Calibrate Colour Sensor")
                return False
        self._colour_hexdrive = colour_hexdrive

        app.update_period = DEFAULT_ACTIVE_UPDATE_PERIOD
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
        self._display_refresh_time = 0
        self._time_since_line_detected = _MAX_TIME_WITHOUT_LINE # start with time since line detected at max so we don't try to continue moving until we see a line.
        self._enable_movement = False
        #self._calibration_msg_shown = False # don't reset this so we don't keep showing the message if the user has already seen it once.

        # Load PID / tuning parameters from settings
        self.kp = app.settings['pid_kp'].v
        self.ki = app.settings['pid_ki'].v
        self.kd = app.settings['pid_kd'].v
        self._mid_hue = _HUE_SCALE_FACTOR * (app.settings['mid_hue'].v if 'mid_hue' in app.settings else _DEFAULT_MID_HUE)
        self._max_hue = _HUE_SCALE_FACTOR * (app.settings['max_hue'].v if 'max_hue' in app.settings else _DEFAULT_MAX_HUE)
        self.line_power = MOTOR_POWER_SCALE_FACTOR * (app.settings['line_power'].v if 'line_power' in app.settings else _DEFAULT_FOLLOWER_POWER)

        if self.ki > 0:
            self.integral_limit = self._app.max_power // self.ki
        else:
            self.integral_limit = 0

        # optionally enable range sensor for obstacle detection
        range_hexdrive = sensor_mgr.active_range_hexdrive() if sensor_mgr is not None else None
        if range_hexdrive is not None and sensor_mgr is not None:
            if sensor_mgr.enable_range_sensor(range_hexdrive):
                self._range_hexdrive = range_hexdrive
                self._min_obstacle_distance = app.settings['min_range'].v if 'min_range' in app.settings else _DEFAULT_MIN_OBSTACLE_DISTANCE
                self._obstacle_detection_count = 0

        if self._logging:
            print("B:Line Follower mode started")
        return True


    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta) -> bool:
        """Handle Line Follower UI.  Returns True if handled."""
        app = self._app

        if app.sensor_test_mgr.colour_sensor_stats.update(delta):
            if self._logging:
                print(f"B:LF:CS={app.sensor_test_mgr.colour_sensor_stats.rate_str}")
        if app.sensor_test_mgr.range_sensor_stats.update(delta):
            if self._logging:
                print(f"B:LF:RS={app.sensor_test_mgr.range_sensor_stats.rate_str}")

        # We don't want to update display every sample, so we use a refresh timer to limit the update rate.
        self._display_refresh_time += delta
        if self._display_refresh_time >= self._display_refresh_interval:
            if self._new_sample:
                self._new_sample = False
                self._display_refresh_time = 0
                app.refresh = True

        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            app.performance_mode = False
            if len(app.hexdrive_apps) > 0:
                app.hexdrive_apps[0].set_motors((0, 0))
                app.hexdrive_apps[0].set_power(False)
            sensor_mgr = app.sensor_test_mgr
            if sensor_mgr is not None and self._colour_hexdrive is not None:
                sensor_mgr.disable_colour_sensor(self._colour_hexdrive)
            self._colour_hexdrive = None
            if sensor_mgr is not None and self._range_hexdrive is not None:
                sensor_mgr.disable_range_sensor(self._range_hexdrive)
            self._range_hexdrive = None
            app.set_ring_colour(None)
            self.pid_integral = 0
            self.pid_previous_error = 0


            # persist any changes to the line follower settings before returning to the menu
            app.settings['mid_hue'].persist()
            app.settings['pid_kp'].persist()
            app.settings['pid_ki'].persist()
            app.settings['pid_kd'].persist()
            app.return_to_menu()
            return True
        elif app.button_states.get(BUTTON_TYPES["RIGHT"]):
            # Left/Right cycle which editable field is selected.
            app.button_states.clear()
            self._selected_field = (self._selected_field + 1) % len(_EDIT_FIELDS)
            app.refresh = True
            app.performance_mode = False  # disable performance mode while adjusting settings so we can see the display update
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            app.button_states.clear()
            self._signed_steering_gain *= -1
            if self._logging:
                print(f"B:LF:Steering gain sign reversed to {self._signed_steering_gain}")
            app.performance_mode = False  # disable performance mode while adjusting settings so we can see the display update
        elif app.button_states.get(BUTTON_TYPES["UP"]):
            # Up/Down adjust the value of the currently selected field.
            app.button_states.clear()
            self._adjust_selected_field(1)
            app.refresh = True
            app.performance_mode = False  # disable performance mode while adjusting settings so we can see the display update
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            app.button_states.clear()
            self._adjust_selected_field(-1)
            app.refresh = True
            app.performance_mode = False  # disable performance mode while adjusting settings so we can see the display update
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            self._enable_movement = not self._enable_movement
            if self._enable_movement:
                if self._logging:
                    print("B:LF:Start")
                self._app.set_ring_colour(None) # Turn off the ring colour to indicate we are actively following the line
            else:
                if self._logging:
                    print("B:LF:Stop")
                self.motor_output = (0, 0)      # Stop
            app.refresh = True
            app.performance_mode = False  # disable performance mode while adjusting settings so we can see the display update
        return True


    def _adjust_selected_field(self, direction: int):
        """Adjust the currently selected editable field's setting up (direction > 0)
        or down (direction < 0) using MySetting.inc/dec with the field's 'l' step,
        then refresh the runtime copies used by draw and background_update.
        Fields flagged 'wrap' roll over between min and max at the ends."""
        field = _EDIT_FIELDS[self._selected_field]
        setting_name = field[_EDIT_FIELDS_SETTING_INDEX]
        step_level = field[_EDIT_FIELDS_L_INDEX]
        setting = self._app.settings[setting_name]
        if direction > 0:
            setting.v = setting.inc(setting.v, l=step_level)
        else:
            setting.v = setting.dec(setting.v, l=step_level)
        self._apply_field_settings()


    def _apply_field_settings(self):
        """Refresh the runtime copies of the editable settings used by the
        background_update and draw functions.  All editable copies are refreshed so
        the caller does not need to identify which specific field changed."""
        app = self._app
        self._mid_hue = _HUE_SCALE_FACTOR * app.settings['mid_hue'].v
        self.kp = app.settings['pid_kp'].v
        self.kd = app.settings['pid_kd'].v


    # ------------------------------------------------------------------
    # Background update (called from the fast loop)
    # ------------------------------------------------------------------

    def background_update(self, delta) -> tuple[int, int] | None:  # pylint: disable=unused-argument
        """Line follower motor control based on the colour sensor hue.
        Returns motor output tuple, or None if not active."""
        sensor_mgr = self._app.sensor_test_mgr
        if sensor_mgr is None or self._colour_hexdrive is None:
            return None

        # Poll the shared range sensor if we are allowed to move
        if self._range_hexdrive and self._enable_movement:
            new_sample, range_mm = sensor_mgr.read_range(self._range_hexdrive)
            if new_sample:
                #if self._logging:
                #    print(f"B:LF:Range={range_mm}mm")
                if range_mm < self._min_obstacle_distance and self._enable_movement:
                    # Stop immediately if an obstacle is detected within 100mm
                    self._obstacle_detection_count += 1
                    if self._obstacle_detection_count > 2:  # require 3 consecutive detections to avoid false positives
                        if self._logging:
                            print(f"B:LF:Obstacle detected @{range_mm}mm, auto stop")
                        self._enable_movement = False
                        app = self._app
                        app.performance_mode = False
                        app.notification = Notification("Stop Obstacle", self._range_hexdrive.config.port)
                    else:
                        # start to slow down while we wait for the next sample to confirm the obstacle is still there
                        if self._logging:
                            print(f"B:LF:Obstacle detected @{range_mm}mm, slowing down ({self._obstacle_detection_count})")
                    output = (0, 0)
                    self.motor_output = output
                    return output
                else:
                    self._obstacle_detection_count = 0

        # Poll the shared colour sensor; read_colour also updates the ring colour on change.
        # Force a read to ensure we get the latest sample, as the colour sensor is only polled in the background by the HexDrive
        if self._colour_hexdrive and self._colour_hexdrive.colour_sensor:
            _ = self._colour_hexdrive.colour_sensor.read()

        new_sample, hue, name, _raw = sensor_mgr.read_colour(self._colour_hexdrive)
        if new_sample:
            #if self._logging:
            #    print(f"B:LF:Hue={hue//10}.{hue%10}° Name={name}")
            self._last_colour_hue = hue
            self._last_colour_name = name
            self._last_colour = _raw
            self._new_sample = True

            if self._last_colour_name == "Black":
                # Stop Immediately if the colour is black
                if self._logging:
                    print("B:LF:Black detected, auto stop")
                self._last_colour_hue = _HUE_COLOUR_UNKNOWN
                self._enable_movement = False
                self._time_since_line_detected = _MAX_TIME_WITHOUT_LINE
                output = (0, 0)
                app = self._app
                app.performance_mode = False  # stop performance mode if we have lost the line for a while
                app.notification = Notification("Stop On Black", self._colour_hexdrive.config.port)
            elif self._last_colour_name in ("White", "Grey"):
                # Allow a short period to pick up the line again if the colour is white or grey (i.e. no line detected)
                self._last_colour_hue = _HUE_COLOUR_UNKNOWN
                self._time_since_line_detected += delta
                if self._time_since_line_detected < _MAX_TIME_WITHOUT_LINE:
                    output = self.motor_output
                else:
                    self._time_since_line_detected = _MAX_TIME_WITHOUT_LINE  # clamp to max so we don't overflow
                    output = (0, 0)
                    self._app.performance_mode = False  # stop performance mode if we have lost the line for a while
            else:
                hue_difference_from_mid = _signed_hue_delta(self._last_colour_hue, self._mid_hue)
                if self.follower_mode == _FOLLOWER_MODE_BINARY:
                    # place holder for another algorithm that uses a binary on/off approach to steering, rather than a PID controller.
                    output = (0, 0)
                elif self.follower_mode == _FOLLOWER_MODE_DIFFERENTIAL:
                    # Use circular hue distance so values near 0/3600 wrap correctly.
                    # need setting to flip the sign of the steering input to reverse the direction of the turn if needed
                    if abs(hue_difference_from_mid) < self._max_hue:
                        steering_input = self._signed_steering_gain * hue_difference_from_mid
                        if self._enable_movement:
                            output = self.compute_differential_output(steering_input)
                            self._app.performance_mode = True  # enable performance mode while we are actively following the line
                        else:
                            output = (0, 0)
                            self._app.performance_mode = False  # disable performance mode if we are not actively following the line
                        self._time_since_line_detected = 0
                    else:
                        self._time_since_line_detected += delta
                        if self._time_since_line_detected < _MAX_TIME_WITHOUT_LINE:
                            output = self.motor_output  # continue with last output while we see a colour that is too far from the mid hue, to allow the robot to continue on its path until it finds the line again.
                        else:
                            output = (0, 0)
                            self._time_since_line_detected = _MAX_TIME_WITHOUT_LINE  # clamp to max so we don't overflow
                            self._app.performance_mode = False  # stop performance mode if we have lost the line for a while
                            # as we are not moving we don't need to get updates as fast.
                    #print(f"B:LF:hue_diff={hue_difference_from_mid}, Output={output}")
                else:
                    # Unknown follower mode, so stop the motors
                    output = (0, 0)
                    self._app.performance_mode = False  # disable performance mode if we are not actively following the line

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

        # adjust forward power to be faster when well aligned with the line, and slower when far from the line, to avoid overshooting
        # scale the line power based on the absolute error, so that when the error is small the line power is at maximum, and when the error is large the line power is reduced to avoid overshooting
        abs_error = abs(error)

        # scale line power linearly from max to min (25%) based on the error when too far from the line mid point, to avoid overshooting.  When the error is small, use full line power.
        if abs_error >= (self._max_hue // 2):
            line_power = self.line_power * ((self._max_hue + (self._max_hue//4)) - abs_error) // self._max_hue
        else:
            line_power = self.line_power

        # Combine correction with base forward power to get output for each motor & limit output to max power
        max_power = self._app.max_power
        output = (_clamp(line_power + correction, -max_power, max_power), _clamp(line_power - correction, -max_power, max_power))

        if self._logging:
            print(f"B:LF:PID:Err={error} P={p_term} I={i_term} D={d_term} Corr={correction} Out={output}")

        return output


    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------



    def draw(self, ctx) -> bool:
        """Render Line Follower UI.  Returns True if handled."""

        #Remind User to calibrate Colour Sensor if this is the first time the Line Follower has been started since the app was launched.
        if not self._calibration_msg_shown:
            self._app.show_message(["Line Follower:", "For best tracking", "ensure Colour", "Sensor calibration", "is recent"], [(0.5,1.0,0.5),(1,1,1),(1,1,1),(1,1,1),(1,1,1)], return_state = STATE_FOLLOWER)
            self._calibration_msg_shown = True
            return True

        # ================================================
        # draw a box to show the deviation from mid hue:
        # ================================================
        half_height = label_font_size
        half_width = 100

        # outer box shows the maximum hue deviation possible, inner box shows the maximum hue deviation used, and a line shows the current deviation position
        ctx.rgb(0.25,0.25,0.25).rectangle(-half_width, -half_height, 2 * half_width, label_font_size * 2).fill()

        # grey highlight behind the currently selected editable field
        hx, hy, hw, hh = _EDIT_FIELDS[self._selected_field][_EDIT_FIELDS_RECTANGLE_INDEX]
        ctx.rgb(0.33, 0.33, 0.33).rectangle(hx, hy, hw, hh).fill()

        # 'rainbow' like bands to show the hue ranges for the different colours between the minimum and maximum hue deviation, with the neutral hue in the centre
        width = 2
        ctx.line_width = width
        max_deviation_x = (half_width * self._max_hue) // (180 * _HUE_SCALE_FACTOR)
        # only need to draw lines every line_width pixels, so we can skip some to reduce the number of lines drawn
        for x in range(-max_deviation_x, max_deviation_x + 1, width):
            pixel_x = x + width // 2
            hue_offset = (x * 180 * _HUE_SCALE_FACTOR) // half_width
            if self._signed_steering_gain < 0:
                # reverse the hue direction if the steering gain is negative
                hue_offset = -hue_offset
            hue = (self._mid_hue + hue_offset) % _HUE_CIRCLE
            ctx.rgb(*hue_to_rgb(hue)).move_to(pixel_x, -half_height).line_to(pixel_x, half_height).stroke()

        # if we have a valid colour reading, draw the current deviation line and the hue value
        if self._last_colour is not None:
            # 'mid XXX° hue' label in the middle of the box, with the hue value in the colour of the mid hue
            hue_y = -(3 * label_font_size)//2
            ctx.font_size = small_font_size
            mid_rgb = hue_to_rgb(self._mid_hue)
            ctx.rgb(*mid_rgb)
            ctx.move_to(-80, hue_y).text("Mid")
            ctx.move_to( 50, hue_y).text("Hue")

            # draw the mid hue value in the colour of the mid hue, centred in the box
            ctx.font_size = label_font_size
            mid_hue_text = f"{self._mid_hue // _HUE_SCALE_FACTOR}°"
            mid_rgb = hue_to_rgb(self._mid_hue)
            ctx.rgb(*mid_rgb).move_to(-ctx.text_width(mid_hue_text)//2, hue_y).text(mid_hue_text)

            # You can see the current colour in the outer ring - so no need to duplicate with this...
            # 'last colour' label in the middle of the box, with the colour name in the colour of the last detected hue
            #ctx.font_size = small_font_size
            #display_rgb = self._app.sensor_test_mgr.colour_card_rgb(self._last_colour_name) if self._app.sensor_test_mgr is not None else (0.5, 0.5, 0.5)
            #ctx.rgb(*display_rgb).move_to(-ctx.text_width(f"{self._last_colour_name}")//2, 2 * label_font_size).text(f"{self._last_colour_name}")

            # Kp / Kd gains in yellow, centred below the box at fixed positions
            # that line up with the highlight rectangles in _EDIT_FIELDS.
            ctx.font_size = small_font_size
            gains_y = 2 * label_font_size
            kp_text = f"Kp:{self.kp // _FOLLOWER_PID_SCALE_FACTOR}.{self.kp % _FOLLOWER_PID_SCALE_FACTOR:01d}"
            kd_text = f"Kd:{self.kd // _FOLLOWER_PID_SCALE_FACTOR}.{self.kd % _FOLLOWER_PID_SCALE_FACTOR:01d}"
            ctx.rgb(1, 1, 0).move_to(-51 - ctx.text_width(kp_text)//2, gains_y).text(kp_text)
            ctx.rgb(1, 1, 0).move_to( 51 - ctx.text_width(kd_text)//2, gains_y).text(kd_text)

            if self._last_colour_hue != _HUE_COLOUR_UNKNOWN:
                # If the last colour was sufficiently saturated to have a hue (hue is known) then draw a line to show the current deviation from the mid hue, in white.
                deviation = _signed_hue_delta(self._last_colour_hue, self._mid_hue)
                deviation_x = (half_width * deviation) // (180 * _HUE_SCALE_FACTOR)
                if self._signed_steering_gain < 0:
                    # reverse the deviation direction if the steering gain is negative
                    deviation_x = -deviation_x

                # current deviation line in white
                ctx.line_width = 4
                ctx.rgb(1,1,1).move_to(deviation_x, -half_height-10).line_to(deviation_x, half_height+10).stroke()

        # tick mark for the centre in black
        ctx.line_width = 2
        ctx.rgb(0,0,0).move_to(0, -half_height).line_to(0, 0).stroke()

        # draw the button labels - up/down adjust the selected field, left/right cycle fields
        sel_label = _EDIT_FIELDS[self._selected_field][_EDIT_FIELDS_LABEL_INDEX]
        confirm_label = "Start" if not self._enable_movement else "Stop" # was "\u25C0"

        button_labels(ctx, cancel_label="Back", confirm_label=confirm_label,
                      up_label=f"+{sel_label}", down_label=f"-{sel_label}",
                      left_label="Direction", right_label="\u25B6")

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
