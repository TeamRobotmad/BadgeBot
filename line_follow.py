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

from .app import MOTOR_POWER_SCALE_FACTOR, STATE_FOLLOWER, DEFAULT_ACTIVE_UPDATE_PERIOD, MOTOR_ENABLE_USER_STATE
from .sensor_test import COLOUR_LIST

try:
    from micropython import const
except ImportError:
    # CPython / simulator fallback – const() is just an identity function
    # on MicroPython; replicate that so module-level const() calls work.
    const = lambda x: x         #pylint: disable=unnecessary-lambda-assignment


# Line Follower constants
_MAX_TIME_WITHOUT_LINE = const(500)  # milliseconds to continue moving after losing the line before stopping
_DEFAULT_DISPLAY_REFRESH_INTERVAL_MS = const(100)
_CALIBRATION_MSG_TIMEOUT_MS = const(4000)  # auto-dismiss the calibration reminder so remote (BLE) activation isn't left stuck on it

# Automatic Stop based on Colour Sensor (from COLOUR_LIST)
_DEFAULT_COLOUR_STOP = const(0)  # default colour index to stop on ("Black")

# Automatic Stop based on Range Sensor
_DEFAULT_MIN_OBSTACLE_DISTANCE = const(100)      # minimum distance in mm to an obstacle before stopping
_MIN_MIN_OBSTACLE_DISTANCE_MM  = const(20)       # Minimum allowed value for the minimum range setting
_MIN_MAX_OBSTACLE_DISTANCE_MM  = const(800)      # Maximum allowed value for the minimum range setting

# For integer Hue values we use 0.1-degree units, so 360 degrees = 3600 units.
_DEFAULT_MID_HUE = const(300)      # Default 'mid hue' for colour sensor, midway between red and blue (300 = 300.0 degrees)
_DEFAULT_MAX_HUE = const( 70)      # Clamp steering input to this hue-distance from neutral (degree units)
                                   # if the detected Hue is further away than this from the mid hue then we consider it to be off the line and stop moving.

_HUE_CIRCLE = const(3600)
_HUE_HALF_CIRCLE = const(_HUE_CIRCLE // 2)
_HUE_COLOUR_UNKNOWN = const(-1)  # special value for unknown hue (Achromatic colours - so not indicated in UI)
_HUE_SCALE_FACTOR = const(10)  # scale factor for hue values to allow finer adjustment in settings

# PID Gains for Steering Control (scaled up by 1000 for integer maths)
_DEFAULT_FOLLOWER_PID_KP = const( 60)
_DEFAULT_FOLLOWER_PID_KI = const(  0)
_DEFAULT_FOLLOWER_PID_KD = const( 25)
_FOLLOWER_PID_SCALE_FACTOR = const(1)        # if you change the number of digits in this scale factor then update the draw formatting

# Do not set this too high otherwise there is no scope for one wheel to be given more power than the other to steer. (max is 127)
_DEFAULT_FOLLOWER_POWER = const(60)
_MIN_LINE_POWER = const(10)
_MAX_LINE_POWER = const(127)

# Line Follower Modes
_FOLLOWER_MODE_DIFFERENTIAL = const(0)
_FOLLOWER_MODE_BINARY       = const(1)    # not implemented

# Plot Selection Modes
_PLOT_SELECTION_NONE   = const(0)
_PLOT_SELECTION_COLOUR = const(1)
_PLOT_SELECTION_RANGE  = const(2)
_PLOT_SELECTION_PID    = const(3)
_PLOT_SELECTION_POWER  = const(4)
_PLOT_SELECTION_LABELS = ("None", "Colour", "Range", "PID", "Power")


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
    ("Kp",  "pid_kp",  0, (-92, 33, 82, 22)),
    ("Kd",  "pid_kd",  0, (10, 33, 82, 22)),
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
    group = MySetting.GROUP_LINE_FOLLOWER
    s['line_power']     = MySetting(
        s, _DEFAULT_FOLLOWER_POWER, _MIN_LINE_POWER, _MAX_LINE_POWER,
        group=group, order=10, title="Line power",
        description="Motor power while following a line")
    s['mid_hue']        = MySetting(
        s, _DEFAULT_MID_HUE, 0, 360, wrap=True,
        group=group, order=20, title="Mid hue",
        description="Line colour used as neutral steering")
    s['max_hue']        = MySetting(
        s, _DEFAULT_MAX_HUE, 0, 180,
        group=group, order=30, title="Hue range",
        description="Largest hue error accepted as the line")
    s['pid_kp']         = MySetting(
        s, _DEFAULT_FOLLOWER_PID_KP, 0, 2000,
        group=group, order=40, title="PID Kp",
        description="Proportional steering gain")
    s['pid_ki']         = MySetting(
        s, _DEFAULT_FOLLOWER_PID_KI, 0, 2000,
        group=group, order=45, title="PID Ki",
        description="Accumulated steering correction gain")
    s['pid_kd']         = MySetting(
        s, _DEFAULT_FOLLOWER_PID_KD, 0, 2000,
        group=group, order=50, title="PID Kd",
        description="Steering response to error changes")
    s['min_range']      = MySetting(
        s, _DEFAULT_MIN_OBSTACLE_DISTANCE,
        _MIN_MIN_OBSTACLE_DISTANCE_MM, _MIN_MAX_OBSTACLE_DISTANCE_MM,
        group=group, order=90, title="Stop range",
        description="Distance to obstacle that stops driving (mm)")
    s['colour_stop']    = MySetting(
        s, _DEFAULT_COLOUR_STOP, 0, 9, labels=COLOUR_LIST,
        group=group, order=80, title="Stop colour",
        description="Colour that stops line following")
    s['plot_type']      = MySetting(
        s, _PLOT_SELECTION_NONE, _PLOT_SELECTION_NONE, _PLOT_SELECTION_POWER,
        labels=_PLOT_SELECTION_LABELS, group=group, order=100, title="Plot",
        description="Data shown on BLE Plot")


# ---- Line Follower Manager -------------------------------------------------

class LineFollowMgr:
    """Manages the Line Follower functionality.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """
    __slots__ = ("_app", "_logging", "sensor_rate", "follower_mode",
                 "line_power", "_pid_integral", "_pid_previous_error",
                 "kp", "ki", "kd", "integral_limit", "motor_output",
                 "_last_colour", "_last_colour_hue", "_last_colour_name", "_colour_hexdrive", "_range_hexdrive",
                 "_colour_stop", "_mid_hue", "_max_hue", "_new_sample", "_display_refresh_time", "_display_refresh_interval", "_signed_steering_gain",
                 "_time_since_line_detected", "_selected_field", "_enable_movement", "_min_obstacle_distance", "_obstacle_detection_count", "_calibration_msg_shown",
                 "_last_range_mm", "_plot_selection", "_last_p_term", "_last_i_term", "_last_d_term", "_time_since_last_update_ms")

    def __init__(self, app, logging: bool = True):
        self._app = app
        self._logging: bool = logging
        self.sensor_rate: int = 0     # sample rate
        self.follower_mode: int = _FOLLOWER_MODE_DIFFERENTIAL   # Default follower mode
        self.line_power: int = _DEFAULT_FOLLOWER_POWER                # Default line follower power
        self._pid_integral: int = 0                              # Accumulated integral term for PID controller
        self._pid_previous_error: int = 0                        # Previous error for derivative term of PID controller
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
        self._colour_stop: str = COLOUR_LIST[_DEFAULT_COLOUR_STOP]  # default colour to stop on ("Black")
        self._min_obstacle_distance: int = _DEFAULT_MIN_OBSTACLE_DISTANCE  # minimum distance in mm to an obstacle before stopping
        self._obstacle_detection_count: int = 0
        self._calibration_msg_shown: bool = False  # whether the calibration message has been shown
        self._last_range_mm: int = -1
        self._plot_selection: int = 0  # 0 = colour, 1 = range, 2 = PID output
        self._last_p_term: int = 0
        self._last_i_term: int = 0
        self._last_d_term: int = 0
        self._time_since_last_update_ms: int = 0
        if self._logging:
            print("B:LineFollowMgr initialised")


    @property
    def logging(self) -> bool:
        """Whether to print debug logs to the console."""
        return self._logging

    @logging.setter
    def logging(self, value: bool):
        """Enable or disable logging for the Line Follower Manager."""
        self._logging = value


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


        # Load any persisted colour calibration, then enable the colour sensor for polling
        # (no events, no interrupts).
        if sensor_mgr is not None:
            if not sensor_mgr.enable_colour_sensor(colour_hexdrive, period=DEFAULT_ACTIVE_UPDATE_PERIOD):
                app.enable_motors(False, MOTOR_ENABLE_USER_STATE)
                app.notification = Notification("Colour Sensor not available")
                return False
            # Load any persisted colour calibration
            if not sensor_mgr.apply_colour_calibration(colour_hexdrive):
                app.enable_motors(False, MOTOR_ENABLE_USER_STATE)
                app.notification = Notification("Please Calibrate Colour Sensor")
                return False
        self._colour_hexdrive = colour_hexdrive

        if not app.enable_motors(True, MOTOR_ENABLE_USER_STATE):
            if self._logging:
                print("HexDrive initialisation failed for Line Follower")
            app.notification = Notification("HexDrive Init Failed")
            return False

        app.set_menu(None)
        app.button_states.clear()
        app.refresh = True
        app.auto_repeat_clear()

        # Reset controller and reading state
        self.stop_movement()
        self._last_range_mm = -1
        self._last_colour_hue = 0
        self._last_colour = (0, 0, 0)
        self._last_colour_name = "unknown"
        self._new_sample = False
        self._display_refresh_time = 0
        self._time_since_line_detected = _MAX_TIME_WITHOUT_LINE # start with time since line detected at max so we don't try to continue moving until we see a line.
        #self._calibration_msg_shown = False # don't reset this so we don't keep showing the message if the user has already seen it once.
        app.reset_hue_history()  # start the LED hue history from black

        # Load PID / tuning parameters from settings
        self.kp = app.settings['pid_kp'].v
        self.ki = app.settings['pid_ki'].v
        self.kd = app.settings['pid_kd'].v
        self._mid_hue = _HUE_SCALE_FACTOR * (app.settings['mid_hue'].v if 'mid_hue' in app.settings else _DEFAULT_MID_HUE)
        self._max_hue = _HUE_SCALE_FACTOR * (app.settings['max_hue'].v if 'max_hue' in app.settings else _DEFAULT_MAX_HUE)
        self.line_power = MOTOR_POWER_SCALE_FACTOR * (app.settings['line_power'].v if 'line_power' in app.settings else _DEFAULT_FOLLOWER_POWER)
        self._colour_stop = COLOUR_LIST[app.settings['colour_stop'].v if 'colour_stop' in app.settings else _DEFAULT_COLOUR_STOP]
        self._plot_selection = app.settings['plot_type'].v if 'plot_type' in app.settings else _PLOT_SELECTION_RANGE
        if self._logging:
            print(f"B:LF:line_power={self.line_power}, colour_stop='{self._colour_stop}'")

        if self.ki > 0:
            self.integral_limit = self._app.max_power // self.ki
        else:
            self.integral_limit = 0
        self._time_since_last_update_ms = 0

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

        #Remind User to calibrate Colour Sensor if this is the first time the Line Follower has been started since the app was launched.
        if not self._calibration_msg_shown:
            self._app.show_message(["Line Follower:", "For best tracking", "ensure Colour", "Sensor calibration", "is recent"], [(0.5,1.0,0.5),(1,1,1),(1,1,1),(1,1,1),(1,1,1)], return_state = STATE_FOLLOWER, timeout = _CALIBRATION_MSG_TIMEOUT_MS)
            self._calibration_msg_shown = True
            return True

        if app.sensor_test_mgr.colour_sensor_stats.update(delta):
            #if self._logging:
            print(f"B:LF:CS={app.sensor_test_mgr.colour_sensor_stats.rate_str}")
        if app.sensor_test_mgr.range_sensor_stats.update(delta):
            #if self._logging:
            #print(f"B:LF:RS={app.sensor_test_mgr.range_sensor_stats.rate_str}")
            pass

        # We don't want to update display/plot every sample, so we use a refresh timer to limit the update rate.
        self._display_refresh_time += delta
        if self._display_refresh_time >= self._display_refresh_interval:
            if self._new_sample:
                self._new_sample = False
                self._display_refresh_time = 0
                app.refresh = True
                if app.bluetooth_mgr is not None and app.bluetooth_mgr.is_connected and self._last_range_mm >= 0:
                    # send data according to which is selected for transmission (colour, range or PID output).
                    if self._plot_selection == _PLOT_SELECTION_COLOUR:
                        # send the colour sensor hue to the phone app via BLE for plotting
                        app.bluetooth_mgr.send_plotter_data([self._last_colour_hue])
                    elif self._plot_selection == _PLOT_SELECTION_RANGE:
                        # send the range sensor distance to the phone app via BLE for plotting
                        app.bluetooth_mgr.send_plotter_data([self._last_range_mm])
                    elif self._plot_selection == _PLOT_SELECTION_PID:
                        # send the PID values to the phone app via BLE for plotting
                        app.bluetooth_mgr.send_plotter_data([self._last_p_term, self._last_i_term, self._last_d_term])
                    elif self._plot_selection == _PLOT_SELECTION_POWER:
                        app.bluetooth_mgr.send_plotter_data([self.motor_output[0], self.motor_output[1]])

        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            app.performance_mode = False
            app.enable_motors(False, MOTOR_ENABLE_USER_STATE)
            sensor_mgr = app.sensor_test_mgr
            if sensor_mgr is not None and self._colour_hexdrive is not None:
                sensor_mgr.disable_colour_sensor(self._colour_hexdrive)
            self._colour_hexdrive = None
            if sensor_mgr is not None and self._range_hexdrive is not None:
                sensor_mgr.disable_range_sensor(self._range_hexdrive)
            self._range_hexdrive = None
            app.set_ring_colour(None)
            self.clear_pid()
            self._last_range_mm = -1

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
            self.toggle_direction()
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
            self.toggle_movement()
        return True


    def toggle_movement(self):
        """Toggle the line-follower start/stop movement state (CONFIRM action).
        Shared by the local CONFIRM button and remote (BLE) control."""
        self._enable_movement = not self._enable_movement
        if self._logging:
            print(f"B:LF:{'Start' if self._enable_movement else 'Stop'}")
        if not self._enable_movement:
            self.stop_movement()
        self._app.refresh = True
        self._app.performance_mode = False   # disable performance mode while adjusting settings so we can see the display update


    def toggle_direction(self):
        """Reverse the steering-gain sign (LEFT action).  Shared by the local LEFT
        button and remote (BLE) control so a remote user can match the line side."""
        self._signed_steering_gain *= -1
        self._app.notification = Notification("Direction: Reversed")
        if self._logging:
            print(f"B:LF:Steering gain sign reversed to {self._signed_steering_gain}")


    def current_led_hue(self):
        """Hue data source for the app's LED history buffer.  Returns the hue
        (0-3600, 0.1-degree units) only while the robot is actively on the line;
        otherwise returns None so the LED history renders black."""
        if self._time_since_line_detected == 0:
            return self._last_colour_hue
        return None


    def clear_pid(self):
        """Clear the PID controller state (integral and previous error)."""
        self._pid_integral = 0
        self._pid_previous_error = 0
        self._last_p_term = 0
        self._last_i_term = 0
        self._last_d_term = 0


    def stop_movement(self):
        """Stop the line follower and disable the colour sensor."""
        self.clear_pid()
        self._enable_movement = False
        self._app.performance_mode = False
        self.motor_output = (0, 0)  # Stop


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

        output = (0, 0)

        # Poll the shared range sensor if we are allowed to move
        if self._range_hexdrive:
            new_sample, range_mm = sensor_mgr.read_range(self._range_hexdrive)
            if new_sample:
                self._last_range_mm = range_mm
                if range_mm < self._min_obstacle_distance and self._enable_movement:
                    # Stop immediately if an obstacle is detected within 100mm
                    self._obstacle_detection_count += 1
                    if self._obstacle_detection_count > 2:  # require 3 consecutive detections to avoid false positives
                        if self._logging:
                            print(f"B:LF:Obstacle detected @{range_mm}mm, auto stop")
                        self.stop_movement()
                        self._app.notification = Notification("Stop Obstacle", self._range_hexdrive.config.port)
                    else:
                        # start to slow down while we wait for the next sample to confirm the obstacle is still there
                        if self._logging:
                            print(f"B:LF:Obstacle detected @{range_mm}mm, slowing down ({self._obstacle_detection_count})")
                    self.motor_output = output
                    return output
                else:
                    self._obstacle_detection_count = 0

        # Poll the shared colour sensor; read_colour also updates the ring colour on change.
        # Force a read to ensure we get the latest sample, as the colour sensor is only polled in the background by the HexDrive.
        colour_sensor = getattr(self._colour_hexdrive, "colour_sensor", None)
        if colour_sensor is not None:
            _ = colour_sensor.read()

        new_sample, hue, _, name, _raw = sensor_mgr.read_colour(self._colour_hexdrive)
        self._time_since_last_update_ms += delta
        if new_sample:
            #if self._logging:
            #    print(f"B:LF:Hue={hue//10}.{hue%10}° Name={name}")
            self._last_colour_hue = hue
            self._last_colour_name = name
            self._last_colour = _raw
            self._new_sample = True

            if self._last_colour_name == self._colour_stop:
                if self._enable_movement:
                    # Stop Immediately if the colour is the stop colour (e.g. black)
                    if self._logging:
                        print(f"B:LF:{self._colour_stop} detected, auto stop")
                    self.stop_movement()
                    self._time_since_line_detected = _MAX_TIME_WITHOUT_LINE
                    self._app.notification = Notification(f"Stop On {self._colour_stop}", self._colour_hexdrive.config.port)
            if self._last_colour_name in ("White", "Grey", "Black"):
                self._last_colour_hue = _HUE_COLOUR_UNKNOWN
                # Colours with no hue (achromatic) are treated as "no line detected" and we allow a short period to pick up the line again.
                if self._enable_movement:
                    # Allow a short period to pick up the line again if the colour is white or grey (i.e. no line detected)
                    self._time_since_line_detected += delta
                    if self._time_since_line_detected < _MAX_TIME_WITHOUT_LINE:
                        output = self.motor_output
                    else:
                        self._time_since_line_detected = _MAX_TIME_WITHOUT_LINE  # clamp to max so we don't overflow
                        self._app.performance_mode = False  # stop performance mode if we have lost the line for a while
            else:
                # We are actually seeing a colour that is not black, white or grey, so we can potentially use it to steer the robot.
                hue_difference_from_mid = _signed_hue_delta(self._last_colour_hue, self._mid_hue)
                #if self.follower_mode == _FOLLOWER_MODE_BINARY:
                #    # place holder for another algorithm that uses a binary on/off approach to steering, rather than a PID controller.
                if self.follower_mode == _FOLLOWER_MODE_DIFFERENTIAL:
                    # Use circular hue distance so values near 0/3600 wrap correctly.
                    # need setting to flip the sign of the steering input to reverse the direction of the turn if needed
                    if abs(hue_difference_from_mid) < self._max_hue:
                        steering_input = self._signed_steering_gain * hue_difference_from_mid
                        if self._enable_movement:
                            output = self.compute_differential_output(steering_input, self._time_since_last_update_ms)
                            self._time_since_last_update_ms = 0
                            self._app.performance_mode = True  # enable performance mode while we are actively following the line
                        else:
                            self._app.performance_mode = False  # disable performance mode if we are not actively following the line
                        self._time_since_line_detected = 0
                    else:
                        self._time_since_line_detected += delta
                        if self._time_since_line_detected < _MAX_TIME_WITHOUT_LINE:
                            output = self.motor_output  # continue with last output while we see a colour that is too far from the mid hue, to allow the robot to continue on its path until it finds the line again.
                        else:
                            self._time_since_line_detected = _MAX_TIME_WITHOUT_LINE  # clamp to max so we don't overflow
                            self._app.performance_mode = False  # stop performance mode if we have lost the line for a while
                            # as we are not moving we don't need to get updates as fast.
                    #print(f"B:LF:hue_diff={hue_difference_from_mid}, Output={output}")
                else:
                    # Unknown follower mode, so stop the motors
                    self._app.performance_mode = False  # disable performance mode if we are not actively following the line

            self.motor_output = output
        else:
            output = self.motor_output
        return output


    @micropython.viper
    def _compute_steering_control(self, error: int, delta: int, kp: int, ki:int, kd:int) -> int:
        """Compute steering control using a full PID controller for differential line following."""

        # Proportional term
        p_term: int = kp * error
        i_term: int = 0
        d_term: int = 0

        # Integral term - accumulate error over time with anti-windup clamping
        if ki > 0:
            # Cast object attributes to int before doing math
            integral: int = int(self._pid_integral) + error
            limit: int = int(self.integral_limit)

            # Inline _clamp logic (much faster in Viper)
            if integral < -limit:
                integral = -limit
            elif integral > limit:
                integral = limit

            self._pid_integral = integral
            i_term = ki * integral
        else:
            i_term = 0

        # Derivative term - rate of change of error
        if delta > 0:
            prev_error: int = int(self._pid_previous_error)
            d_term: int = kd * (error - prev_error) * 1000
            d_term = d_term // delta
        else:
            d_term: int = 0

        self._pid_previous_error = error
        self._last_p_term = p_term
        self._last_i_term = i_term
        self._last_d_term = d_term

        # Combined PID output
        correction: int = p_term + i_term + d_term

        # NOT IN USE AT PRESENT:
        # adjust forward power to be faster when well aligned with the line, and slower when far from the line, to avoid overshooting
        # scale the line power based on the absolute error, so that when the error is small the line power is at maximum, and when the error is large the line power is reduced to avoid overshooting
        #abs_error = abs(error)
        # scale line power linearly from max to min (25%) based on the error when too far from the line mid point, to avoid overshooting.  When the error is small, use full line power.
        #if abs_error >= (self._max_hue // 4):
        #    line_power = (self.line_power * (self._max_hue + (self._max_hue//4) - abs_error)) // self._max_hue
        #else:

        # limit the maximum correction to 1.5x the line power, so that the robot does not turn too sharply and overshoot the line.
        line_power: int = int(self.line_power)

        correction_limit: int = (3 * line_power) >> 1

        # Inline _clamp logic for correction output
        if correction < -correction_limit:
            correction = -correction_limit
        elif correction > correction_limit:
            correction = correction_limit

        return correction


    def compute_differential_output(self, error: int, delta: int) -> tuple[int, int]:
        """Compute motor output using a full PID controller for differential line following.

        Uses the difference between left and right sensor readings as the error signal,
        and applies proportional, integral, and derivative terms to compute a steering correction.
        Returns a tuple of (left_motor, right_motor) power values, clamped to max_power.
        Uses integer maths for efficiency, scaling down the PID gains and error values to avoid overflow.
        """
        correction = self._compute_steering_control(error, delta, self.kp, self.ki, self.kd)

        # Combine correction with base forward power to get output for each motor & limit output to max power
        line_power = self.line_power
        max_power = self._app.max_power
        output = (_clamp(line_power + correction, -max_power, max_power), _clamp(line_power - correction, -max_power, max_power))

        if self._logging:
            print(f"B:LF:PID:Err={error} P={self._last_p_term} I={self._last_i_term} D={self._last_d_term} Corr={correction} Out={output}")

        return output


    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw_tracker(self, ctx):
        """Draw the line follower tracker UI element."""
        # ================================================
        # draw a box to show the deviation from mid hue:
        # ================================================
        half_height = label_font_size
        half_width = 100

        # outer box shows the maximum hue deviation possible, inner box shows the maximum hue deviation used, and a line shows the current deviation position
        ctx.rgb(0.25,0.25,0.25).rectangle(-half_width, -half_height, 2 * half_width, label_font_size * 2).fill()

        # grey highlight behind the currently selected editable field
        selected_field = _EDIT_FIELDS[self._selected_field]
        hx, hy, hw, hh = selected_field[3]
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
            kp_text = f"Kp:{self.kp}"      # // _FOLLOWER_PID_SCALE_FACTOR}.{self.kp % _FOLLOWER_PID_SCALE_FACTOR:01d}"
            kd_text = f"Kd:{self.kd}"      # // _FOLLOWER_PID_SCALE_FACTOR}.{self.kd % _FOLLOWER_PID_SCALE_FACTOR:01d}"
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


    def draw(self, ctx) -> bool:
        """Render Line Follower UI.  Returns True if handled."""

        if not self._enable_movement:
            # draw the line-following tracker only when the robot is not moving, so we can see the current deviation from mid hue.
            self.draw_tracker(ctx)

            # draw the button labels - up/down adjust the selected field, left/right cycle fields
            sel_label = _EDIT_FIELDS[self._selected_field][_EDIT_FIELDS_LABEL_INDEX]
            confirm_label = "Start" if not self._enable_movement else "Stop"

            button_labels(ctx, cancel_label="Back", confirm_label=confirm_label,
                        up_label=f"+{sel_label}", down_label=f"-{sel_label}",
                        left_label="Direction", right_label="\u25B6")
        else:
            # draw the button labels - confirm to stop, left/right to reverse direction
            button_labels(ctx, confirm_label="Stop", left_label="Direction")

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
