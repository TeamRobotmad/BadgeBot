# Colour Drive Module for BadgeBot
#
# Reads a downward-facing TCS3472 colour sensor to detect coloured cards
# on the ground and translates each colour into a robot movement.  The
# robot drives over a trail of cards, reading each one in turn:
#
#   Red     = Stop (end of trail)
#   Green   = Drive forward
#   Blue    = Drive backward
#   Yellow  = Turn left
#   Magenta = Turn right
#   White   = Pause (do nothing, then read again)
#
# The sensor is expected to be on the same hexpansion slot as the
# HexDrive (i.e. shared I2C port).  Falls back to a manual port picker
# if the HexDrive port is unavailable.
#
# Public interface (called by the main app):
#   __init__(app)            – wire up to BadgeBotApp
#   start()                  – enter the colour-drive flow (from menu)
#   update(delta)            – per-tick state machine update
#   draw(ctx)                – render colour-drive-related UI
#   background_update(delta) – called from the fast background loop;
#                              returns motor output tuple or None
#   init_settings(settings)  – register colour-drive specific settings

import asyncio
from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification

from .app import DEFAULT_BACKGROUND_UPDATE_PERIOD

# ---------------------------------------------------------------------------
# Sub-state constants
# ---------------------------------------------------------------------------
_SUB_SELECT_PORT = 0   # Manual port selection (only when HexDrive port unavailable)
_SUB_READY       = 1   # Waiting for user to start
_SUB_READ        = 2   # Reading sensor, classifying colour
_SUB_DISPLAY     = 3   # Briefly showing detected colour before executing
_SUB_EXECUTE     = 4   # Performing the movement
_SUB_PAUSE       = 5   # White card — short pause then re-read
_SUB_STOPPED     = 6   # Red card or error — trail complete
_SUB_ADVANCE     = 7   # Drive forward after a turn to clear the card

# ---------------------------------------------------------------------------
# Colour names (used as dict keys and display labels)
# ---------------------------------------------------------------------------
COLOUR_RED     = "Red"
COLOUR_GREEN   = "Green"
COLOUR_BLUE    = "Blue"
COLOUR_YELLOW  = "Yellow"
COLOUR_MAGENTA = "Magenta"
COLOUR_WHITE   = "White"
COLOUR_UNKNOWN = "Unknown"

# Colour → action description (for display)
_COLOUR_ACTIONS = {
    COLOUR_RED:     "Stop",
    COLOUR_GREEN:   "Forward",
    COLOUR_BLUE:    "Backward",
    COLOUR_YELLOW:  "Turn Left",
    COLOUR_MAGENTA: "Turn Right",
    COLOUR_WHITE:   "Pause",
    COLOUR_UNKNOWN: "---",
}

# Colour → display RGB tuple (for drawing a colour swatch)
_COLOUR_RGB = {
    COLOUR_RED:     (1.0, 0.0, 0.0),
    COLOUR_GREEN:   (0.0, 1.0, 0.0),
    COLOUR_BLUE:    (0.0, 0.3, 1.0),
    COLOUR_YELLOW:  (1.0, 1.0, 0.0),
    COLOUR_MAGENTA: (1.0, 0.0, 1.0),
    COLOUR_WHITE:   (1.0, 1.0, 1.0),
    COLOUR_UNKNOWN: (0.4, 0.4, 0.4),
}

# ---------------------------------------------------------------------------
# Timing defaults
# ---------------------------------------------------------------------------
_SENSOR_READ_INTERVAL_MS = 100    # how often to poll the sensor
_DISPLAY_HOLD_MS         = 200    # how long to show the detected colour before acting
_PAUSE_MS                = 1000   # how long to pause on a white card
_TICK_MS                 = 10     # smallest motor ramp unit
_CONFIRM_READS           = 2      # consecutive identical reads required before acting
_MAX_UNKNOWN_READS       = 20     # consecutive unknowns before auto-stop (~2s at 100ms interval)

# ---------------------------------------------------------------------------
# Colour classification thresholds
# ---------------------------------------------------------------------------
_MIN_CLEAR      = 50      # minimum clear channel to attempt classification
_SAT_THRESHOLD  = 0.15    # below this saturation → white

# Default settings
_DEFAULT_CD_DRIVE_MM  = 75    # distance per forward/backward card (mm)
_DEFAULT_CD_TURN_DEG  = 90    # turn angle per left/right card (degrees)
_DEFAULT_CD_SPEED     = 34000 # motor PWM power for colour-drive movements (~26% max)


# ---- Colour classification -------------------------------------------------

def classify_colour(red, green, blue, clear):
    """Classify raw TCS3472 RGBC readings into a named colour.

    Uses a normalised-RGB to hue conversion with a saturation check
    for white detection.  Returns one of the COLOUR_* constants.

    Parameters
    ----------
    red, green, blue : int
        Raw 16-bit channel counts from the TCS3472.
    clear : int
        Raw 16-bit clear (unfiltered) channel count.

    Returns
    -------
    str
        One of COLOUR_RED, COLOUR_GREEN, COLOUR_BLUE, COLOUR_YELLOW,
        COLOUR_MAGENTA, COLOUR_WHITE, or COLOUR_UNKNOWN.
    """
    if clear < _MIN_CLEAR:
        return COLOUR_UNKNOWN

    total = red + green + blue
    if total == 0:
        return COLOUR_UNKNOWN

    # Normalise each channel to 0.0–1.0 fraction of total
    rn = red / total
    gn = green / total
    bn = blue / total

    # Saturation estimate: 1 − 3·min(rn, gn, bn)
    # When all channels are equal, saturation = 0 (white/grey).
    # When one channel dominates, saturation approaches 1.
    sat = 1.0 - 3.0 * min(rn, gn, bn)

    if sat < _SAT_THRESHOLD:
        return COLOUR_WHITE

    # Hue calculation (0–360°) from normalised RGB.
    # Simplified version of the standard RGB→HSV hue formula.
    max_c = max(rn, gn, bn)
    min_c = min(rn, gn, bn)
    delta = max_c - min_c

    if delta < 0.001:
        return COLOUR_WHITE  # effectively grey

    if max_c == rn:
        hue = 60.0 * (((gn - bn) / delta) % 6.0)
    elif max_c == gn:
        hue = 60.0 * (((bn - rn) / delta) + 2.0)
    else:
        hue = 60.0 * (((rn - gn) / delta) + 4.0)

    if hue < 0:
        hue += 360.0

    # Map hue ranges to colour names.
    # These ranges are tuned for common coloured card under indoor lighting.
    # Adjustments may be needed for different lighting conditions.
    if hue < 25 or hue >= 340:
        return COLOUR_RED
    elif hue < 70:
        return COLOUR_YELLOW
    elif hue < 165:
        return COLOUR_GREEN
    elif hue < 260:
        return COLOUR_BLUE
    elif hue < 340:
        return COLOUR_MAGENTA

    return COLOUR_UNKNOWN  # should not reach here


# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting: type):
    """Register colour-drive-specific settings in the shared settings dict."""
    s['cd_drive_mm']  = MySetting(s, _DEFAULT_CD_DRIVE_MM,  10, 1000)
    s['cd_turn_deg']  = MySetting(s, _DEFAULT_CD_TURN_DEG,  5,   360)
    s['cd_speed']     = MySetting(s, _DEFAULT_CD_SPEED,     1000, 65535)


# ---- Colour Drive manager --------------------------------------------------

class ColourDriveMgr:
    """Manages the Colour Drive workflow — reading coloured cards and
    translating them into robot movements.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """

    def __init__(self, app):
        self.app = app
        self._sub_state: int = _SUB_READY
        self._port_selected: int = 1
        self._colour: str = COLOUR_UNKNOWN
        self._action: str = ""
        self._raw_rgbc: tuple = (0, 0, 0, 0)  # (R, G, B, C) for display
        self._hue: float = 0.0                 # for debug display
        self._sat: float = 0.0                 # for debug display
        self._read_timer: int = 0
        self._display_timer: int = 0
        self._pause_timer: int = 0
        self._execute_timer: int = 0
        self._card_count: int = 0              # number of cards read so far
        self._confirm_colour: str = COLOUR_UNKNOWN  # colour being confirmed
        self._confirm_count: int = 0                # consecutive matching reads
        self._unknown_count: int = 0                # consecutive UNKNOWN reads
        self._last_acted: str = COLOUR_UNKNOWN      # last colour we acted on
        self._seen_different: bool = True            # seen a different reading since last action
        self._was_turn: bool = False            # True when EXECUTE is a turn
        self._mc_task = None                   # async task for motor controller
        self._active: bool = False
        self._sensor_selected: bool = False    # True once TCS3472 is selected
        self.target_output: tuple = (0, 0)
        self.motor_output: tuple = (0, 0)

    def _log(self, msg: str):
        """Print a diagnostic message when the logging setting is enabled."""
        if self.app.settings['logging'].v:
            print(f"CD:{msg}")

    # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Enter the Colour Drive flow from the main menu."""
        app = self.app

        # Try to open sensor on HexDrive port first
        sensor_open = False
        sensor_test = app.sensor_test_mgr

        if sensor_test.sensor_mgr is not None and sensor_test.sensor_mgr.is_open:
            sensor_open = True
        else:
            ports_to_try = []
            if app.hexdrive_port is not None:
                ports_to_try.append(app.hexdrive_port)
            if sensor_test.port_selected not in ports_to_try:
                ports_to_try.append(sensor_test.port_selected)
            for probe_port in ports_to_try:
                if sensor_test.open_sensor_port(probe_port):
                    sensor_test.port_selected = probe_port
                    sensor_open = True
                    break

        if not sensor_open:
            # Fall back to manual port selection
            app.set_menu(None)
            app.button_states.clear()
            app.refresh = True
            self._sub_state = _SUB_SELECT_PORT
            self._port_selected = app.hexdrive_port if app.hexdrive_port is not None else 1
            return True

        # Try to select the TCS3472 sensor
        self._sensor_selected = self._select_tcs3472()
        if not self._sensor_selected:
            app.notification = Notification("  No TCS3472\n  Sensor")
            return False

        # Sensor is ready — enter the ready sub-state
        app.set_menu(None)
        app.button_states.clear()
        app.refresh = True
        self._sub_state = _SUB_READY
        self._card_count = 0
        self._colour = COLOUR_UNKNOWN
        self._action = ""
        self._active = False
        self._log(f"start: sensor={self._sensor_selected} port={app.sensor_test_mgr.port_selected}")
        return True

    # ------------------------------------------------------------------
    # Internal helpers — sensor
    # ------------------------------------------------------------------

    def _select_tcs3472(self) -> bool:
        """Ensure the TCS3472 is the currently selected sensor.
        Returns True if found and selected."""
        sm = self.app.sensor_test_mgr.sensor_mgr
        if sm is None or not sm.is_open:
            return False
        return sm.select_sensor("TCS3472")

    def _read_colour(self) -> str:
        """Read the TCS3472 via the shared SensorManager and classify."""
        sm = self.app.sensor_test_mgr.sensor_mgr
        if sm is None or not sm.is_open:
            return COLOUR_UNKNOWN
        try:
            data = sm.read_current()
            r = int(data.get("red", "0"))
            g = int(data.get("green", "0"))
            b = int(data.get("blue", "0"))
            c = int(data.get("clear", "0"))
            self._raw_rgbc = (r, g, b, c)

            # Compute hue/sat for debug display
            total = r + g + b
            rn = gn = bn = 0.0
            if total > 0 and c >= _MIN_CLEAR:
                rn = r / total
                gn = g / total
                bn = b / total
                self._sat = 1.0 - 3.0 * min(rn, gn, bn)
                max_c = max(rn, gn, bn)
                min_c = min(rn, gn, bn)
                delta = max_c - min_c
                if delta >= 0.001:
                    if max_c == rn:
                        self._hue = 60.0 * (((gn - bn) / delta) % 6.0)
                    elif max_c == gn:
                        self._hue = 60.0 * (((bn - rn) / delta) + 2.0)
                    else:
                        self._hue = 60.0 * (((rn - gn) / delta) + 4.0)
                    if self._hue < 0:
                        self._hue += 360.0
                else:
                    self._hue = 0.0
            else:
                self._hue = 0.0
                self._sat = 0.0

            colour = classify_colour(r, g, b, c)
            self._log(f"read RGBC=({r},{g},{b},{c}) nRGB=({rn:.2f},{gn:.2f},{bn:.2f}) H={self._hue:.0f} S={self._sat:.2f} -> {colour}")
            return colour
        except Exception as e:  # pylint: disable=broad-exception-caught
            self._log(f"read error: {e}")
            return COLOUR_UNKNOWN

    # ------------------------------------------------------------------
    # Internal helpers — motor control
    # ------------------------------------------------------------------

    def _start_motors(self):
        """Power on the HexDrive motors and suppress PWM logging."""
        if self.app.hexdrive_app is not None:
            self.app.hexdrive_app.set_logging(False)
            self.app.hexdrive_app.set_power(True)
        self._log("motors ON")

    def _stop_motors(self):
        """Zero motor output immediately."""
        self.target_output = (0, 0)
        self.motor_output = (0, 0)
        if self.app.hexdrive_app is not None:
            self.app.hexdrive_app.set_motors((0, 0))
        self._log("motors STOP")

    def _begin_forward(self):
        """Drive forward by cd_drive_mm using the MotorController's
        accelerometer-aided distance drive, with timed fallback."""
        mc = self.app.motor_controller
        drive_mm = int(self.app.settings['cd_drive_mm'].v)
        if mc is not None:
            self._log(f"FWD {drive_mm}mm via MC")
            self._mc_task = asyncio.get_event_loop().create_task(
                mc.forward_mm(drive_mm)
            )
        else:
            self._log(f"FWD {drive_mm}mm timed fallback")
            speed = int(self.app.settings['cd_speed'].v)
            self.target_output = (speed, speed)
        self._execute_timer = 0

    def _begin_backward(self):
        """Drive backward by cd_drive_mm using the MotorController's
        accelerometer-aided distance drive, with timed fallback."""
        mc = self.app.motor_controller
        drive_mm = int(self.app.settings['cd_drive_mm'].v)
        if mc is not None:
            self._log(f"BWD {drive_mm}mm via MC")
            self._mc_task = asyncio.get_event_loop().create_task(
                mc.backward_mm(drive_mm)
            )
        else:
            self._log(f"BWD {drive_mm}mm timed fallback")
            speed = int(self.app.settings['cd_speed'].v)
            self.target_output = (-speed, -speed)
        self._execute_timer = 0

    def _begin_turn_left(self):
        """Turn left (CCW) by cd_turn_deg using the MotorController's
        gyro-aided turn, with timed differential drive fallback."""
        mc = self.app.motor_controller
        turn_deg = int(self.app.settings['cd_turn_deg'].v)
        if mc is not None:
            self._log(f"TURN LEFT {turn_deg}deg via MC")
            self._mc_task = asyncio.get_event_loop().create_task(
                mc.turn(-turn_deg)
            )
        else:
            self._log(f"TURN LEFT {turn_deg}deg timed fallback")
            speed = int(self.app.settings['cd_speed'].v)
            self.target_output = (-speed, speed)
        self._execute_timer = 0

    def _begin_turn_right(self):
        """Turn right (CW) by cd_turn_deg using the MotorController's
        gyro-aided turn, with timed differential drive fallback."""
        mc = self.app.motor_controller
        turn_deg = int(self.app.settings['cd_turn_deg'].v)
        if mc is not None:
            self._log(f"TURN RIGHT {turn_deg}deg via MC")
            self._mc_task = asyncio.get_event_loop().create_task(
                mc.turn(turn_deg)
            )
        else:
            self._log(f"TURN RIGHT {turn_deg}deg timed fallback")
            speed = int(self.app.settings['cd_speed'].v)
            self.target_output = (speed, -speed)
        self._execute_timer = 0

    def _is_execute_done(self, delta: int) -> bool:
        """Check if the current movement is complete."""
        if self._mc_task is not None:
            if self._mc_task.done():
                self._mc_task = None
                return True
            return False
        # Timed fallback (only used when MotorController is unavailable)
        self._execute_timer += delta
        # rough fallback: drive_mm as ms, or turn_deg * 10 as ms
        fallback_ms = max(int(self.app.settings['cd_drive_mm'].v),
                          int(self.app.settings['cd_turn_deg'].v) * 10)
        return self._execute_timer >= fallback_ms

    @staticmethod
    def _slew(current: int, target: int, step: int) -> int:
        """Slew-rate limiter for smooth motor ramping."""
        if current < target:
            return min(current + step, target)
        if current > target:
            return max(current - step, target)
        return current

    def _apply_output_ramp(self, delta: int):
        """Smoothly ramp motor output towards target."""
        accel = max(1, int(self.app.settings['acceleration'].v))
        ticks = max(1, delta // _TICK_MS)
        step = accel * ticks
        max_power = int(self.app.settings['max_power'].v)

        target_l = max(-max_power, min(max_power, int(self.target_output[0])))
        target_r = max(-max_power, min(max_power, int(self.target_output[1])))
        cur_l = int(self.motor_output[0])
        cur_r = int(self.motor_output[1])

        self.motor_output = (
            self._slew(cur_l, target_l, step),
            self._slew(cur_r, target_r, step),
        )

    # ------------------------------------------------------------------
    # Public interface — called by BadgeBotApp dispatch tables
    # ------------------------------------------------------------------

    def update(self, delta: int):
        """Main update tick, called from BadgeBotApp.update when in STATE_COLOUR_DRIVE."""
        # CANCEL always exits cleanly
        if self.app.button_states.get(BUTTON_TYPES["CANCEL"]):
            self.app.button_states.clear()
            self._shutdown()
            self.app.return_to_menu()
            return

        if self._sub_state == _SUB_SELECT_PORT:
            self._update_select_port(delta)
        elif self._sub_state == _SUB_READY:
            self._update_ready(delta)
        elif self._sub_state == _SUB_READ:
            self._update_read(delta)
        elif self._sub_state == _SUB_DISPLAY:
            self._update_display(delta)
        elif self._sub_state == _SUB_EXECUTE:
            self._update_execute(delta)
        elif self._sub_state == _SUB_ADVANCE:
            self._update_advance(delta)
        elif self._sub_state == _SUB_PAUSE:
            self._update_pause(delta)
        elif self._sub_state == _SUB_STOPPED:
            self._update_stopped(delta)

        # Ramp motors smoothly (only for direct-drive phases, not MC tasks)
        if self._active and self._mc_task is None:
            self._apply_output_ramp(delta)

    def draw(self, ctx):
        """Render the Colour Drive UI."""
        if self._sub_state == _SUB_SELECT_PORT:
            self._draw_select_port(ctx)
        elif self._sub_state == _SUB_READY:
            self._draw_ready(ctx)
        elif self._sub_state == _SUB_STOPPED:
            self._draw_stopped(ctx)
        else:
            self._draw_running(ctx)

    def background_update(self, delta: int):       # pylint: disable=unused-argument
        """Feed motors from the background loop so the HexDrive watchdog stays happy.
        When the MotorController is executing a task it manages motors directly,
        so we return None.  Only provide direct output for the forward-while-reading
        phase (no MC task) or the timed fallback path."""
        if not self._active:
            return None
        if self._mc_task is not None:
            return None  # MotorController manages motors directly
        return self.motor_output

    # ------------------------------------------------------------------
    # Sub-state: SELECT_PORT
    # ------------------------------------------------------------------

    def _update_select_port(self, delta: int):     # pylint: disable=unused-argument
        app = self.app
        if app.button_states.get(BUTTON_TYPES["RIGHT"]):
            app.button_states.clear()
            self._port_selected = (self._port_selected % 6) + 1
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            app.button_states.clear()
            self._port_selected = ((self._port_selected - 2) % 6) + 1
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            sensor_test = app.sensor_test_mgr
            if sensor_test.open_sensor_port(self._port_selected):
                sensor_test.port_selected = self._port_selected
                self._sensor_selected = self._select_tcs3472()
                if self._sensor_selected:
                    self._sub_state = _SUB_READY
                    self._card_count = 0
                    self._colour = COLOUR_UNKNOWN
                    self._action = ""
                    self._active = False
                    app.refresh = True
                else:
                    app.notification = Notification("  No TCS3472\n  on port")
            else:
                app.notification = Notification("  No sensors\n  on port")

    def _draw_select_port(self, ctx):
        self.app.draw_message(
            ctx,
            ["Colour Drive", f"Sensor Port: {self._port_selected}"],
            [(1, 1, 1), (0, 1, 1)],
            label_font_size,
        )
        button_labels(ctx, left_label="<Port", right_label="Port>",
                      confirm_label="Scan", cancel_label="Back")

    # ------------------------------------------------------------------
    # Sub-state: READY
    # ------------------------------------------------------------------

    def _update_ready(self, delta: int):           # pylint: disable=unused-argument
        app = self.app
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            # Start the colour-drive run
            self._start_motors()
            self._active = True
            self._card_count = 0
            self._colour = COLOUR_UNKNOWN
            self._read_timer = 0
            app.update_period = 10
            self._enter_read()
            app.refresh = True

    def _draw_ready(self, ctx):
        lines = [
            "Colour Drive",
            "Place robot on",
            "first card then",
            "press Confirm",
        ]
        colours = [(1, 1, 1), (0.8, 0.8, 0.8), (0.8, 0.8, 0.8), (0, 1, 0)]
        self.app.draw_message(ctx, lines, colours, label_font_size)
        button_labels(ctx, confirm_label="Start", cancel_label="Back")

    # ------------------------------------------------------------------
    # Sub-state: READ
    # ------------------------------------------------------------------

    def _enter_read(self):
        """Transition into the READ sub-state — keep driving forward while
        reading, since the sensor faces down and the robot drives over cards."""
        self._sub_state = _SUB_READ
        self._read_timer = 0
        self._confirm_colour = COLOUR_UNKNOWN
        self._confirm_count = 0
        # Maintain forward drive while reading — don't stop between cards
        speed = int(self.app.settings['cd_speed'].v)
        self.target_output = (speed, speed)
        self._log("-> READ (fwd)")

    def _update_read(self, delta: int):
        self._read_timer += delta
        if self._read_timer >= _SENSOR_READ_INTERVAL_MS:
            self._read_timer = 0
            colour = self._read_colour()
            self._action = _COLOUR_ACTIONS.get(colour, "---")
            self.app.refresh = True

            if colour == COLOUR_UNKNOWN:
                # Reset confirmation streak on unknown
                self._confirm_colour = COLOUR_UNKNOWN
                self._confirm_count = 0
                self._seen_different = True
                self._unknown_count += 1
                if self._unknown_count >= _MAX_UNKNOWN_READS:
                    self._log(f"auto-stop: {self._unknown_count} consecutive unknowns")
                    self._stop_motors()
                    self._enter_stopped("Lost track (no card)")
                return

            # Reset unknown counter on any valid colour
            self._unknown_count = 0

            # Red = emergency stop — act on a single read, no confirmation
            if colour == COLOUR_RED:
                self._colour = COLOUR_RED
                self._card_count += 1
                self._action = _COLOUR_ACTIONS[COLOUR_RED]
                self._log(f"ACTION colour=Red card#{self._card_count} (immediate)")
                self._stop_motors()
                self._enter_stopped("Trail complete")
                return

            # Last-card filter: after acting on a colour, ignore that same
            # colour until we've seen at least one different reading.
            # This prevents the floor (which may read similarly to the last
            # card) from re-triggering the same action.
            if colour != self._last_acted:
                self._seen_different = True
            if colour == self._last_acted and not self._seen_different:
                self._log(f"skip {colour} (same as last action, waiting for different)")
                self._confirm_colour = COLOUR_UNKNOWN
                self._confirm_count = 0
                return

            # Confirmation: require _CONFIRM_READS consecutive identical
            # reads to filter out transient edge reads between cards.
            if colour == self._confirm_colour:
                self._confirm_count += 1
            else:
                self._confirm_colour = colour
                self._confirm_count = 1
                self._log(f"confirm {colour} ({self._confirm_count}/{_CONFIRM_READS})")
                return

            self._log(f"confirm {colour} ({self._confirm_count}/{_CONFIRM_READS})")

            if self._confirm_count < _CONFIRM_READS:
                return

            # Confirmed — accept
            self._colour = colour
            self._card_count += 1
            self._last_acted = colour
            self._seen_different = False

            # Show briefly what we detected, then act
            self._sub_state = _SUB_DISPLAY
            self._display_timer = 0

    def _draw_read(self, ctx):
        """Draw the reading-in-progress screen."""
        r, g, b, c = self._raw_rgbc
        lines = [
            "Reading...",
            f"R:{r} G:{g} B:{b}",
            f"C:{c} H:{self._hue:.0f} S:{self._sat:.2f}",
        ]
        colours = [(1, 1, 1), (1, 1, 0), (0.7, 0.7, 0.7)]
        self.app.draw_message(ctx, lines, colours, label_font_size)
        button_labels(ctx, cancel_label="Stop")

    # ------------------------------------------------------------------
    # Sub-state: DISPLAY — briefly show what was detected
    # ------------------------------------------------------------------

    def _update_display(self, delta: int):
        self._display_timer += delta
        self.app.refresh = True
        if self._display_timer >= _DISPLAY_HOLD_MS:
            # Transition to the appropriate action
            self._log(f"ACTION colour={self._colour} card#{self._card_count}")
            if self._colour == COLOUR_RED:
                self._enter_stopped("Trail complete")
            elif self._colour == COLOUR_WHITE:
                self._stop_motors()
                self._sub_state = _SUB_PAUSE
                self._pause_timer = 0
                self._log("-> PAUSE")
            elif self._colour == COLOUR_GREEN:
                # Green = keep driving forward — no stop, go straight to read
                self._enter_read()
            elif self._colour == COLOUR_BLUE:
                self._stop_motors()
                self._sub_state = _SUB_EXECUTE
                self._was_turn = False
                self._begin_backward()
            elif self._colour == COLOUR_YELLOW:
                self._stop_motors()
                self._sub_state = _SUB_EXECUTE
                self._was_turn = True
                self._begin_turn_left()
            elif self._colour == COLOUR_MAGENTA:
                self._stop_motors()
                self._sub_state = _SUB_EXECUTE
                self._was_turn = True
                self._begin_turn_right()
            else:
                # Unknown — re-read
                self._log("UNKNOWN -> re-read")
                self._enter_read()

    # ------------------------------------------------------------------
    # Sub-state: EXECUTE — performing the movement
    # ------------------------------------------------------------------

    def _update_execute(self, delta: int):
        if self._is_execute_done(delta):
            if self._was_turn:
                # After a turn, drive forward to clear the card
                self._was_turn = False
                self._sub_state = _SUB_ADVANCE
                self._log("EXECUTE done (turn) -> ADVANCE")
                self._begin_forward()
                self.app.refresh = True
            else:
                # Movement complete — resume forward drive and read next card
                self._log("EXECUTE done -> READ")
                self._enter_read()
                self.app.refresh = True

    def _update_advance(self, delta: int):
        """After a turn, drive forward to clear the card, then resume reading."""
        if self._is_execute_done(delta):
            self._log("ADVANCE done -> READ")
            self._enter_read()
            self.app.refresh = True

    # ------------------------------------------------------------------
    # Sub-state: PAUSE — white card
    # ------------------------------------------------------------------

    def _update_pause(self, delta: int):
        self._pause_timer += delta
        if self._pause_timer >= _PAUSE_MS:
            self._enter_read()
            self.app.refresh = True

    # ------------------------------------------------------------------
    # Sub-state: STOPPED — red card or end
    # ------------------------------------------------------------------

    def _enter_stopped(self, reason: str):
        self._sub_state = _SUB_STOPPED
        self._stop_motors()
        self._active = False
        if self.app.hexdrive_app is not None:
            self.app.hexdrive_app.set_logging(True)  # restore PWM logging
            self.app.hexdrive_app.set_power(False)
        self._action = reason
        self._log(f"STOPPED: {reason} after {self._card_count} cards")
        self.app.refresh = True

    def _update_stopped(self, delta: int):         # pylint: disable=unused-argument
        if self.app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            self.app.button_states.clear()
            # Restart
            self._start_motors()
            self._active = True
            self._card_count = 0
            self._colour = COLOUR_UNKNOWN
            self._read_timer = 0
            self.app.update_period = 10
            self._enter_read()
            self.app.refresh = True

    def _draw_stopped(self, ctx):
        lines = [
            "Colour Drive",
            self._action,
            f"Cards: {self._card_count}",
            "Confirm=Restart",
        ]
        colours = [(1, 1, 1), (1, 0.3, 0.3), (1, 1, 0), (0, 1, 0)]
        self.app.draw_message(ctx, lines, colours, label_font_size)
        button_labels(ctx, confirm_label="Restart", cancel_label="Back")

    # ------------------------------------------------------------------
    # Shared draw for running states (READ, DISPLAY, EXECUTE, PAUSE)
    # ------------------------------------------------------------------

    def _draw_running(self, ctx):
        """Draw the main running UI showing detected colour and action."""
        r, g, b, c = self._raw_rgbc
        colour_rgb = _COLOUR_RGB.get(self._colour, (0.4, 0.4, 0.4))
        sub_labels = {
            _SUB_READ:    "Reading...",
            _SUB_DISPLAY: f"Detected: {self._colour}",
            _SUB_EXECUTE: f"{self._action}...",
            _SUB_ADVANCE: "Advancing...",
            _SUB_PAUSE:   "Pausing...",
        }
        status = sub_labels.get(self._sub_state, "")

        lines = [
            "Colour Drive",
            status,
            f"R:{r} G:{g} B:{b} C:{c}",
            f"H:{self._hue:.0f} S:{self._sat:.2f}",
            f"Cards: {self._card_count}",
        ]
        colours = [
            (1, 1, 1),
            colour_rgb,
            (1, 1, 0),
            (0.7, 0.7, 0.7),
            (0, 1, 1),
        ]
        self.app.draw_message(ctx, lines, colours, label_font_size)
        button_labels(ctx, cancel_label="Stop")

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def _shutdown(self):
        """Clean shutdown — zero motors, restore logging, and cut power."""
        if self._mc_task is not None:
            self._mc_task.cancel()
            self._mc_task = None
        self._stop_motors()
        self._active = False
        if self.app.hexdrive_app is not None:
            self.app.hexdrive_app.set_logging(True)  # restore PWM logging
            self.app.hexdrive_app.set_power(False)
        self._log(f"shutdown after {self._card_count} cards")
        self.app.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
