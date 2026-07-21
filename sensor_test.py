"""Sensor Test Module for BadgeBot"""
#
# Manages the I2C sensor test feature.  Handles port selection, sensor
# scanning, live reading display, and sensor switching.
#
# Public interface (called by the main app):
#   __init__(app)            – wire up to BadgeBotApp
#   start()                  – enter the sensor-test flow (from menu)
#   update(delta)            – per-tick state machine update
#   draw(ctx)                – render sensor-test-related UI
#   init_settings(settings)  – register sensor-test specific settings (none currently)

import time
from system.eventbus import eventbus
from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification
import settings as platform_settings
import micropython

from .app import (SETTINGS_NAME_PREFIX, DEFAULT_BACKGROUND_UPDATE_PERIOD)
#from .diagnostics import output as diagnostics_output

try:
    from machine import mem32, disable_irq, enable_irq
except ImportError:
    class _Mem32Shim:
        def __getitem__(self, _addr: int) -> int:
            return 0

        def __setitem__(self, _addr: int, _value: int) -> None:
            return None

    # Simulator fallback: keep imports working even when direct register access
    # and IRQ controls are not exposed by the simulated machine module.
    mem32 = _Mem32Shim()

    def disable_irq() -> int:
        """Disable interrupts and return previous state (if supported)."""
        # No-op in simulator fallback.
        return 0

    def enable_irq(_state: int) -> None:
        """Restore interrupts to the given state (if supported)."""
        # No-op in simulator fallback.
        _ = _state
        return None


try:
    from micropython import const
except ImportError:
    # CPython / simulator fallback – const() is just an identity function
    # on MicroPython; replicate that so module-level const() calls work.
    const = lambda x: x         #pylint: disable=unnecessary-lambda-assignment

_TIME_SLEEP_MS = getattr(time, "sleep_ms", None)

def _sleep_ms(delay_ms: int) -> None:
    if _TIME_SLEEP_MS is not None:
        _TIME_SLEEP_MS(delay_ms)
        return
    time.sleep(delay_ms / 1000)     #pylance:disable=reportAttributeIssue


_SENSOR_COLOUR = "Colour"
_SENSOR_RANGE  = "Range"
_COLOUR_BLACK  = "Black"
_COLOUR_WHITE  = "White"
_COLOUR_RED    = "Red"
_COLOUR_GREEN  = "Green"
_COLOUR_BLUE   = "Blue"
_COLOUR_YELLOW = "Yellow"
_COLOUR_CYAN   = "Cyan"
_COLOUR_MAGENTA= "Magenta"
_COLOUR_ORANGE = "Orange"
_COLOUR_GRAY   = "Gray"

# Constants
_SLOTS = const(6)

# Local sub-states (internal to Sensor Test)
_SUB_SELECT_PORT = const(0)
_SUB_READING     = const(1)


# Pages of information to show for each sensor (can be switched with up/down buttons)
_PAGE_RAW = const(0)
_PAGE_STATS = const(1)
_PAGE_DATA = const(2)
_PAGE_CAL = const(3)
_PAGE_NAMES = {
    _PAGE_RAW: "Raw",
    _PAGE_STATS: "Stats",
    _PAGE_DATA: "Data",
    _PAGE_CAL: "Cal",
}

_DEFAULT_MIN_UPDATE_PERIOD_MS = const(200)  # Minimum update period in milliseconds to avoid overwhelming the display with too many updates per second
_READING_BACKGROUND_UPDATE_PERIOD_MS = const(20)  # Background update period in milliseconds for polling the sensors for new readings (if not using events)

class SensorEntry:
    """Represents a sensor entry in the sensor list."""
    __slots__ = ("name", "sensor_type", "stats")

    def __init__(self, name: str, sensor_type: str, stats: "SensorStats"):
        self.name = name
        self.sensor_type = sensor_type
        self.stats = stats

_COLOUR_CALIBRATION_PREFIX = "stc"

# Colour Test Cards
_COLOUR_TEST_CARDS = [_COLOUR_BLACK, _COLOUR_WHITE, _COLOUR_RED, _COLOUR_GREEN, _COLOUR_BLUE, _COLOUR_YELLOW, _COLOUR_CYAN, _COLOUR_MAGENTA, _COLOUR_ORANGE, _COLOUR_GRAY]

# RGB float tuples (0.0-1.0) for each named colour, used for the ring and text colouring.
# "Orange" and "Gray" are included as lookup_colour_RGB can return them.
_COLOUR_CARD_RGB = {
    _COLOUR_BLACK:   (0.0,  0.0,  0.0),
    _COLOUR_WHITE:   (1.0,  1.0,  1.0),
    _COLOUR_RED:     (1.0,  0.0,  0.0),
    _COLOUR_GREEN:   (0.0,  1.0,  0.0),
    _COLOUR_BLUE:    (0.0,  0.0,  1.0),
    _COLOUR_YELLOW:  (1.0,  1.0,  0.0),
    _COLOUR_CYAN:    (0.0,  1.0,  1.0),
    _COLOUR_MAGENTA: (1.0,  0.0,  1.0),
    _COLOUR_ORANGE:  (1.0,  0.5,  0.0),
    _COLOUR_GRAY:    (0.5,  0.5,  0.5),
}


# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting: type):       # pylint: disable=unused-argument, invalid-name
    """Register sensor-test-specific settings in the shared settings dict."""
    # No settings currently, but this is where they would be registered if needed.


# ---- Sensor Test manager ---------------------------------------------------

class SensorTestMgr:
    """Manages the Sensor Test workflow.
    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """
    __slots__ = ("_app", "_sub_state", "_last_sub_state", "_port_selected", "_new_sample", "_use_events", "_update_timer",
                 "_min_update_period_ms", "_hexdrive_app", "_sensor_selected", "_sensor_type", "_sensor_name",
                 "_display_data", "_page_selected", "_page_count", "_test_card", "_logging", "_draw_stats",
                 "_last_range", "_last_colour", "_last_colour_name", "_last_colour_hue", "_display_colour", "_range_sensor_stats",
                 "_colour_sensor_stats", "_range_sensor", "_colour_sensor", "_sensor_list",
                 "_last_colour_sequence", "_last_range_sequence")

    def __init__(self, app, logging: bool = False):
        self._app = app
        self._sub_state: int = _SUB_SELECT_PORT
        self._last_sub_state: int | None = None
        self._port_selected: int = 1
        self._new_sample: bool = False
        self._use_events: bool = False  # NOT using events - they proved to limit the update rate too much, so we poll the sensors instead.
        self._update_timer: int = 0
        self._min_update_period_ms: int = _DEFAULT_MIN_UPDATE_PERIOD_MS  # Minimum update period in milliseconds
        self._hexdrive_app = None
        self._sensor_selected: int = 0
        self._sensor_type: str = "None"
        self._sensor_name: str = "None"
        self._display_data: dict = {}
        self._page_selected: int = _PAGE_RAW
        self._page_count: int = 3
        self._test_card: str = _COLOUR_TEST_CARDS[0] # default to black test card
        self._logging: bool = logging
        self._draw_stats = SensorStats("Draw")

        # Range sensor specifics
        self._last_range: int | None = None

        # Colour sensor specifics
        self._last_colour: tuple[int, int, int, int] | None = None
        self._last_colour_name: str = "unknown"
        self._last_colour_hue: int = 0
        self._display_colour: tuple[float, float, float] = (1.0, 1.0, 0.0)  # default to yellow for non-colour sensors
        self._range_sensor_stats = SensorStats(_SENSOR_RANGE)
        self._colour_sensor_stats = SensorStats(_SENSOR_COLOUR)
        self._range_sensor = None
        self._colour_sensor = None
        self._last_range_sequence: int = 0
        self._last_colour_sequence: int = 0

        # Ultimately this list needs to be populated dynamically based on the sensors detected on the selected HexDrive, but for now we hardcode the known sensor types.
        self._sensor_list: list[SensorEntry] = [
            SensorEntry("VL53L0X", _SENSOR_RANGE, self._range_sensor_stats),
            SensorEntry("OPT4060", _SENSOR_COLOUR, self._colour_sensor_stats),
        ]
        if self._logging:
            print("SensorTestMgr initialised")


    # ------------------------------------------------------------------

    @property
    def logging(self) -> bool:
        """Whether to print debug logs to the console."""
        return self._logging


    @logging.setter
    def logging(self, value: bool):
        """Set whether to print debug logs to the console."""
        self._logging = value


    @property
    def colour(self) -> tuple:
        """Currently detected colour as an (r, g, b) tuple with values in the 0.0-1.0 range.
        Suitable for use in the ctx display drawing functions."""
        return self._display_colour


    @colour.setter
    def colour(self, value: tuple):
        self._display_colour = value


    @property
    def num_sensors(self) -> int:
        """Return the number of sensors detected on the selected HexDrive."""
        if self._hexdrive_app is None:
            return 0
        return len(self._sensor_list)


    # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Enter the Sensor Test flow from the main menu."""
        app = self._app
        app.set_menu(None)
        app.button_states.clear()
        app.refresh = True

        self._display_data = {}
        self._last_colour = None
        self._last_colour_name = "unknown"
        self._last_colour_hue = 0
        self._last_range = None
        self.colour = (1.0, 1.0, 0.0)  # reset to yellow when starting sensor test
        self._new_sample = False
        self._update_timer = self._min_update_period_ms # so that the first reading is displayed immediately

        num_hexdrives = len(app.hexdrive_apps)
        if num_hexdrives == 0:
            app.notification = Notification("No HexDrives", port=0)
            return False
        elif num_hexdrives == 1:
            self._hexdrive_app = app.hexdrive_apps[0]
            self._port_selected = self._hexdrive_app.config.port
            self._setup_for_sensor_type()
            self._sub_state = _SUB_READING
            return True
        self._sub_state = _SUB_SELECT_PORT

        if not self._use_events:
            # Polling the sensors for new readings in the background (if not using events)
            app.update_period = _READING_BACKGROUND_UPDATE_PERIOD_MS

        return True


    # ------------------------------------------------------------------
    # Background update (called from the fast loop)
    # ------------------------------------------------------------------

    def background_update(self, delta) -> tuple[int, int] | None:  # pylint: disable=unused-argument
        """Perform background updates based on the current sub-state."""
        if self._sub_state == _SUB_READING:
            if self._hexdrive_app is not None:
                # Poll the sensors for new readings
                if self._sensor_type == _SENSOR_RANGE:
                    range_sensor = getattr(self._hexdrive_app, "range_sensor", None)
                    if range_sensor is not None:
                        s = range_sensor.sequence
                        if s != self._last_range_sequence:
                            # If so, update the display values and stats.
                            self._last_range_sequence = s
                            self._last_range = range_sensor.range
                            self._new_sample = True
                            self._range_sensor_stats.new_sample(s)
                elif self._sensor_type == _SENSOR_COLOUR:
                    colour_sensor = getattr(self._hexdrive_app, "colour_sensor", None)
                    if colour_sensor is not None:
                        s = colour_sensor.sequence
                        if s != self._last_colour_sequence:
                            # If so, update the display values and stats.
                            self._last_colour_sequence = s
                            self._last_colour = colour_sensor.colour
                            colour_name = colour_sensor.colour_name
                            self._last_colour_hue = colour_sensor.colour_hue
                            if colour_name != self._last_colour_name:
                                self._last_colour_name = colour_name
                                self._app.set_ring_colour(_COLOUR_CARD_RGB.get(self._last_colour_name, (0.5, 0.5, 0.5)))
                            self._new_sample = True
                            self._colour_sensor_stats.new_sample(s)
        return None


    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta: int):
        """Handle Sensor Test states."""
        if self._draw_stats.update(delta):
            self._app.refresh = True
        if self._sub_state == _SUB_SELECT_PORT:
            self._update_select_port(delta)
        elif self._sub_state == _SUB_READING:
            self._update_reading(delta)
        # diagnostics if the sub-state changes
        if self._sub_state != self._last_sub_state:
            if self.logging and self._last_sub_state is not None:
                print(f"B:Sub-state changed from {self._last_sub_state} to {self._sub_state}")
            self._last_sub_state = self._sub_state


    def _setup_for_sensor_type(self):
        # Reset all sensor and display data when starting to read a new sensor
        if self._hexdrive_app is None or self._sensor_selected >= len(self._sensor_list):
            return
        self._display_data = {}
        self._update_timer = self._min_update_period_ms
        self.colour = (1.0, 1.0, 0.0)  # reset to yellow when switching sensors
        # Which Sensor is selected
        selected_sensor = self._sensor_list[self._sensor_selected]
        self._sensor_type = selected_sensor.sensor_type
        self._sensor_name = selected_sensor.name
        self._enable_sensors(self._sensor_selected)
        self._update_page_count()

        if self._sensor_type == _SENSOR_RANGE:
            range_sensor = getattr(self._hexdrive_app, "range_sensor", None)
            if range_sensor is None:
                if self.logging:
                    print("B:Range sensor not available on this HexDrive.")
        elif self._sensor_type is _SENSOR_COLOUR:
            self._test_card: str = _COLOUR_TEST_CARDS[0] # default to black test card
            colour_sensor = getattr(self._hexdrive_app, "colour_sensor", None)
            if colour_sensor is not None:
                sensor_white_gains = getattr(colour_sensor, "white_gains", None)
                if sensor_white_gains is not None:
                    calibrated = getattr(colour_sensor, "calibrated", None)
                    if calibrated is None:
                        if self.logging:
                            print("B:Colour sensor does not have a 'calibrated' attribute.")
                    elif not calibrated:
                        settings_white_gains = self._load_colour_calibration("gain")
                        if settings_white_gains is not None:
                            colour_sensor.white_gains = settings_white_gains
                            if self.logging:
                                print(f"B:Loaded white gains from settings: {settings_white_gains}")
                            settings_black_reference = self._load_colour_calibration("black")
                            if settings_black_reference is not None:
                                colour_sensor.black_reference = settings_black_reference
                                if self.logging:
                                    print(f"B:Loaded black reference from settings: {settings_black_reference}")
                            elif self.logging:
                                print("B:Black reference not found in settings.")
                        else:
                            if self.logging:
                                print("B:White gains not found in settings.")
                                # Check if sensor now shows as calibrated
                        calibrated = getattr(colour_sensor, "calibrated", True)
                        if not calibrated:
                            # Colour sensor needs calibration, so prompt the user to perform calibration
                            self._page_selected = _PAGE_CAL
                    else:
                        if self.logging:
                            print("B:Colour sensor already calibrated.")
            else:
                if self.logging:
                    print("B:Colour sensor not available on this HexDrive.")
        selected_sensor.stats.reset()  # reset stats for the selected sensor


    def _update_select_port(self, delta: int):   # pylint: disable=unused-argument
        app = self._app
        if app.button_states.get(BUTTON_TYPES["RIGHT"]):
            app.button_states.clear()
            self._port_selected = (self._port_selected % _SLOTS) + 1
            if self._hexdrive_app is not None:
                # Tidy up the previous HexDrive App before switching to a new port
                self._disable_sensors()
                self._hexdrive_app = None
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            app.button_states.clear()
            self._port_selected = ((self._port_selected - 2) % _SLOTS) + 1
            if self._hexdrive_app is not None:
                # Tidy up the previous HexDrive App before switching to a new port
                self._disable_sensors()
                self._hexdrive_app = None
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):    # "Scan"
            app.button_states.clear()
            # Check if there is a HexDrive App known on this port
            for hexdrive_app in app.hexdrive_apps:
                if hexdrive_app.config.port == self._port_selected:
                    if self._logging:
                        print(f"B: Selected HexDrive App on port {self._port_selected}")
                    # Check if this HexDrive App has any sensors attached
                    try:
                        if hexdrive_app.capabilities & (hexdrive_app.CAPABILITY_RANGE | hexdrive_app.CAPABILITY_COLOUR):
                            if self.logging:
                                print(f"B: HexDrive App on port {self._port_selected} has sensors: {hex(hexdrive_app.capabilities & 0xFFFF)}")
                            app.notification = Notification("Selected", port=self._port_selected)
                            self._hexdrive_app = hexdrive_app
                            self._sensor_selected = 0
                            self._setup_for_sensor_type()
                            self._sub_state = _SUB_READING
                            break
                    except Exception:  # pylint: disable=broad-except
                        pass
            app.refresh = True
            if self._hexdrive_app is None:
                app.notification = Notification("No Sensors", port=self._port_selected)
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if self.logging:
                print("Exiting Sensor Test")
            self._disable_sensors()
            self._hexdrive_app = None
            app.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
            app.return_to_menu()


    @staticmethod
    def _ordered_display_items(display_data: dict) -> list[tuple[str, str]]:
        """ with dicts not maintinaing order we need to force into a list in the order we want to display"""
        items = []
        seen = set()
        for key in ("r", "g", "b", "w"):
            if key in display_data:
                items.append((key, str(display_data[key])))
                seen.add(key)
        for key, value in display_data.items():
            if key not in seen:
                items.append((key, str(value)))
        return items


    @staticmethod
    def _colour_setting_keys(key: str) -> tuple[str, str, str, str]:
        base = f"{_COLOUR_CALIBRATION_PREFIX}_{key}_"
        return (f"{base}r", f"{base}g", f"{base}b", f"{base}w")


    def _load_colour_calibration(self, key: str) -> tuple[int, int, int, int] | None:
        setting_keys = self._colour_setting_keys(key)
        values = []
        for setting_key in setting_keys:
            value = platform_settings.get(f"{SETTINGS_NAME_PREFIX}.{setting_key}", None)
            if value is None:
                print(f"B:Colour calibration setting '{setting_key}' not found in platform settings.")
                return None
            values.append(int(value))
        gains = (values[0], values[1], values[2], values[3])
        return gains


    def _update_page_count(self) -> None:
        self._page_count = 3  # default to 3 pages for all sensors
        if self._sensor_type is _SENSOR_COLOUR:
            colour_sensor = getattr(self._hexdrive_app, "colour_sensor", None)
            if colour_sensor is not None:
                calibrated = getattr(colour_sensor, "calibrated", None)
                if calibrated is not None:
                    if self.logging:
                        print("B:Colour sensor has a 'calibrated' attribute.")
                    self._page_count = 4
        if self._page_selected >= self._page_count:
            self._page_selected = _PAGE_RAW


    def _capture_white_reference(self) -> bool:
        if self._hexdrive_app is None or self._last_colour is None:
            return False
        sensor = getattr(self._hexdrive_app, "colour_sensor", None)
        if sensor is None:
            return False

        # Capture the current RGBW reading as the white reference for the colour sensor
        try:
            # This will automatically update the white_gains property based on the captured reading
            sensor.white_reference = self._last_colour
        except Exception as e:          #pylint: disable=broad-except
            print(f"Error setting white reference: {e}")
            return False

        # Read back the gains from the HexDrive App so we can save them to platform settings for persistence across sessions and availability in other modules
        gains = getattr(sensor, "white_gains", None)
        if gains is None:
            return False

        # Save white gains to platform settings for persistence across sessions and availability in other modules
        setting_keys = self._colour_setting_keys("gain")
        for setting_key, gain in zip(setting_keys, gains):
            platform_settings.set(f"{SETTINGS_NAME_PREFIX}.{setting_key}", gain)
            print(f"B:Wrote white gain {gain} to platform settings under key '{SETTINGS_NAME_PREFIX}.{setting_key}'")
        platform_settings.save()
        if self._logging:
            print(f"B:Saved white gains: {gains}")
        self._app.notification = Notification("Calibration Saved", port=self._port_selected)
        return True


    def _capture_black_reference(self) -> bool:
        if self._hexdrive_app is None or self._last_colour is None:
            return False
        sensor = getattr(self._hexdrive_app, "colour_sensor", None)
        if sensor is None:
            return False

        # Capture and persist black reference so white-gain calibration remains valid across restarts.
        try:
            sensor.black_reference = self._last_colour
        except (AttributeError, TypeError, ValueError) as e:
            print(f"Error capturing black reference: {e}")
            return False

        setting_keys = self._colour_setting_keys("black")
        for setting_key, value in zip(setting_keys, self._last_colour):
            platform_settings.set(f"{SETTINGS_NAME_PREFIX}.{setting_key}", value)
            print(f"B:Wrote black reference {value} to platform settings under key '{SETTINGS_NAME_PREFIX}.{setting_key}'")
        platform_settings.save()
        if self._logging:
            print(f"B:Saved black reference: {self._last_colour}")
        return True


    # ------------------------------------------------------------------
    # Sensor selection
    # ------------------------------------------------------------------

    def change_sensor(self, direction: int) -> None:
        """Change the sensor in the list by the given direction (1 for next, -1 for previous)."""
        # tidy up the previous sensor before switching to the next one
        if self._hexdrive_app is not None:
            self._disable_sensors(self._sensor_selected)
        if self._sensor_list:
            self._sensor_selected = (self._sensor_selected + direction) % len(self._sensor_list)
            self._setup_for_sensor_type()  # reset any sensor-specific settings for the new sensor
        self._update_display_values()


    def _update_display_values(self):      # pylint: disable=unused-argument
        # clear old display data
        self._display_data = {}
        #self._update_page_count()

        # Sensor-specific display logic based on sensor type and available data
        colour_sensor = getattr(self._hexdrive_app, "colour_sensor", None)
        if self._sensor_type is _SENSOR_COLOUR and self._hexdrive_app is not None and colour_sensor is not None:
            # Colour Sensor with RGBW data
            if self._page_selected == _PAGE_DATA:
                self._display_data["colour"] = self._last_colour_name
            elif self._page_selected == _PAGE_RAW and self._last_colour is not None:
                self._display_data["r"] = self._last_colour[0]
                self._display_data["g"] = self._last_colour[1]
                self._display_data["b"] = self._last_colour[2]
                self._display_data["w"] = self._last_colour[3]
            elif self._page_selected == _PAGE_CAL:
                # NB you can't depend on the order of display_data dict items when drawn
                self._display_data["colour"] = self._last_colour_name
                self._display_data["match"] = "PASS" if self._last_colour_name == self._test_card else "FAIL"
            self.colour = _COLOUR_CARD_RGB.get(self._last_colour_name, (0.5, 0.5, 0.5))

        elif self._sensor_type is _SENSOR_RANGE:
            if self._page_selected == _PAGE_DATA and self._last_range is not None:
                range_mm = self._last_range
                if range_mm < 50:
                    range_str = "V Close"
                elif range_mm < 100:
                    range_str = "Close"
                elif range_mm < 500:
                    range_str = "Medium"
                else:
                    range_str = "Far"
                self._display_data["Range"] = range_str
            elif self._page_selected == _PAGE_RAW:
                self._display_data["range(mm)"] = self._last_range

        if self._page_selected == _PAGE_STATS:
            # get the rate from the stats object for the current sensor and display it
            rate = self._sensor_list[self._sensor_selected].stats.rate
            missed = self._sensor_list[self._sensor_selected].stats.missed
            self._display_data["sample"] = f"{rate//10}.{rate % 10}Hz"
            self._display_data["missed"] = f"{missed}"
            self._display_data["queue"] = f"{eventbus.event_queue.qsize()}"
            draw_rate = self._draw_stats.rate
            self._display_data["draw"] = f"{draw_rate // 10}.{draw_rate % 10}Hz"


    def _update_reading(self, delta: int):      # pylint: disable=unused-argument
        app = self._app
        # perform update call on all sensor stats
        for sensor in self._sensor_list:
            if sensor.stats.update(delta):
                app.refresh = True

        self._update_timer += delta
        if self._update_timer >= self._min_update_period_ms:
            # we do not display every sample to avoid overwhelming the display with too many updates per second
            if self._new_sample:
                if self._use_events and self._hexdrive_app is not None and self._sensor_type is _SENSOR_COLOUR:
                    # only the raw RGB values are received via events, so we need to update the colour name here
                    colour_sensor = getattr(self._hexdrive_app, "colour_sensor", None)
                    if colour_sensor is not None:
                        try:
                            self._last_colour_name = colour_sensor.colour_name
                        except Exception as e:          # pylint: disable=broad-except
                            print(f"Error updating colour sensor name and display colour: {e}")
                self._new_sample = False
                self._update_timer = 0
                app.refresh = True

        if app.refresh:
            self._update_display_values()

        if app.button_states.get(BUTTON_TYPES["RIGHT"]) and self.num_sensors > 1:
            app.button_states.clear()
            self.change_sensor(1)
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]) and self.num_sensors > 1:
            app.button_states.clear()
            self.change_sensor(-1)
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["UP"]):
            app.button_states.clear()
            if self._page_count > 0:
                self._page_selected = (self._page_selected - 1) % self._page_count
                self._update_display_values()
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            app.button_states.clear()
            if self._page_count > 0:
                self._page_selected = (self._page_selected + 1) % self._page_count
                self._update_display_values()
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            if self._page_selected == _PAGE_CAL and self._hexdrive_app is not None and self._sensor_type is _SENSOR_COLOUR:
                # Capture a colour card reading for the currently selected test card
                hexdrive_app = self._hexdrive_app
                colour_sensor = getattr(hexdrive_app, "colour_sensor", None)
                if colour_sensor is not None:
                    calibrated = getattr(colour_sensor, "calibrated", True)
                    if self._test_card in (_COLOUR_BLACK, _COLOUR_WHITE) and not calibrated:
                        if self._test_card is _COLOUR_BLACK and self._last_colour is not None:
                            # Capture the black reference for the colour sensor
                            if self._capture_black_reference():
                                self._test_card = _COLOUR_WHITE  # advance to the next test card (white) for the next capture
                        elif self._test_card is _COLOUR_WHITE and self._last_colour is not None:
                            # Capture the white reference for the colour sensor
                            self._capture_white_reference()
                    elif calibrated:
                        # Clear Colour sensor white gains calibration
                        try:
                            colour_sensor.calibrated = False
                        except Exception as e:              # pylint: disable=broad-except
                            print(f"Error clearing calibration: {e}")
                        self._test_card = _COLOUR_BLACK  # reset to black test card after clearing calibration
                        print("B:Colour sensor calibration cleared.")
                        self._app.notification = Notification("Calibration Cleared", port=self._port_selected)
                    #else:
                        # Check if the last detected colour name matches the expected test card colour name, and display a PASS/FAIL notification accordingly.
                        #match = self._last_colour_name == self._test_card
                        #self._app.notification = Notification(f"{'PASS' if match else 'FAIL'}", port=self._port_selected)


                # Automatically advance to the next test card - DO NOT loop back to the first card after the last one, as this will cause the user to have to re-capture the black reference again.
                #index = _COLOUR_TEST_CARDS.index(self._test_card)
                #index = (index + 1) if index + 1 < len(_COLOUR_TEST_CARDS) else index
                #self._test_card = _COLOUR_TEST_CARDS[index]

                self._update_display_values()
                app.refresh = True
            else:
                print(f"B:Confirm button pressed on page {_PAGE_NAMES.get(self._page_selected, 'Unknown')} for sensor {self._sensor_name} - no action defined.")
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            self._disable_sensors()
            self._sub_state = _SUB_SELECT_PORT
            app.refresh = True


    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx):
        """Render sensor test UI."""
        if self._sub_state == _SUB_SELECT_PORT:
            self._draw_select_port(ctx)
            return True
        elif self._sub_state == _SUB_READING:
            self._draw_stats.new_sample()
            self._draw_reading(ctx)
            return True
        return False


    def _draw_select_port(self, ctx):
        self._app.draw_message(ctx,
            ["Sensor Test", f"Port: {self._port_selected}"],
            [(1, 1, 0), (0, 1, 1)],
            label_font_size)
        button_labels(ctx, left_label="\u25C0Port", right_label="Port\u25B6",
                      confirm_label="Scan", cancel_label="Back")


    def _draw_reading(self, ctx):
        up_label = down_label = confirm_label = ""
        #ctx.font = "Arimo Regular" - the font doesn't appear to change
        lines = []
        colours = []
        if self._sensor_type is _SENSOR_COLOUR:
            colour_sensor = getattr(self._hexdrive_app, "colour_sensor", None)
            if self._page_selected == _PAGE_CAL and colour_sensor is not None:
                calibrated = getattr(colour_sensor, "calibrated", None)
                if calibrated is None:
                    pass
                elif calibrated:
                    confirm_label = "Clear"
                    lines += ["Colour Sensor", "Calibrated"]
                    colours += [(0.5, 1, 0.5), (0.5, 1, 0.5)]
                else:
                    confirm_label = "Confirm"
                    lines += ["Colour Sensor", "Calibration:", "Place over", f"{self._test_card} Surface"]
                    colours += [(1, 1, 0), (1, 1, 0), (1, 1, 0), (1, 1, 0)]
        if self._page_selected in (_PAGE_RAW, _PAGE_STATS, _PAGE_DATA):
            if self._page_count == 0:
                lines += [self._sensor_type]
            else:
                lines += [f"{self._sensor_type}:{_PAGE_NAMES[self._page_selected]}"]
            colours += [(1, 1, 0)]

            if self._display_data:
                for label, value in self._ordered_display_items(self._display_data):
                    lines += [f"{label}:{value}"]
                    colours += [(1, 1, 1)]
            else:
                lines += ["Reading..."]
                colours += [(0.5, 0.5, 0.5)]

        # Slot # - squeeze between top labels
        #ctx.rgb(1.0, 1.0, 1.0).move_to(-ctx.text_width(f"{self._port_selected}")//2, -ctx.font_size*2.5).text(f"{self._port_selected}")

        if self._page_count > 0:
            # Button labels: up/down show destination page names
            down_page = (self._page_selected + 1) % self._page_count
            up_page = (self._page_selected - 1) % self._page_count
            # Only show the DOWN label if there is more than 1 page, otherwise there is no other page to go to
            down_label=_PAGE_NAMES[down_page] if self._page_count > 1 else ""
            # only show the UP label if there are more than 2 pages, otherwise it would just show the same as the DOWN
            up_label=_PAGE_NAMES[up_page] if self._page_count > 2 else ""

            if self.num_sensors > 1:
                button_labels(ctx, left_label="\u25C0Prev", right_label="Next\u25B6",
                            up_label=up_label, down_label=down_label, cancel_label="Back", confirm_label=confirm_label)
            else:
                button_labels(ctx, up_label=up_label, down_label=down_label, cancel_label="Back", confirm_label=confirm_label)

        # Ensure there are always 5 lines to draw for consistent layout, even if some are blank
        while len(lines) < 5:
            lines.append("")
            colours.append((1, 1, 1))

        self._app.draw_message(ctx, lines, colours, label_font_size)


    def _enable_sensors(self, sensor: int | None = None) -> bool:
        """Enable the range and colour sensors on the HexDrive if present."""
        num_sensors_enabled = 0
        if self._hexdrive_app is None:
            return False
        hexdrive_app = self._hexdrive_app
        # Ensure compatibility with all versions of HexDriveApp
        capabilities = getattr(hexdrive_app, "capabilities", 0)
        capability_range = getattr(hexdrive_app, "CAPABILITY_RANGE", 0)
        capability_colour = getattr(hexdrive_app, "CAPABILITY_COLOUR", 0)

        # the hexdrive_app could be for V1 or V2, only V2 has the range sensor support
        if (sensor is not None and self._sensor_list[sensor].sensor_type != "Range") or 0 == (capabilities & capability_range):
            pass  # Don't enable the range sensor if the selected sensor is not a range sensor or if the hexdrive_app does not support range sensors
        else:
            range_enable = getattr(hexdrive_app, "range_enable", None)
            if range_enable is not None:
                try:
                    range_enable(True, events = self._use_events, interrupts = True)
                except (TypeError, RuntimeError) as e:
                    print(f"B:Error enabling range sensor: {e}")
                set_range_period = getattr(hexdrive_app, "set_range_period", None)
                if set_range_period is not None:
                    set_range_period(100)  # Set the range sensor period to 100ms (example value)
                print("B:Range Enabled")
                num_sensors_enabled += 1
                if self._use_events:
                    range_event = getattr(hexdrive_app, "RangeEvent", None)
                    if range_event is not None:
                        eventbus.on(
                            range_event,
                            self._handle_range_event,
                            self
                        )
                        print("B:Range Event enabled")

        if (sensor is not None and self._sensor_list[sensor].sensor_type is not _SENSOR_COLOUR) or 0 == (capabilities & capability_colour):
            pass  # Don't enable the colour sensor if the selected sensor is not a colour sensor or if the hexdrive_app does not support colour sensors
        else:
            colour_enable = getattr(hexdrive_app, "colour_enable", None)
            if colour_enable is not None:
                set_flood_led = getattr(hexdrive_app, "set_flood_led", None)
                if set_flood_led is not None:
                    set_flood_led(True)  # Enable the flood LED for the colour sensor (and interrupt pull up) - this is required for the colour sensor to work properly
                try:
                    colour_enable(True, events = self._use_events)
                except (TypeError, RuntimeError) as e:
                    print(f"B:Error enabling colour sensor: {e}")
                #set_colour_period = getattr(hexdrive_app, "set_colour_period", None)
                #if set_colour_period is not None:
                #    set_colour_period(100)  # Set the colour sensor period to 100ms (example value)

                print("B:Colour Enabled")
                num_sensors_enabled += 1
                if self._use_events:
                    colour_event = getattr(hexdrive_app, "ColourEvent", None)
                    if colour_event is not None:
                        eventbus.on(
                            colour_event,
                            self._handle_colour_event,
                            self
                        )
                        print("B:Colour Event enabled")
        return num_sensors_enabled > 0


    def _disable_sensors(self, sensor: int | None = None):
        """Disable the range and colour sensors on the HexDrive if present."""
        if self._hexdrive_app is None:
            return
        hexdrive_app = self._hexdrive_app
        if (sensor is not None and self._sensor_list[sensor].sensor_type is not _SENSOR_RANGE):
            pass  # Don't disable the range sensor if the selected sensor is not a range sensor
        else:
            # Range Sensor
            range_enable = getattr(hexdrive_app, "range_enable", None)
            if range_enable is not None:
                try:
                    range_enable(False)
                except (TypeError, RuntimeError) as e:
                    print(f"B:Error disabling range sensor: {e}")
                print("B:Range Disabled")
                #if self._use_events:
                #    range_event = getattr(hexdrive_app, "RangeEvent", None)
                #    if range_event is not None:
                #        eventbus.remove(range_event, self._handle_range_event, self)
                #        print("B:Range Event disabled")
            self._range_sensor_stats.reset()

        if (sensor is not None and self._sensor_list[sensor].sensor_type is not _SENSOR_COLOUR):
            pass  # Don't disable the colour sensor if the selected sensor is not a colour sensor
        else:
            # Colour Sensor
            set_flood_led = getattr(hexdrive_app, "set_flood_led", None)
            if set_flood_led is not None:
                set_flood_led(False)
            colour_enable = getattr(hexdrive_app, "colour_enable", None)
            if colour_enable is not None:
                try:
                    colour_enable(False)
                except (TypeError, RuntimeError) as e:
                    print(f"B:Error disabling colour sensor: {e}")
                print("B:Colour Disabled")
                #if self._use_events:
                #    colour_event = getattr(hexdrive_app, "ColourEvent", None)
                #    if colour_event is not None:
                #        eventbus.remove(colour_event, self._handle_colour_event, self)
                #        print("B:Colour Event disabled")
            self._colour_sensor_stats.reset()


    # ------------------------------------------------------------------
    # Event Handlers
    # ------------------------------------------------------------------

    def _handle_range_event(self, event):
        """Handle range events from the range sensor."""
        #print(f"B:EventRange:{event.range}mm")
        self._last_range = event.range
        self._range_sensor_stats.new_sample()
        if self._sensor_type is _SENSOR_RANGE:
            self._new_sample = True
        elif self.logging:
            print(f"B:Received range event for {self._sensor_type}")


    def _handle_colour_event(self, event):
        """Handle colour events from the colour sensor."""
        #print(f"B:EventColour:{event.colour} ({event.colour_name})")
        self._last_colour = event.colour
        self._colour_sensor_stats.new_sample()
        if self._sensor_type is _SENSOR_COLOUR:
            self._new_sample = True
        elif self.logging:
            print(f"B:Received colour event for {self._sensor_type}")


#------------------------------------------------------------------
# ESP32S3 PCNT (Pulse Counter) hardware register definitions and bit masks
# Supports all 4 PCNT units (0-3) on the ESP32-S3.
#-------------------------------------------------------------------

_SYSTEM_BASE      = const(0x600C0000)
_GPIO_BASE        = const(0x60004000)
_PCNT_BASE        = const(0x60017000)

_PCNT_NUM_UNITS   = 4   # ESP32-S3 has 4 PCNT units

_PCNT_CLK_BIT     = const(1 << 10)  # SYSTEM_PCNT_CLK_EN / SYSTEM_PCNT_RST (bit 10)

# System/Clock registers
_CLK_EN0_REG      = const(_SYSTEM_BASE + 0x0018)
_RST_EN0_REG      = const(_SYSTEM_BASE + 0x0020)

# GPIO Matrix Base
_GPIO_FUNC_IN_SEL_CFG_BASE = const(_GPIO_BASE + 0x0154)
_SIG_IN_SEL_BIT   = const(1 << 6) # Enable routing via GPIO Matrix

# PCNT register offsets (per-unit, relative to _PCNT_BASE)
#   CONF0: _PCNT_BASE + unit * 0x0C
#   CONF1: _PCNT_BASE + unit * 0x0C + 0x04
#   CONF2: _PCNT_BASE + unit * 0x0C + 0x08
#   CNT:   _PCNT_BASE + 0x30 + unit * 4
#   STATUS:_PCNT_BASE + 0x50 + unit * 4
_PCNT_CTRL_REG    = const(_PCNT_BASE + 0x0060)

# _PCNT_CTRL_REG bits — per-unit reset and pause at (unit * 2) and (unit * 2 + 1)
_PCNT_CTRL_CLK_EN = const(1 << 16)  # Register clock gate — must be 1 for register access

# CONF0 bit layout (same layout for all units)
_CONF0_FILTER_THRES_M  = const(0x3FF)   # bits [9:0]
_CONF0_FILTER_EN       = const(1 << 10)

# GPIO signal index base for PCNT: Unit N, CH0 pulse = 33 + N*4, CH0 ctrl = 35 + N*4
_PCNT_SIG_BASE    = 33

# APB clock frequency for filter calculation (Hz)
_APB_CLK_HZ       = const(80_000_000)

# Badge hexpansion HS pin to ESP32-S3 GPIO number mapping
# HexpansionConfig(port).pin[i] is Pin(gpio) where gpio = _HS_PIN_TO_GPIO[port][i]
_HS_PIN_TO_GPIO = {
    1: (39, 40, 41, 42),
    2: (35, 36, 37, 38),
    3: (34, 33, 47, 48),
    4: (11, 14, 13, 12),
    5: (18, 16, 15, 17),
    6: ( 3,  4,  5,  6),
}

# Reverse lookup: GPIO number -> (port, pin_index) for diagnostics
_GPIO_TO_HS = {}
for _port, _gpios in _HS_PIN_TO_GPIO.items():
    for _idx, _gpio in enumerate(_gpios):
        _GPIO_TO_HS[_gpio] = (_port, _idx)

_PCNT_UNIT_STRIDE = const(0x0C)
_PCNT_CONF1_OFFSET = const(0x04)
_PCNT_CONF2_OFFSET = const(0x08)
_PCNT_CNT_OFFSET = const(0x30)

_CONF0_CH0_NEG_MODE_S = const(16)
_CONF0_CH0_POS_MODE_S = const(18)
_CONF0_CH0_HCTRL_MODE_S = const(20)
_CONF0_CH0_LCTRL_MODE_S = const(22)
_CONF0_CH1_NEG_MODE_S = const(24)
_CONF0_CH1_POS_MODE_S = const(26)
_CONF0_CH1_HCTRL_MODE_S = const(28)
_CONF0_CH1_LCTRL_MODE_S = const(30)

_PCNT_COUNT_DISABLE = const(0)
_PCNT_COUNT_INCREMENT = const(1)
_PCNT_COUNT_DECREMENT = const(2)

_PCNT_CTRL_KEEP = const(0)
_PCNT_CTRL_REVERSE = const(1)
_PCNT_CTRL_HOLD = const(2)

_PCNT_GPIO_CONST_HIGH = const(0x38)
_PCNT_COUNTER_MASK = const(0xFFFF)
_PCNT_COUNTER_SIGN_BIT = const(0x8000)
_PCNT_COUNTER_MODULO = const(0x10000)
_PCNT_COUNTER_MAX = const(0x7FFF)
_PCNT_DEFAULT_MAX = const(0x7FFF)


def _pcnt_conf0_addr(unit: int) -> int:
    return _PCNT_BASE + unit * _PCNT_UNIT_STRIDE


def _pcnt_conf1_addr(unit: int) -> int:
    return _pcnt_conf0_addr(unit) + _PCNT_CONF1_OFFSET


def _pcnt_conf2_addr(unit: int) -> int:
    return _pcnt_conf0_addr(unit) + _PCNT_CONF2_OFFSET


def _pcnt_cnt_addr(unit: int) -> int:
    return _PCNT_BASE + _PCNT_CNT_OFFSET + unit * 4


def _pcnt_rst_bit(unit: int) -> int:
    return 1 << (unit * 2)


def _pcnt_signal_index(unit: int, channel: int, control: bool = False) -> int:
    return _PCNT_SIG_BASE + unit * 4 + channel + (2 if control else 0)


def _pcnt_gpio_label(gpio: int) -> str:
    hs_pin = _GPIO_TO_HS.get(gpio)
    if hs_pin is None:
        return f"GPIO {gpio}"
    return f"GPIO {gpio} (port {hs_pin[0]} HS pin {hs_pin[1]})"


def _pcnt_filter_bits(filter_ns: int | None) -> int:
    if filter_ns is None or filter_ns <= 0:
        return 0
    filter_val = (_APB_CLK_HZ * filter_ns) // 1_000_000_000
    if filter_val > 1023:
        filter_val = 1023
    return (filter_val & _CONF0_FILTER_THRES_M) | _CONF0_FILTER_EN


def _pcnt_enable_peripheral() -> None:
    mem32[_CLK_EN0_REG] |= _PCNT_CLK_BIT
    mem32[_RST_EN0_REG] &= ~_PCNT_CLK_BIT


def _pcnt_disable_peripheral() -> None:
    mem32[_CLK_EN0_REG] &= ~_PCNT_CLK_BIT
    mem32[_RST_EN0_REG] |= _PCNT_CLK_BIT


def _pcnt_route_input(signal_index: int, gpio: int | None) -> None:
    route = _SIG_IN_SEL_BIT | (_PCNT_GPIO_CONST_HIGH if gpio is None else gpio)
    mem32[_GPIO_FUNC_IN_SEL_CFG_BASE + (signal_index * 4)] = route


def _pcnt_read_count_signed(unit: int) -> int:
    raw = mem32[_pcnt_cnt_addr(unit)] & _PCNT_COUNTER_MASK
    if raw & _PCNT_COUNTER_SIGN_BIT:
        return raw - _PCNT_COUNTER_MODULO
    return raw


def _pcnt_reset_counter(unit: int) -> None:
    rst_bit = _pcnt_rst_bit(unit)
    irq_state = disable_irq()
    mem32[_PCNT_CTRL_REG] |= rst_bit
    mem32[_PCNT_CTRL_REG] &= ~rst_bit
    enable_irq(irq_state)


def _pcnt_unit_in_use(unit: int, logging: bool = False) -> bool:
    """Return True when *unit* appears to be configured and active."""
    clk_on = (mem32[_CLK_EN0_REG] & _PCNT_CLK_BIT) != 0
    if not clk_on:
        if logging:
            print(f"PCNT: unit {unit} - peripheral clock off, unit free")
        return False

    ctrl = mem32[_PCNT_CTRL_REG]
    if not ctrl & _PCNT_CTRL_CLK_EN:
        if logging:
            print(f"PCNT: unit {unit} - register clock gate off, unit free")
        return False

    rst_bit = _pcnt_rst_bit(unit)
    if ctrl & rst_bit:
        if logging:
            print(f"PCNT: unit {unit} - held in reset, unit free")
        return False

    conf0 = mem32[_pcnt_conf0_addr(unit)]
    if conf0 in (0, 0x3C10):
        if logging:
            print(f"PCNT: unit {unit} - CONF0=0x{conf0:08X} (unconfigured), unit free")
        return False

    if logging:
        cnt = mem32[_pcnt_cnt_addr(unit)] & _PCNT_COUNTER_MASK
        pulse_sig = _pcnt_signal_index(unit, 0)
        gpio_route = mem32[_GPIO_FUNC_IN_SEL_CFG_BASE + pulse_sig * 4]
        routed_gpio = gpio_route & 0x3F
        print(f"PCNT: unit {unit} - IN USE: CONF0=0x{conf0:08X}, count={cnt}, routed to GPIO {routed_gpio}")
    return True


def _pcnt_allocate_unit(unit: int | None, logging: bool = False) -> int | None:
    if unit is not None:
        if unit < 0 or unit >= _PCNT_NUM_UNITS:
            if logging:
                print(f"PCNT: unit {unit} out of range (0-{_PCNT_NUM_UNITS - 1})")
            return None
        if _pcnt_unit_in_use(unit, logging):
            if logging:
                print(f"PCNT: requested unit {unit} is already in use")
            return None
        if logging:
            print(f"PCNT: using requested unit {unit}")
        return unit

    for candidate in range(_PCNT_NUM_UNITS):
        if not _pcnt_unit_in_use(candidate, logging):
            if logging:
                print(f"PCNT: auto-selected unit {candidate}")
            return candidate

    if logging:
        print("PCNT: all units in use, no free unit available")
    return None


def _pcnt_disable_peripheral_if_unused(logging: bool = False) -> None:
    if any(_pcnt_unit_in_use(unit) for unit in range(_PCNT_NUM_UNITS)):
        return
    _pcnt_disable_peripheral()
    if logging:
        print("PCNT: all units released, peripheral clock disabled")


class _PCNTUnitBase:
    """Shared low-level PCNT unit allocation and teardown helpers."""
    __slots__ = ("unit", "_configured", "logging")

    def __init__(self, unit: int | None, logging: bool = False):
        self.logging = logging
        self.unit = _pcnt_allocate_unit(unit, logging)
        self._configured = False

    def _log(self, message: str) -> None:
        if self.logging:
            print(message)

    def _begin_configuration(self) -> tuple[int, int]:
        unit = self.unit
        if unit is None:
            raise ValueError("PCNT unit not available")

        _pcnt_enable_peripheral()

        ctrl = mem32[_PCNT_CTRL_REG]
        ctrl |= _PCNT_CTRL_CLK_EN | _pcnt_rst_bit(unit)
        mem32[_PCNT_CTRL_REG] = ctrl

        mem32[_pcnt_conf0_addr(unit)] = 0
        mem32[_pcnt_conf1_addr(unit)] = 0
        mem32[_pcnt_conf2_addr(unit)] = 0
        return _pcnt_conf0_addr(unit), _pcnt_cnt_addr(unit)

    def _finish_configuration(self, conf0_addr: int, cnt_addr: int) -> None:
        unit = self.unit
        if unit is None:
            raise ValueError("PCNT unit not available")

        ctrl = mem32[_PCNT_CTRL_REG]
        ctrl &= ~_pcnt_rst_bit(unit)
        mem32[_PCNT_CTRL_REG] = ctrl
        _pcnt_reset_counter(unit)
        self._configured = True

        self._log(
            f"PCNT U{unit}: configured OK, CONF0=0x{mem32[conf0_addr]:08X}, "
            f"CTRL=0x{mem32[_PCNT_CTRL_REG]:08X}, CNT={mem32[cnt_addr] & _PCNT_COUNTER_MASK}"
        )

    def deinit(self):
        """Release the PCNT unit and make it available again."""
        if not self._configured or self.unit is None:
            return

        unit = self.unit
        mem32[_PCNT_CTRL_REG] |= _pcnt_rst_bit(unit)
        mem32[_pcnt_conf0_addr(unit)] = 0
        mem32[_pcnt_conf1_addr(unit)] = 0
        mem32[_pcnt_conf2_addr(unit)] = 0
        self._configured = False

        self._log(f"PCNT U{unit}: released")
        _pcnt_disable_peripheral_if_unused(self.logging)


class Counter(_PCNTUnitBase):
    """Wrapper around ESP32-S3 PCNT hardware for counting rising edges."""
    __slots__ = ("pin",)

    def __init__(self, unit: int | None, src: int, filter_ns: int = 0, logging: bool = False):
        self.pin = src
        super().__init__(unit, logging)
        if self.unit is None:
            return
        if not self.init(src, filter_ns):
            self._log(f"PCNT: failed to configure unit {self.unit}")
            self.unit = None

    def __str__(self):
        if self.unit is None:
            return "Counter(not configured)"
        return f"Counter(unit={self.unit}, GPIO={self.pin}, count={self.value()})"

    __repr__ = __str__

    def init(self, src: int, filter_ns: int | None = None) -> bool:
        """Configure the unit to count rising edges on *src*."""
        unit = self.unit
        if unit is None:
            return False

        self.pin = src
        conf0_addr, cnt_addr = self._begin_configuration()
        pulse_sig = _pcnt_signal_index(unit, 0)
        ctrl_sig = _pcnt_signal_index(unit, 0, control=True)
        aux_pulse_sig = _pcnt_signal_index(unit, 1)
        aux_ctrl_sig = _pcnt_signal_index(unit, 1, control=True)

        self._log(f"PCNT U{unit}: counter on {_pcnt_gpio_label(src)}, filter_ns={filter_ns}ns")

        try:
            _pcnt_route_input(pulse_sig, src)
            _pcnt_route_input(ctrl_sig, None)
            _pcnt_route_input(aux_pulse_sig, None)
            _pcnt_route_input(aux_ctrl_sig, None)

            config = _pcnt_filter_bits(filter_ns)
            config |= _PCNT_COUNT_INCREMENT << _CONF0_CH0_POS_MODE_S
            mem32[conf0_addr] = config

            self._finish_configuration(conf0_addr, cnt_addr)
        except Exception as exc:          # pylint: disable=broad-exception-caught
            self._log(f"PCNT U{unit}: error configuring counter: {exc}")
            self._configured = False
            return False

        if self.logging:
            print(f"PCNT U{unit}: configured - CONF0=0x{mem32[conf0_addr]:08X}, CTRL=0x{mem32[_PCNT_CTRL_REG]:08X}, CNT={mem32[cnt_addr] & 0xFFFF}")
        return True

    def value(self, value: int | None = None) -> int:
        """Return the current count, optionally read-and-reset on ``value(0)``."""
        if not self._configured or self.unit is None:
            return 0

        count = mem32[_pcnt_cnt_addr(self.unit)] & _PCNT_COUNTER_MASK
        if value == 0:
            _pcnt_reset_counter(self.unit)
        return count


class Encoder(_PCNTUnitBase):
    """4x quadrature encoder wrapper built on a single ESP32-S3 PCNT unit."""
    __slots__ = ("phase_a", "phase_b", "_position", "_cycles", "_last_raw", "_range_min", "_range_max", "_range_enabled")

    def __init__(
        self,
        unit: int | None,
        phase_a: int,
        phase_b: int,
        filter_ns: int = 0,
        max_count: int | None = None,
        min_count: int = 0,
        logging: bool = False,
    ):
        self.phase_a = phase_a
        self.phase_b = phase_b
        self._position = 0
        self._cycles = 0
        self._last_raw = 0
        self._range_min = 0
        self._range_max = 0
        self._range_enabled = False
        super().__init__(unit, logging)
        if self.unit is None:
            return
        if not self.init(phase_a, phase_b, filter_ns=filter_ns, max_count=max_count, min_count=min_count):
            self._log(f"PCNT: failed to configure encoder on unit {self.unit}")
            self.unit = None

    def __str__(self):
        if self.unit is None:
            return "Encoder(not configured)"
        return (
            f"Encoder(unit={self.unit}, phase_a={self.phase_a}, phase_b={self.phase_b}, position={self.value()}, cycles={self._cycles})"
        )

    __repr__ = __str__

    def init(
        self,
        phase_a: int,
        phase_b: int,
        filter_ns: int = 0,
        max_count: int | None = None,
        min_count: int = 0,
    ) -> bool:
        """Configure the unit for 4x quadrature decoding on *phase_a* and *phase_b*."""
        unit = self.unit
        if unit is None:
            return False
        if phase_a == phase_b:
            self._log("PCNT: encoder phase_a and phase_b must use different GPIOs")
            return False

        range_enabled = max_count is not None and not (max_count == 0 and min_count == 0)
        range_max = 0 if max_count is None else max_count
        range_min = 0 if max_count is None else min_count
        if range_enabled and range_max < range_min:
            self._log(f"PCNT U{unit}: invalid encoder range min={range_min}, max={range_max}")
            return False

        self.phase_a = phase_a
        self.phase_b = phase_b

        conf0_addr, cnt_addr = self._begin_configuration()
        range_desc = "hardware range"
        if range_enabled:
            range_desc = f"min={range_min}, max={range_max}"
        self._log(
            f"PCNT U{unit}: encoder on {_pcnt_gpio_label(phase_a)} and {_pcnt_gpio_label(phase_b)}, phases=4, filter_ns={filter_ns}ns, {range_desc}"
        )

        try:
            _pcnt_route_input(_pcnt_signal_index(unit, 0), phase_a)
            _pcnt_route_input(_pcnt_signal_index(unit, 0, control=True), phase_b)
            _pcnt_route_input(_pcnt_signal_index(unit, 1), phase_b)
            _pcnt_route_input(_pcnt_signal_index(unit, 1, control=True), phase_a)

            config = _pcnt_filter_bits(filter_ns)
            config |= _PCNT_COUNT_INCREMENT << _CONF0_CH0_NEG_MODE_S
            config |= _PCNT_COUNT_DECREMENT << _CONF0_CH0_POS_MODE_S
            config |= _PCNT_CTRL_REVERSE << _CONF0_CH0_LCTRL_MODE_S
            config |= _PCNT_COUNT_DECREMENT << _CONF0_CH1_NEG_MODE_S
            config |= _PCNT_COUNT_INCREMENT << _CONF0_CH1_POS_MODE_S
            config |= _PCNT_CTRL_REVERSE << _CONF0_CH1_LCTRL_MODE_S
            mem32[conf0_addr] = config

            self._finish_configuration(conf0_addr, cnt_addr)
        except Exception as exc:          # pylint: disable=broad-exception-caught
            self._log(f"PCNT U{unit}: error configuring encoder: {exc}")
            self._configured = False
            return False

        self._range_min = range_min
        self._range_max = range_max
        self._range_enabled = range_enabled
        self._position = range_min if self._range_enabled else 0
        self._cycles = 0
        self._last_raw = _pcnt_read_count_signed(unit)
        return True

    def _update_position(self) -> None:
        if not self._configured or self.unit is None:
            return

        raw = _pcnt_read_count_signed(self.unit)
        delta = raw - self._last_raw
        if delta > _PCNT_COUNTER_MAX:
            delta -= _PCNT_COUNTER_MODULO
        elif delta < -_PCNT_COUNTER_SIGN_BIT:
            delta += _PCNT_COUNTER_MODULO
        self._last_raw = raw

        if delta == 0:
            return

        previous_cycles = self._cycles
        if self._range_enabled:
            span = (self._range_max - self._range_min) + 1
            absolute = self._cycles * span + (self._position - self._range_min)
            absolute += delta
            self._cycles, offset = divmod(absolute, span)
            self._position = self._range_min + offset
        else:
            self._position += delta

        if self.logging:
            wrap_note = " (wrapped)" if self._cycles != previous_cycles else ""
            print(
                f"PCNT U{self.unit}: encoder delta={delta}, position={self._position}, cycles={self._cycles}{wrap_note}"
            )

    def value(self, value: int | None = None) -> int:
        """Return the current position and optionally reset it with ``value(0)``."""
        if not self._configured or self.unit is None:
            return 0

        self._update_position()
        position = self._position
        if value == 0:
            if self._range_enabled and not self._range_min <= 0 <= self._range_max:
                raise ValueError("0 outside configured encoder range")
            _pcnt_reset_counter(self.unit)
            self._last_raw = _pcnt_read_count_signed(self.unit)
            self._position = 0
            self._cycles = 0
            self._log(f"PCNT U{self.unit}: encoder position reset to 0, cycles=0")
        return position

    def cycles(self) -> int:
        """Return the current logical wrap/underflow cycle count."""
        if not self._configured or self.unit is None:
            return 0

        self._update_position()
        return self._cycles


# a class to hold sensor meta-data statistics for display in the UI, it is told when a new sensor reading is available and it keeps track of the
# sample update rate averaged over a defined period of time e.g. default to 5 seconds
# it does NOT use time functions but takes in a delta when called from update
# it is initialised with a name of the sensor for which it is providing stats
class SensorStats():
    """A class to track sensor statistics, including sample rate and count."""
    __slots__ = ("_name", "_sample_period_ms", "_sample_count", "_sample_timer", "_sample_rate", "_missed_samples", "_last_sequence_number")

    def __init__(self, name: str, sample_period_ms: int=5000):
        self._name: str = name
        self._sample_period_ms: int = sample_period_ms
        self._sample_count: int = 0
        self._sample_timer: int = 0
        self._sample_rate: int = 0
        self._missed_samples: int = 0
        self._last_sequence_number: int = -1


    @property
    def name(self) -> str:
        """Return the name of the sensor."""
        return self._name


    @property
    def missed(self) -> int:
        """Return the number of missed samples."""
        return self._missed_samples


    @micropython.native
    def update(self, delta: int) -> bool:
        """Update the sample timer and calculate the sample rate."""
        self._sample_timer += delta
        if self._sample_timer >= self._sample_period_ms:
            # Calculate the sample rate in units of 0.1Hz
            self._sample_rate = (10000 * self._sample_count) // self._sample_timer
            # Reset the counters for the next period
            self._sample_count = 0
            self._sample_timer = 0
            return True
        return False


    def new_sample(self, sequence_number: int | None = None) -> None:
        """Increment the sample count when a new sensor reading is available."""
        self._sample_count += 1
        if sequence_number is not None:
            if self._last_sequence_number != -1 and sequence_number != self._last_sequence_number + 1:
                self._missed_samples += sequence_number - self._last_sequence_number - 1
            self._last_sequence_number = sequence_number


    def reset(self) -> None:
        """Reset the sample count and timer."""
        self._sample_count = 0
        self._sample_timer = 0
        self._sample_rate = 0
        self._missed_samples = 0
        self._last_sequence_number = -1


    @property
    def rate(self) -> int:
        """Return the current sample rate in 0.1Hz units."""
        return self._sample_rate
