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

from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification
from system.hexpansion.config import HexpansionConfig
import settings as platform_settings

from .sensor_manager import SensorManager

from .app import SETTINGS_NAME_PREFIX, DEFAULT_BACKGROUND_UPDATE_PERIOD

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
    time.sleep(delay_ms / 1000)


# Local sub-states (internal to Sensor Test)
_SUB_SELECT_PORT = 0
_SUB_READING     = 1


# Pages of information to show for each sensor (can be switched with up/down buttons)
_PAGE_RAW = 0
_PAGE_STATS = 1
_PAGE_DATA = 2
_PAGE_CAL = 3
_PAGE_NAMES = {
    0: "Raw",
    1: "Stats",
    2: "Data",
    3: "Cal",
}

_WHITE_CAL_SCALE = 1024
_WHITE_CAL_GAIN_PREFIX = "stc"

# Mapping colour RGB/XY values to human readable colour names.
# Values are based on the CIE 1931 Chromaticity Diagram
COLOR_REGIONS = [
    {"name": "White",   "x": (0.28, 0.35), "y": (0.28, 0.35)},
    {"name": "Orange",  "x": (0.50, 0.65), "y": (0.35, 0.45)},
    {"name": "Red",     "x": (0.45, 0.75), "y": (0.25, 0.40)},
    {"name": "Green",   "x": (0.10, 0.40), "y": (0.45, 0.85)},
    {"name": "Blue",    "x": (0.10, 0.25), "y": (0.05, 0.25)},
    {"name": "Yellow",  "x": (0.40, 0.50), "y": (0.45, 0.55)},
    {"name": "Cyan",    "x": (0.10, 0.30), "y": (0.25, 0.45)},
    {"name": "Magenta", "x": (0.30, 0.55), "y": (0.10, 0.25)},
    {"name": "Gray",    "x": (0.30, 0.45), "y": (0.25, 0.45)},
]

_ENABLE_PIN  = const(0)     # First LS pin used to enable the SMPSU
_COLOUR_INT_PIN = const(1)  # Second LS pin used to detect interrupts from the colour sensor to trigger readings without polling
_LED_PIN  = const(2)        # Third LS pin used to control an LED to illuminate the area under the colour sensor for better readings of reflected light from the surface below.
_DIST_INT_PIN = const(3)    # Fourth LS pin used to detect interrupts from the distance sensor to trigger readings without polling
_DIST_XSHUT_PIN = const(4)  # Fifth LS pin used to control the XSHUT pin of the distance sensor to allow it to be power cycled for reset or power saving

# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting: type):       # pylint: disable=unused-argument, invalid-name
    """Register sensor-test-specific settings in the shared settings dict."""
    # No settings currently, but this is where they would be registered if needed.
    pass

# ---- Sensor Test manager ---------------------------------------------------

class SensorTestMgr:
    """Manages the Sensor Test workflow.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """

    def __init__(self, app, logging: bool = False):
        self._app = app
        self._sub_state = _SUB_SELECT_PORT
        self._sensor_mgr: SensorManager | None = None
        self._port_selected: int = 1
        self._sensor_data: dict = {}
        self._display_data: dict = {}
        self._page_selected: int = _PAGE_RAW
        self._page_count: int = 3
        self._logging: bool = logging
        self._read_timer: int = 0    # ms since last sensor read
        self._sample_count: int = 0
        self._count_timer: int = 0  # ms
        self._sample_rate: int = 0  # Hz
        self._new_sample: bool = False
        self._colour: tuple[float, float, float] = (1.0, 1.0, 0.0)  # default to yellow for non-colour sensors
        self._white_gains: tuple[int, int, int, int] | None = None  # white reference gains for RGBC channels, scaled by _WHITE_CAL_SCALE
        self._test_results: dict = {}  # dict to hold test results

        if self._logging:
            print("SensorTestMgr initialised")


    # ------------------------------------------------------------------

    @property
    def logging(self) -> bool:
        """Whether to print debug logs to the console."""
        return self._logging

    @logging.setter
    def logging(self, value: bool):
        # Update logging setting in this manager and also in the SensorManager if it exists, so that sensor manager logs will be shown/hidden based on the current app setting
        if self._sensor_mgr is not None:
            self._sensor_mgr.logging = value
        self._logging = value

    @property
    def sample_count(self) -> int:
        """Number of sensor samples read since starting the current test."""
        return self._sample_count

    @sample_count.setter
    def sample_count(self, value: int):
        self._sample_count = value


    # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Enter the Sensor Test flow from the main menu."""
        app = self._app
        app.set_menu(None)
        app.button_states.clear()
        self._sensor_data = {}
        self._display_data = {}
        app.refresh = True
        sensor_mgr = self._ensure_sensor_mgr()
        self._colour = (1.0, 1.0, 0.0)  # reset to yellow when starting sensor test
        if app.hexdrive_ports is not None:
            # If a HexDrive is present try its port for sensors
            for port in app.hexdrive_ports:
                if sensor_mgr.open(port):
                    self._port_selected = port
                    app.update_period = sensor_mgr.read_interval
                    self._sub_state = _SUB_READING
                    break
        else:
            # Otherwise, start in port selection mode
            self._port_selected = 1
            self._sub_state = _SUB_SELECT_PORT
        return True


    # ------------------------------------------------------------------
    # Sensor Manager access
    # ------------------------------------------------------------------

    def _ensure_sensor_mgr(self) -> SensorManager:
        """Lazy-import and create SensorManager if needed."""
        if self._sensor_mgr is None:
            #from .sensor_manager import SensorManager
            self._sensor_mgr = SensorManager(logging=self._logging)
        else:
            self._sensor_mgr.close()
        return self._sensor_mgr


    def open_sensor_port(self, port: int) -> bool:
        """Open a sensor port.  Returns True if sensors found.
        Can be called by other modules (e.g. AutoDriveMgr) that
        need to reuse the SensorManager."""
        sensor_mgr = self._ensure_sensor_mgr()
        return sensor_mgr.open(port)


    @property
    def sensor_mgr(self):
        """Direct access to the underlying SensorManager instance."""
        return self._sensor_mgr


    @property
    def port_selected(self) -> int:
        """Currently selected I2C port number (1–6)."""
        return self._port_selected

    @port_selected.setter
    def port_selected(self, value: int):
        self._port_selected = value


    @property
    def colour(self) -> tuple:
        """Currently detected colour as an (r, g, b) tuple with values in the 0.0-1.0 range."""
        return self._colour

    @colour.setter
    def colour(self, value: tuple):
        self._colour = value


    @staticmethod
    def lookup_color_XYZ(x: int, y: int, z: int, brightness_threshold: int = 10) -> str:    #pylint: disable=invalid-name
        """
        Identifies a color name by searching through the COLOR_REGIONS table.
            Parameters:
            x, y, z: The CIE 1931 XYZ color space coordinates. values 0-1024 (10-bit)
            brightness_threshold: An integer representing the minimum brightness (Y) required to consider a color valid. Colors with Y below this threshold will be classified as "Black".
        """
        # 1. Handle total darkness/Black (also guarantees no divide-by-zero in the next step)
        if y < brightness_threshold:
            return "Black"

        total = x + y + z

        # 2. Calculate coordinates
        x_coord = x / total
        y_coord = y / total

        # 3. Search the lookup table
        for region in COLOR_REGIONS:
            x_min, x_max = region["x"]
            y_min, y_max = region["y"]

            if x_min <= x_coord <= x_max and y_min <= y_coord <= y_max:
                return region["name"]

        return "Unknown"

    # Example: Testing a Red measurement
    # result = lookup_color(0.6, 0.3, 0.1)
    # print(f"Detected: {result}")


    @staticmethod
    def lookup_colour_RGB(r: int, g: int, b: int, clear: int = 0) -> str:    #pylint: disable=invalid-name  #pylint: disable=invalid-name
        """Identifies a color name from raw RGB channel readings using HSV colour space.

        HSV naturally separates chromatic colour (hue) from achromatic attributes
        (saturation and brightness), giving far more accurate named-colour matching
        than projecting R/G/B onto the CIE xy diagram.

        Parameters:
            r, g, b : raw channel values (any consistent scale — need not be 0-255).
            clear   : optional broadband clear-channel reading from the same sensor.
                      When provided it is used as a brightness reference to distinguish
                      Black from Gray from White in low-saturation scenes.
        """
        max_c = max(r, g, b)
        if max_c == 0:
            return "Black"

        min_c = min(r, g, b)
        delta = max_c - min_c

        # Saturation (0.0 – 1.0): how far from grey the colour is
        s = delta / max_c

        # --- Achromatic branch (low saturation) ---
        if s < 0.20:
            # Use the clear channel as a brightness reference when available;
            # otherwise fall back to max_c compared to the channel ceiling.
            brightness_ref = clear if clear > 0 else max_c
            if brightness_ref > 0:
                reflectance = max_c / brightness_ref
            else:
                reflectance = 0.0
            if reflectance < 0.15:
                return "Black"
            if reflectance > 0.65:
                return "White"
            return "Gray"

        # --- Chromatic branch: compute hue (0 – 360°) ---
        if max_c == r:
            h = 60.0 * (((g - b) / delta) % 6)
        elif max_c == g:
            h = 60.0 * ((b - r) / delta + 2)
        else:
            h = 60.0 * ((r - g) / delta + 4)

        if h < 20 or h >= 340:
            return "Red"
        if h < 45:
            return "Orange"
        if h < 70:
            return "Yellow"
        if h < 150:
            return "Green"
        if h < 200:
            return "Cyan"
        if h < 260:
            return "Blue"
        return "Magenta"


    @staticmethod
    def _apply_white_reference(r: int, g: int, b: int, w: int = 0, white_gains: tuple[int, int, int, int] | None = None) -> tuple[int, int, int, int]:
        """Apply white reference gains to raw RGBC values and return adjusted RGBC tuple."""
        if white_gains is None:
            return (r, g, b, w)
        return (
            max(0, ((r * white_gains[0]) + (_WHITE_CAL_SCALE // 2)) // _WHITE_CAL_SCALE),
            max(0, ((g * white_gains[1]) + (_WHITE_CAL_SCALE // 2)) // _WHITE_CAL_SCALE),
            max(0, ((b * white_gains[2]) + (_WHITE_CAL_SCALE // 2)) // _WHITE_CAL_SCALE),
            max(0, ((w * white_gains[3]) + (_WHITE_CAL_SCALE // 2)) // _WHITE_CAL_SCALE) if w > 0 else 0,
        )


    # ------------------------------------------------------------------
    # Background update (called from the fast loop)
    # ------------------------------------------------------------------

    def background_update(self, delta) -> tuple[int, int] | None:  # pylint: disable=unused-argument
        """Perform background updates based on the current sub-state."""
        if self._sub_state == _SUB_READING:
            sensor_mgr = self._sensor_mgr
            if sensor_mgr is None:
                return None

            self._count_timer += delta
            if self._count_timer >= 1000:
                # compute sample rate every second based on the number of samples read and the elapsed time
                self._sample_rate = ((1000 * self.sample_count) + 500) // self._count_timer # sample rate in Hz
                self._count_timer = 0
                self.sample_count = 0
                self._new_sample = True

            # need per sensor read timing here to balance responsiveness with CPU load,
            # since some sensors can be slow to read and we don't want to bog down the system by reading too frequently.
            # We also want to update the displayed sample rate at a regular interval (e.g. every second) based on the number of samples read in that time.
            #self._read_timer += delta
            #if self._read_timer >= self._sensor_mgr.read_interval:
                #print(f"ST:Reading sensor (S:read_timer={self._read_timer}ms, count_timer={self._count_timer}ms, sample_count={self.sample_count})")
                #self._count_timer += self._read_timer
                #self._read_timer = 0
            # Read sensor data in the background and update sample count and rate calculation
            # TODO - make this more generic - interrupt property of sensor, and avoid having code split between sensor test and sensor manager...
            # if colour sensor - see if the interrupt pin is active (low) before trying to read, to avoid long waits when the sensor is not ready with new data
            config = HexpansionConfig(self._port_selected)
            if sensor_mgr.type == "Colour":
                if config.ls_pin[_COLOUR_INT_PIN].value():
                    # interrupt pin active low - NOT active, so sensor not ready with new data
                    return None
                self._test_results["colour int low"] = True
            elif sensor_mgr.type == "Distance":
                if config.ls_pin[_DIST_INT_PIN].value():
                    #return None
                    pass
                else:
                    self._test_results["distance int low"] = True

            try:
                self._sensor_data = sensor_mgr.read_current()
                self.sample_count = self.sample_count + 1
            except Exception as e:      # pylint: disable=broad-exception-caught
                self._sensor_data = {"Error": str(e)}

            if sensor_mgr.type == "Colour":
                if config.ls_pin[_COLOUR_INT_PIN].value():
                    self._test_results["colour int high"] = True
            elif sensor_mgr.type == "Distance":
                if config.ls_pin[_DIST_INT_PIN].value():
                    self._test_results["distance int high"] = True

        return None


    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta: int):
        """Handle Sensor Test states."""
        if self._sub_state == _SUB_SELECT_PORT:
            self._update_select_port(delta)
        elif self._sub_state == _SUB_READING:
            self._update_reading(delta)


    def _setup_for_sensor_type(self):
        sensor_mgr = self._sensor_mgr
        if sensor_mgr is None:
            return

        if self.logging:
            print(f"ST:Opened sensor port {self._port_selected} with read_interval {sensor_mgr.read_interval}ms")
        self._app.update_period = sensor_mgr.read_interval

        # Reset all sensor and display data when starting to read a new sensor
        self._sensor_data = {}
        self._display_data = {}
        self._read_timer = 0
        self._count_timer = 0
        self._sample_rate = 0
        self._sample_count = 0
        self._new_sample = False
        self._colour = (1.0, 1.0, 0.0)  # reset to yellow when switching sensors

        # Sensor specific setup
        if sensor_mgr.type == "Colour":
            self._white_gains = self._load_white_gains("ref")
            if self._white_gains is not None and self.logging:
                print(f"ST:Loaded white gains from settings: {self._white_gains}")


    def _update_select_port(self, delta: int):   # pylint: disable=unused-argument
        app = self._app
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
            sensor_mgr = self._ensure_sensor_mgr()
            app.refresh = True
            if sensor_mgr.open(self._port_selected):
                self._setup_for_sensor_type()
                self._sub_state = _SUB_READING
            else:
                app.notification = Notification("      No      Sensors", port=self._port_selected)
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if self.logging:
                print("Exiting Sensor Test")
            if self._sensor_mgr is not None:
                self._sensor_mgr.close()
            app.return_to_menu()


    @staticmethod
    def _ordered_display_items(display_data: dict) -> list[tuple[str, str]]:
        """ with dicts not maintinaing order we need to force into a list in the order we want to display"""
        items = []
        seen = set()
        for key in ("r", "g", "b"):
            if key in display_data:
                items.append((key, str(display_data[key])))
                seen.add(key)
        for key, value in display_data.items():
            if key not in seen:
                items.append((key, str(value)))
        return items


    @staticmethod
    def _white_gain_setting_keys(key: str) -> tuple[str, str, str, str]:
        base = f"{_WHITE_CAL_GAIN_PREFIX}_{key}_"
        return (f"{base}r", f"{base}g", f"{base}b", f"{base}w")


    @staticmethod
    def _reference_to_gains(r: int, g: int, b: int, w: int = 0) -> tuple[int, int, int, int]:
        ref_r = max(int(r), 1)
        ref_g = max(int(g), 1)
        ref_b = max(int(b), 1)
        ref_w = max(int(w), 1) if w > 0 else _WHITE_CAL_SCALE
        gain_scale = _WHITE_CAL_SCALE * _WHITE_CAL_SCALE
        return (
            (gain_scale + (ref_r // 2)) // ref_r,
            (gain_scale + (ref_g // 2)) // ref_g,
            (gain_scale + (ref_b // 2)) // ref_b,
            (gain_scale + (ref_w // 2)) // ref_w,
        )


    def _load_white_gains(self, key: str) -> tuple[int, int, int, int] | None:
        setting_keys = self._white_gain_setting_keys(key)
        values = []
        for setting_key in setting_keys:
            value = platform_settings.get(f"{SETTINGS_NAME_PREFIX}.{setting_key}", None)
            if value is None:
                return None
            values.append(int(value))
        gains = (values[0], values[1], values[2], values[3])
        return gains


    def _update_page_count(self) -> None:
        sensor_mgr = self._sensor_mgr
        self._page_count = 4 if sensor_mgr is not None and sensor_mgr.type == "Colour" else 3
        if self._page_selected >= self._page_count:
            self._page_selected = _PAGE_RAW


    def _capture_white_reference(self) -> bool:
        sensor_mgr = self._sensor_mgr
        if sensor_mgr is None or sensor_mgr.type != "Colour":
            return False
        if not all(key in self._sensor_data for key in ("r", "g", "b")):
            return False

        gains = self._reference_to_gains(
            int(self._sensor_data["r"]),
            int(self._sensor_data["g"]),
            int(self._sensor_data["b"]),
            int(self._sensor_data.get("w", 0)),
        )
        # Update white gains in this manager
        self._white_gains = gains

        # Save white gains to platform settings for persistence across sessions and availability in other modules
        setting_keys = self._white_gain_setting_keys("ref")
        for setting_key, gain in zip(setting_keys, gains):
            platform_settings.set(f"{SETTINGS_NAME_PREFIX}.{setting_key}", gain)
        if self._logging:
            print(f"ST:Stored white gains: {gains}")
        self._app.notification = Notification("White Cal    Saved", port=self._port_selected)
        return True


    def _update_display_values(self):      # pylint: disable=unused-argument
        # clear old display data
        self._display_data = {}
        self._update_page_count()

        # Sensor-specific display logic based on sensor type and available data
        if self._sensor_mgr and self._sensor_mgr.type == "Colour":
            # Colour Sensors typically provide either raw RGB channels or CIE XYZ channels, so check which we have and process accordingly.
            if all(k in self._sensor_data for k in ("x", "y", "z")):
                try:
                    x = int(self._sensor_data["x"])
                    y = int(self._sensor_data["y"])
                    z = int(self._sensor_data["z"])

                    if self._sensor_mgr.current_sensor_name == "OPT4048":
                        # OPT4048 requires a matrix transform to convert from its raw XYZ to standard CIE1931 XYZ.
                        x1 = int( 2.3489 * x + 0.4075 * y + 0.9286 * z)
                        y1 = int(-0.1990 * x + 1.9896 * y - 0.1697 * z)
                        z1 = int( 0.1281 * x - 0.1588 * y + 6.7402 * z)
                        x = x1
                        y = y1
                        z = z1

                    if self._page_selected == _PAGE_DATA:
                        # Look up the colour name based on the chromaticity coordinates and brightness
                        colour_name = self.lookup_color_XYZ(x, y, z)
                        if colour_name == "Unknown":
                            total = x + y + z
                            x_f = x / total
                            y_f = y / total
                            colour_name = f"x={x_f:.2f}, y={y_f:.2f}"
                        self._display_data["colour"] = colour_name
                    elif self._page_selected == _PAGE_RAW:
                        self._display_data = self._sensor_data
                    elif self._page_selected == _PAGE_CAL:
                        self._display_data["ref"] = "white"
                        self._display_data["press"] = "CONFIRM"

                    #convert CIE1931 XYZ to RGB using a simple matrix transform
                    r = int( 3.2406 * x - 1.5372 * y - 0.4986 * z)
                    g = int(-0.9689 * x + 1.8758 * y + 0.0415 * z)
                    b = int( 0.0557 * x - 0.2040 * y + 1.0570 * z)


                except Exception as e:    # pylint: disable=broad-exception-caught
                    print(f"ST:Colour conversion error: {e}")
                    r = g = b = 0

            elif all(k in self._sensor_data for k in ("r", "g", "b")):
                try:
                    r = int(self._sensor_data["r"])
                    g = int(self._sensor_data["g"])
                    b = int(self._sensor_data["b"])
                    w = int(self._sensor_data.get("w", 0))
                    calibrated_r, calibrated_g, calibrated_b, calibrated_w = self._apply_white_reference(r, g, b, w, self._white_gains)

                    if self._page_selected == _PAGE_DATA:
                        colour_name = self.lookup_colour_RGB(calibrated_r, calibrated_g, calibrated_b, calibrated_w)
                        self._display_data["colour"] = colour_name
                    elif self._page_selected == _PAGE_RAW:
                        self._display_data = self._sensor_data
                    elif self._page_selected == _PAGE_CAL:
                        self._display_data["ref"] = "white"
                        self._display_data["press"] = "CONFIRM"

                except Exception as e:    # pylint: disable=broad-exception-caught
                    print(f"ST:Colour conversion error: {e}")
                    r = g = b = 0
            else:
                r = g = b = 0

            # if the sensor is a colour sensor: create colour which can be used with ctx to show the detected colour
            max_channel = max(r, g, b)
            if max_channel > 0:
                # display colour from sensed colour
                red_f   = r / max_channel
                green_f = g / max_channel
                blue_f  = b / max_channel
                self._colour = (red_f, green_f, blue_f)
            else:
                self._colour = (1.0,1.0,0.0)  # default to yellow if all channels are zero to avoid divide-by-zero and to provide a visible colour for non-colour sensors
        elif self._sensor_mgr and self._sensor_mgr.type == "Distance":
            if self._page_selected == _PAGE_DATA and "dist" in self._sensor_data:
                try:
                    dist_mm = int(self._sensor_data["dist"])
                    if dist_mm < 20:
                        distance_str = "V Close"
                    elif dist_mm < 100:
                        distance_str = "Close"
                    elif dist_mm < 500:
                        distance_str = "Medium"
                    else:
                        distance_str = "Far"
                    self._display_data["Range"] = distance_str
                    self._display_data["Dist"] = f"{dist_mm}mm"
                except Exception as e:    # pylint: disable=broad-exception-caught
                    print(f"ST:Distance processing error: {e}")
            elif self._page_selected == _PAGE_RAW:
                self._display_data = self._sensor_data
        elif self._page_selected == _PAGE_RAW:
            self._display_data = self._sensor_data

        if self._page_selected == _PAGE_STATS:
            if self._sample_rate > 0:
                self._display_data["rate"] = f"{self._sample_rate}Hz"


    def _update_reading(self, delta: int):      # pylint: disable=unused-argument
        app = self._app
        if self._new_sample:  # reading is handled in background_update, which updates self._sensor_data when a new reading is available
            self._new_sample = False
            self._update_display_values()
            app.refresh = True

        if app.button_states.get(BUTTON_TYPES["RIGHT"]) and self._sensor_mgr and self._sensor_mgr.num_sensors > 1:
            app.button_states.clear()
            self._sensor_mgr.next_sensor()
            self._setup_for_sensor_type()  # reset any sensor-specific settings for the new sensor
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]) and self._sensor_mgr and self._sensor_mgr.num_sensors > 1:
            app.button_states.clear()
            self._sensor_mgr.prev_sensor()
            self._setup_for_sensor_type()  # reset any sensor-specific settings for the new sensor
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
            if self._page_selected == _PAGE_CAL and self._capture_white_reference():
                self._update_display_values()
                app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            sensor_mgr = self._sensor_mgr
            if sensor_mgr is not None:
                sensor_mgr.close()
            app.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
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
            self._draw_reading(ctx)
            return True
        return False


    def _draw_select_port(self, ctx):
        self._app.draw_message(ctx,
            ["Sensor Test", f"Port: {self._port_selected}"],
            [(1, 1, 0), (0, 1, 1)],
            label_font_size)
        button_labels(ctx, left_label="<Port", right_label="Port>",
                      confirm_label="Scan", cancel_label="Back")


    def _draw_reading(self, ctx):
        up_label = down_label = confirm_label = ""
        sensor_mgr = self._sensor_mgr
        num_sensors = sensor_mgr.num_sensors if sensor_mgr else 1
        sensor_name = sensor_mgr.current_sensor_name if sensor_mgr else "Sensor"
        if num_sensors > 1:
            current_sensor_index = sensor_mgr.current_sensor_index if sensor_mgr else 0
            lines = [f"Slot {self._port_selected}-{current_sensor_index + 1}/{num_sensors}"]
        else:
            lines = [f"Slot {self._port_selected}"]
        colours = [(1, 1, 0)]
        if self._page_count == 0:
            lines += [sensor_name]
        else:
            lines += [f"{sensor_name}-{_PAGE_NAMES[self._page_selected]}"]
        colours += [(1, 0, 1)]
        if self._display_data:
            for label, value in self._ordered_display_items(self._display_data):
                lines += [f"{label}:{value}"]
                colours += [self._colour]
        else:
            lines += ["Reading..."]
            colours += [(0.5, 0.5, 0.5)]

        if self._page_count > 0:
            # Button labels: up/down show destination page names
            down_page = (self._page_selected + 1) % self._page_count
            up_page = (self._page_selected - 1) % self._page_count
            # Only show the DOWN label if there is more than 1 page, otherwise there is no other page to go to
            down_label=_PAGE_NAMES[down_page] if self._page_count > 1 else ""
            # only show the UP label if there are more than 2 pages, otherwise it would just show the same as the DOWN
            up_label=_PAGE_NAMES[up_page] if self._page_count > 2 else ""
            if self._page_selected == _PAGE_CAL and sensor_mgr is not None and sensor_mgr.type == "Colour":
                confirm_label = "Store"

            if num_sensors > 1:
                button_labels(ctx, left_label="<Prev", right_label="Next>",
                            up_label=up_label, down_label=down_label, cancel_label="Back", confirm_label=confirm_label)
            else:
                button_labels(ctx, up_label=up_label, down_label=down_label, cancel_label="Back", confirm_label=confirm_label)

        # Ensure there are always 5 lines to draw for consistent layout, even if some are blank
        while len(lines) < 5:
            lines.append("")
            colours.append((1, 1, 1))

        self._app.draw_message(ctx, lines, colours, label_font_size)


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
    if not (ctrl & _PCNT_CTRL_CLK_EN):
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

    def __init__(
        self,
        unit: int | None,
        phase_a: int,
        phase_b: int,
        filter_ns: int = 0,
        max: int | None = None,
        min: int = 0,
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
        if not self.init(phase_a, phase_b, filter_ns=filter_ns, max=max, min=min):
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
        max: int | None = None,
        min: int = 0,
    ) -> bool:
        """Configure the unit for 4x quadrature decoding on *phase_a* and *phase_b*."""
        unit = self.unit
        if unit is None:
            return False
        if phase_a == phase_b:
            self._log("PCNT: encoder phase_a and phase_b must use different GPIOs")
            return False

        range_enabled = max is not None and not (max == 0 and min == 0)
        range_max = 0 if max is None else max
        range_min = 0 if max is None else min
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
            if self._range_enabled and not (self._range_min <= 0 <= self._range_max):
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
