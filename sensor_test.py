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
from system.hexpansion.util import detect_eeprom_addr, get_hexpansion_block_devices, read_hexpansion_header
import settings as platform_settings
import vfs

from egpio import ePin
from .sensor_manager import SensorManager

from .app import SETTINGS_NAME_PREFIX, DEFAULT_BACKGROUND_UPDATE_PERIOD, MOTOR_PWM_FREQ, STATE_SENSOR

try:
    from machine import Pin, mem32, disable_irq, enable_irq
except ImportError:
    from machine import Pin

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


# Constants for rotation rate measurement and motor test mode.
_ROTATION_RATE_MEASUREMENT_PERIOD_MS = 2500     # how often to update the displayed rotation rate measurement in ms (tradeoff between display responsiveness and stability of the reading)
_DEFAULT_ROTATION_RATE_EMITTER_DUTY = 20        # default duty cycle for the IR emitter when doing rate testing, 0-255 (0=off, 255=full on)
_DEFAULT_SPOKES_PER_ROTATION = 3                # number of times the photodiode will be triggered per full rotation of the wheel
_MOTOR_TEST_BACKGROUND_UPDATE_PERIOD = 1000     # background update period in ms to use during motor test mode (tradeoff between display responsiveness and CPU load)
_ROTATION_RATE_EMITTER_PINS = [1, 2]            # LS_B & LS_C pins used to drive the IR emitter for rotation rate testing
_ROTATION_RATE_SENSOR_PINS = [0, 1]             # HS_F & HS_G pins used to read the phottransistors for rotation rate testing
_ROTATION_RATE_SENSOR_ENABLE_PINS = [3, 4]      # LS_D & LS_E pins used to enable the phototransistors for rotation rate testing (set to output and high to enable, input to disable)
_IR_EMITTER_PWM_STEP_SIZE = 2                   # Step size for adjusting IR emitter brightness in manual mode, 0-255 (0=off, 255=full on)


# Local sub-states (internal to Sensor Test)
_SUB_SELECT_PORT = 0
_SUB_READING     = 1
_SUB_MOTOR_TEST  = 2

# Rotation Rate Auto scan configuration
_AUTO_SCAN_STEPS       = 60     # Number of power levels to test during auto scan
_AUTO_SCAN_SETTLE_MS   = 320    # ms to wait after setting power before starting actual measurement period
_AUTO_SCAN_MEASURE_MS  = 5000   # ms measurement window per step (maximum)
_AUTO_RESULTS_FILENAME = "mtrtst.csv"
_AUTO_RESULTS_DEST_LABELS = ("badge fs", "hex fs")


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

# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting: type):       # pylint: disable=unused-argument, invalid-name
    """Register sensor-test-specific settings in the shared settings dict.
    Currently only the motor-test CSV destination is exposed here."""
    s["path"] = MySetting(s, 0, 0, len(_AUTO_RESULTS_DEST_LABELS) - 1, labels=_AUTO_RESULTS_DEST_LABELS)


# ---- Sensor Test manager ---------------------------------------------------

class SensorTestMgr:
    """Manages the Sensor Test workflow.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """

    def __init__(self, app, hextest_port: int | None = None, logging: bool = False):
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

        self._rotation_rate_emitter_duty: int = _DEFAULT_ROTATION_RATE_EMITTER_DUTY # duty cycle for the IR emitter when doing rate testing, 0-255 (0=off, 255=full on)
        self._rotation_rate_counters = []                           # hardware counters used to count photodiode pulses for rate testing
        self._rotation_rate_rpms: list[int] = []                    # computed RPM values derived from counter deltas
        self._rotation_rate_measurement_period: int = _ROTATION_RATE_MEASUREMENT_PERIOD_MS
        self._rotation_rate_measurement_period_elapsed: int = 0     # ticks since last rate check, used to compute pulse rate in Hz based on the change in the counter value
        self._rotation_rate_motor_power: int = 0                    # Power applied to motors in TEST mode
        self._rotation_rate_spokes: int = _DEFAULT_SPOKES_PER_ROTATION

        # Auto scan state
        self._auto_mode: bool = False             # True = auto scanning, False = manual
        self._auto_direction: int = 1             # 1 = forwards, -1 = reverse
        self._auto_step: int = 0                  # current step index (0.._AUTO_SCAN_STEPS-1)
        self._auto_settling: bool = True          # True = in settle phase, False = in measure phase
        self._rotation_detected: bool = False     # True once motion has been observed during auto scan
        self._auto_results: list[tuple[int, list[int], int | None]] = []   # list of (power, rpm list, current mA)
        self._auto_max_rpm: int = 0               # max rpm seen during scan
        self._auto_max_current_ma: int = 0        # max current seen during scan
        self._auto_last_current_ma: int = 0       # latest current sampled in auto mode
        self._auto_done: bool = False             # True = scan complete
        self._motor_calibration_fit: list[tuple[float, float] | None] = []  # list of (slope, intercept) fits, indexed by motor number

        self._ina226 = None
        self._ina226_sensor_mgr = None  # SensorManager used exclusively for motor-test INA226 discovery
        self._ina226_reading: dict[str, int] = {}
        self._ina226_sum_current_ma: int = 0
        self._ina226_sum_bus_mv: int = 0
        self._ina226_sample_count: int = 0

        # Use HS pins on a spare Hexpansion to measure rotation rate
        self._test_support_hexpansion_config: HexpansionConfig | None = None
        self.hextest_setup(hextest_port)

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

    @property
    def _rotation_rate_rounding(self) -> int:
        return (self._rotation_rate_measurement_period * self._rotation_rate_spokes) // 2


    def hextest_setup(self, port: int | None):
        """Use HS pins on a spare Hexpansion to make rotation rate measurements."""
        if self._test_support_hexpansion_config is not None and port != self._test_support_hexpansion_config.port:
            try:
                for i in range(4):
                    self._test_support_hexpansion_config.pin[i].init(mode=Pin.IN)
                if self._sub_state == _SUB_MOTOR_TEST:
                    if self._logging:
                        print(f"Test Hexpansion {'removed' if port is None else 'changed'}")
                    self._app.notification = Notification("Motor Test   aborted", port=self._test_support_hexpansion_config.port)
                    self._stop_motor_test_mode()
            except AttributeError:
                pass  # Simulator Pin stubs lack .init()
            self._test_support_hexpansion_config = None
        if port is not None and self._test_support_hexpansion_config is None:
            if self._logging:
                print(f"Setting up Hexpansion on port {port} for rotation rate measurement")
            self._test_support_hexpansion_config = HexpansionConfig(port)
            self._rotation_rate_enable(False)  # start with rotation rate emitter and sensors off until we enter motor test mode


    def _rotation_rate_sensor_pair(self, pair_index: int = 0) -> tuple[int, int] | None:
        """Return the requested HS sensor pin pair from `_ROTATION_RATE_SENSOR_PINS`."""
        start = pair_index * 2
        if start < 0 or start + 1 >= len(_ROTATION_RATE_SENSOR_PINS):
            return None
        return _ROTATION_RATE_SENSOR_PINS[start], _ROTATION_RATE_SENSOR_PINS[start + 1]


    def encoder_smoke_test(
        self,
        samples: int = 12,
        interval_ms: int = 250,
        filter_ns: int = 1_000_000,
        max: int | None = 3,
        min: int = 0,
    ) -> bool:
        """Run a short console-based encoder smoke test on the first HexTest sensor pair."""
        if samples <= 0:
            print("S:Encoder smoke test requires at least one sample")
            return False
        if interval_ms < 0:
            print("S:Encoder smoke test requires interval_ms >= 0")
            return False
        if self._sub_state == _SUB_MOTOR_TEST:
            print("S:Encoder smoke test unavailable while motor test mode is active")
            return False

        config = self._test_support_hexpansion_config
        if config is None:
            print("S:Encoder smoke test requires a HexTest Hexpansion")
            return False

        hs_pair = self._rotation_rate_sensor_pair(0)
        if hs_pair is None:
            print("S:Encoder smoke test requires at least one HS sensor pin pair")
            return False

        gpios = _HS_PIN_TO_GPIO.get(config.port)
        if gpios is None:
            print(f"S:Encoder smoke test does not know the GPIO mapping for port {config.port}")
            return False

        phase_a_pin, phase_b_pin = hs_pair
        phase_a_gpio = gpios[phase_a_pin]
        phase_b_gpio = gpios[phase_b_pin]
        range_desc = "hardware range" if max is None else f"min={min}, max={max}"

        self._rotation_rate_enable(True)
        encoder = Encoder(
            None,
            phase_a_gpio,
            phase_b_gpio,
            filter_ns=filter_ns,
            max=max,
            min=min,
            logging=True,
        )
        if encoder.unit is None:
            print(
                f"S:Encoder smoke test failed on HexTest port {config.port} "
                f"HS pins {phase_a_pin}/{phase_b_pin}"
            )
            self._rotation_rate_enable(False)
            return False

        print(
            f"S:Encoder smoke test on HexTest port {config.port}, "
            f"HS pins {phase_a_pin}/{phase_b_pin}, GPIOs {phase_a_gpio}/{phase_b_gpio}, {range_desc}"
        )
        print("S:Rotate the wheel by hand and watch position/cycles for direction and wrap behaviour")

        try:
            print(f"S:Encoder initial: position={encoder.value()}, cycles={encoder.cycles()}")
            for sample_index in range(samples):
                _sleep_ms(interval_ms)
                print(
                    f"S:Encoder sample {sample_index + 1}/{samples}: "
                    f"position={encoder.value()}, cycles={encoder.cycles()}"
                )

            final_position = encoder.value()
            final_cycles = encoder.cycles()
            encoder.value(0)
            print(
                f"S:Encoder reset after position={final_position}, cycles={final_cycles}; "
                f"now position={encoder.value()}, cycles={encoder.cycles()}"
            )
            print("S:Encoder smoke test complete")
            return True
        finally:
            encoder.deinit()
            self._rotation_rate_enable(False)


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
        # If a HexTest is present then go straight to motor test mode.
        if len(app.hexdrive_apps) > 0 and self._test_support_hexpansion_config is not None and self._start_motor_test_mode():
            self._port_selected = self._test_support_hexpansion_config.port
            self._sub_state = _SUB_MOTOR_TEST
        elif app.hexdrive_ports is not None:
            # If a HexDrive is present try its port for sensors
            for port in app.hexdrive_ports:
                if sensor_mgr.open(port):
                    self._port_selected = port
                    app.update_period = sensor_mgr.read_interval
                    self._sub_state = _SUB_READING
                    break
        elif app.hexsense_port is not None and sensor_mgr.open(app.hexsense_port):
            # If no HexDrive, but a HexSense is present, try its port
            self._port_selected = app.hexsense_port
            app.update_period = sensor_mgr.read_interval
            self._sub_state = _SUB_READING
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


    @property
    def rotation_rate_emitter_duty(self) -> int:
        """Duty cycle (0-255) for the IR emitter when doing rotation rate testing."""
        return self._rotation_rate_emitter_duty

    @rotation_rate_emitter_duty.setter
    def rotation_rate_emitter_duty(self, value: int):
        self._rotation_rate_emitter_duty = value
        if self._test_support_hexpansion_config is not None:
            for pin_num in _ROTATION_RATE_EMITTER_PINS:
                self._test_support_hexpansion_config.ls_pin[pin_num].duty(self._rotation_rate_emitter_duty)


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
            config = HexpansionConfig(self._app.hexdrive_ports[0])
            if sensor_mgr.type == "Colour":
                if config.ls_pin[1].value():
                    # interrupt pin active low - NOT active, so sensor not ready with new data
                    return None
                self._test_results["colour int low"] = True
            elif sensor_mgr.type == "Distance":
                if config.ls_pin[3].value():
                    return None
                self._test_results["distance int low"] = True

            try:
                self._sensor_data = sensor_mgr.read_current()
                self.sample_count = self.sample_count + 1
            except Exception as e:      # pylint: disable=broad-exception-caught
                self._sensor_data = {"Error": str(e)}

            if sensor_mgr.type == "Colour":
                if config.ls_pin[1].value():
                    self._test_results["colour int high"] = True
            elif sensor_mgr.type == "Distance":
                if config.ls_pin[3].value():
                    self._test_results["distance int high"] = True

        elif self._sub_state == _SUB_MOTOR_TEST:
            self._sample_ina226_in_background()
            return (self._rotation_rate_motor_power, self._rotation_rate_motor_power)
        return None


    def _auto_rotation_rate_step(self):
        self._auto_step += 1
        self._app.refresh = True
        if self._auto_step >= _AUTO_SCAN_STEPS:
            # Scan complete — stop motors
            self._auto_done = True
            self._rotation_detected = False
            self._rotation_rate_motor_power = 0
            self._auto_direction *= -1  # reverse direction for next scan
            self._auto_fit_calculate()
            self._save_auto_results_csv()
        else:
            # Advance to next power level
            self._rotation_rate_motor_power = self._auto_direction * (65535 * self._auto_step) // (_AUTO_SCAN_STEPS - 1)
        self._rotation_rate_measurement_period_elapsed = 0
        self._auto_settling = True


    def _auto_results_dest_mode(self) -> int:
        setting = self._app.settings.get("path")
        if setting is None:
            return 0
        try:
            return int(setting.v)
        except Exception:      # pylint: disable=broad-exception-caught
            return 0


    def _mount_hexdrive_fs(self, port: int) -> tuple[str | None, bool]:
        mountpoint = f"/hexpansion_{port}"
        config = HexpansionConfig(port)
        eeprom_addr, addr_len = detect_eeprom_addr(config.i2c)
        if eeprom_addr is None or addr_len is None:
            print(f"ST:No EEPROM found on hexdrive port {port}")
            return None, False
        header = read_hexpansion_header(config.i2c, eeprom_addr=eeprom_addr, addr_len=addr_len)
        if header is None:
            print(f"ST:Failed to read hexdrive header on port {port}")
            return None, False
        try:
            _, partition = get_hexpansion_block_devices(config.i2c, header, eeprom_addr, addr_len=addr_len)
        except RuntimeError as exc:
            print(f"ST:Failed to get hexdrive block device: {exc}")
            return None, False
        mounted_here = True
        try:
            vfs.mount(partition, mountpoint, readonly=False)
        except OSError as exc:
            if exc.args and exc.args[0] == 1:
                mounted_here = False
            else:
                print(f"ST:Failed to mount {mountpoint}: {exc}")
                return None, False
        except Exception as exc:      # pylint: disable=broad-exception-caught
            print(f"ST:Failed to mount {mountpoint}: {exc}")
            return None, False
        return mountpoint, mounted_here


    def _auto_results_path(self) -> tuple[str | None, str | None, bool]:
        if self._auto_results_dest_mode() == 1:
            if len(self._app.hexdrive_ports) == 0:
                print("ST:No HexDrive present for hex fs CSV save")
                return None, None, False
            mountpoint, mounted_here = self._mount_hexdrive_fs(self._app.hexdrive_ports[0])
            if mountpoint is None:
                return None, None, False
            return f"{mountpoint}/{_AUTO_RESULTS_FILENAME}", mountpoint, mounted_here
        return f"/{_AUTO_RESULTS_FILENAME}", None, False


    def _save_auto_results_csv(self) -> bool:
        if len(self._auto_results) == 0:
            return False
        output_path, mountpoint, mounted_here = self._auto_results_path()
        if output_path is None:
            return False

        rpm_count = len(self._rotation_rate_rpms)
        header = ["pwr"] + [f"rpm{index + 1}" for index in range(rpm_count)] + ["ma"]

        try:
            with open(output_path, "wb") as csv_file:
                csv_file.write((",".join(header) + "\n").encode())
                for power, rpms, current_ma in self._auto_results:
                    row = [str(power)]
                    row.extend(str(rpm) for rpm in rpms)
                    row.append(str(current_ma))
                    csv_file.write(",".join(row).encode())
        except Exception as exc:      # pylint: disable=broad-exception-caught
            print(f"ST:Failed to save CSV {output_path}: {exc}")
            return False
        finally:
            if mounted_here and mountpoint is not None:
                try:
                    vfs.umount(mountpoint)
                except Exception as exc:      # pylint: disable=broad-exception-caught
                    print(f"ST:Failed to unmount {mountpoint}: {exc}")

        print(f"ST:Saved auto motor test CSV to {output_path}")
        return True


    @staticmethod
    def _linear_regression(points: list[tuple[int, int]]) -> tuple[float, float] | None:
        if len(points) < 2:
            return None
        count = len(points)
        sum_x = sum(point[0] for point in points)
        sum_y = sum(point[1] for point in points)
        sum_xx = sum(point[0] * point[0] for point in points)
        sum_xy = sum(point[0] * point[1] for point in points)
        denominator = (count * sum_xx) - (sum_x * sum_x)
        if denominator == 0:
            return None
        slope = ((count * sum_xy) - (sum_x * sum_y)) / denominator
        intercept = (sum_y - (slope * sum_x)) / count
        return slope, intercept


    def _auto_fit_calculate(self) -> None:
        self._motor_calibration_fit = []
        for index in range(len(self._rotation_rate_rpms)):
            points = [(power, rpms[index]) for power, rpms, _ in self._auto_results if index < len(rpms)]
            self._motor_calibration_fit.append(self._linear_regression(points))


    def _show_auto_results_fit(self) -> None:
        lines = ["Auto Scan Fit"]
        colours: list[tuple[float, float, float]] = [(1, 1, 0)]
        for index in range(len(self._rotation_rate_rpms)):
            fit = self._motor_calibration_fit[index] if index < len(self._motor_calibration_fit) else None
            if fit is None:
                lines.append(f"M{index + 1}: n/a")
            else:
                slope, intercept = fit
                lines.append(f"M{index + 1}: r={slope:.3f}p{intercept:+.1f}")
            colours.append(self._colour_for_index(index))
        self._app.show_message(lines, colours, return_state=STATE_SENSOR)


    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta: int):
        """Handle Sensor Test states."""
        if self._sub_state == _SUB_SELECT_PORT:
            self._update_select_port(delta)
        elif self._sub_state == _SUB_READING:
            self._update_reading(delta)
        elif self._sub_state == _SUB_MOTOR_TEST:
            self._update_motor_test_mode(delta)


    def _rotation_rate_enable(self, enable: bool = True) -> bool:
        if self._test_support_hexpansion_config is None:
            return False
        try:
            if enable:
                if self._logging:
                    print("ST:Enabling rotation rate emitters and sensors")
                for pin_num in _ROTATION_RATE_EMITTER_PINS:
                    self._test_support_hexpansion_config.ls_pin[pin_num].init(mode=ePin.PWM)  # Set LS pins to output mode to turn on the IR emitters
                    self._test_support_hexpansion_config.ls_pin[pin_num].duty(self.rotation_rate_emitter_duty)  # Set LS pins to the current duty cycle to drive the IR emitters)
                for pin_num in _ROTATION_RATE_SENSOR_ENABLE_PINS:
                    self._test_support_hexpansion_config.ls_pin[pin_num].init(mode=Pin.OUT)  # Set LS pins to output mode to enable the phototransistors for rotation rate measurement
                    self._test_support_hexpansion_config.ls_pin[pin_num].value(1)  # Set LS enable pins high to turn on the phototransistors for rotation rate measurement
            else:
                if self._logging:
                    print("ST:Disabling rotation rate emitters and sensors")
                for pin_num in _ROTATION_RATE_EMITTER_PINS:
                    self._test_support_hexpansion_config.ls_pin[pin_num].init(mode=Pin.IN)  # Set LS pins to input mode to turn off the IR emitters
                for pin_num in _ROTATION_RATE_SENSOR_ENABLE_PINS:
                    self._test_support_hexpansion_config.ls_pin[pin_num].init(mode=Pin.IN)  # Set LS pins to input mode to turn off the phototransistors for rotation rate measurement

            for pin_num in _ROTATION_RATE_SENSOR_PINS:
                self._test_support_hexpansion_config.pin[pin_num].init(mode=Pin.IN)  # Set HS pins to input mode to read the phototransistors for rotation rate measurement
        except AttributeError:
            pass  # Simulator Pin stubs lack .init()
        return True


    def _init_ina226_for_motor_test(self) -> bool:
        self._ina226 = None
        self._ina226_sensor_mgr = None
        self._ina226_reading = {}
        self._reset_ina226_accumulators()
        try:
            #from .sensor_manager import SensorManager
            mgr = SensorManager(logging=self._logging)
            # The INA226 sensor can't be on a port with an EEPROM because that would clash with the UUT EEPROM.
            for port in range(1, 7):
                if not mgr.open(port):
                    mgr.close()
                    if self._logging:
                        print(f"ST:INA226 - no sensors found on port {port}")
                    continue
                # Find the first INA226 sensor in the discovered list
                sensor = mgr.get_sensor_by_name("INA226")
                if sensor is not None:
                    self._ina226 = sensor
                    self._ina226_sensor_mgr = mgr
                    if self._logging:
                        print(f"ST:INA226 found @ 0x{sensor.i2c_addr:02X} on port {port}")
                    return True
                # No INA226 found; close the manager
                mgr.close()
        except Exception as e:      # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"ST:INA226 init failed: {e}")
        return False


    def _reset_ina226_accumulators(self) -> None:
        self._ina226_sum_current_ma = 0
        self._ina226_sum_bus_mv = 0
        self._ina226_sample_count = -1


    def _sample_ina226_in_background(self) -> None:
        sensor = self._ina226
        if sensor is None:
            return
        data = sensor.read_sample_if_ready()
        if data is None:
            return
        try:
            if self._ina226_sample_count >= 0:
                # only use samples after the first one, to allow the INA226 to settle after a power change before we start accumulating data for averaging
                self._ina226_sum_current_ma += int(data.get("mA", 0))
                self._ina226_sum_bus_mv += int(data.get("mV", 0))
            self._ina226_sample_count += 1
        except Exception as e:       # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"ST:INA226 sample error: {e}")
            return


    def _consume_ina226_average(self) -> int | None:
        if self._ina226_sample_count <= 0:
            self._ina226_reading = {}
            return None
        count = self._ina226_sample_count
        current_ma = self._ina226_sum_current_ma // count
        voltage_mv = self._ina226_sum_bus_mv // count
        self._ina226_reading = {
            "mA": current_ma,
            "mV": voltage_mv,
        }
        self._reset_ina226_accumulators()
        return current_ma


    def _update_motor_test_mode(self, delta: int):  # pylint: disable=unused-argument
        app = self._app
        if self._test_support_hexpansion_config is None:
            self._stop_motor_test_mode()
            return

        # CANCEL always exits motor test mode
        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            self._show_auto_results_fit()
            self._stop_motor_test_mode()
            return

        # CONFIRM toggles between manual and auto mode
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            self._rotation_rate_motor_power = 0
            self._auto_last_current_ma = 0
            self._rotation_rate_measurement_period_elapsed = 0
            self._reset_ina226_accumulators()
            for counter in self._rotation_rate_counters:
                if counter is not None:
                    counter.value(0)      # reset counter
            if self._auto_mode:
                # Switch back to manual
                self._show_auto_results_fit()
                self._rotation_rate_measurement_period = _ROTATION_RATE_MEASUREMENT_PERIOD_MS
                self._auto_mode = False
                self._auto_done = False
            else:
                # Start auto scan
                self._auto_mode = True
                self._auto_done = False
                self._auto_step = 0
                self._rotation_rate_measurement_period = _AUTO_SCAN_MEASURE_MS
                self._auto_settling = True
                self._auto_results = []
                self._auto_max_rpm = 10
                self._auto_max_current_ma = 50
                self._rotation_detected = False
            app.refresh = True
            return

        if self._auto_mode:
            if not self._auto_done:
                self._rotation_rate_measurement_period_elapsed += delta
                if self._auto_settling:
                    if self._rotation_rate_measurement_period_elapsed >= _AUTO_SCAN_SETTLE_MS:
                        # Settle phase done — discard counter and start measuring
                        count = 0
                        for counter in self._rotation_rate_counters:
                            if counter is not None:
                                count += counter.value(0)  # read-and-reset to discard
                        if count == 0 and not self._rotation_detected:

                            # There has been no motion from any motors - so we can skip the measure phase and move straight to the next power level
                            current_ma = self._consume_ina226_average()
                            if current_ma is not None:
                                current_abs = abs(current_ma)
                                self._auto_last_current_ma = current_ma
                                if current_abs > self._auto_max_current_ma:
                                    self._auto_max_current_ma = current_abs
                            power = self._rotation_rate_motor_power
                            self._rotation_rate_rpms = [0] * len(self._rotation_rate_counters)
                            if self._logging:
                                print(f"ST:Auto Scan Step {self._auto_step}/{_AUTO_SCAN_STEPS} - Power: {power}, Rate: 0 rpm, Current: {current_ma}mA")
                            self._auto_results.append((power//66, [0] * len(self._rotation_rate_counters), current_ma))
                            self._auto_rotation_rate_step()

                        else:
                            self._rotation_detected = True
                            # estimate how long we need to measure for based on the count we got during the settle period, to ensure we get a good RPM (2%)
                            # reading even at low speeds, while still keeping the overall scan time reasonable#
                            cpm = (60000 * count) // self._rotation_rate_measurement_period_elapsed # rounded down - never displayed
                            self._rotation_rate_measurement_period = min(_AUTO_SCAN_MEASURE_MS, (60000 * 50) // cpm) if cpm > 0 else _AUTO_SCAN_MEASURE_MS
                            self._rotation_rate_measurement_period_elapsed = 0
                            self._auto_settling = False
                            self._reset_ina226_accumulators()
                else:
                    if self._rotation_rate_measurement_period_elapsed >= self._rotation_rate_measurement_period:
                        # Measure phase done — read counter and record result
                        self._rotation_rate_rpms = [0] * len(self._rotation_rate_counters)
                        for index, counter in enumerate(self._rotation_rate_counters):
                            if counter is not None:
                                count = counter.value(0)
                                rpm = ((60000 * count) + self._rotation_rate_rounding) // (self._rotation_rate_measurement_period_elapsed * self._rotation_rate_spokes)
                                if rpm > self._auto_max_rpm:
                                    self._auto_max_rpm = rpm
                                self._rotation_rate_rpms[index] = rpm

                        ### duplicate of block above - could be a method
                        current_ma = self._consume_ina226_average()
                        if current_ma is not None:
                            current_abs = abs(current_ma)
                            self._auto_last_current_ma = current_ma
                            if current_abs > self._auto_max_current_ma:
                                self._auto_max_current_ma = current_abs
                        power = self._rotation_rate_motor_power
                        if self._logging:
                            print(f"ST:Auto Scan Step {self._auto_step}/{_AUTO_SCAN_STEPS} - Power: {power}, Rates: {self._rotation_rate_rpms} rpm, Current: {current_ma}mA")
                        self._auto_results.append((power//66, self._rotation_rate_rpms, current_ma))
                        self._auto_rotation_rate_step()

            # In auto mode, no manual button control for power/IR
            return
        else:
            # manual measurement mode
            self._rotation_rate_measurement_period_elapsed += delta
            if self._rotation_rate_measurement_period_elapsed >= self._rotation_rate_measurement_period:
                count = 0
                for index, counter in enumerate(self._rotation_rate_counters):
                    if counter is not None:
                        count = counter.value(0)  # read-and-reset to get the count for the elapsed period
                        self._rotation_rate_rpms[index] = ((60000 * count) + self._rotation_rate_rounding) // (self._rotation_rate_measurement_period_elapsed * self._rotation_rate_spokes)
                self._rotation_rate_measurement_period_elapsed = 0
                self._consume_ina226_average()
                #if self.logging:
                #    print(f"ST:Rotation Rates: {self._rotation_rate_rpms}")

        # Manual mode button handling
        if app.button_states.get(BUTTON_TYPES["UP"]):
            app.button_states.clear()
            self.rotation_rate_emitter_duty = min(255, self.rotation_rate_emitter_duty + _IR_EMITTER_PWM_STEP_SIZE)
            if self.logging:
                print(f"ST:IR+Emitter Duty: {self.rotation_rate_emitter_duty}")
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            app.button_states.clear()
            self.rotation_rate_emitter_duty = max(0, self.rotation_rate_emitter_duty - _IR_EMITTER_PWM_STEP_SIZE)
            if self.logging:
                print(f"ST:IR-Emitter Duty: {self.rotation_rate_emitter_duty}")
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["RIGHT"]):
            app.button_states.clear()
            self._rotation_rate_motor_power = min(65535, self._rotation_rate_motor_power + 1000)
            if self.logging:
                print(f"ST:Motor+Power: {self._rotation_rate_motor_power}")
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            app.button_states.clear()
            self._rotation_rate_motor_power = max(-65535, self._rotation_rate_motor_power - 1000)
            if self.logging:
                print(f"ST:Motor-Power: {self._rotation_rate_motor_power}")
            app.refresh = True


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
            motor_test_port = self._test_support_hexpansion_config.port if self._test_support_hexpansion_config is not None else 0
            if self._port_selected == motor_test_port:
                if self._start_motor_test_mode():
                    app.notification = Notification("Motor Test", port=self._port_selected)
                    if self.logging:
                        print(f"ST:Entering Motor Test mode on port {self._port_selected}")
                    self._sub_state = _SUB_MOTOR_TEST
                    app.refresh = True
            else:
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
                self._rotation_rate_enable(False)
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


    def _start_motor_test_mode(self) -> bool:
        # enable HexDrive power, ...
        app = self._app
        if len(app.hexdrive_apps) > 0 and self._test_support_hexpansion_config is not None:
            app.hexdrive_apps[0].set_logging(True)
            # Read INA226:
            if self._init_ina226_for_motor_test():
                if self._ina226 is not None:
                    ina226 = self._ina226
                    data = ina226.read(timeout=160)
                    try:
                        volts = int(data.get("mV", 0))
                        amps = int(data.get("mA", 0))
                    except Exception as e:          # pylint: disable=broad-exception-caught
                        print(f"ST:Error reading INA226 data: {e}")
                    else:
                        if 3000 <= volts <= 3200 and amps < 5:
                            if self.logging:
                                print("ST:INA226 initial voltage & current reading OK")
                            self._test_results["Power Off"] = True
                        else:
                            self._test_results["Power Off"] = False

            if app.hexdrive_apps[0].initialise() and app.hexdrive_apps[0].set_power(True) and app.hexdrive_apps[0].set_freq(MOTOR_PWM_FREQ):
                app.hexdrive_apps[0].set_keep_alive(2000)   # Updates can be quite slow as we are using the draw function
                #app.hexdrive_apps[0].set_motors((-1,-1))    # Try forcing PWM to be reinitialised by swapping direction.
                # Enable the IR emitter for measuring wheel rotation rate
                self._rotation_rate_enable(True)

                # Enable the phototransistor input for measuring wheel rotation rate
                for pin_num in _ROTATION_RATE_SENSOR_PINS:
                    # configure the ESP32S3 hardware to count pulses on the HS_F pin
                    # Counter not yet available in this Micropython port so we have created our own...
                    gpio_num = _HS_PIN_TO_GPIO[self._test_support_hexpansion_config.port][pin_num]
                    counter = Counter(None, gpio_num, filter_ns=1000000, logging=False)  # auto-select PCNT unit
                    if counter is not None and counter.unit is not None:
                        self._rotation_rate_counters.append(counter)
                    else:
                        if self.logging:
                            print(f"ST:Failed to allocate PCNT counter for pin {pin_num} (GPIO {gpio_num})")
                        app.notification = Notification("PCNT Init     Failed")
                        # deinit any counters we did manage to create before returning
                        for c in self._rotation_rate_counters:
                            if c is not None:
                                c.deinit()
                        self._rotation_rate_counters = []
                        return False
                if self.logging:
                    print(f"ST:Rate counter {self._rotation_rate_counters}")
                self._rotation_rate_measurement_period_elapsed = 0
                self._rotation_rate_rpms = [0] * len(self._rotation_rate_counters)

                if self._ina226_sensor_mgr is not None:
                    app.update_period = self._ina226_sensor_mgr.read_interval  # update at the sensor read interval
                else:
                    app.update_period = _MOTOR_TEST_BACKGROUND_UPDATE_PERIOD

                # If we don't have a distance sensor then we can do a simple loopback test
                sensor_mgr = self._sensor_mgr
                if sensor_mgr is not None and sensor_mgr.get_sensor_by_name("VL53L0X") is None:
                    # Loop back test for XSHUT - DIST_INT
                    config = HexpansionConfig(self._app.hexdrive_ports[0])
                    self._app.hexdrive_apps[0].set_dist_xshut(1)
                    if 1 == config.ls_pin[3].value():
                        self._test_results["XSHUT high"] = True
                        self._test_results["dist int high"] = True
                    else:
                        self._test_results["XSHUT high"] = False

                    self._app.hexdrive_apps[0].set_dist_xshut(0)
                    if 0 == config.ls_pin[3].value():
                        self._test_results["XSHUT low"] = True
                        self._test_results["dist int low"] = True
                    else:
                        self._test_results["XSHUT low"] = False
                app.update_period = _MOTOR_TEST_BACKGROUND_UPDATE_PERIOD
                return True
        if self.logging:
            print("ST:Failed to initialise for motor test mode")
        app.notification = Notification("Test Init     Failed")
        return False


    def _stop_motor_test_mode(self):
        if self._logging:
            print("ST:Stopping Motor Test mode and cleaning up")

        # Take voltage reading before we power down
        if self._ina226 is not None:
            ina226 = self._ina226
            data = ina226.read(timeout=160)
            try:
                volts = int(data.get("mV", 0))
            except Exception as e:              # pylint: disable=broad-exception-caught
                print(f"ST:Error reading INA226 data: {e}")
            else:
                self._test_results["5V Voltage"] = volts
                if 4900 <= volts <= 5300:
                    self._test_results["Power On"] = True
                else:
                    self._test_results["Power On"] = False


        # confirm all tests passed:
        if all(self._test_results.get(test, False) for test in ("Power Off", "Power On","XSHUT high", "XSHUT low", "colour int high", "colour int low", "dist int high", "dist int low")):
            if self.logging:
                print("ST:***** Test PASSED *****")
            self._app.notification = Notification("    Test     PASSED", port=self._port_selected)
        # Report test results
        print(f"ST:Test results: {self._test_results}")

        app = self._app
        self._auto_mode = False
        self._auto_done = False
        self._rotation_rate_motor_power = 0
        self._ina226_reading = {}
        self._reset_ina226_accumulators()
        if self._ina226 is not None:
            if self._ina226_sensor_mgr is not None:
                try:
                    self._ina226_sensor_mgr.close()
                except Exception as exc:          # pylint: disable=broad-exception-caught
                    if self._logging:
                        print("INA226 sensor manager close failed:", exc)
                self._ina226_sensor_mgr = None
        self._ina226 = None

        if len(app.hexdrive_apps) > 0:
            app.hexdrive_apps[0].set_freq(0)
            app.hexdrive_apps[0].set_power(False)

        for c in self._rotation_rate_counters:
            if c is not None:
                c.deinit()
        self._rotation_rate_counters = []

        app.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
        self._rotation_rate_enable(False)
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
        elif self._sub_state == _SUB_MOTOR_TEST:
            self._draw_motor_test_mode(ctx)
            return True
        return False


    def _draw_motor_test_mode(self, ctx):
        if self._test_support_hexpansion_config is None:
            return
        if self._auto_mode:
            self._draw_auto_scan(ctx)
            return
        #print("DRAWING")
        # Manual mode: show the current emitter duty cycle as a percentage in the label, and show the current photodiode reading and rate counter value in the display data
        lines = [f"IR:{int(self.rotation_rate_emitter_duty * 100 // 255)}%"]
        colours = [(1, 1, 0)]
        # Show power
        lines += [f"Pwr:{self._rotation_rate_motor_power}"]
        colours += [(0, 1, 1)]
        for index, rpm in enumerate(self._rotation_rate_rpms):
            if rpm is not None:
                lines += [f"{index}: {rpm}rpm"]
                colours += [(1, 0, 1)]
        if self._ina226_reading:
            lines += [f"I:{self._ina226_reading.get('mA', 0)}mA"]
            colours += [(0.3, 0.8, 1.0)]
            #lines += [f"V:{self._ina226_reading.get('mV', 0)}mV"]
            #colours += [(0.3, 0.8, 1.0)]
        self._app.draw_message(ctx, lines, colours, label_font_size)
        button_labels(ctx, up_label="IR+", down_label="IR-", cancel_label="Back",
                      left_label="Pwr-", right_label="Pwr+", confirm_label="Auto")


    def _draw_auto_scan(self, ctx):
        """Draw a chart of power vs RPM from the auto scan results."""
        # Chart area within the 240x240 circular display (origin at centre)
        chart_left = -90
        chart_right = 90
        chart_top = -65
        chart_bottom = 35
        chart_w = chart_right - chart_left
        chart_h = chart_bottom - chart_top

        # Background
        ctx.rgb(0.05, 0.05, 0.05).rectangle(chart_left - 5, chart_top - 5, chart_w + 10, chart_h + 10).fill()

        # Axes
        ctx.rgb(0.4, 0.4, 0.4)
        ctx.move_to(chart_left, chart_bottom).line_to(chart_right, chart_bottom).stroke()  # X axis
        ctx.move_to(chart_left, chart_bottom).line_to(chart_left, chart_top).stroke()      # Y axis

        n = len(self._auto_results)
        max_rpm = self._auto_max_rpm if self._auto_max_rpm > 0 else 1
        max_current_ma = self._auto_max_current_ma if self._auto_max_current_ma > 0 else 1

        if n > 1:
            # Plot data points as small bars.
            # Auto-scan results may contain either a scalar RPM or a list/tuple
            # of per-counter RPMs. Reduce multi-counter readings to a single
            # scalar for this chart by using the maximum measured RPM.
            bar_w = max(1, chart_w // _AUTO_SCAN_STEPS)
            for i in range(n):
                power, rpms, current_ma = self._auto_results[i]
                x = chart_left + (abs(power) * chart_w) // 100
                for index, rpm in enumerate(rpms):
                    h = (rpm * chart_h) // max_rpm
                    if h > 0:
                        # colour by index to differentiate multiple counters if present
                        ctx.rgb(*self._colour_for_index(index)).rectangle(x, chart_bottom - h - 1, bar_w, 2).fill()
                if current_ma is not None:
                    current_h = (abs(current_ma) * chart_h) // max_current_ma
                    marker_y = chart_bottom - current_h
                    ctx.rgb(1.0, 0.2, 0.2)
                    ctx.rectangle(x, marker_y - 1, bar_w, 2).fill()

        # Title and max RPM label
        ctx.font_size = label_font_size
        if self._auto_done:
            ctx.move_to(-50, chart_top - 25).text("Complete")

            ctx.font_size = label_font_size - 8
            ctx.rgb(0.0, 1.0, 1.0).move_to(chart_left, chart_bottom + 5 + ctx.font_size).text("0%")
            width = ctx.text_width("Power")
            ctx.move_to(-width//2, chart_bottom + 5 + ctx.font_size).text("Power")
            width = ctx.text_width("100%")
            ctx.move_to(chart_right - width, chart_bottom + 5 + ctx.font_size).text("100%")
            # provide a legend for the colours on the graph for the rpms only
            for index in range(len(self._rotation_rate_counters)):
                ctx.rgb(*self._colour_for_index(index)).move_to(chart_left+20, chart_bottom + 5 + ((index + 2) * (ctx.font_size))).text(f"Motor {index+1} RPM")
                # Plot best fit line
                fit = self._motor_calibration_fit[index] if index < len(self._motor_calibration_fit) else None
                if fit is None:
                    continue
                slope, intercept = fit
                # get min and max power values from the scan range
                left_power = self._auto_results[0][0]
                right_power = self._auto_results[n-1][0]
                # is intercept going to be with X or Y axis as only positive quadrant shown
                if intercept < 0:
                    x1 = chart_left - ((intercept * max_rpm) // slope)
                    y1 = chart_bottom
                else:
                    x1 = chart_left
                    y1 = chart_bottom - ((slope * left_power + intercept) * chart_h) // max_rpm
                # is line going to leave chart along the top or right edge?
                if slope * right_power + intercept > max_rpm:
                    x2 = chart_left + ((max_rpm - intercept) * right_power) // slope
                    y2 = chart_top
                else:
                    x2 = chart_right
                    y2 = chart_bottom - ((slope * right_power + intercept) * chart_h) // max_rpm
                    print(f"ST:Motor {index+1} calibration line: slope={slope}, intercept={intercept}, x1={x1}, y1={y1}, x2={x2}, y2={y2}")
                ctx.rgb(*self._colour_for_index(index)).move_to(x1, y1).line_to(x2, y2).stroke()

        else:
            progress = (self._auto_step * 100) // _AUTO_SCAN_STEPS
            ctx.rgb(1.0,1.0,1.0).move_to(-50, chart_top - 25).text(f"Scan {progress}%")

            # Instantaneous current label (updated live during the scan)
            ctx.font_size = label_font_size - 8
            for index, rpm in enumerate(self._rotation_rate_rpms):
                ctx.rgb(*self._colour_for_index(index)).move_to(chart_left+20, chart_bottom + 5 + ((index + 2) * (ctx.font_size))).text(f"Mtr{index+1}: {rpm}rpm")
            ctx.rgb(1.0, 0.2, 0.2).move_to(15, chart_bottom + 5 + ctx.font_size).text(f"{self._auto_last_current_ma}mA")

        # Y axis Maximum RPM and Current labels
        ctx.font_size = label_font_size - 8
        ctx.rgb(0.0, 1.0, 0.5).move_to(chart_left+20, chart_top - 5).text(f"rpm:{max_rpm}")
        ctx.rgb(1.0, 0.2, 0.2).move_to(5, chart_top - 5).text(f"mA:{max_current_ma}")

        #button_labels(ctx, cancel_label="Back", confirm_label="Manual")

    def _colour_for_index(self, index: int) -> tuple[float, float, float]:
        if index == 0:
            return (0.0, 1.0, 0.5)
        elif index == 1:
            return (1.0, 0.5, 0.0)
        else:
            return (1.0, 1.0, 1.0)

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
