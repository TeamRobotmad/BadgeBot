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

from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification
from system.hexpansion.config import HexpansionConfig
try:
    from egpio import ePin
except ImportError:
    class ePin:  # pylint: disable=invalid-name
        """Simulator stub for egpio.ePin – used only for ePin.PWM mode constant."""
        PWM = None
from .app import DEFAULT_BACKGROUND_UPDATE_PERIOD, MOTOR_PWM_FREQ
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


# Constants for rotation rate measurement and motor test mode.
_ROTATION_RATE_MEASUREMENT_PERIOD_MS = 2500     # how often to update the displayed rotation rate measurement in ms (tradeoff between display responsiveness and stability of the reading)
_DEFAULT_ROTATION_RATE_EMITTER_DUTY = 20        # default duty cycle for the IR emitter when doing rate testing, 0-255 (0=off, 255=full on)
_DEFAULT_SPOKES_PER_ROTATION = 3                # number of times the photodiode will be triggered per full rotation of the wheel
_MOTOR_TEST_BACKGROUND_UPDATE_PERIOD = 1000     # background update period in ms to use during motor test mode (tradeoff between display responsiveness and CPU load)
_ROTATION_RATE_EMITTER_PINS = [0,1,2,3,4]       # LS_C & LS_D pins used to drive the IR emitter for rotation rate testing
_ROTATION_RATE_SENSOR_PINS = [0, 1]             # HS_F & HS_G pins used to read the phottransistors for rotation rate testing
_ROTATION_RATE_SENSOR_ENABLE_PINS = [3]         # LS_D pins used to enable the phototransistors for rotation rate testing (set to output and high to enable, input to disable)
_IR_EMITTER_PWM_STEP_SIZE = 2                   # Step size for adjusting IR emitter brightness in manual mode, 0-255 (0=off, 255=full on)
# Temporary - while there is no EEPROM on the Test Hexpansion
_ROTATION_RATE_PORT = 1                         # Hexpansion slot used for rotation rate measurement

# Local sub-states (internal to Sensor Test)
_SUB_SELECT_PORT = 0
_SUB_READING     = 1
_SUB_MOTOR_TEST  = 2

# Rotation Rate Auto scan configuration
_AUTO_SCAN_STEPS       = 50     # Number of power levels to test during auto scan
_AUTO_SCAN_SETTLE_MS   = 200    # ms to wait after setting power before discarding counter
_AUTO_SCAN_MEASURE_MS  = 2000   # ms measurement window per step


# Pages of information to show for each sensor (can be switched with up/down buttons)
_PAGE_RAW = 0
_PAGE_STATS = 1
_PAGE_DATA = 2
_PAGE_NAMES = {
    0: "Raw",
    1: "Stats",
    2: "Data",
}

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
    Currently no dedicated settings, but the hook exists for future use."""
    # no sensor-test-specific settings at this time


# ---- Sensor Test manager ---------------------------------------------------

class SensorTestMgr:
    """Manages the Sensor Test workflow.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """

    def __init__(self, app, hextest_port: int | None = _ROTATION_RATE_PORT, logging: bool = False):
        self._app = app
        self._sub_state = _SUB_SELECT_PORT
        self._sensor_mgr = None  # SensorManager instance (lazy-imported)
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
        self._colour: tuple = (1.0, 1.0, 0.0)  # default to yellow for non-colour sensors

        self._rotation_rate_emitter_duty: int = _DEFAULT_ROTATION_RATE_EMITTER_DUTY # duty cycle for the IR emitter when doing rate testing, 0-255 (0=off, 255=full on)
        self._rotation_rate_counters = []                           # hardware counters used to count photodiode pulses for rate testing
        self._rotation_rate_rpms: list[int | None] = []             # computed RPM values derived from counter deltas
        self._rotation_rate_measurement_period_elapsed: int = 0     # ticks since last rate check, used to compute pulse rate in Hz based on the change in the counter value
        self._rotation_rate_motor_power: int = 0                    # Power applied to motors in TEST mode
        self._rotation_rate_spokes: int = _DEFAULT_SPOKES_PER_ROTATION
        self._rotation_rate_rounding: int = (_ROTATION_RATE_MEASUREMENT_PERIOD_MS * self._rotation_rate_spokes) // 2

        # Auto scan state
        self._auto_mode: bool = False             # True = auto scanning, False = manual
        self._auto_direction: int = 1             # 1 = forwards, -1 = reverse
        self._auto_step: int = 0                  # current step index (0.._AUTO_SCAN_STEPS-1)
        self._auto_timer: int = 0                 # elapsed ms within current phase
        self._auto_settling: bool = True          # True = in settle phase, False = in measure phase
        self._auto_results: list[tuple[int, list[int], int | None]] = []   # list of (power, rpm list, current mA)
        self._auto_max_rpm: int = 0               # max rpm seen during scan
        self._auto_max_current_ma: int = 0        # max current seen during scan
        self._auto_last_current_ma: int = 0       # latest current sampled in auto mode
        self._auto_done: bool = False             # True = scan complete
        self._ina226 = None
        self._ina226_sensor_mgr = None  # SensorManager used exclusively for motor-test INA226 discovery
        self._ina226_reading: dict[str, int] = {}
        self._ina226_sum_current_ma: int = 0
        self._ina226_sum_bus_mv: int = 0
        self._ina226_sum_power_mw: int = 0
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


    def hextest_setup(self, port: int | None):
        """Use HS pins on a spare Hexpansion to make rotation rate measurements."""
        if self._test_support_hexpansion_config is not None and port != self._test_support_hexpansion_config.port:
            try:
                for i in range(4):
                    self._test_support_hexpansion_config.pin[i].init(mode=Pin.IN)
                if self._sub_state == _SUB_MOTOR_TEST:
                    if self._logging:
                        print(f"Test Hexpansion {'removed' if port is None else 'changed'}")
                    self._app.notification = Notification("Motor Test - aborted", port=self._test_support_hexpansion_config.port)
                    self._stop_motor_test_mode()
            except AttributeError:
                pass  # Simulator Pin stubs lack .init()
            self._test_support_hexpansion_config = None
        if port is not None and self._test_support_hexpansion_config is None:
            if self._logging:
                print(f"Setting up Hexpansion on port {port} for rotation rate measurement")
            self._test_support_hexpansion_config = HexpansionConfig(port)
            self._rotation_rate_enable(False)  # start with rotation rate emitter and sensors off until we enter motor test mode


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
        self.colour = (1.0, 1.0, 0.0)  # reset to yellow when starting sensor test
        # If a HexDrive is present, try its port first
        if app.hexdrive_ports is not None:
            for port in app.hexdrive_ports:
                if sensor_mgr.open(port):
                    self._port_selected = port
                    app.update_period = sensor_mgr.read_interval
                    self._sub_state = _SUB_READING
                    break
        # If no HexDrive, but a HexSense is present, try its port next
        elif app.hexsense_port is not None and sensor_mgr.open(app.hexsense_port):
            self._port_selected = app.hexsense_port
            app.update_period = sensor_mgr.read_interval
            self._sub_state = _SUB_READING
        # Otherwise, start in port selection mode
        else:
            self._port_selected = 1
            self._sub_state = _SUB_SELECT_PORT
        return True


    # ------------------------------------------------------------------
    # Sensor Manager access
    # ------------------------------------------------------------------

    def _ensure_sensor_mgr(self) -> "SensorManager":
        """Lazy-import and create SensorManager if needed."""
        if self._sensor_mgr is None:
            from .sensor_manager import SensorManager
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


    # ------------------------------------------------------------------
    # Background update (called from the fast loop)
    # ------------------------------------------------------------------

    def background_update(self, delta) -> tuple[int, int] | None:  # pylint: disable=unused-argument
        """Perform background updates based on the current sub-state."""
        if self._sub_state == _SUB_READING:
            sensor_mgr = self._sensor_mgr
            if sensor_mgr is None:
                return None
            # need per sensor read timing here to balance responsiveness with CPU load, since some sensors can be slow to read and we don't want to bog down the system by reading too frequently.  We also want to update the displayed sample rate at a regular interval (e.g. every second) based on the number of samples read in that time.
            #self._read_timer += delta
            #if self._read_timer >= self._sensor_mgr.read_interval:
                #print(f"S:Reading sensor (S:read_timer={self._read_timer}ms, count_timer={self._count_timer}ms, sample_count={self.sample_count})")
                #self._count_timer += self._read_timer
                #self._read_timer = 0
            # Read sensor data in the background and update sample count and rate calculation
            try:
                self._sensor_data = sensor_mgr.read_current()
                self.sample_count = self.sample_count + 1
            except Exception as e:      # pylint: disable=broad-exception-caught
                self._sensor_data = {"Error": str(e)}

            self._count_timer += delta
            if self._count_timer >= 1000:
                # compute sample rate every second based on the number of samples read and the elapsed time
                self._sample_rate = ((1000 * self.sample_count) + 500) // self._count_timer # sample rate in Hz
                self._count_timer = 0
                self.sample_count = 0
                self._new_sample = True
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
            self._rotation_rate_motor_power = 0
            self._auto_direction *= -1  # reverse direction for next scan
        else:
            # Advance to next power level
            self._rotation_rate_motor_power = self._auto_direction * (65535 * self._auto_step) // (_AUTO_SCAN_STEPS - 1)
            self._auto_timer = 0
            self._auto_settling = True


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
                    print("Enabling rotation rate emitter and sensors")
                for pin_num in _ROTATION_RATE_EMITTER_PINS:
                    self._test_support_hexpansion_config.ls_pin[pin_num].init(mode=ePin.PWM)  # Set LS pins to output mode to turn on the IR emitters
                    self._test_support_hexpansion_config.ls_pin[pin_num].duty(self.rotation_rate_emitter_duty)  # Set LS pins to the current duty cycle to drive the IR emitters)
                for pin_num in _ROTATION_RATE_SENSOR_ENABLE_PINS:
                    self._test_support_hexpansion_config.ls_pin[pin_num].init(mode=Pin.OUT)  # Set LS pins to output mode to enable the phototransistors for rotation rate measurement
                    self._test_support_hexpansion_config.ls_pin[pin_num].value(1)  # Set LS enable pins high to turn on the phototransistors for rotation rate measurement
            else:
                if self._logging:
                    print("Disabling rotation rate emitter and sensors")
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
        if self._test_support_hexpansion_config is None:
            return False
        try:
            from .sensor_manager import SensorManager
            mgr = SensorManager(logging=self._logging)
            port = self._test_support_hexpansion_config.port
            if not mgr.open(port):
                mgr.close()
                if self._logging:
                    print(f"S:INA226 – no sensors found on port {port}")
                return False
            # Find the first INA226 sensor in the discovered list
            sensor = mgr.get_sensor_by_name("INA226")
            if sensor is not None:
                self._ina226 = sensor
                self._ina226_sensor_mgr = mgr
                if self._logging:
                    print(f"S:INA226 found @ 0x{sensor.i2c_addr:02X}")
                return True
            # No INA226 found; close the manager
            mgr.close()
        except Exception as e:      # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"S:INA226 init failed: {e}")
        return False

    def _reset_ina226_accumulators(self) -> None:
        self._ina226_sum_current_ma = 0
        self._ina226_sum_bus_mv = 0
        self._ina226_sum_power_mw = 0
        self._ina226_sample_count = 0

    def _sample_ina226_in_background(self) -> None:
        sensor = self._ina226
        if sensor is None:
            return
        data = sensor.read_sample_if_ready()
        if data is None:
            return
        try:
            self._ina226_sum_current_ma += int(data.get("current_mA", 0))
            self._ina226_sum_bus_mv += int(data.get("bus_mV", 0))
            self._ina226_sum_power_mw += int(data.get("power_mW", 0))
            self._ina226_sample_count += 1
        except Exception as e:       # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"S:INA226 sample error: {e}")
            return

    def _consume_ina226_average(self) -> int | None:
        if self._ina226_sample_count <= 0:
            self._ina226_reading = {}
            return None
        count = self._ina226_sample_count
        current_ma = self._ina226_sum_current_ma // count
        self._ina226_reading = {
            "current_mA": current_ma,
            "bus_mV": self._ina226_sum_bus_mv // count,
            "power_mW": self._ina226_sum_power_mw // count,
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
                self._auto_mode = False
                self._auto_done = False
            else:
                # Start auto scan
                self._auto_mode = True
                self._auto_done = False
                self._auto_step = 0
                self._auto_timer = 0
                self._auto_settling = True
                self._auto_results = []
                self._auto_max_rpm = 0
                self._auto_max_current_ma = 0
            app.refresh = True
            return

        if self._auto_mode:
            if not self._auto_done:
                self._auto_timer += delta
                if self._auto_settling:
                    if self._auto_timer >= _AUTO_SCAN_SETTLE_MS:
                        # Settle phase done — discard counter and start measuring
                        count = 0
                        for counter in self._rotation_rate_counters:
                            if counter is not None:
                                count += counter.value(0)  # read-and-reset to discard
                        if count == 0:
                            # There has been no motion from any motors - so we can skip the measure phase and move straight to the next power level
                            self._auto_rotation_rate_step()
                        else:
                            self._auto_timer = 0
                            self._auto_settling = False
                            self._reset_ina226_accumulators()
                else:
                    if self._auto_timer >= _AUTO_SCAN_MEASURE_MS:
                        # Measure phase done — read counter and record result
                        rounding = (_AUTO_SCAN_MEASURE_MS * self._rotation_rate_spokes) // 2
                        rate = [0] * len(self._rotation_rate_counters)
                        for index, counter in enumerate(self._rotation_rate_counters):
                            if counter is not None:
                                count = counter.value(0)
                                rpm = ((60000 * count) + rounding) // (_AUTO_SCAN_MEASURE_MS * self._rotation_rate_spokes)
                                if rpm > self._auto_max_rpm:
                                    self._auto_max_rpm = rpm
                                rate[index] = rpm
                        current_ma = self._consume_ina226_average()
                        if current_ma is not None:
                            current_abs = abs(current_ma)
                            self._auto_last_current_ma = current_ma
                            if current_abs > self._auto_max_current_ma:
                                self._auto_max_current_ma = current_abs
                        power = self._rotation_rate_motor_power
                        self._auto_results.append((power, rate, current_ma))
                        self._auto_rotation_rate_step()
            # In auto mode, no manual button control for power/IR
            return
        else:
            # manual measurement mode
            self._rotation_rate_measurement_period_elapsed += delta
            if self._rotation_rate_measurement_period_elapsed >= _ROTATION_RATE_MEASUREMENT_PERIOD_MS:
                count = 0
                for index, counter in enumerate(self._rotation_rate_counters):
                    if counter is not None:
                        count = counter.value(0)  # read-and-reset to get the count for the elapsed period
                        self._rotation_rate_rpms[index] = ((60000 * count) + self._rotation_rate_rounding) // (self._rotation_rate_measurement_period_elapsed * self._rotation_rate_spokes)
                self._rotation_rate_measurement_period_elapsed = 0
                self._consume_ina226_average()
                if self.logging:
                    print(f"S:Rotation Rates: {self._rotation_rate_rpms}")

        # Manual mode button handling
        if app.button_states.get(BUTTON_TYPES["UP"]):
            app.button_states.clear()
            self.rotation_rate_emitter_duty = min(255, self.rotation_rate_emitter_duty + _IR_EMITTER_PWM_STEP_SIZE)
            if self.logging:
                print(f"S:IR+Emitter Duty: {self.rotation_rate_emitter_duty}")
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            app.button_states.clear()
            self.rotation_rate_emitter_duty = max(0, self.rotation_rate_emitter_duty - _IR_EMITTER_PWM_STEP_SIZE)
            if self.logging:
                print(f"S:IR-Emitter Duty: {self.rotation_rate_emitter_duty}")
        elif app.button_states.get(BUTTON_TYPES["RIGHT"]):
            app.button_states.clear()
            self._rotation_rate_motor_power = min(65535, self._rotation_rate_motor_power + 1000)
            if self.logging:
                print(f"S:Motor+Power: {self._rotation_rate_motor_power}")
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            app.button_states.clear()
            self._rotation_rate_motor_power = max(-65535, self._rotation_rate_motor_power - 1000)
            if self.logging:
                print(f"S:Motor-Power: {self._rotation_rate_motor_power}")




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
            if self._port_selected == motor_test_port and self._start_motor_test_mode():
                app.notification = Notification("Motor Test", port=self._port_selected)
                if self.logging:
                    print(f"S:Entering Motor Test mode on port {self._port_selected}")
                self._sub_state = _SUB_MOTOR_TEST
                app.refresh = True
            else:
                sensor_mgr = self._ensure_sensor_mgr()
                self._sensor_data = {}
                self._display_data = {}
                self._read_timer = 0
                self._count_timer = 0
                self._sample_rate = 0
                self._sample_count = 0
                self._new_sample = False
                app.refresh = True
                if sensor_mgr.open(self._port_selected):
                    app.update_period = sensor_mgr.read_interval
                    if self.logging:
                        print(f"Opened sensor port {self._port_selected} with read_interval {sensor_mgr.read_interval}ms")
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


    def _update_display_values(self):      # pylint: disable=unused-argument
        # clear old display data
        self._display_data = {}

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
                        self._display_data = {k: str(v) for k, v in self._sensor_data.items()}

                    #convert CIE1931 XYZ to RGB using a simple matrix transform
                    r = int( 3.2406 * x - 1.5372 * y - 0.4986 * z)
                    g = int(-0.9689 * x + 1.8758 * y + 0.0415 * z)
                    b = int( 0.0557 * x - 0.2040 * y + 1.0570 * z)


                except Exception as e:    # pylint: disable=broad-exception-caught
                    print(f"S:Colour conversion error: {e}")
                    r = g = b = 0

            elif all(k in self._sensor_data for k in ("red", "green", "blue")):
                try:
                    r = int(self._sensor_data["red"])
                    g = int(self._sensor_data["green"])
                    b = int(self._sensor_data["blue"])

                    if self._page_selected == _PAGE_DATA:
                        if "clear" in self._sensor_data:
                            clear = int(self._sensor_data["clear"])
                            colour_name = self.lookup_colour_RGB(r, g, b, clear)
                        else:
                            colour_name = self.lookup_colour_RGB(r, g, b)
                        self._display_data["colour"] = colour_name
                    elif self._page_selected == _PAGE_RAW:
                        self._display_data = {k: str(v) for k, v in self._sensor_data.items()}

                except Exception as e:    # pylint: disable=broad-exception-caught
                    print(f"S:Colour conversion error: {e}")
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
                self.colour = (red_f, green_f, blue_f)
            else:
                self.colour = (1.0,1.0,0.0)  # default to yellow if all channels are zero to avoid divide-by-zero and to provide a visible colour for non-colour sensors
        elif self._sensor_mgr and self._sensor_mgr.type == "Distance":
            if self._page_selected == _PAGE_DATA and "dist_mm" in self._sensor_data:
                try:
                    dist_mm = int(self._sensor_data["dist_mm"])
                    if dist_mm < 20:
                        distance_str = f"{dist_mm}mm (Very Close)"
                    elif dist_mm < 100:
                        distance_str = f"{dist_mm}mm (Close)"
                    elif dist_mm < 500:
                        distance_str = f"{dist_mm}mm (Medium)"
                    else:
                        distance_str = f"{dist_mm}mm (Far)"
                    self._display_data["Distance"] = distance_str
                except Exception as e:    # pylint: disable=broad-exception-caught
                    print(f"S:Distance processing error: {e}")
        elif self._page_selected == _PAGE_RAW:
            self._display_data = {k: str(v) for k, v in self._sensor_data.items()}

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
            self.colour = (1.0, 1.0, 0.0)  # reset to yellow when switching sensors
            self._sensor_mgr.next_sensor()
            self._sensor_data = {}
            self._display_data = {}
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]) and self._sensor_mgr and self._sensor_mgr.num_sensors > 1:
            app.button_states.clear()
            self.colour = (1.0, 1.0, 0.0)  # reset to yellow when switching sensors
            self._sensor_mgr.prev_sensor()
            self._sensor_data = {}
            self._display_data = {}
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
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            sensor_mgr = self._sensor_mgr
            if sensor_mgr is not None:
                sensor_mgr.close()
            app.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
            self._sub_state = _SUB_SELECT_PORT
            app.refresh = True


    def _start_motor_test_mode(self) -> bool:
        # enable HexDrive power
        app = self._app
        if len(app.hexdrive_apps) > 0 and self._test_support_hexpansion_config is not None:
            app.hexdrive_apps[0].set_logging(True)
            if app.hexdrive_apps[0].initialise() and app.hexdrive_apps[0].set_power(True) and app.hexdrive_apps[0].set_freq(MOTOR_PWM_FREQ):
                app.hexdrive_apps[0].set_keep_alive(2000)   # Updates can be quite slow as we are using the draw function
                app.hexdrive_apps[0].set_motors((-1,-1))    # Try forcing PWM to be reinitialised by swapping direction.
                # Enable the IR emitter for measuring wheel rotation rate
                self._rotation_rate_enable(True)

                # Enable the phototransistor input for measuring wheel rotation rate
                for pin_num in _ROTATION_RATE_SENSOR_PINS:
                    # configure the ESP32S3 hardware to count pulses on the HS_F pin
                    # Counter not yet available in this Micropython port so we have created our own...
                    gpio_num = _HS_PIN_TO_GPIO[self._test_support_hexpansion_config.port][pin_num]
                    counter = Counter(None, gpio_num, filter_ns=1000000, logging=self.logging)  # auto-select PCNT unit
                    if counter is not None and counter.unit is not None:
                        self._rotation_rate_counters.append(counter)
                    else:
                        if self.logging:
                            print(f"S:Failed to allocate PCNT counter for pin {pin_num} (GPIO {gpio_num})")
                        app.notification = Notification("PCNT Init Failed")
                        # deinit any counters we did manage to create before returning
                        for c in self._rotation_rate_counters:
                            if c is not None:
                                c.deinit()
                        self._rotation_rate_counters = []
                        return False
                if self.logging:
                    print(f"S:Rate counter {self._rotation_rate_counters}")
                self._rotation_rate_measurement_period_elapsed = 0
                self._rotation_rate_rpms = [0] * len(self._rotation_rate_counters)
                self._init_ina226_for_motor_test()
                app.update_period = _MOTOR_TEST_BACKGROUND_UPDATE_PERIOD  # update every 1000ms to give a responsive display without overwhelming the CPU with updates
                return True
        if self.logging:
            print("H:Failed to initialise HexDrive for motor test mode")
        app.notification = Notification("HexDrive Init Failed")
        return False


    def _stop_motor_test_mode(self):
        if self._logging:
            print("Stopping Motor Test mode and cleaning up")
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
            app.hexdrive_apps[0].set_pwm((0, 0, 0, 0))
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
            lines += [f"I:{self._ina226_reading.get('current_mA', 0)}mA"]
            colours += [(1, 0.3, 0.3)]
            lines += [f"V:{self._ina226_reading.get('bus_mV', 0)}mV"]
            colours += [(0.3, 0.8, 1.0)]
        self._app.draw_message(ctx, lines, colours, label_font_size)
        button_labels(ctx, up_label="IR+", down_label="IR-", cancel_label="Back",
                      left_label="Pwr-", right_label="Pwr+", confirm_label="Auto")


    def _draw_auto_scan(self, ctx):
        """Draw a chart of power vs RPM from the auto scan results."""
        # Chart area within the 240x240 circular display (origin at centre)
        chart_left = -90
        chart_right = 90
        chart_top = -65
        chart_bottom = 65
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
                x = chart_left + (abs(power) * chart_w) // 65535
                for index, rpm in enumerate(rpms):
                    h = (rpm * chart_h) // max_rpm
                    if h > 0:
                        # colour by index to differentiate multiple counters if present
                        if index == 0:
                            ctx.rgb(0.0, 1.0, 0.5)
                        else:
                            ctx.rgb(1.0, 0.5, 0.0)
                        ctx.rectangle(x, chart_bottom - h, bar_w, h).fill()
                if current_ma is not None:
                    current_h = (abs(current_ma) * chart_h) // max_current_ma
                    marker_y = chart_bottom - current_h
                    ctx.rgb(1.0, 0.2, 0.2)
                    ctx.rectangle(x + bar_w, marker_y - 1, 2, 2).fill()

        # Title and max RPM label
        ctx.rgb(1, 1, 0)
        ctx.font_size = label_font_size
        if self._auto_done:
            ctx.move_to(-55, chart_top - 5).text("Complete")
        else:
            progress = (self._auto_step * 100) // _AUTO_SCAN_STEPS
            ctx.move_to(-55, chart_top - 5).text(f"Scan {progress}%")

        ctx.rgb(0, 1, 1)
        ctx.move_to(-60, chart_bottom + label_font_size + 2).text(f"Max:{max_rpm}rpm")
        ctx.rgb(1.0, 0.2, 0.2)
        ctx.move_to(15, chart_bottom + label_font_size + 2).text(f"Ipk:{max_current_ma}mA")

        button_labels(ctx, cancel_label="Back", confirm_label="Manual")


    def _draw_select_port(self, ctx):
        self._app.draw_message(ctx,
            ["Sensor Test", f"Port: {self._port_selected}"],
            [(1, 1, 0), (0, 1, 1)],
            label_font_size)
        button_labels(ctx, left_label="<Port", right_label="Port>",
                      confirm_label="Scan", cancel_label="Back")


    def _draw_reading(self, ctx):
        up_label = down_label = ""
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
            for label, value in self._display_data.items():
                lines += [f"{label}:{value}"]
                colours += [self.colour]
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

            if num_sensors > 1:
                button_labels(ctx, left_label="<Prev", right_label="Next>",
                            up_label=up_label, down_label=down_label, cancel_label="Back")
            else:
                button_labels(ctx, up_label=up_label, down_label=down_label, cancel_label="Back")

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
_CONF0_CH0_POS_MODE_S  = const(18)      # bits [19:18]

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

class Counter:
    """Wrapper around ESP32-S3 PCNT hardware for counting rising edges.

    Parameters
    ----------
    src : int
        The ESP32-S3 GPIO number to count pulses on.
        Use ``_HS_PIN_TO_GPIO[port][index]`` to convert from a badge HS pin.
    id : int | None
        PCNT unit to use (0-3).  If ``None``, the first available (unused) unit
        is auto-selected.  If the requested unit is already in use, ``__init__``
        sets ``self.unit = None`` to signal failure.
    filter_ns : int
        Minimum pulse width in nanoseconds.  Pulses shorter than this are
        rejected by the hardware glitch filter.  Set to 0 to disable filtering.
    logging : bool
        Print diagnostic messages to the console.

    CURRENTLY ONLY COUNTS UP ON RISING EDGES
    """

    def __init__(self, unit: int | None, src: int, filter_ns: int = 0, logging: bool = False):
        self.logging = logging
        self._configured = False

        if unit is not None:
            if unit < 0 or unit >= _PCNT_NUM_UNITS:
                if self.logging:
                    print(f"PCNT: unit {unit} out of range (0-{_PCNT_NUM_UNITS - 1})")
                self.unit = None
                return
            if self._unit_in_use(unit):
                self.unit = None
                return
            self.unit = unit
        else:
            # Auto-select first available unit
            self.unit = None
            for u in range(_PCNT_NUM_UNITS):
                if not self._unit_in_use(u):
                    self.unit = u
                    break
            if self.unit is None:
                if self.logging:
                    print("PCNT: all units in use, no free unit available")
                return

        if not self.init(src, filter_ns):
            if self.logging:
                print(f"PCNT: failed to configure unit {self.unit}")
            self.unit = None


    def _unit_in_use(self, unit: int) -> bool:
        """Check whether a PCNT unit appears to already be in use.

        A unit is considered in use if:
        - The peripheral clock is enabled AND
        - The register clock gate is enabled AND
        - The unit is NOT held in reset AND
        - CONF0 is non-zero (has been configured)
        """
        # Check peripheral clock
        clk_on = (mem32[_CLK_EN0_REG] & _PCNT_CLK_BIT) != 0
        if not clk_on:
            if self.logging:
                print(f"PCNT: unit {unit} - peripheral clock off, unit free")
            return False

        ctrl = mem32[_PCNT_CTRL_REG]

        # Check register clock gate
        if not ctrl & _PCNT_CTRL_CLK_EN:
            if self.logging:
                print(f"PCNT: unit {unit} - register clock gate off, unit free")
            return False

        # Check if held in reset (reset bit = unit * 2)
        rst_bit = 1 << (unit * 2)
        if ctrl & rst_bit:
            if self.logging:
                print(f"PCNT: unit {unit} - held in reset, unit free")
            return False

        # Check CONF0 register
        conf0_addr = _PCNT_BASE + unit * 0x0C
        conf0 = mem32[conf0_addr]
        if conf0 == 0x3C10: # a slightly odd reset state
            if self.logging:
                print(f"PCNT: unit {unit} - CONF0=0x3C10 (unconfigured), unit free")
            return False

        # Unit appears to be actively configured and running
        if self.logging:
            cnt_addr = _PCNT_BASE + 0x30 + unit * 4
            cnt = mem32[cnt_addr] & 0xFFFF
            pulse_sig = _PCNT_SIG_BASE + unit * 4
            gpio_route = mem32[_GPIO_FUNC_IN_SEL_CFG_BASE + pulse_sig * 4]
            routed_gpio = gpio_route & 0x3F
            print(f"PCNT: unit {unit} - IN USE: CONF0=0x{conf0:08X}, "
                  f"count={cnt}, routed to GPIO {routed_gpio}")
        return True


    def __str__(self):
        if self.unit is None:
            return "Counter(not configured)"
        count = self.value()
        return f"Counter(unit={self.unit}, GPIO={self.pin}, count={count})"


    def init(self, src: int, filter_ns: int | None = None) -> bool:
        """Configure a PCNT unit to count rising edges on the GPIO pin specified by src."""
        self.pin = src

        unit = self.unit
        if unit is None:
            return False
        conf0_addr = _PCNT_BASE + unit * 0x0C
        cnt_addr = _PCNT_BASE + 0x30 + unit * 4
        rst_bit = 1 << (unit * 2)
        pulse_sig = _PCNT_SIG_BASE + unit * 4       # PCNT_SIG_CH0_INn
        ctrl_sig = _PCNT_SIG_BASE + unit * 4 + 2    # PCNT_CTRL_CH0_INn

        if self.logging:
            hs = _GPIO_TO_HS.get(self.pin)
            hs_str = f" port {hs[0]} HS pin {hs[1]})" if hs else ""
            print(f"PCNT U{unit}: on GPIO {self.pin}{hs_str}, filter_ns={filter_ns}ns")
            print(f"  CONF0 addr=0x{conf0_addr:08X}, CNT addr=0x{cnt_addr:08X}")
            print(f"  pulse_sig={pulse_sig}, ctrl_sig={ctrl_sig}")

        try:
            # --- 1. ENABLE PERIPHERAL CLOCK ---
            mem32[_CLK_EN0_REG] |= _PCNT_CLK_BIT
            mem32[_RST_EN0_REG] &= ~_PCNT_CLK_BIT

            # --- 2. ENABLE REGISTER CLOCK GATE, HOLD THIS UNIT IN RESET ---
            # Read-modify-write to preserve other units' state
            ctrl = mem32[_PCNT_CTRL_REG]
            ctrl |= _PCNT_CTRL_CLK_EN | rst_bit
            mem32[_PCNT_CTRL_REG] = ctrl

            # --- 3. ROUTE GPIO VIA MATRIX ---
            mem32[_GPIO_FUNC_IN_SEL_CFG_BASE + (pulse_sig * 4)] = _SIG_IN_SEL_BIT | self.pin
            # Route constant high (0x38) to control signal
            mem32[_GPIO_FUNC_IN_SEL_CFG_BASE + (ctrl_sig * 4)] = _SIG_IN_SEL_BIT | 0x38

            # --- 4. CONFIGURE COUNTING ---
            # Calculate filter threshold from min pulse width
            if filter_ns is not None and filter_ns > 0:
                filter_val = (_APB_CLK_HZ * filter_ns) // 1_000_000_000
                if filter_val > 1023:
                    filter_val = 1023
                config = (filter_val & _CONF0_FILTER_THRES_M) | _CONF0_FILTER_EN
            else:
                config = 0
            config |= (1 << _CONF0_CH0_POS_MODE_S)  # Inc on rising edge
            mem32[conf0_addr] = config

            # --- 5. RELEASE FROM RESET ---
            ctrl = mem32[_PCNT_CTRL_REG]
            ctrl &= ~rst_bit
            mem32[_PCNT_CTRL_REG] = ctrl

            self._configured = True

        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"PCNT U{unit}: error configuring: {e}")
            return False

        if self.logging:
            print(f"PCNT U{unit}: configured OK, "
                  f"CONF0=0x{mem32[conf0_addr]:08X}, "
                  f"CTRL=0x{mem32[_PCNT_CTRL_REG]:08X}, "
                  f"CNT={mem32[cnt_addr] & 0xFFFF}")
        return True


    def value(self, value: int | None = None) -> int:
        """Read the current count and optionally reset the counter to zero.
          DOES NOT SUPPORT SETTING THE COUNTER TO AN ARBITRARY VALUE, ONLY RESETTING TO ZERO."""
        if not self._configured:
            return 0

        unit = self.unit
        if unit is None:
            return 0

        rst_bit = 1 << (unit * 2)
        cnt_addr = _PCNT_BASE + 0x30 + unit * 4
        if value is not None and value == 0:
            irq_state = disable_irq()
            count = mem32[cnt_addr] & 0xFFFF
            mem32[_PCNT_CTRL_REG] |= rst_bit
            mem32[_PCNT_CTRL_REG] &= ~rst_bit
            enable_irq(irq_state)
        else:
            count = mem32[cnt_addr] & 0xFFFF
        return count



    def deinit(self):
        """Release the PCNT unit: hold it in reset and clear its CONF0."""
        if not self._configured or self.unit is None:
            return
        unit = self.unit
        conf0_addr = _PCNT_BASE + unit * 0x0C
        rst_bit = 1 << (unit * 2)
        mem32[_PCNT_CTRL_REG] |= rst_bit   # hold in reset
        mem32[conf0_addr] = 0               # clear config so unit appears free
        self._configured = False

        if self.logging:
            print(f"PCNT U{unit}: released")

        # disable the peripheral clock if no units are in use to save power
        if not any(self._unit_in_use(u) for u in range(_PCNT_NUM_UNITS)):
            mem32[_CLK_EN0_REG] &= ~_PCNT_CLK_BIT
            mem32[_RST_EN0_REG] |= _PCNT_CLK_BIT
            if self.logging:
                print("PCNT: all units released, peripheral clock disabled")
