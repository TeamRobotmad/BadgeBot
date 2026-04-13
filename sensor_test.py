# Sensor Test Module for BadgeBot
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
from system.hexpansion.config import HexpansionConfig, ePin
from machine import Pin, mem32
from micropython import const
from .app import DEFAULT_BACKGROUND_UPDATE_PERIOD, MOTOR_PWM_FREQ

# Temporary - while thre is no EEPROM on the Test Hexpansion
_ROTATION_RATE_PORT = 2                         # Hexpansion slot used for rotation rate measurement
_ROTATION_RATE_EMITTER_PIN = 2                  # LS_C pin used to drive the IR emitter for rotation rate testing
_ROTATION_RATE_SENSOR_PIN = 0                   # HS_F pin used to read the phottransistor for rotation rate testing
_ROTATION_RATE_MEASUREMENT_PERIOD_MS = 2500     # how often to update the displayed rotation rate measurement in ms (tradeoff between display responsiveness and stability of the reading)
_DEFAULT_ROTATION_RATE_EMITTER_DUTY = 64        # default duty cycle for the IR emitter when doing rate testing, 0-255 (0=off, 255=full on)
_DEFAULT_SPOKES_PER_ROTATION = 3                # number of times the photodiode will be triggered per full rotation of the wheel

# Local sub-states (internal to Sensor Test)
_SUB_SELECT_PORT = 0
_SUB_READING     = 1
_SUB_MOTOR_TEST  = 2


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

def init_settings(s, MySetting: type):       # pylint: disable=unused-argument
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

    def __init__(self, app, logging: bool = False):
        self._app = app
        self._sub_state = _SUB_SELECT_PORT
        self._sensor_mgr = None          # SensorManager instance (lazy-imported)
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
        self._rotation_rate_sensor_pin: int | None = None           # pin used to read the photodiode for rate testing
        self._rotation_rate_counter: Counter | None = None          # hardware counter used to count photodiode pulses for rate testing
        self._rotation_rate_rpm: int | None = None                  # current value of the hardware counter (for diagnostics, should increase as the photodiode detects pulses when the emitter is on)
        self._rotation_rate_measurement_period_elapsed: int = 0     # ticks since last rate check, used to compute pulse rate in Hz based on the change in the counter value
        self._rotation_rate_motor_power: int = 0                    # Power applied to motors in TEST mode
        self._rotation_rate_spokes: int = _DEFAULT_SPOKES_PER_ROTATION
        self._rotation_rate_rounding: int = ((_ROTATION_RATE_MEASUREMENT_PERIOD_MS * self._rotation_rate_spokes) // 2)

        # Use HS pins on a spare Hexpansion to measure rotation rate
        self._test_support_hexpansion_config: HexpansionConfig | None = None
        if _ROTATION_RATE_PORT is not None:
            self._test_support_hexpansion_config = HexpansionConfig(_ROTATION_RATE_PORT)    # Create a config instance to access the LED pin for diagnostics
            # HS_F for photodiode input, LS_C for IR emitter output (only enabled when on)
            self._test_support_hexpansion_config.ls_pin[_ROTATION_RATE_EMITTER_PIN].init(mode=ePin.IN)  # LS_C
            self._test_support_hexpansion_config.pin[_ROTATION_RATE_SENSOR_PIN].init(mode=Pin.IN)       # HS_F

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
        self._ensure_sensor_mgr()
        self.colour = (1.0, 1.0, 0.0)  # reset to yellow when starting sensor test
        # If a HexDrive is present, try its port first
        if app.hexdrive_port is not None and self._sensor_mgr.open(app.hexdrive_port):
            self._port_selected = app.hexdrive_port
            app.update_period = self._sensor_mgr.read_interval
            self._sub_state = _SUB_READING
        # If no HexDrive, but a HexSense is present, try its port next
        elif app.hexsense_config is not None and app.hexsense_config.port is not None and self._sensor_mgr.open(app.hexsense_config.port):
            self._port_selected = app.hexsense_config.port
            app.update_period = self._sensor_mgr.read_interval
            self._sub_state = _SUB_READING
        # Otherwise, start in port selection mode
        else:
            self._port_selected = 1
            self._sub_state = _SUB_SELECT_PORT
        return True


    # ------------------------------------------------------------------
    # Sensor Manager access
    # ------------------------------------------------------------------

    def _ensure_sensor_mgr(self):
        """Lazy-import and create SensorManager if needed."""
        if self._sensor_mgr is None:
            from .sensor_manager import SensorManager
            self._sensor_mgr = SensorManager(logging=self._logging)
        else:
            self._sensor_mgr.close()

    def open_sensor_port(self, port: int) -> bool:
        """Open a sensor port.  Returns True if sensors found.
        Can be called by other modules (e.g. AutoDriveMgr) that
        need to reuse the SensorManager."""
        self._ensure_sensor_mgr()
        return self._sensor_mgr.open(port)


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
        return self._rotation_rate_emitter_duty

    @rotation_rate_emitter_duty.setter
    def rotation_rate_emitter_duty(self, value: int):
        self._rotation_rate_emitter_duty = value
        if self._test_support_hexpansion_config is not None:
            self._test_support_hexpansion_config.ls_pin[_ROTATION_RATE_EMITTER_PIN].duty(self._rotation_rate_emitter_duty)


    @staticmethod
    def lookup_color_XYZ(x: int, y: int, z: int, brightness_threshold: int = 10) -> str:
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
        x = x / total
        y = y / total

        # 3. Search the lookup table
        for region in COLOR_REGIONS:
            x_min, x_max = region["x"]
            y_min, y_max = region["y"]
            
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return region["name"]

        return "Unknown"

    # Example: Testing a Red measurement
    # result = lookup_color(0.6, 0.3, 0.1)
    # print(f"Detected: {result}")


    @staticmethod
    def lookup_colour_RGB(r: int, g: int, b: int, clear: int = 0) -> str:
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
        if self._sub_state == _SUB_READING:
            #self._read_timer += delta
            #if self._read_timer >= self._sensor_mgr.read_interval:
                #print(f"S:Reading sensor (S:read_timer={self._read_timer}ms, count_timer={self._count_timer}ms, sample_count={self.sample_count})")
                #self._count_timer += self._read_timer
                #self._read_timer = 0
            # Read sensor data in the background and update sample count and rate calculation
            try:
                self._sensor_data = self._sensor_mgr.read_current()
                self.sample_count = self.sample_count + 1
            except Exception as e:      # pylint: disable=broad-exception-caught
                self._sensor_data = {"Error": str(e)}

            self._count_timer += delta
            if self._count_timer >= 1000:
                # compute sample rate every second based on the number of samples read and the elapsed time
                self._sample_rate = (((1000 * self.sample_count) + 500) // self._count_timer) # sample rate in Hz
                self._count_timer = 0
                self.sample_count = 0
                self._new_sample = True
        elif self._sub_state == _SUB_MOTOR_TEST:
            return (self._rotation_rate_motor_power, self._rotation_rate_motor_power)

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
        elif self._sub_state == _SUB_MOTOR_TEST and self._test_support_hexpansion_config is not None:
            self._update_motor_test_mode(delta)


    def _update_motor_test_mode(self, delta: int):   # pylint: disable=unused-argument
        # use UP and DOWN to adjust the _rotation_rate_emitter_duty, which controls the brightness of the IR emitter when doing rate testing
        app = self._app
        if app.button_states.get(BUTTON_TYPES["UP"]):
            app.button_states.clear()
            self.rotation_rate_emitter_duty = min(255, self.rotation_rate_emitter_duty + 8)  # increase duty cycle by 8 (about 3.125%) up to a maximum of 255
            if self.logging:
                print(f"S:IR+Emitter Duty: {self._rotation_rate_emitter_duty}")
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            app.button_states.clear()
            self.rotation_rate_emitter_duty = max(0, self.rotation_rate_emitter_duty - 8)  # decrease duty cycle by 8 (about 3.125%) down to a minimum of 0
            if self.logging:
                print(f"S:IR-Emitter Duty: {self._rotation_rate_emitter_duty}")
        # use left and rigth to adjust the _rotation_rate_motor_power, which is returned from background_update to allow testing the motors with different power levels in TEST mode
        elif app.button_states.get(BUTTON_TYPES["RIGHT"]):
            app.button_states.clear()
            self._rotation_rate_motor_power = min(65535, self._rotation_rate_motor_power + 1000)  # increase power by 1000 up to a maximum of 65535
            if self.logging:
                print(f"S:Motor+Power: {self._rotation_rate_motor_power}")
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            app.button_states.clear()
            self._rotation_rate_motor_power = max(0, self._rotation_rate_motor_power - 1000)  # decrease power by 1000 down to a minimum of 0
            if self.logging:
                print(f"S:Motor-Power: {self._rotation_rate_motor_power}")
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if self.logging:
                print("Exiting Test mode")
            if app.hexdrive_app is not None:
                app.hexdrive_app.set_power(False)
            app.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
            self._test_support_hexpansion_config.ls_pin[_ROTATION_RATE_EMITTER_PIN].init(mode=Pin.IN)    # Set the IR Emitter pin to input mode to turn it off
            self._rotation_rate_motor_power = 0
            self._sub_state = _SUB_SELECT_PORT
            app.refresh = True

        self._rotation_rate_measurement_period_elapsed += delta
        if self._rotation_rate_measurement_period_elapsed >= _ROTATION_RATE_MEASUREMENT_PERIOD_MS:  # update the display values every 2500ms
            count = self._rotation_rate_counter.value()
            self._rotation_rate_rpm = ((60000 * count) + self._rotation_rate_rounding) // (self._rotation_rate_measurement_period_elapsed * self._rotation_rate_spokes)
            self._rotation_rate_measurement_period_elapsed = 0
            if self.logging:
                print(f"S:Count {count} = Rotation Rate: {self._rotation_rate_rpm}")


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
            if self._port_selected == _ROTATION_RATE_PORT and self._start_motor_test_mode():
                app.notification = Notification("Motor Test", port=self._port_selected)
                if self.logging:
                    print(f"S:Entering Motor Test mode on port {self._port_selected}")                              
                self._sub_state = _SUB_MOTOR_TEST
                app.refresh = True
            else:
                self._ensure_sensor_mgr()
                self._sensor_data = {}
                self._display_data = {}
                self._read_timer = 0
                self._count_timer = 0
                self._sample_rate = 0
                self._sample_count = 0
                self._new_sample = False
                app.refresh = True
                if self._sensor_mgr.open(self._port_selected):
                    app.update_period = self._sensor_mgr.read_interval
                    if self.logging:
                        print(f"Opened sensor port {self._port_selected} with read_interval {self._sensor_mgr.read_interval}ms")
                    self._sub_state = _SUB_READING
                else:
                    app.notification = Notification("      No      Sensors", port=self._port_selected)
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if self.logging:
                print("Exiting Sensor Test")
            if self._sensor_mgr is not None:
                self._sensor_mgr.close()
                if self._test_support_hexpansion_config is not None:
                    self._test_support_hexpansion_config.ls_pin[_ROTATION_RATE_EMITTER_PIN].init(mode=Pin.IN)    # Set the IR Emitter pin to input mode
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
            self._sensor_mgr.close()
            app.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
            self._sub_state = _SUB_SELECT_PORT
            app.refresh = True


    def _start_motor_test_mode(self) -> bool:
        # enable HexDrive power
        app = self._app
        if app.hexdrive_app is not None and self._test_support_hexpansion_config is not None:
            app.hexdrive_app.set_logging(True)
            if app.hexdrive_app.initialise() and app.hexdrive_app.set_power(True) and app.hexdrive_app.set_freq(MOTOR_PWM_FREQ):
                # Enable the IR emitter for measuring wheel rotation rate
                if self.logging:
                    print("S:IR Emitter On")
                self._test_support_hexpansion_config.ls_pin[_ROTATION_RATE_EMITTER_PIN].init(mode=ePin.PWM)     # Set the IR Emitter pin to PWM mode so we can control the brightness
                self._test_support_hexpansion_config.ls_pin[_ROTATION_RATE_EMITTER_PIN].duty(self._rotation_rate_emitter_duty)  # Set the IR Emitter to the configured duty cycle

                # Enable the phototransistor input for measuring wheel rotation rate
                self._rotation_rate_sensor_pin = self._test_support_hexpansion_config.pin[_ROTATION_RATE_SENSOR_PIN]           # HS_F
                self._rotation_rate_sensor_pin.init(mode=Pin.IN)
                # configure the ESP32S3 hardware to count pulses on the HS_F pin and make the count available as a regular GPIO input that we can read in software.  This allows us to do high-speed counting of photodiode pulses without needing to do it in software with interrupts or tight loops, which would be unreliable due to the cooperative multitasking nature of the app.
                # Counter not yet available in this Micropython port so we have created our own...
                self._rotation_rate_counter = Counter(unit=0, pin=self._rotation_rate_sensor_pin)     # create Counter and begin counting
                if self.logging:
                    print(f"S:Rate counter {self._rotation_rate_counter}")
                self._rotation_rate_measurement_period_elapsed = 0
                self._rotation_rate_rpm = 0
                app.update_period = 250  # update every 250ms to give a responsive display without overwhelming the CPU with updates
                return True
        if self.logging:
            print("H:Failed to initialise HexDrive for motor moves")
        app.notification = Notification("HexDrive Init Failed")
        return False


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
        # show the current emitter duty cycle as a percentage in the label, and show the current photodiode reading and rate counter value in the display data
        lines = [f"IR:{int(self.rotation_rate_emitter_duty * 100 // 255)}%"]
        colours = [(1, 1, 0)]
        # Show power
        lines += [f"Pwr:{self._rotation_rate_motor_power}"]
        colours += [(0, 1, 1)]
        if self._rotation_rate_counter is not None:
            lines += [f"{self._rotation_rate_rpm}rpm"]
            colours += [(1, 0, 1)]
        self._app.draw_message(ctx, lines, colours, label_font_size)
        button_labels(ctx, up_label="IR+", down_label="IR-", cancel_label="Back", left_label="Pwr-", right_label="Pwr+")


    def _draw_select_port(self, ctx):
        self._app.draw_message(ctx,
            ["Sensor Test", f"Port: {self._port_selected}"],
            [(1, 1, 0), (0, 1, 1)],
            label_font_size)
        button_labels(ctx, left_label="<Port", right_label="Port>",
                      confirm_label="Scan", cancel_label="Back")
        

    def _draw_reading(self, ctx):
        up_label = down_label = ""
        num_sensors = self._sensor_mgr.num_sensors if self._sensor_mgr else 1
        sensor_name = self._sensor_mgr.current_sensor_name if self._sensor_mgr else "Sensor"
        if num_sensors > 1:
            lines = [f"Slot {self._port_selected}-{self._sensor_mgr.current_sensor_index + 1}/{num_sensors}"]
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
#-------------------------------------------------------------------

_SYSTEM_BASE      = const(0x600C0000)
_GPIO_BASE        = const(0x60004000)
_PCNT_BASE        = const(0x60017000)

_PCNT_CLK_BIT     = const(1 << 10)  # SYSTEM_PCNT_CLK_EN / SYSTEM_PCNT_RST (bit 10)

# System/Clock Base
_CLK_EN0_REG      = const(_SYSTEM_BASE + 0x0018)
_RST_EN0_REG      = const(_SYSTEM_BASE + 0x0020)

# GPIO Matrix Base
_GPIO_FUNC_IN_SEL_CFG_BASE = const(_GPIO_BASE + 0x0154)
_U0_PULSE_SIG_IDX = 33  # PCNT_SIG_CH0_IN0_IDX  — Unit 0, Ch 0, Pulse signal
_U0_CTRL_SIG_IDX  = 35  # PCNT_CTRL_CH0_IN0_IDX — Unit 0, Ch 0, Control signal
_SIG_IN_SEL_BIT   = const(1 << 6) # Enable routing via GPIO Matrix

# PCNT registers (ESP32-S3 layout from pcnt_reg.h)
_PCNT_U0_CONF0_REG      = const(_PCNT_BASE + 0x0000)
_PCNT_PULSE_CNT_U0_REG  = const(_PCNT_BASE + 0x0030)  # PCNT_PULSE_CNT_U0_REG
_PCNT_CTRL_REG          = const(_PCNT_BASE + 0x0060)  # PCNT_CTRL_REG

# _PCNT_PULSE_CNT_U0_REG bits
_PCNT_PULSE_CNT_RST_U0 = const(1 << 0)

# _PCNT_CTRL_REG bits
_PCNT_CTRL_CLK_EN   = const(1 << 16)  # Register clock gate — must be 1 for register access
_PCNT_CTRL_RST_U0   = const(1 << 0)   # Counter reset for unit 0 (default 1 = held in reset)

# _PCNT_U0_CONF0_REG bit layout (ESP32-S3):
#   [9:0]   FILTER_THRES        [17:16] CH0_NEG_MODE (0=none,1=inc,2=dec)
#   [10]    FILTER_EN           [19:18] CH0_POS_MODE (0=none,1=inc,2=dec)
#   [11]    THR_ZERO_EN         [21:20] CH0_HCTRL_MODE
#   [12]    THR_H_LIM_EN        [23:22] CH0_LCTRL_MODE
#   [13]    THR_L_LIM_EN        [25:24] CH1_NEG_MODE
#   [14]    THR_THRES0_EN       [27:26] CH1_POS_MODE
#   [15]    THR_THRES1_EN       [29:28] CH1_HCTRL_MODE
_CONF0_FILTER_THRES_M  = const(0x3FF)   # bits [9:0]
_CONF0_FILTER_EN       = const(1 << 10)
_CONF0_CH0_NEG_MODE_S  = const(16)      # bits [17:16]
_CONF0_CH0_POS_MODE_S  = const(18)      # bits [19:18]

class Counter:
    """Simple wrapper for ESP32S3 pulse counting hardware, which counts the number of rising
    edges on a given pin. __init__() initializes the counter on a given pin, and value() returns the current count."""                              

    def __init__(self, unit: int | None = 0, pin: Pin = None):
        self.unit = unit
        # unable to find a way to get at the pin number from the Pin object, so just hardcoding it for now since we only need it for one pin in this app
        self.pin = 35

        if not self.configure():
            return None


    def __str__(self):
        # method called by printing the Counter object
        count = self.value()
        return f"Counter(unit={self.unit}, pin={self.pin}, count={count})"


    def configure(self) -> bool:
        """Configure the ESP32S3 PCNT hardware to count rising edges on the specified pin."""
        # print the current state of the relevant registers for debugging
        print(f"Initial CLK_EN0_REG[{hex(_CLK_EN0_REG)}]: {mem32[_CLK_EN0_REG]:032b}")
        print(f"Initial RST_EN0_REG[{hex(_RST_EN0_REG)}]: {mem32[_RST_EN0_REG]:032b}")
        print(f"Initial PCNT_CTRL_REG[{hex(_PCNT_CTRL_REG)}]: {mem32[_PCNT_CTRL_REG]:032b}")

        try:    
            # --- 1. ENABLE PERIPHERAL CLOCK ---
            # Enable PCNT clock and clear reset bit
            mem32[_CLK_EN0_REG] |= _PCNT_CLK_BIT
            mem32[_RST_EN0_REG] &= ~_PCNT_CLK_BIT

            # --- 2. ENABLE PCNT REGISTER CLOCK GATE ---
            # The PCNT module has an internal register clock gate (PCNT_CTRL_REG bit 16).
            # Registers cannot be read or written until this bit is set.
            # Also hold counter in reset (bit 0 = 1) while configuring.
            mem32[_PCNT_CTRL_REG] = _PCNT_CTRL_CLK_EN | _PCNT_CTRL_RST_U0

            print(f"PCNT_CTRL_REG after gate enable[{hex(_PCNT_CTRL_REG)}]: {mem32[_PCNT_CTRL_REG]:032b}")
            print(f"U0_CONF0_REG after gate enable[{hex(_PCNT_U0_CONF0_REG)}]: {mem32[_PCNT_U0_CONF0_REG]:032b}")

            # --- 3. ROUTE GPIO TO PCNT SIGNAL ---
            # GPIO Matrix: Route physical GPIO pin to Signal 39 (U0_PULSE_CH0_IN)
            mem32[_GPIO_FUNC_IN_SEL_CFG_BASE + (_U0_PULSE_SIG_IDX * 4)] = _SIG_IN_SEL_BIT | self.pin

            # Route constant "1" (0x38) to Control Signal so it's always high
            mem32[_GPIO_FUNC_IN_SEL_CFG_BASE + (_U0_CTRL_SIG_IDX * 4)] = _SIG_IN_SEL_BIT | 0x38

            # --- 4. CONFIGURE PCNT COUNTING ---
            # U0_CONF0_REG layout (ESP32-S3):
            #   [9:0]   FILTER_THRES    [17:16] CH0_NEG_MODE
            #   [10]    FILTER_EN       [19:18] CH0_POS_MODE (1=inc on rising)
            FILTER_VAL = 1023   # 1023 = ~12.8 microsecond filter at 80MHz APB clock
            config = 0
            config |= (FILTER_VAL & _CONF0_FILTER_THRES_M)  # Filter threshold in bits [9:0]
            config |= _CONF0_FILTER_EN                       # Enable filter (bit 10)
            config |= (1 << _CONF0_CH0_POS_MODE_S)           # POS_MODE = 1: Inc on rising edge
            # NEG_MODE = 0 (default): No effect on falling edge
            mem32[_PCNT_U0_CONF0_REG] = config

            # --- 5. RELEASE COUNTER FROM RESET ---
            # Clear RST bit (bit 0) to start counting, keep CLK_EN (bit 16)
            mem32[_PCNT_CTRL_REG] = _PCNT_CTRL_CLK_EN

        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"S:Error configuring PCNT: {e}")
            return False

        print(f"CLK_EN0_REG[{hex(_CLK_EN0_REG)}]: {mem32[_CLK_EN0_REG]:032b}")
        print(f"RST_EN0_REG[{hex(_RST_EN0_REG)}]: {mem32[_RST_EN0_REG]:032b}")
        print(f"PCNT_CTRL_REG[{hex(_PCNT_CTRL_REG)}]: {mem32[_PCNT_CTRL_REG]:032b}")
        print(f"GPIO_FUNC_IN_SEL_CFG for Pulse Signal {_U0_PULSE_SIG_IDX}[{hex(_GPIO_FUNC_IN_SEL_CFG_BASE + (_U0_PULSE_SIG_IDX * 4))}]: {mem32[_GPIO_FUNC_IN_SEL_CFG_BASE + (_U0_PULSE_SIG_IDX * 4)]:032b}")
        print(f"GPIO_FUNC_IN_SEL_CFG for Control Signal {_U0_CTRL_SIG_IDX}[{hex(_GPIO_FUNC_IN_SEL_CFG_BASE + (_U0_CTRL_SIG_IDX * 4))}]: {mem32[_GPIO_FUNC_IN_SEL_CFG_BASE + (_U0_CTRL_SIG_IDX * 4)]:032b}")
        print(f"U0_CONF0_REG[{hex(_PCNT_U0_CONF0_REG)}]: {mem32[_PCNT_U0_CONF0_REG]:032b}")
        print(f"U0_CNT_REG[{hex(_PCNT_PULSE_CNT_U0_REG)}]: {mem32[_PCNT_PULSE_CNT_U0_REG]:032b}")
        
        return True


    def value(self) -> int:
        """Read the current count value and reset the counter to zero."""
        try:
            # Read the 16-bit counter value
            count = mem32[_PCNT_PULSE_CNT_U0_REG] & 0xFFFF
            mem32[_PCNT_CTRL_REG] |= _PCNT_PULSE_CNT_RST_U0  # Write to the reset register to reset the count to zero
            mem32[_PCNT_CTRL_REG] &= ~_PCNT_PULSE_CNT_RST_U0 # Clear the reset bit to allow counting to resume
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"S:Error reading PCNT count: {e}")
            return 0
        
        return count

