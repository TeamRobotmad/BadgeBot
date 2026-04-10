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

from .app import DEFAULT_BACKGROUND_UPDATE_PERIOD

# Local sub-states (internal to Sensor Test)
_SUB_SELECT_PORT = 0
_SUB_READING     = 1


# Pages of information to show for each sensor (can be switched with up/down buttons)
_PAGE_RAW = 0
_PAGE_STATS = 1
_PAGE_DATA = 2
_PAGE_NAMES = {
    0: "Raw",
    1: "Stats",
    2: "Data",
}

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
        #self._read_timer += delta
        #if self._read_timer >= self._sensor_mgr.read_interval:
            #print(f"S:Reading sensor (S:read_timer={self._read_timer}ms, count_timer={self._count_timer}ms, sample_count={self.sample_count})")
            #self._count_timer += self._read_timer
            #self._read_timer = 0
        try:
            self._sensor_data = self._sensor_mgr.read_current()
            self.sample_count = self.sample_count + 1
        except Exception as e:      # pylint: disable=broad-exception-caught
            self._sensor_data = {"Error": str(e)}

        self._count_timer += delta
        if self._count_timer >= 1000:
            # compute sample rate
            self._sample_rate = (((1000 * self.sample_count) + 500) // self._count_timer) # sample rate in Hz
            self._count_timer = 0
            self.sample_count = 0
            self._new_sample = True
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
            if self._sensor_mgr is not None:
                self._sensor_mgr.close()
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
