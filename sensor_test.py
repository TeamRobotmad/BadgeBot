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


# Local sub-states (internal to Sensor Test)
_SUB_SELECT_PORT = 0
_SUB_READING     = 1

# Timing
_SENSOR_READ_INTERVAL_MS = 250


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
        self.app = app
        self._sub_state = _SUB_SELECT_PORT
        self._sensor_mgr = None          # SensorManager instance (lazy-imported)
        self._port_selected: int = 1
        self._sensor_data: dict = {}
        self._logging: bool = logging
        self._read_timer: int = 0
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


    # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Enter the Sensor Test flow from the main menu."""
        app = self.app
        app.set_menu(None)
        app.button_states.clear()
        self._sensor_data = {}
        app.refresh = True
        self._ensure_sensor_mgr()
        self._read_timer = 0
        # If a HexDrive is present, try its port first
        if app.hexdrive_port is not None and self._sensor_mgr.open(app.hexdrive_port):
            self._port_selected = app.hexdrive_port
            self._sub_state = _SUB_READING
        # If no HexDrive, but a HexSense is present, try its port next
        elif app.hexsense_config is not None and app.hexsense_config.port is not None and self._sensor_mgr.open(app.hexsense_config.port):
            self._port_selected = app.hexsense_config.port
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
            self._ensure_sensor_mgr()
            self._sensor_data = {}
            self._read_timer = 0
            app.refresh = True
            if self._sensor_mgr.open(self._port_selected):
                self._sub_state = _SUB_READING
            else:
                app.notification = Notification("      No      Sensors", port=self._port_selected)
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if self._sensor_mgr is not None:
                self._sensor_mgr.close()
            app.return_to_menu()

    def _update_reading(self, delta: int):
        app = self.app
        self._read_timer += delta
        if self._read_timer >= _SENSOR_READ_INTERVAL_MS:
            self._read_timer = 0
            try:
                self._sensor_data = self._sensor_mgr.read_current()
            except Exception as e:      # pylint: disable=broad-exception-caught
                self._sensor_data = {"Error": str(e)}
            app.refresh = True
        if app.button_states.get(BUTTON_TYPES["RIGHT"]):
            app.button_states.clear()
            self._sensor_mgr.next_sensor()
            self._sensor_data = {}
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            app.button_states.clear()
            self._sensor_mgr.prev_sensor()
            self._sensor_data = {}
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            self._sensor_mgr.close()
            self._sub_state = _SUB_SELECT_PORT
            app.refresh = True

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx):
        """Render sensor test UI."""
        app = self.app
        if self._sub_state == _SUB_SELECT_PORT:
            app.draw_message(ctx,
                ["Sensor Test", f"Port: {self._port_selected}"],
                [(1, 1, 1), (0, 1, 1)],
                label_font_size)
            button_labels(ctx, left_label="<Port", right_label="Port>",
                          confirm_label="Scan", cancel_label="Back")
        else:
            num_sensors = self._sensor_mgr.num_sensors if self._sensor_mgr else 1
            sensor_name = self._sensor_mgr.current_sensor_name if self._sensor_mgr else "Sensor"
            if num_sensors > 1:
                title = f"{sensor_name} {self._sensor_mgr.current_sensor_index + 1}/{num_sensors}"
            else:
                title = sensor_name
            lines = [title]
            colours = [(0, 1, 1)]
            if self._sensor_data:
                for label, value in self._sensor_data.items():
                    lines.append(f"{label}:{value}")
                    colours.append((1, 1, 0))
            else:
                lines.append("Reading...")
                colours.append((0.5, 0.5, 0.5))
            app.draw_message(ctx, lines, colours, label_font_size)
            if num_sensors > 1:
                button_labels(ctx, left_label="<Prev", right_label="Next>",
                              cancel_label="Back")
            else:
                button_labels(ctx, cancel_label="Back")
