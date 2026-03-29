"""
SensorManager — opens an I2C port, probes for known sensors, and
manages the currently displayed sensor in the BadgeBot sensor-test mode.

Usage (lazy import pattern to conserve badge RAM):
    from .sensor_manager import SensorManager
    mgr = SensorManager(logging=True)
    if mgr.open(port=2):
        data = mgr.read_current()   # {label: value_str, ...}
        mgr.next_sensor()
    mgr.close()
"""

import machine
from .sensors import ALL_SENSOR_CLASSES


class SensorManager:
    def __init__(self, logging: bool = False):
        self._logging: bool = logging
        self._i2c = None
        self._port: int = None
        self._sensors = []      # list of initialised SensorBase instances
        self._index: int = 0         # currently selected sensor
        self._last_data = {}
        if self._logging:
            print("SensorManager initialised")


    # ------------------------------------------------------------------

    @property
    def logging(self) -> bool:
        return self._logging
    
    @logging.setter
    def logging(self, value: bool):
        self._logging = value


    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self, port: int) -> bool:
        """Open hexpansion I2C port (1–6), scan, and initialise any found sensors.
        Returns True if at least one sensor was found."""
        self.close()
        self._port = port

        try:
            self._i2c = machine.I2C(port)
        except Exception as e:      # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"SM:Cannot open I2C port {port}: {e}")
            return False

        try:
            found_addrs = set(self._i2c.scan())
        except Exception as e:      # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"SM:I2C scan failed on port {port}: {e}")
            return False

        if self._logging:
            print(f"SM:Port {port} scan: {[hex(a) for a in found_addrs]}")

        for cls in ALL_SENSOR_CLASSES:
            if cls.I2C_ADDR in found_addrs:
                sensor = cls()
                if sensor.begin(self._i2c):
                    self._sensors.append(sensor)
                    if self._logging:
                        print(f"SM:  + {cls.NAME} @ 0x{cls.I2C_ADDR:02X}")
                else:
                    if self._logging:
                        print(f"SM:  - {cls.NAME} begin() failed")

        self._index = 0
        self._last_data = {}
        return len(self._sensors) > 0

    def close(self):
        """Shutdown all sensors and release the I2C bus."""
        for s in self._sensors:
            try:
                s.reset()
            except Exception:       # pylint: disable=broad-exception-caught
                pass
        self._sensors = []
        self._index = 0
        self._last_data = {}
        self._i2c = None
        self._port = None

    # ------------------------------------------------------------------
    # Sensor selection
    # ------------------------------------------------------------------

    def next_sensor(self):
        if self._sensors:
            self._index = (self._index + 1) % len(self._sensors)
            self._last_data = {}

    def prev_sensor(self):
        if self._sensors:
            self._index = (self._index - 1) % len(self._sensors)
            self._last_data = {}

    def select_sensor(self, name: str) -> bool:
        """Select sensor by NAME. Returns True if found."""
        for idx, sensor in enumerate(self._sensors):
            if sensor.NAME == name:
                self._index = idx
                self._last_data = {}
                return True
        return False

    # ------------------------------------------------------------------
    # Reading
    # ------------------------------------------------------------------

    def read_current(self) -> dict:
        """Read the currently selected sensor; cache result in last_data."""
        if not self._sensors:
            return {"Error": "no sensors"}
        self._last_data = self._sensors[self._index].read()
        return self._last_data

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def num_sensors(self) -> int:
        return len(self._sensors)

    @property
    def current_sensor_name(self) -> str:
        if not self._sensors:
            return "none"
        return self._sensors[self._index].NAME

    @property
    def current_sensor_index(self) -> int:
        return self._index

    @property
    def last_data(self) -> dict:
        return self._last_data

    @property
    def port(self):
        return self._port

    @property
    def is_open(self) -> bool:
        return self._i2c is not None and len(self._sensors) > 0

    def sensor_list(self) -> list:
        """Return [(index, name), ...] for all found sensors."""
        return [(i, s.NAME) for i, s in enumerate(self._sensors)]
