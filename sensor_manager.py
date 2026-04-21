"""
SensorManager — opens an I2C port, probes for known sensors, and
manages the currently displayed sensor in the BadgeBot sensor-test mode.

Usage (lazy import pattern to conserve badge RAM):
    from .sensor_manager import SensorManager
    mgr = SensorManager(app, logging=True)
    if mgr.open(port=2):
        data = mgr.read_current()   # {label: value_str, ...}
        mgr.next_sensor()
    mgr.close()
"""

from machine import I2C, Pin
from system.hexpansion.config import HexpansionConfig
from .sensors import ALL_SENSOR_CLASSES
from .sensors.sensor_base import SensorBase

#HexSense LED pin
_LED_PIN = 1        # LED to illumiinate area under colour sensor to mmeasure reflected light from surface below.
_INTERRUPT_PIN = 2  # Not currently used, but we can set it up as an input for future interrupt-based drivers


class SensorManager:
    def __init__(self, logging: bool = False):
        self._logging: bool = logging
        self._i2c = None
        self._port: int | None = None
        self._sensors: list[SensorBase] = []      # list of initialised SensorBase instances
        self._index: int = 0         # currently selected sensor
        self._last_data = {}
        self._read_interval_ms = 10
        self._type = "Generic"
        if self.logging:
            print("SensorManager initialised")


    # ------------------------------------------------------------------

    @property
    def logging(self) -> bool:
        return self._logging
    
    @logging.setter
    def logging(self, value: bool):
        self._logging = value

    @property
    def read_interval(self) -> int:
        return self._read_interval_ms

    @property
    def type(self) -> str:
        return self._type    


    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self, port: int) -> bool:
        """Open hexpansion I2C port (1–6), scan, and initialise any found sensors.
        Returns True if at least one sensor was found."""
        self.close()
        self._port = port

        try:
            self._i2c = I2C(port)
        except Exception as e:      # pylint: disable=broad-exception-caught
            if self.logging:
                print(f"SM:Cannot open I2C port {port}: {e}")
            return False

        try:
            found_addrs = set(self._i2c.scan())
        except Exception as e:      # pylint: disable=broad-exception-caught
            if self.logging:
                print(f"SM:I2C scan failed on port {port}: {e}")
            return False

        if self.logging:
            print(f"SM:Port {port} scan: {[hex(a) for a in found_addrs]}")

        for cls in ALL_SENSOR_CLASSES:
            addresses = getattr(cls, "I2C_ADDRS", (getattr(cls, "I2C_ADDR", 0),))
            for address in addresses:
                if address not in found_addrs:
                    continue
                try:
                    sensor = cls(i2c_addr=address)
                except TypeError:
                    sensor = cls()
                if sensor.begin(self._i2c):
                    self._sensors.append(sensor)
                    if self.logging:
                        print(f"SM:  + {cls.NAME} @ 0x{sensor.i2c_addr:02X} {cls.TYPE}")
                elif self.logging:
                    print(f"SM:  - {cls.NAME} @ 0x{address:02X} begin() failed")

        self._index = 0
        self._last_data = {}

        # Set read interval from the first found sensor, or default to 250ms
        if self._sensors:
            self._read_interval_ms = getattr(self._sensors[0], 'READ_INTERVAL_MS', 250)
            self._type = getattr(self._sensors[0], 'TYPE', 'Generic')
        else:
            self._read_interval_ms = 250
            self._type = "Generic"

        # Enable LED if there is at least one sensor
        if len(self._sensors) > 0:
            if self.logging:
                print(f"SM:LED On port {port}")
            config = HexpansionConfig(port)    
            config.ls_pin[_LED_PIN].init(mode=Pin.OUT)
            config.ls_pin[_LED_PIN].value(1)
            config.ls_pin[_INTERRUPT_PIN].init(mode=Pin.IN)

        return len(self._sensors) > 0


    def report_interrupt(self) -> bool:
        """Check if the interrupt pin is active (low)."""
        if self._port is None:
            return False
        config = HexpansionConfig(self._port)
        v = config.ls_pin[_INTERRUPT_PIN].value()
        print(f"INT pin value: {v}")
        return v == 0
    

    def close(self):
        """Shutdown all sensors and release the I2C bus."""
        for s in self._sensors:
            try:
                s.reset()
            except Exception:       # pylint: disable=broad-exception-caught
                pass
        if self._port is not None:
            if self.logging:
                print(f"SM:LED Off port {self._port}")
            config = HexpansionConfig(self._port)    
            if config is not None:
                config.ls_pin[_LED_PIN].value(0)
                config.ls_pin[_LED_PIN].init(mode=Pin.IN)
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
            self._read_interval_ms = getattr(self._sensors[self._index], 'READ_INTERVAL_MS', 250)
            self._type = getattr(self._sensors[self._index], 'TYPE', 'Generic')


    def prev_sensor(self):
        if self._sensors:
            self._index = (self._index - 1) % len(self._sensors)
            self._last_data = {}
            self._read_interval_ms = getattr(self._sensors[self._index], 'READ_INTERVAL_MS', 250)
            self._type = getattr(self._sensors[self._index], 'TYPE', 'Generic')
   
   
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
        sensor = self._sensors[self._index]
        return f"{sensor.NAME}@0x{sensor.i2c_addr:02X}"

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
