"""
SensorBase - Abstract base class for all BadgeBot I2C sensor drivers.

Each concrete driver must implement:
  - I2C_ADDR   : int  - default 7-bit I2C address
  - NAME        : str  - human-readable sensor name (≤10 chars for display)
  - begin(i2c)  - initialise the sensor; returns True on success
  - read()      - take a measurement; returns a dict {label: value_string}
  - reset()     - put sensor to a safe/low-power state (called on cleanup)

The manager calls begin() once after confirming the address is present on the
bus, then calls read() periodically while the sensor is selected in the UI.
"""


class SensorBase:
    # Sub-classes must override these
    I2C_ADDR = 0x00
    NAME = "Unknown"
    READ_INTERVAL_MS = 250
    TYPE = "Generic"

    def __init__(self, i2c_addr: int | None = None):
        self._i2c = None
        self._ready = False
        self._i2c_addr = self.I2C_ADDR if i2c_addr is None else i2c_addr

    # ------------------------------------------------------------------
    # Public API (called by SensorManager / app.py)
    # ------------------------------------------------------------------

    def begin(self, i2c) -> bool:
        """Initialise the sensor on the given I2C bus.

        Returns True if the sensor is found and configured successfully.
        Store the i2c object for later use in read().
        """
        self._i2c = i2c
        self._ready = False
        try:
            self._ready = self._init()
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"S:{self.NAME} begin error: {e}")
            self._ready = False
        return self._ready

    def read(self) -> dict:
        """Return the latest measurement as {label: value_string}.

        Returns an empty dict or {'Error': 'msg'} on failure.
        """
        if not self._ready:
            return {"Error": "not ready"}
        try:
            return self._measure()
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"S:{self.NAME} read error: {e}")
            return {"Error": str(e)}

    def reset(self):
        """Put the sensor into a low-power / safe state."""
        try:
            self._shutdown()
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"S:{self.NAME} reset error: {e}")
        self._ready = False

    def shutdown(self):
        """Put the sensor into a low-power state without changing ready state."""
        if self._i2c is None:
            return
        try:
            self._shutdown()
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"S:{self.NAME} shutdown error: {e}")

    @property
    def is_ready(self) -> bool:
        return self._ready

    @property
    def i2c_addr(self) -> int:
        return self._i2c_addr

    # ------------------------------------------------------------------
    # Internal helpers - override in sub-classes
    # ------------------------------------------------------------------

    def _init(self) -> bool:
        """Hardware initialisation. Return True on success."""
        raise NotImplementedError

    def _measure(self) -> dict:
        """Perform measurement. Return dict of {label: value_str}."""
        raise NotImplementedError

    def _shutdown(self):
        """Optional power-down hook.

        Subclasses can implement register writes here when hardware supports
        an explicit shutdown mode. Drivers without dedicated power management
        can leave this as a no-op.
        """
        return

    # ------------------------------------------------------------------
    # Utility helpers available to all drivers
    # ------------------------------------------------------------------

    def _write_reg(self, reg: int, data: bytes):
        if self._i2c is None:
            raise RuntimeError("I2C not initialized")
        self._i2c.writeto_mem(self._i2c_addr, reg, data)

    def _read_reg(self, reg: int, n: int = 1) -> bytes:
        if self._i2c is None:
            raise RuntimeError("I2C not initialized")
        return self._i2c.readfrom_mem(self._i2c_addr, reg, n)

    def _read_u8(self, reg: int) -> int:
        return self._read_reg(reg, 1)[0]

    def _read_u16_le(self, reg: int) -> int:
        d = self._read_reg(reg, 2)
        return d[0] | (d[1] << 8)

    def _read_u16_be(self, reg: int) -> int:
        d = self._read_reg(reg, 2)
        return (d[0] << 8) | d[1]

    def _read_s16_be(self, reg: int) -> int:
        value = self._read_u16_be(reg)
        if value & 0x8000:
            value -= 0x10000
        return value

    def _write_u8(self, reg: int, value: int):
        self._write_reg(reg, bytes([value & 0xFF]))

    def _write_u16_be(self, reg: int, value: int):
        self._write_reg(reg, bytes([(value >> 8) & 0xFF, value & 0xFF]))
