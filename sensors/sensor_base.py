"""
SensorBase — abstract base class for all BadgeBot I2C sensor drivers.

Subclasses must set class attributes:
    I2C_ADDR : int   — default 7-bit I2C address
    NAME     : str   — human-readable name shown in the UI

And override (at minimum):
    _init()    -> bool      — configure the chip; return False on failure
    _measure() -> dict      — take one reading; return {label: str, ...}

Optional override:
    _shutdown()             — power-down / reset the chip
"""

import struct
import time


class SensorBase:
    I2C_ADDR: int = 0x00
    NAME: str = "Sensor"

    def __init__(self):
        self._i2c = None
        self._ok = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def begin(self, i2c) -> bool:
        """Attach to *i2c* bus and initialise the chip.  Returns True on success."""
        self._i2c = i2c
        try:
            self._ok = self._init()
        except Exception as e:
            print(f"S:{self.NAME} begin error: {e}")
            self._ok = False
        return self._ok

    def read(self) -> dict:
        """Return the latest measurement as {label: value_str, ...}."""
        if not self._ok:
            return {"Error": "not ready"}
        try:
            return self._measure()
        except Exception as e:
            print(f"S:{self.NAME} read error: {e}")
            return {"Error": str(e)}

    def reset(self):
        """Shutdown and re-initialise the sensor."""
        try:
            self._shutdown()
        except Exception:
            pass
        self._ok = False
        if self._i2c is not None:
            self.begin(self._i2c)

    # ------------------------------------------------------------------
    # Subclass hooks
    # ------------------------------------------------------------------

    def _init(self) -> bool:
        raise NotImplementedError

    def _measure(self) -> dict:
        raise NotImplementedError

    def _shutdown(self):
        pass  # optional

    # ------------------------------------------------------------------
    # Low-level I2C helpers  (8-bit register addresses)
    # ------------------------------------------------------------------

    def _write_u8(self, reg: int, value: int):
        self._i2c.writeto_mem(self.I2C_ADDR, reg, bytes([value & 0xFF]))

    def _read_u8(self, reg: int) -> int:
        return self._i2c.readfrom_mem(self.I2C_ADDR, reg, 1)[0]

    def _read_u16_le(self, reg: int) -> int:
        d = self._i2c.readfrom_mem(self.I2C_ADDR, reg, 2)
        return d[0] | (d[1] << 8)

    def _read_u16_be(self, reg: int) -> int:
        d = self._i2c.readfrom_mem(self.I2C_ADDR, reg, 2)
        return (d[0] << 8) | d[1]

    def _read_reg(self, reg: int, n: int) -> bytes:
        return self._i2c.readfrom_mem(self.I2C_ADDR, reg, n)

    def _write_reg(self, reg: int, data: bytes):
        self._i2c.writeto_mem(self.I2C_ADDR, reg, data)
