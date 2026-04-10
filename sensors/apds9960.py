"""
APDS-9960 Gesture / Proximity / Colour sensor driver.

Default I2C address: 0x39
In this driver we enable proximity and colour (RGBC) measurement.
Gesture detection is left as a future extension.

Measurements:
  - prox  : proximity count (0-255, higher = closer)
  - clear : clear (white) channel count
  - red   : red channel count
  - green : green channel count
  - blue  : blue channel count

Datasheet: https://docs.broadcom.com/doc/AV02-4191EN
"""

import time
from .sensor_base import SensorBase


_ID_REG        = 0x92
_ID_EXPECT_1   = 0xAB
_ID_EXPECT_2   = 0xA8   # some batches report this

_ENABLE        = 0x80
_ATIME         = 0x81
_CONTROL       = 0x8F
_STATUS        = 0x93
_CDATAL        = 0x94   # CDATA 16-bit LE, then R, G, B each 16-bit LE
_PDATA         = 0x9C   # proximity 8-bit

# ENABLE register bits
_PON   = 0x01   # power on
_AEN   = 0x02   # ALS/colour enable
_PEN   = 0x04   # proximity enable
_WEN   = 0x08   # wait enable

# STATUS bits
_AVALID = 0x01   # colour data valid
_PVALID = 0x02   # proximity data valid

_AGAIN_4X  = 0x01   # ALS gain control
_ATIME_VAL = 0xC0   # integration time ≈ 154ms (~18,000 lux full-scale)


class APDS9960(SensorBase):
    I2C_ADDR = 0x39
    NAME = "APDS9960"

    def _init(self) -> bool:
        chip_id = self._read_u8(_ID_REG)
        if chip_id not in (_ID_EXPECT_1, _ID_EXPECT_2):
            print(f"S:APDS9960 unexpected ID 0x{chip_id:02X}")
            return False

        # Power off, reset
        self._write_u8(_ENABLE, 0x00)
        time.sleep_ms(10)

        # ALS integration time
        self._write_u8(_ATIME, _ATIME_VAL)
        # AGAIN = 4x
        self._write_u8(_CONTROL, _AGAIN_4X)
        # Power on + enable ALS + enable proximity
        self._write_u8(_ENABLE, _PON | _AEN | _PEN)
        time.sleep_ms(5)

        return True

    def _measure(self) -> dict:
        # Wait for valid data; ATIME 0xC0 → ~178 ms integration, allow 2x margin
        deadline = time.ticks_add(time.ticks_ms(), 400)
        while True:
            st = self._read_u8(_STATUS)
            if (st & _AVALID) and (st & _PVALID):
                break
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return {"Error": "timeout"}
            time.sleep_ms(5)

        # Read CRGB (4 x 16-bit little-endian = 8 bytes)
        raw = self._read_reg(_CDATAL, 8)
        clear = raw[0] | (raw[1] << 8)
        red   = raw[2] | (raw[3] << 8)
        green = raw[4] | (raw[5] << 8)
        blue  = raw[6] | (raw[7] << 8)

        # Read proximity (8-bit)
        prox = self._read_u8(_PDATA)

        return {
            "prox":  str(prox),
            "clear": str(clear),
            "red":   str(red),
            "green": str(green),
            "blue":  str(blue),
        }

    def _shutdown(self):
        self._write_u8(_ENABLE, 0x00)
