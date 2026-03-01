"""
APDS-9960 proximity + RGBC colour sensor driver.

Default I2C address: 0x39
Measurements:
  - prox  : proximity (0–255, higher = closer)
  - clear : clear channel count
  - red   : red channel count
  - green : green channel count
  - blue  : blue channel count

Datasheet: https://docs.broadcom.com/doc/AV02-4191EN
"""

import time
from .sensor_base import SensorBase

_ID_REG     = 0x92
_ID_APDS    = 0xAB   # APDS-9960

_ENABLE     = 0x80
_ATIME      = 0x81
_PPULSE     = 0x8E
_CONTROL    = 0x8F
_STATUS     = 0x93
_CDATAL     = 0x94   # 8 bytes: C 16LE, R 16LE, G 16LE, B 16LE
_PDATA      = 0x9C

# ENABLE bits
_PON  = 0x01
_AEN  = 0x02
_PEN  = 0x04
_WEN  = 0x08

# STATUS bits
_AVALID = 0x01
_PVALID = 0x02

# Gain
_AGAIN_4X  = 0x01
_PGAIN_4X  = 0x0C

# ATIME: integration cycles = 256 - ATIME_VAL, each cycle = 2.78 ms
_ATIME_VAL = 0xC0   # 64 cycles ≈ 178 ms integration


class APDS9960(SensorBase):
    I2C_ADDR = 0x39
    NAME = "APDS9960"

    def _init(self) -> bool:
        chip_id = self._read_u8(_ID_REG)
        if chip_id != _ID_APDS:
            print(f"S:APDS9960 unexpected ID 0x{chip_id:02X}")
            return False

        # Power off briefly
        self._write_u8(_ENABLE, 0x00)
        time.sleep_ms(5)

        # ALS integration time
        self._write_u8(_ATIME, _ATIME_VAL)
        # AGAIN = 4x
        self._write_u8(_CONTROL, _AGAIN_4X)
        # Proximity: 8 pulses, 16 µs, PGAIN 4x
        self._write_u8(_PPULSE, 0x87)   # pulse len=16µs, count=8
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
