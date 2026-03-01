"""
TCS3439 Colour (RGBW) sensor driver.

Default I2C address: 0x39
The TCS3439 is from the same ams-OSRAM family as the TCS3472 but uses a
different I2C address and a slightly different register set (SMBUS protocol).

Measurements:
  - white : white (W) channel (16-bit, raw count)
  - red   : red channel
  - green : green channel
  - blue  : blue channel
  - lux   : approximate illuminance (using green channel)

Datasheet: https://ams-osram.com/products/sensors/color-sensors/ams-tcs3439-color-sensor

NOTE: The TCS3439 register map is compatible with the TCS34903 / TCS3430
      family (SMBUS command protocol, 16-bit addresses with auto-increment).
      Confirm final register values against your specific silicon revision.
"""

import time
from .sensor_base import SensorBase


# Command register (auto-increment block read)
_CMD_AUTO   = 0xA0

_ENABLE     = 0xA0 | 0x00
_ATIME      = 0xA0 | 0x01
_WTIME      = 0xA0 | 0x03
_CONTROL    = 0xA0 | 0x0F
_ID_REG     = 0xA0 | 0x12
_STATUS     = 0xA0 | 0x13
_RDATAL     = 0xA0 | 0x14  # 8 bytes: R 16LE, G 16LE, B 16LE, W 16LE

_ID_EXPECT  = 0x90   # TCS3439 / TCS34903 family identifier; verify with your part

# ENABLE bits
_PON  = 0x01
_AEN  = 0x02

# STATUS bits
_AVALID = 0x01

# Gain: 0x00=1x, 0x01=4x, 0x02=16x, 0x03=64x
_GAIN_4X  = 0x01

# ATIME: integration = (256 - ATIME_VAL) * 2.78 ms (TCS3439 specific)
_ATIME_VAL = 0xEB    # ~35 ms


class TCS3439(SensorBase):
    I2C_ADDR = 0x39
    NAME = "TCS3439"

    def _cmd_write(self, cmd_reg: int, value: int):
        self._i2c.writeto_mem(self.I2C_ADDR, cmd_reg, bytes([value & 0xFF]))

    def _cmd_read(self, cmd_reg: int, n: int = 1) -> bytes:
        return self._i2c.readfrom_mem(self.I2C_ADDR, cmd_reg, n)

    def _init(self) -> bool:
        chip_id = self._cmd_read(_ID_REG, 1)[0]
        if chip_id != _ID_EXPECT:
            # Warn but proceed — early silicon may report a different ID
            print(f"S:TCS3439 ID 0x{chip_id:02X} (expected 0x{_ID_EXPECT:02X}) - proceeding")

        # Power on
        self._cmd_write(_ENABLE, _PON)
        time.sleep_ms(3)
        # Enable ALS
        self._cmd_write(_ENABLE, _PON | _AEN)
        # Integration time and gain
        self._cmd_write(_ATIME, _ATIME_VAL)
        self._cmd_write(_CONTROL, _GAIN_4X)

        return True

    def _measure(self) -> dict:
        # Wait for valid data; integration time ≈ 58 ms, allow 3x margin
        deadline = time.ticks_add(time.ticks_ms(), 200)
        while True:
            st = self._cmd_read(_STATUS, 1)[0]
            if st & _AVALID:
                break
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return {"Error": "timeout"}
            time.sleep_ms(5)

        # Read 8 bytes: R, G, B, W each 16-bit LE
        raw = self._cmd_read(_RDATAL, 8)
        red   = raw[0] | (raw[1] << 8)
        green = raw[2] | (raw[3] << 8)
        blue  = raw[4] | (raw[5] << 8)
        white = raw[6] | (raw[7] << 8)

        # Rough lux from green channel (adjust coefficient for your diffuser/glass)
        lux = green * 0.1

        return {
            "red":   str(red),
            "green": str(green),
            "blue":  str(blue),
            "white": str(white),
            "lux":   f"{lux:.0f}lx",
        }

    def _shutdown(self):
        self._cmd_write(_ENABLE, 0x00)
