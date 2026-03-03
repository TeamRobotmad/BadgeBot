"""
TCS3472 Colour (RGBC) sensor driver.

Default I2C address: 0x29
Measurements:
  - clear : clear (white) channel (16-bit, raw count)
  - red   : red channel
  - green : green channel
  - blue  : blue channel
  - cct   : correlated colour temperature (K, approximate)
  - lux   : approximate illuminance

The TCS34725 / TCS34723 / TCS34721 share the same register map.

Datasheet: https://ams.com/documents/20143/36005/TCS3472_DS000390_3-00.pdf
"""

import time
from .sensor_base import SensorBase

# The command bit must be set in every register access
_CMD        = 0x80
_CMD_AUTO   = 0xA0  # Auto-increment (combined with register address)

_ID_REG     = _CMD | 0x12
_ID_TCS3472 = 0x44  # TCS34725 / TCS34723
_ID_TCS3471 = 0x4D  # TCS34721 / TCS34729 - same driver works

_ENABLE     = _CMD | 0x00
_ATIME      = _CMD | 0x01
_CONTROL    = _CMD | 0x0F
_STATUS     = _CMD | 0x13
_CDATAL     = _CMD_AUTO | 0x14  # 8 bytes: C 16LE, R 16LE, G 16LE, B 16LE

# ENABLE bits
_PON  = 0x01
_AEN  = 0x02

# STATUS bits
_AVALID = 0x01

# Gain
_GAIN_4X  = 0x01

# ATIME: integration time = (256 - ATIME) * 2.4ms
_ATIME_VAL = 0xEB    # ~35 ms (good for indoor lighting)

_GLASS_ATTENUATION = 1.0   # adjust if sensor is behind glass / diffuser


class TCS3472(SensorBase):
    I2C_ADDR = 0x29
    NAME = "TCS3472"

    # The TCS3472 register addresses already include the command bit,
    # so we don't use the base _read_reg / _write_reg helpers directly.

    def _cmd_write(self, cmd_reg: int, value: int):
        self._i2c.writeto_mem(self.I2C_ADDR, cmd_reg, bytes([value & 0xFF]))

    def _cmd_read(self, cmd_reg: int, n: int = 1) -> bytes:
        return self._i2c.readfrom_mem(self.I2C_ADDR, cmd_reg, n)

    def _init(self) -> bool:
        chip_id = self._cmd_read(_ID_REG, 1)[0]
        if chip_id not in (_ID_TCS3472, _ID_TCS3471):
            print(f"S:TCS3472 unexpected ID 0x{chip_id:02X}")
            return False

        # Power on, wait, then enable ALS
        self._cmd_write(_ENABLE, _PON)
        time.sleep_ms(3)
        self._cmd_write(_ENABLE, _PON | _AEN)

        # Integration time and gain
        self._cmd_write(_ATIME, _ATIME_VAL)
        self._cmd_write(_CONTROL, _GAIN_4X)

        return True

    def _measure(self) -> dict:
        # Wait for a valid reading; integration time ≈ 50 ms, allow 4x margin
        deadline = time.ticks_add(time.ticks_ms(), 200)
        while True:
            st = self._cmd_read(_STATUS, 1)[0]
            if st & _AVALID:
                break
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return {"Error": "timeout"}
            time.sleep_ms(5)

        raw = self._cmd_read(_CDATAL, 8)
        clear = raw[0] | (raw[1] << 8)
        red   = raw[2] | (raw[3] << 8)
        green = raw[4] | (raw[5] << 8)
        blue  = raw[6] | (raw[7] << 8)

        result = {
            "clear": str(clear),
            "red":   str(red),
            "green": str(green),
            "blue":  str(blue),
        }

        # Approximate CCT (McCamy's formula)
        if clear > 0:
            r = red   / clear
            g = green / clear
            b = blue  / clear
            x = -0.14282 * r + 1.54924 * g + -0.95641 * b
            y = -0.32466 * r + 1.57837 * g + -0.73191 * b
            z = -0.68202 * r + 0.77073 * g +  0.56332 * b
            if (x + y + z) > 0:
                xc = x / (x + y + z)
                yc = y / (x + y + z)
                n  = (xc - 0.3320) / (0.1858 - yc) if (0.1858 - yc) != 0 else 0
                cct = int(449.0 * n**3 + 3525.0 * n**2 + 6823.3 * n + 5520.33)
                result["cct"] = f"{cct}K"

            # Approximate lux (Taos DN25)
            lux = (-0.32466 * red + 1.57837 * green + -0.73191 * blue) * _GLASS_ATTENUATION
            result["lux"] = f"{max(0.0, lux):.0f}lx"

        return result

    def _shutdown(self):
        self._cmd_write(_ENABLE, 0x00)
