"""
TCS3430 Colour (XYZ) sensor driver.

Default I2C address: 0x39
The TCS3430 is from the ams-OSRAM family

Measurements:
  - Z : Zchannel
  - Y : Ychannel
  - I : Ichannel
  - X : Xchannel

"""

import time
from .sensor_base import SensorBase

# Registers 
_ENABLE     = 0x80
_ATIME      = 0x81
_WTIME      = 0x83
_CFG1_REG   = 0x90
_REVID_REG  = 0x91
_ID_REG     = 0x92
_STATUS     = 0x93
_RDATAL     = 0x94  # 8 bytes: R 16LE, G 16LE, B 16LE, W 16LE or might be other e.g. XYZI depending on part/revision; confirm with your part
_CFG3_REG   = 0xAB
_INTENABLE  = 0xDD

_ID_EXPECT  = 0xDC   # TCS3430 family identifier; verify with your part
_REVID_EXPECT = 0x41   # TCS3430 revision; verify with your part

# ENABLE bits
_PON  = 0x01
_AEN  = 0x02
_WEN  = 0x08

# STATUS/INT bits
_AINT = 0x10
_ASAT = 0x80

# CFG3 bits
_INT_READ_CLEAR = 0x80  # Clear interrupt on any status read

# Gain: 0x00=1x, 0x01=4x, 0x02=16x, 0x03=64x
_GAIN_1X  = 0x00
_GAIN_4X  = 0x01
_GAIN_16X = 0x02
_GAIN_64X = 0x03

# ATIME: integration = (256 - ATIME_VAL) * 2.78 ms (TCS3430 specific)
_ATIME_VAL = 0x02 # ~9ms


class TCS3430(SensorBase):
    I2C_ADDR = 0x39 # 00111001b
    NAME = "TCS3430"
    READ_INTERVAL_MS = 10
    TYPE = "Colour"
    
    _saturation: bool = False

    def _cmd_write(self, cmd_reg: int, value: int):
        self._i2c.writeto_mem(self.I2C_ADDR, cmd_reg, bytes([value & 0xFF]))

    def _cmd_read(self, cmd_reg: int, n: int = 1) -> bytes:
        return self._i2c.readfrom_mem(self.I2C_ADDR, cmd_reg, n)

    def _init(self) -> bool:
        chip_id = self._cmd_read(_ID_REG, 1)[0]
        if chip_id != _ID_EXPECT:
            # Warn but proceed — early silicon may report a different ID
            print(f"S:TCS3430 ID 0x{chip_id:02X} (expected 0x{_ID_EXPECT:02X}) - proceeding")

        chip_id = self._cmd_read(_REVID_REG, 1)[0]
        if chip_id != _REVID_EXPECT:
            # Warn but proceed — early silicon may report a different ID
            print(f"S:TCS3430 REV 0x{chip_id:02X} (expected 0x{_REVID_EXPECT:02X}) - proceeding")

        # Power on
        self._cmd_write(_ENABLE, _PON)
        time.sleep_ms(3)
        # Enable ALS
        self._cmd_write(_ENABLE, _PON | _AEN)
        # Integration time and gain
        self._cmd_write(_ATIME, _ATIME_VAL)
        self._cmd_write(_CFG1_REG, _GAIN_64X)
        self._cmd_write(_INTENABLE, _AINT)  # interrupt on ALS complete
        self._cmd_write(_CFG3_REG, _INT_READ_CLEAR)  # clear on any status read

        return True

    def _measure(self) -> dict:
        # Wait for valid data; integration time ≈ 3x2.78=8.34 ms
        deadline = time.ticks_add(time.ticks_ms(), 10)
        while True:
            st = self._cmd_read(_STATUS, 1)[0]
            if st & _AINT:
                if st & _ASAT:
                    self._saturation = True
                else:
                    self._saturation = False
                break
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return {"Error": "timeout"}
            time.sleep_ms(2)

        # Read 8 bytes: Z, Y, I, X each 16-bit LE
        raw = self._cmd_read(_RDATAL, 8)
        z   = raw[0] | (raw[1] << 8)
        y   = raw[2] | (raw[3] << 8)
        i   = raw[4] | (raw[5] << 8)
        x   = raw[6] | (raw[7] << 8)

        return {
            "x":     str(x),
            "y":     str(y),
            "z":     str(z),
        }

    def _shutdown(self):
        self._cmd_write(_ENABLE, 0x00)
