"""
VL6180X Time-of-Flight proximity + ambient light sensor driver.

Default I2C address: 0x29
Measurements:
  - range_mm : distance in mm (0-100 mm typical)
  - lux      : ambient light in lux (requires ALS gain setting)

Datasheet: https://www.st.com/resource/en/datasheet/vl6180x.pdf
"""

import time
from .sensor_base import SensorBase


_MODEL_ID_REG    = 0x000
_MODEL_ID_EXPECT = 0xB4

# Key registers (16-bit addresses)
_SYSTEM_INTERRUPT_CONFIG      = 0x014   # configure which conditions flag the status register
_SYSTEM_FRESH_OUT_OF_RESET    = 0x016
_SYSRANGE_START               = 0x018
_SYSALS_INTEGRATION_PERIOD_HI = 0x040
_SYSALS_INTEGRATION_PERIOD_LO = 0x041
_SYSALS_START                 = 0x038
_SYSALS_ANALOGUE_GAIN         = 0x03F
_RESULT_INTERRUPT_STATUS_GPIO = 0x04F
_RESULT_RANGE_STATUS          = 0x04D  # lower nibble = error code; 0 = valid
_RESULT_RANGE_VAL             = 0x062
_RESULT_ALS_VAL               = 0x050  # 2 bytes, big-endian
_SYSTEM_INTERRUPT_CLEAR       = 0x015

# SYSTEM__INTERRUPT_CONFIG: bits[2:0]=range condition, bits[5:3]=ALS condition
# 0b100 = "new sample ready" for each channel
_INT_NEW_SAMPLE_RANGE = 0x04
_INT_NEW_SAMPLE_ALS   = 0x04 << 3   # = 0x20
_INT_CONFIG_BOTH      = _INT_NEW_SAMPLE_RANGE | _INT_NEW_SAMPLE_ALS  # 0x24

_ALS_GAIN_1X = 0x46   # register encoding for gain = 1.0x

# ALS integration time: register value = period_ms - 1
_ALS_INT_PERIOD_MS = 100
_ALS_INT_HI = ((_ALS_INT_PERIOD_MS - 1) >> 8) & 0xFF
_ALS_INT_LO  = (_ALS_INT_PERIOD_MS - 1) & 0xFF

_RANGE_TIMEOUT_MS = 100   # > typical range time (~20 ms)
_ALS_TIMEOUT_MS   = 250   # > integration period (100 ms) + margin


class VL6180X(SensorBase):
    I2C_ADDR = 0x29
    NAME = "VL6180X"
    READ_INTERVAL_MS = 100
    TYPE = "Distance"

    # The VL6180X uses 16-bit register addresses, so we override the helpers.
    def _write_reg16(self, reg: int, data: bytes):
        addr_bytes = bytes([(reg >> 8) & 0xFF, reg & 0xFF])
        self._i2c.writeto(self.I2C_ADDR, addr_bytes + data)

    def _read_reg16(self, reg: int, n: int = 1) -> bytes:
        addr_bytes = bytes([(reg >> 8) & 0xFF, reg & 0xFF])
        self._i2c.writeto(self.I2C_ADDR, addr_bytes, False)
        return self._i2c.readfrom(self.I2C_ADDR, n)

    def _write_u8_16(self, reg: int, value: int):
        self._write_reg16(reg, bytes([value & 0xFF]))

    def _read_u8_16(self, reg: int) -> int:
        return self._read_reg16(reg, 1)[0]

    def _read_u16_be_16(self, reg: int) -> int:
        d = self._read_reg16(reg, 2)
        return (d[0] << 8) | d[1]

    def _poll_status(self, mask: int, timeout_ms: int) -> bool:
        """Poll RESULT__INTERRUPT_STATUS_GPIO until (status & mask) is non-zero.

        Returns True on success, False on timeout.
        """
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        while True:
            st = self._read_u8_16(_RESULT_INTERRUPT_STATUS_GPIO)
            if st & mask:
                return True
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return False
            time.sleep_ms(2)

    def _init(self) -> bool:
        model = self._read_u8_16(_MODEL_ID_REG)
        if model != _MODEL_ID_EXPECT:
            print(f"S:VL6180X unexpected ID 0x{model:02X} (expected 0x{_MODEL_ID_EXPECT:02X})")
            return False

        # Apply mandatory private settings if fresh out of reset
        fresh = self._read_u8_16(_SYSTEM_FRESH_OUT_OF_RESET)
        if fresh == 1:
            self._mandatory_settings()
            self._write_u8_16(_SYSTEM_FRESH_OUT_OF_RESET, 0x00)

        # Configure "new sample ready" interrupt condition for range and ALS.
        # Without this, RESULT__INTERRUPT_STATUS_GPIO bits are never raised and
        # every poll times out (default INTERRUPT_CONFIG = 0x00 = no condition).
        self._write_u8_16(_SYSTEM_INTERRUPT_CONFIG, _INT_CONFIG_BOTH)

        # ALS integration time = 100 ms
        self._write_u8_16(_SYSALS_INTEGRATION_PERIOD_HI, _ALS_INT_HI)
        self._write_u8_16(_SYSALS_INTEGRATION_PERIOD_LO, _ALS_INT_LO)

        # ALS analogue gain = 1x
        self._write_u8_16(_SYSALS_ANALOGUE_GAIN, _ALS_GAIN_1X)

        # Clear any stale interrupts
        self._write_u8_16(_SYSTEM_INTERRUPT_CLEAR, 0x07)

        return True

    def _mandatory_settings(self):
        """Private registers that must be set per ST datasheet."""
        settings = [
            (0x0207, 0x01), (0x0208, 0x01), (0x0096, 0x00), (0x0097, 0xFD),
            (0x00E3, 0x00), (0x00E4, 0x04), (0x00E5, 0x02), (0x00E6, 0x01),
            (0x00E7, 0x03), (0x00F5, 0x02), (0x00D9, 0x05), (0x00DB, 0xCE),
            (0x00DC, 0x03), (0x00DD, 0xF8), (0x009F, 0x00), (0x00A3, 0x3C),
            (0x00B7, 0x00), (0x00BB, 0x3C), (0x00B2, 0x09), (0x00CA, 0x09),
            (0x0198, 0x01), (0x01B0, 0x17), (0x01AD, 0x00), (0x00FF, 0x05),
            (0x0100, 0x05), (0x0199, 0x05), (0x01A6, 0x1B), (0x01AC, 0x3E),
            (0x01A7, 0x1F), (0x0030, 0x00),
        ]
        for reg, val in settings:
            self._write_u8_16(reg, val)

    def _measure(self) -> dict:
        result = {}

        # --- Range ---
        self._write_u8_16(_SYSTEM_INTERRUPT_CLEAR, 0x07)  # clear stale flags
        self._write_u8_16(_SYSRANGE_START, 0x01)           # single shot

        if self._poll_status(_INT_NEW_SAMPLE_RANGE, _RANGE_TIMEOUT_MS):
            status = self._read_u8_16(_RESULT_RANGE_STATUS) >> 4  # upper nibble = error code
            if status == 0:
                dist_mm = self._read_u8_16(_RESULT_RANGE_VAL)
                result["dist_mm"] = f"{dist_mm}mm"
            else:
                # Non-zero = measurement error (e.g. 0x0B = no target detected)
                result["dist_mm"] = "error"
        else:
            result["dist_mm"] = "timeout"

        self._write_u8_16(_SYSTEM_INTERRUPT_CLEAR, 0x07)

        # --- ALS ---
        self._write_u8_16(_SYSTEM_INTERRUPT_CLEAR, 0x07)
        self._write_u8_16(_SYSALS_START, 0x01)             # single shot

        if self._poll_status(_INT_NEW_SAMPLE_ALS, _ALS_TIMEOUT_MS):
            als_raw = self._read_u16_be_16(_RESULT_ALS_VAL)
            # lux = count * (0.32 / gain) / (integration_ms / 100)
            lux = als_raw * 0.32  # gain=1x, integration=100 ms
            result["lux"] = f"{lux:.1f}lx"
        else:
            result["lux"] = "timeout"

        self._write_u8_16(_SYSTEM_INTERRUPT_CLEAR, 0x07)

        return result
