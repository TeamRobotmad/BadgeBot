"""
VL6180X Time-of-Flight range + ALS sensor driver.

Default I2C address: 0x29
Measurements:
  - range_mm : distance in mm  (or "timeout" / "err:X")
  - lux      : ambient light in lux (or "timeout")

Uses 16-bit register addresses (ST proprietary protocol).

Datasheet: https://www.st.com/resource/en/datasheet/vl6180x.pdf
"""

import time
from .sensor_base import SensorBase

# -- 16-bit register addresses -------------------------------------------
_IDENTIFICATION_MODEL_ID      = 0x000   # should read 0xB4
_SYSTEM_MODE_GPIO1            = 0x011
_SYSTEM_INTERRUPT_CONFIG      = 0x014   # interrupt condition select
_SYSTEM_INTERRUPT_CLEAR       = 0x015
_SYSTEM_FRESH_OUT_OF_RESET    = 0x016

_SYSRANGE_START               = 0x018
_SYSRANGE_INTERMEASUREMENT    = 0x01B

_SYSALS_START                 = 0x038
_SYSALS_INTEGRATION_PERIOD_HI = 0x040
_SYSALS_INTEGRATION_PERIOD_LO = 0x041
_SYSALS_ANALOGUE_GAIN         = 0x03F

_RESULT_INTERRUPT_STATUS_GPIO = 0x04F
_RESULT_ALS_VAL               = 0x050   # 16-bit, big-endian
_RESULT_RANGE_VAL             = 0x062   # 8-bit

# -- Interrupt-config / status masks -------------------------------------
_INT_NEW_SAMPLE_RANGE = 0x04   # STATUS[2:0] = 0b100 → new sample ready
_INT_NEW_SAMPLE_ALS   = 0x20   # STATUS[5:3] = 0b100 → new sample ready
_INT_CONFIG_BOTH      = 0x24   # enable new-sample for both range and ALS

# -- ALS integration time (register value = period_ms - 1) ---------------
#    100 ms integration → HI=0x00, LO=0x63
_ALS_INT_MS = 100
_ALS_INT_HI = 0x00
_ALS_INT_LO = _ALS_INT_MS - 1   # 0x63

# -- Poll timeouts -------------------------------------------------------
_RANGE_TIMEOUT_MS = 100   # > typical range time (~20 ms)
_ALS_TIMEOUT_MS   = 250   # > integration period (100 ms) + margin


class VL6180X(SensorBase):
    I2C_ADDR = 0x29
    NAME = "VL6180X"

    # ------------------------------------------------------------------
    # 16-bit register helpers (ST VL6180X uses 16-bit addresses)
    # ------------------------------------------------------------------

    def _write_u8_16(self, reg16: int, value: int):
        buf = bytes([(reg16 >> 8) & 0xFF, reg16 & 0xFF, value & 0xFF])
        self._i2c.writeto(self.I2C_ADDR, buf)

    def _read_u8_16(self, reg16: int) -> int:
        buf = bytes([(reg16 >> 8) & 0xFF, reg16 & 0xFF])
        self._i2c.writeto(self.I2C_ADDR, buf, False)
        return self._i2c.readfrom(self.I2C_ADDR, 1)[0]

    def _read_u16_be_16(self, reg16: int) -> int:
        buf = bytes([(reg16 >> 8) & 0xFF, reg16 & 0xFF])
        self._i2c.writeto(self.I2C_ADDR, buf, False)
        d = self._i2c.readfrom(self.I2C_ADDR, 2)
        return (d[0] << 8) | d[1]

    # ------------------------------------------------------------------
    # Interrupt-status polling helper
    # ------------------------------------------------------------------

    def _poll_status(self, mask: int, timeout_ms: int) -> bool:
        """
        Poll RESULT__INTERRUPT_STATUS_GPIO until (status & mask) != 0.
        Returns True on success, False on timeout.
        """
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        while True:
            st = self._read_u8_16(_RESULT_INTERRUPT_STATUS_GPIO)
            if st & mask:
                return True
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return False
            time.sleep_ms(5)

    # ------------------------------------------------------------------
    # SensorBase hooks
    # ------------------------------------------------------------------

    def _init(self) -> bool:
        model_id = self._read_u8_16(_IDENTIFICATION_MODEL_ID)
        if model_id != 0xB4:
            print(f"S:VL6180X unexpected model ID 0x{model_id:02X}")
            return False

        # Check fresh-out-of-reset flag
        fresh = self._read_u8_16(_SYSTEM_FRESH_OUT_OF_RESET)
        if fresh == 0x01:
            # Mandatory private registers from ST AN4545
            self._write_u8_16(0x0207, 0x01)
            self._write_u8_16(0x0208, 0x01)
            self._write_u8_16(0x0096, 0x00)
            self._write_u8_16(0x0097, 0xFD)
            self._write_u8_16(0x00E3, 0x00)
            self._write_u8_16(0x00E4, 0x04)
            self._write_u8_16(0x00E5, 0x02)
            self._write_u8_16(0x00E6, 0x01)
            self._write_u8_16(0x00E7, 0x03)
            self._write_u8_16(0x00F5, 0x02)
            self._write_u8_16(0x00D9, 0x05)
            self._write_u8_16(0x00DB, 0xCE)
            self._write_u8_16(0x00DC, 0x03)
            self._write_u8_16(0x00DD, 0xF8)
            self._write_u8_16(0x009F, 0x00)
            self._write_u8_16(0x00A3, 0x3C)
            self._write_u8_16(0x00B7, 0x00)
            self._write_u8_16(0x00BB, 0x3C)
            self._write_u8_16(0x00B2, 0x09)
            self._write_u8_16(0x00CA, 0x09)
            self._write_u8_16(0x0198, 0x01)
            self._write_u8_16(0x01B0, 0x17)
            self._write_u8_16(0x01AD, 0x00)
            self._write_u8_16(0x00FF, 0x05)
            self._write_u8_16(0x0100, 0x05)
            self._write_u8_16(0x0199, 0x05)
            self._write_u8_16(0x01A6, 0x1B)
            self._write_u8_16(0x01AC, 0x3E)
            self._write_u8_16(0x01A7, 0x1F)
            self._write_u8_16(0x0030, 0x00)
            self._write_u8_16(_SYSTEM_FRESH_OUT_OF_RESET, 0x00)

        # Configure interrupt: new-sample-ready for both range and ALS
        self._write_u8_16(_SYSTEM_INTERRUPT_CONFIG, _INT_CONFIG_BOTH)

        # ALS integration time = 100 ms
        self._write_u8_16(_SYSALS_INTEGRATION_PERIOD_HI, _ALS_INT_HI)
        self._write_u8_16(_SYSALS_INTEGRATION_PERIOD_LO, _ALS_INT_LO)

        # ALS gain = 1x
        self._write_u8_16(_SYSALS_ANALOGUE_GAIN, 0x46)

        # Clear any stale interrupts
        self._write_u8_16(_SYSTEM_INTERRUPT_CLEAR, 0x07)

        return True

    def _measure(self) -> dict:
        result = {}

        # --- Range ---
        self._write_u8_16(_SYSRANGE_START, 0x01)  # single-shot
        if self._poll_status(_INT_NEW_SAMPLE_RANGE, _RANGE_TIMEOUT_MS):
            range_mm = self._read_u8_16(_RESULT_RANGE_VAL)
            self._write_u8_16(_SYSTEM_INTERRUPT_CLEAR, 0x01)
            result["range_mm"] = f"{range_mm}mm"
        else:
            self._write_u8_16(_SYSTEM_INTERRUPT_CLEAR, 0x01)
            result["range_mm"] = "timeout"

        # --- ALS ---
        self._write_u8_16(_SYSALS_START, 0x01)   # single-shot
        if self._poll_status(_INT_NEW_SAMPLE_ALS, _ALS_TIMEOUT_MS):
            als_raw = self._read_u16_be_16(_RESULT_ALS_VAL)
            self._write_u8_16(_SYSTEM_INTERRUPT_CLEAR, 0x02)
            # lux = count * (0.32 / gain) / (integration_ms / 100)
            lux = als_raw * 0.32  # gain=1x, integration=100 ms
            result["lux"] = f"{lux:.1f}lx"
        else:
            self._write_u8_16(_SYSTEM_INTERRUPT_CLEAR, 0x02)
            result["lux"] = "timeout"

        return result

    def _shutdown(self):
        pass
