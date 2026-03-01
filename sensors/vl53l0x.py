"""
VL53L0X Time-of-Flight distance sensor driver.

Default I2C address: 0x29
Measurement:
  - dist_mm : distance in millimetres (string)

This is an abbreviated initialisation — sufficient for most indoor ranging
but omits the full ST reference SPAD-calibration sequence.

Datasheet / API: https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html
"""

import time
from .sensor_base import SensorBase

# Key registers
_REG_IDENTIFICATION_MODEL_ID        = 0xC0   # should read 0xEE
_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89
_REG_MSRC_CONFIG_CONTROL            = 0x60
_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44
_REG_SYSTEM_SEQUENCE_CONFIG         = 0x01
_REG_DYNAMIC_SPAD_INTERMEDIATE_DATA_DEBUG = 0x3E
_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0
_REG_SYSTEM_INTERRUPT_CONFIG_GPIO   = 0x0A
_REG_GPIO_HV_MUX_ACTIVE_HIGH        = 0x84
_REG_SYSTEM_INTERRUPT_CLEAR         = 0x0B
_REG_RESULT_INTERRUPT_STATUS        = 0x13
_REG_SYSRANGE_START                 = 0x00
_REG_RESULT_RANGE_STATUS            = 0x14  # 12-byte block; range at [10:11]

_RANGE_TIMEOUT_MS = 100   # ms to wait for a measurement


class VL53L0X(SensorBase):
    I2C_ADDR = 0x29
    NAME = "VL53L0X"

    def _init(self) -> bool:
        # Confirm device
        model_id = self._read_u8(_REG_IDENTIFICATION_MODEL_ID)
        if model_id != 0xEE:
            print(f"S:VL53L0X unexpected model ID 0x{model_id:02X}")
            return False

        # Enable 2.8 V I/O (needed when VDD = 2.8 V)
        v = self._read_u8(_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV)
        self._write_u8(_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, v | 0x01)

        # Standard initialisation sequence (abbreviated)
        self._write_u8(0x88, 0x00)
        self._write_u8(0x80, 0x01)
        self._write_u8(0xFF, 0x01)
        self._write_u8(0x00, 0x00)
        self._write_u8(0xFF, 0x00)
        self._write_u8(0x80, 0x00)

        # Disable MSRC and TCC (improves speed slightly)
        msrc = self._read_u8(_REG_MSRC_CONFIG_CONTROL)
        self._write_u8(_REG_MSRC_CONFIG_CONTROL, msrc | 0x12)

        # Set signal rate limit to 0.25 MCPS (fixed-point 9.7)
        self._write_u8(_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 0x00)
        self._write_u8(_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT + 1, 0x20)

        # Set timing budget (single-shot)
        self._write_u8(_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)

        # Configure GPIO to trigger on new sample ready
        self._write_u8(_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
        v = self._read_u8(_REG_GPIO_HV_MUX_ACTIVE_HIGH)
        self._write_u8(_REG_GPIO_HV_MUX_ACTIVE_HIGH, v & ~0x10)
        self._write_u8(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)

        return True

    def _measure(self) -> dict:
        # Single-shot ranging
        self._write_u8(_REG_SYSRANGE_START, 0x01)

        # Wait for measurement complete
        deadline = time.ticks_add(time.ticks_ms(), _RANGE_TIMEOUT_MS)
        while True:
            if self._read_u8(_REG_RESULT_INTERRUPT_STATUS) & 0x07:
                break
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return {"dist_mm": "timeout"}
            time.sleep_ms(5)

        # Read 12-byte result block; range at bytes 10–11 (big-endian)
        raw = self._read_reg(_REG_RESULT_RANGE_STATUS, 12)
        dist_mm = (raw[10] << 8) | raw[11]

        # Clear interrupt
        self._write_u8(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)

        return {"dist_mm": f"{dist_mm}mm"}

    def _shutdown(self):
        pass
