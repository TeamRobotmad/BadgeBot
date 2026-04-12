"""
VL53L0X Time-of-Flight distance sensor driver.

Default I2C address: 0x29
Measurement: distance in mm (up to ~1200 mm in default mode).

This driver uses single-shot ranging.  For continuous ranging, call
begin() with continuous=True (not yet implemented - extend as needed).

Datasheet: https://www.st.com/resource/en/datasheet/vl53l0x.pdf
"""

import time
from .sensor_base import SensorBase


_WHO_AM_I_REG    = 0xC0
_WHO_AM_I_EXPECT = 0xEE

# Key registers (abridged - sufficient for single-shot ranging)
_SYSRANGE_START            = 0x00
_RESULT_INTERRUPT_STATUS   = 0x13
_RESULT_RANGE_STATUS       = 0x14
_SYSTEM_SEQUENCE_CONFIG    = 0x01
_MSRC_CONFIG_CONTROL       = 0x60
_FINAL_RANGE_CONF_MIN_CNT  = 0x45
_GLOBAL_CONFIG_VCSEL_WIDTH = 0x70
_SYSTEM_INTERRUPT_CLEAR    = 0x0B
_GPIO_HV_MUX_ACTIVE_HIGH   = 0x84
_SYSTEM_INTERRUPT_CONFIG   = 0x0A

_RANGE_TIMEOUT_MS = 100   # ms to wait for a measurement


class VL53L0X(SensorBase):
    I2C_ADDR = 0x29
    NAME = "VL53L0X"
    READ_INTERVAL_MS = 100
    TYPE = "Distance"
    
    def _init(self) -> bool:
        # Check WHO_AM_I
        who = self._read_u8(_WHO_AM_I_REG)
        if who != _WHO_AM_I_EXPECT:
            print(f"S:VL53L0X unexpected ID 0x{who:02X} (expected 0x{_WHO_AM_I_EXPECT:02X})")
            return False

        # Minimal init sequence to enable single-shot ranging.
        # For a production driver you would replicate ST's full reference
        # init (reading SPAD counts, calibration etc.).  This abbreviated
        # version is sufficient for functional testing of the sensor.

        # Set GPIO interrupt to active-high and configure for range complete
        self._write_u8(_SYSTEM_INTERRUPT_CONFIG, 0x04)  # new sample ready
        gpio = self._read_u8(_GPIO_HV_MUX_ACTIVE_HIGH)
        self._write_u8(_GPIO_HV_MUX_ACTIVE_HIGH, gpio & ~0x10)  # active LOW
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)

        # Disable MSRC and TCC
        self._write_u8(_MSRC_CONFIG_CONTROL, 0x12)

        # Set sequence steps
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)

        return True

    def _measure(self) -> dict:
        # Trigger single-shot measurement
        self._write_u8(_SYSRANGE_START, 0x01)

        # Wait for result (poll interrupt status)
        deadline = time.ticks_add(time.ticks_ms(), _RANGE_TIMEOUT_MS)
        while True:
            status = self._read_u8(_RESULT_INTERRUPT_STATUS)
            if (status & 0x07) != 0:
                break
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return {"dist_mm": "timeout"}
            time.sleep_ms(1)

        # Read range result (bytes 10-11 of RESULT_RANGE_STATUS block)
        data = self._i2c.readfrom_mem(self.I2C_ADDR, _RESULT_RANGE_STATUS + 10, 2)
        dist_mm = (data[0] << 8) | data[1]

        # Clear interrupt
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)

        return {"dist_mm": f"{dist_mm}mm"}
