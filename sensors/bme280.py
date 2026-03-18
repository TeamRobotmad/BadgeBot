"""
BME280 Temperature / Pressure / Humidity sensor driver.

Default I2C address: 0x76 (SDO low) or 0x77 (SDO high).
Measurements:
  - temp   : temperature in °C
  - press  : pressure in hPa
  - humid  : relative humidity in %RH

The driver reads the factory compensation coefficients once during
begin() and applies the full compensation formulas from the datasheet.

Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
"""

import time
from .sensor_base import SensorBase


_CHIP_ID_REG    = 0xD0
_BME280_CHIP_ID = 0x60
_BMP280_CHIP_ID = 0x58   # pressure+temp only, no humidity

_RESET_REG   = 0xE0
_SOFT_RESET  = 0xB6

_CTRL_HUM    = 0xF2
_STATUS      = 0xF3
_CTRL_MEAS   = 0xF4
_CONFIG      = 0xF5

_PRESS_MSB   = 0xF7  # 3 bytes: press, 3 bytes: temp, 2 bytes: hum

# Oversampling x1 for all, forced mode
_OS_1X  = 0b001
_MODE_FORCED = 0b01


class BME280(SensorBase):
    I2C_ADDR = 0x76
    NAME = "BME280"

    def _init(self) -> bool:
        chip_id = self._read_u8(_CHIP_ID_REG)
        if chip_id not in (_BME280_CHIP_ID, _BMP280_CHIP_ID):
            print(f"S:BME280 unexpected ID 0x{chip_id:02X}")
            return False
        self._has_humidity = (chip_id == _BME280_CHIP_ID)

        # Soft reset
        self._write_u8(_RESET_REG, _SOFT_RESET)
        time.sleep_ms(10)

        # Load compensation coefficients
        self._load_calib()

        # Configure: humidity os x1, temp+press os x1
        if self._has_humidity:
            self._write_u8(_CTRL_HUM, _OS_1X)   # must be set before CTRL_MEAS
        return True

    def _load_calib(self):
        """Read temperature and pressure trim from 0x88-0x9F, humidity from 0xA1, 0xE1-0xE7."""
        raw = self._read_reg(0x88, 24)

        def u16(lo, hi): return raw[lo] | (raw[hi] << 8)
        def s16(lo, hi):
            v = u16(lo, hi)
            return v - 65536 if v >= 32768 else v

        # Temperature
        self._T1 = u16(0, 1)
        self._T2 = s16(2, 3)
        self._T3 = s16(4, 5)
        # Pressure
        self._P1 = u16(6, 7)
        self._P2 = s16(8, 9)
        self._P3 = s16(10, 11)
        self._P4 = s16(12, 13)
        self._P5 = s16(14, 15)
        self._P6 = s16(16, 17)
        self._P7 = s16(18, 19)
        self._P8 = s16(20, 21)
        self._P9 = s16(22, 23)

        if self._has_humidity:
            self._H1 = self._read_u8(0xA1)
            raw_h = self._read_reg(0xE1, 7)
            self._H2 = raw_h[0] | (raw_h[1] << 8)
            if self._H2 >= 32768: self._H2 -= 65536
            self._H3 = raw_h[2]
            self._H4 = (raw_h[3] << 4) | (raw_h[4] & 0x0F)
            if self._H4 >= 2048: self._H4 -= 4096
            self._H5 = (raw_h[5] << 4) | (raw_h[4] >> 4)
            if self._H5 >= 2048: self._H5 -= 4096
            self._H6 = raw_h[6]
            if self._H6 >= 128: self._H6 -= 256

    def _measure(self) -> dict:
        # Trigger one forced measurement
        ctrl = (_OS_1X << 5) | (_OS_1X << 2) | _MODE_FORCED
        self._write_u8(_CTRL_MEAS, ctrl)

        # Wait for measurement to complete (~4 ms at os=1x)
        deadline = time.ticks_add(time.ticks_ms(), 30)
        while True:
            if (self._read_u8(_STATUS) & 0x08) == 0:
                break
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return {"Error": "timeout"}
            time.sleep_ms(1)

        # Read raw ADC values
        data = self._read_reg(_PRESS_MSB, 8)
        adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        adc_h = (data[6] << 8)  |  data[7]  if self._has_humidity else 0

        # Temperature compensation
        var1 = ((adc_t / 16384.0) - (self._T1 / 1024.0)) * self._T2
        var2 = ((adc_t / 131072.0) - (self._T1 / 8192.0)) ** 2 * self._T3
        t_fine = var1 + var2
        temp_c = t_fine / 5120.0

        # Pressure compensation
        var1 = t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self._P6 / 32768.0
        var2 = var2 + var1 * self._P5 * 2.0
        var2 = var2 / 4.0 + self._P4 * 65536.0
        var1 = (self._P3 * var1 * var1 / 524288.0 + self._P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self._P1
        if var1 == 0:
            press_hpa = 0.0
        else:
            p = (1048576.0 - adc_p - var2 / 4096.0) * 6250.0 / var1
            var1 = self._P9 * p * p / 2147483648.0
            var2 = p * self._P8 / 32768.0
            p = p + (var1 + var2 + self._P7) / 16.0
            press_hpa = p / 100.0

        result = {
            "temp":  f"{temp_c:.1f}C",
            "press": f"{press_hpa:.1f}hPa",
        }

        if self._has_humidity:
            h = t_fine - 76800.0
            if h != 0:
                h = (adc_h - (self._H4 * 64.0 + (self._H5 / 16384.0) * h)) * \
                    (self._H2 / 65536.0 *
                     (1.0 + self._H6 / 67108864.0 * h *
                      (1.0 + self._H3 / 67108864.0 * h)))
                h = h * (1.0 - self._H1 * h / 524288.0)
                h = max(0.0, min(100.0, h))
            result["humid"] = f"{h:.1f}%"

        return result
