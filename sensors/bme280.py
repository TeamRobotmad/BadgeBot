"""
BME280 / BMP280 temperature + pressure (+ humidity) sensor driver.

Default I2C address: 0x76  (SDO to GND)  or 0x77 (SDO to VDD)
Measurements:
  - temp   : temperature in °C
  - press  : pressure in hPa
  - humid  : relative humidity % (BME280 only; absent for BMP280)

Full Bosch compensation formulae applied.

Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
"""

import time
import struct
from .sensor_base import SensorBase

_REG_ID          = 0xD0
_REG_RESET       = 0xE0
_REG_CTRL_HUM    = 0xF2
_REG_STATUS      = 0xF3
_REG_CTRL_MEAS   = 0xF4
_REG_CONFIG      = 0xF5
_REG_PRESS_MSB   = 0xF7   # 0xF7..0xF9
_REG_TEMP_MSB    = 0xFA   # 0xFA..0xFC
_REG_HUM_MSB     = 0xFD   # 0xFD..0xFE

_REG_CALIB_00    = 0x88   # 26 bytes of T/P calibration
_REG_CALIB_26    = 0xE1   # 7 bytes of H calibration

_ID_BME280 = 0x60
_ID_BMP280_A = 0x56
_ID_BMP280_B = 0x57
_ID_BMP280_C = 0x58


class BME280(SensorBase):
    I2C_ADDR = 0x76
    NAME = "BME280"

    def _init(self) -> bool:
        chip_id = self._read_u8(_REG_ID)
        self._is_bme = chip_id == _ID_BME280
        if chip_id not in (_ID_BME280, _ID_BMP280_A, _ID_BMP280_B, _ID_BMP280_C):
            print(f"S:BME280 unexpected ID 0x{chip_id:02X}")
            return False

        # Soft reset
        self._write_u8(_REG_RESET, 0xB6)
        time.sleep_ms(10)

        # Load calibration
        self._load_calibration()

        if self._is_bme:
            # Humidity oversampling x1
            self._write_u8(_REG_CTRL_HUM, 0x01)

        # Temp x2, Pressure x4, Normal mode
        self._write_u8(_REG_CTRL_MEAS, 0x4F)  # osrs_t=2, osrs_p=4, mode=normal (0b01001111)
        # Standby 125ms, filter x4
        self._write_u8(_REG_CONFIG, 0x14)

        time.sleep_ms(100)
        return True

    def _load_calibration(self):
        raw = self._read_reg(_REG_CALIB_00, 26)
        (self._T1, self._T2, self._T3,
         self._P1, self._P2, self._P3,
         self._P4, self._P5, self._P6,
         self._P7, self._P8, self._P9) = struct.unpack_from('<HhhHhhhhhhhh', raw, 0)

        if self._is_bme:
            h = self._read_reg(_REG_CALIB_26, 7)
            self._H1 = self._read_u8(0xA1)
            self._H2 = struct.unpack_from('<h', h, 0)[0]
            self._H3 = h[2]
            self._H4 = (h[3] << 4) | (h[4] & 0x0F)
            if self._H4 > 2047:
                self._H4 -= 4096
            self._H5 = (h[5] << 4) | (h[4] >> 4)
            if self._H5 > 2047:
                self._H5 -= 4096
            self._H6 = struct.unpack_from('b', h, 6)[0]

    def _compensate_temp(self, adc_T: int):
        var1 = (adc_T / 16384.0 - self._T1 / 1024.0) * self._T2
        var2 = ((adc_T / 131072.0 - self._T1 / 8192.0) ** 2) * self._T3
        self._t_fine = int(var1 + var2)
        return (var1 + var2) / 5120.0

    def _compensate_pressure(self, adc_P: int) -> float:
        var1 = self._t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self._P6 / 32768.0
        var2 += var1 * self._P5 * 2.0
        var2 = var2 / 4.0 + self._P4 * 65536.0
        var1 = (self._P3 * var1 * var1 / 524288.0 + self._P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self._P1
        if var1 == 0:
            return 0.0
        p = 1048576.0 - adc_P
        p = ((p - var2 / 4096.0) * 6250.0) / var1
        var1 = self._P9 * p * p / 2147483648.0
        var2 = p * self._P8 / 32768.0
        return p + (var1 + var2 + self._P7) / 16.0

    def _compensate_humidity(self, adc_H: int) -> float:
        h = self._t_fine - 76800.0
        if h == 0:
            return 0.0
        h = (adc_H - (self._H4 * 64.0 + self._H5 / 16384.0 * h)) * \
            (self._H2 / 65536.0 * (1.0 + self._H6 / 67108864.0 * h *
             (1.0 + self._H3 / 67108864.0 * h)))
        h *= (1.0 - self._H1 * h / 524288.0)
        return max(0.0, min(100.0, h))

    def _measure(self) -> dict:
        # Read all data registers in one burst
        raw = self._read_reg(_REG_PRESS_MSB, 8)

        adc_P = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4)
        adc_T = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4)
        adc_H = (raw[6] << 8)  | raw[7]

        temp  = self._compensate_temp(adc_T)
        press = self._compensate_pressure(adc_P) / 100.0  # Pa -> hPa

        result = {
            "temp":  f"{temp:.1f}C",
            "press": f"{press:.1f}hPa",
        }

        if self._is_bme and adc_H != 0:
            humid = self._compensate_humidity(adc_H)
            result["humid"] = f"{humid:.1f}%"

        return result

    def _shutdown(self):
        # Put into sleep mode
        ctrl = self._read_u8(_REG_CTRL_MEAS)
        self._write_u8(_REG_CTRL_MEAS, ctrl & ~0x03)
