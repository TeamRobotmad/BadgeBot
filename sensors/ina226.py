"""
INA226 current/voltage/power monitor driver.

Default I2C address range: 0x40-0x4F (A0/A1 address pins).
Measurements:
  - bus_mV      : bus voltage in millivolts
  - current_mA  : current in milliamps
  - power_mW    : power in milliwatts
"""

import time

from .sensor_base import SensorBase

try:
    _ticks_ms = time.ticks_ms
    _ticks_add = time.ticks_add
    _ticks_diff = time.ticks_diff
    _sleep_ms = time.sleep_ms
except AttributeError:
    def _ticks_ms() -> int:
        return int(time.time() * 1000)

    def _ticks_add(base: int, delta: int) -> int:
        return base + delta

    def _ticks_diff(a: int, b: int) -> int:
        return a - b

    def _sleep_ms(delay_ms: int) -> None:
        time.sleep(delay_ms / 1000)


# Register map
_REG_CONFIGURATION = 0x00      # Configuration register
_REG_SHUNT_VOLTAGE = 0x01      # Shunt voltage result (signed)
_REG_BUS_VOLTAGE = 0x02        # Bus voltage result (unsigned)
_REG_POWER = 0x03              # Power result (unsigned)
_REG_CURRENT = 0x04            # Current result (signed)
_REG_CALIBRATION = 0x05        # Calibration register
_REG_MASK_ENABLE = 0x06        # Alert mask/enable register
_REG_ALERT_LIMIT = 0x07        # Alert threshold register
_REG_MANUFACTURER_ID = 0xFE    # Manufacturer ID register
_REG_DIE_ID = 0xFF             # Die ID register


# Configuration register bits (0x00)
_CFG_RESET_BIT = 0x8000        # Software reset bit
_CFG_AVG_SHIFT = 12            # Averaging field shift (bits 14:12)
_CFG_VBUSCT_SHIFT = 9          # Bus voltage conversion time field shift (bits 11:9)
_CFG_VSHCT_SHIFT = 6           # Shunt voltage conversion time field shift (bits 8:6)
_CFG_MODE_SHIFT = 0            # Operating mode field shift (bits 2:0)

# AVG field values (bits 14:12)
_CFG_AVG_1 = 0b000             # 1 sample average
_CFG_AVG_4 = 0b001             # 4 sample average
_CFG_AVG_16 = 0b010            # 16 sample average
_CFG_AVG_64 = 0b011            # 64 sample average
_CFG_AVG_128 = 0b100           # 128 sample average
_CFG_AVG_256 = 0b101           # 256 sample average
_CFG_AVG_512 = 0b110           # 512 sample average
_CFG_AVG_1024 = 0b111          # 1024 sample average

# Conversion time field values for VBUSCT/VSHCT (bits 11:9 and 8:6)
_CFG_CT_140US = 0b000          # 140 us conversion time
_CFG_CT_204US = 0b001          # 204 us conversion time
_CFG_CT_332US = 0b010          # 332 us conversion time
_CFG_CT_588US = 0b011          # 588 us conversion time
_CFG_CT_1100US = 0b100         # 1.1 ms conversion time
_CFG_CT_2116US = 0b101         # 2.116 ms conversion time
_CFG_CT_4156US = 0b110         # 4.156 ms conversion time
_CFG_CT_8244US = 0b111         # 8.244 ms conversion time

# Operating mode field values (bits 2:0)
_CFG_MODE_POWER_DOWN = 0b000   # Power-down mode
_CFG_MODE_SHUNT_TRIG = 0b001   # Shunt voltage, triggered
_CFG_MODE_BUS_TRIG = 0b010     # Bus voltage, triggered
_CFG_MODE_SHUNT_BUS_TRIG = 0b011   # Shunt and bus, triggered
_CFG_MODE_ADC_OFF = 0b100      # ADC off (disabled)
_CFG_MODE_SHUNT_CONT = 0b101   # Shunt voltage, continuous
_CFG_MODE_BUS_CONT = 0b110     # Bus voltage, continuous
_CFG_MODE_SHUNT_BUS_CONT = 0b111   # Shunt and bus, continuous


# Mask/Enable register bits (0x06)
_MASK_SOL = 0x8000             # Shunt over-voltage alert flag
_MASK_SUL = 0x4000             # Shunt under-voltage alert flag
_MASK_BOL = 0x2000             # Bus over-voltage alert flag
_MASK_BUL = 0x1000             # Bus under-voltage alert flag
_MASK_POL = 0x0800             # Power over-limit alert flag
_MASK_CNVR = 0x0400            # Conversion ready alert flag
_MASK_AFF = 0x0010             # Alert function flag
_MASK_CVRF = 0x0008            # Conversion ready flag
_MASK_OVF = 0x0004             # Math overflow flag
_MASK_APOL = 0x0002            # Alert pin polarity select
_MASK_LEN = 0x0001             # Alert latch enable


# Device identification
_MANUFACTURER_ID_TI = 0x5449   # Texas Instruments manufacturer ID


# Driver configuration constants (100 mΩ shunt)
_SHUNT_RESISTOR_MILLIOHM = 100
_CALIBRATION_VALUE = 0x0200    # 512 => 0.1 mA current register LSB with 100 mΩ shunt
_CURRENT_LSB_UA = 100          # 0.1 mA current LSB in microamps
_POWER_LSB_UW = 2500           # 2.5 mW power LSB in microwatts
_READ_TIMEOUT_MS = 40

# Default operating configuration:
#  - shunt conversion: 8.244 ms
#  - bus conversion:   1.1 ms
#  - averaging:        16 samples
_DEFAULT_CONFIGURATION = (
    (_CFG_AVG_16 << _CFG_AVG_SHIFT)
    | (_CFG_CT_1100US << _CFG_VBUSCT_SHIFT)
    | (_CFG_CT_8244US << _CFG_VSHCT_SHIFT)
    | (_CFG_MODE_SHUNT_BUS_CONT << _CFG_MODE_SHIFT)
)


class INA226(SensorBase):
    """INA226 sensor driver with integer fixed-point outputs."""

    I2C_ADDR = 0x40
    I2C_ADDRS = tuple(range(0x40, 0x50))
    NAME = "INA226"
    READ_INTERVAL_MS = 100
    TYPE = "Power"

    def _measure_from_registers(self) -> dict[str, int]:
        bus_raw = self._read_u16_be(_REG_BUS_VOLTAGE)
        current_raw = self._read_s16_be(_REG_CURRENT)
        power_raw = self._read_u16_be(_REG_POWER)

        # Bus LSB = 1.25 mV
        bus_mv = (bus_raw * 125) // 100
        # Current LSB from calibration = 100 uA (0.1 mA)
        current_ma = (current_raw * _CURRENT_LSB_UA) // 1000
        # Power LSB from calibration = 2500 uW (2.5 mW)
        power_mw = (power_raw * _POWER_LSB_UW) // 1000

        return {
            "bus_mV": bus_mv,
            "current_mA": current_ma,
            "power_mW": power_mw,
        }

    def read_sample_if_ready(self) -> dict[str, int] | None:
        """Return one sample in integer units when a new conversion is ready.

        This helper is intended for high-rate internal consumers (for example
        background averaging in motor test mode). The public SensorBase `read()`
        API still returns string values for UI rendering consistency.
        """
        if not self._ready:
            return None
        status = self._read_u16_be(_REG_MASK_ENABLE)
        if (status & _MASK_CVRF) == 0:
            return None
        return self._measure_from_registers()

    def _init(self) -> bool:
        manufacturer = self._read_u16_be(_REG_MANUFACTURER_ID)
        if manufacturer != _MANUFACTURER_ID_TI:
            return False

        self._write_u16_be(_REG_CONFIGURATION, _DEFAULT_CONFIGURATION)
        self._write_u16_be(_REG_CALIBRATION, _CALIBRATION_VALUE)
        return True

    def _measure(self) -> dict:
        deadline = _ticks_add(_ticks_ms(), _READ_TIMEOUT_MS)
        while True:
            sample = self.read_sample_if_ready()
            if sample is not None:
                return {
                    "bus_mV": str(sample["bus_mV"]),
                    "current_mA": str(sample["current_mA"]),
                    "power_mW": str(sample["power_mW"]),
                }
            if _ticks_diff(deadline, _ticks_ms()) <= 0:
                return {"Error": "timeout"}
            _sleep_ms(1)

    def _shutdown(self) -> None:
        self._write_u16_be(_REG_CONFIGURATION, _CFG_MODE_POWER_DOWN)
