"""
VL53L0X Time-of-Flight distance sensor driver.

Default I2C address: 0x29
Measurement: distance in mm (up to ~1200 mm in default mode).

This driver uses single-shot ranging.  For continuous ranging, call
begin() with continuous=True (not yet implemented - extend as needed).

Datasheet: https://www.st.com/resource/en/datasheet/vl53l0x.pdf
"""

import time
from ..diagnostics import diagnostics_output
from .sensor_base import SensorBase


_WHO_AM_I_REG    = 0xC0
_WHO_AM_I_EXPECT = 0xEE

# Key registers (abridged - sufficient for single-shot ranging)
_SYSRANGE_START                              = 0x00
_SYSTEM_SEQUENCE_CONFIG                      = 0x01
_SYSTEM_INTERRUPT_CONFIG                     = 0x0A
_SYSTEM_INTERRUPT_CLEAR                      = 0x0B
_RESULT_INTERRUPT_STATUS                     = 0x13
_RESULT_RANGE_STATUS                         = 0x14
_MSRC_CONFIG_CONTROL                         = 0x60
_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44
_GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84
_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0
_GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6
_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E
_DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F
_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89

_STOP_VARIABLE_REG = 0x91
_SPAD_INFO_REG = 0x92
_SPAD_POLL_REG = 0x83
_INTERRUPT_READY_MASK = 0x07

_RANGE_TIMEOUT_MS = 100   # ms to wait for a measurement

_DEFAULT_TUNING_SETTINGS = (
    (0xFF, 0x01), (0x00, 0x00),
    (0xFF, 0x00), (0x09, 0x00), (0x10, 0x00), (0x11, 0x00),
    (0x24, 0x01), (0x25, 0xFF), (0x75, 0x00),
    (0xFF, 0x01), (0x4E, 0x2C), (0x48, 0x00), (0x30, 0x20),
    (0xFF, 0x00), (0x30, 0x09), (0x54, 0x00), (0x31, 0x04),
    (0x32, 0x03), (0x40, 0x83), (0x46, 0x25), (0x60, 0x00),
    (0x27, 0x00), (0x50, 0x06), (0x51, 0x00), (0x52, 0x96),
    (0x56, 0x08), (0x57, 0x30), (0x61, 0x00), (0x62, 0x00),
    (0x64, 0x00), (0x65, 0x00), (0x66, 0xA0),
    (0xFF, 0x01), (0x22, 0x32), (0x47, 0x14), (0x49, 0xFF),
    (0x4A, 0x00),
    (0xFF, 0x00), (0x7A, 0x0A), (0x7B, 0x00), (0x78, 0x21),
    (0xFF, 0x01), (0x23, 0x34), (0x42, 0x00), (0x44, 0xFF),
    (0x45, 0x26), (0x46, 0x05), (0x40, 0x40), (0x0E, 0x06),
    (0x20, 0x1A), (0x43, 0x40),
    (0xFF, 0x00), (0x34, 0x03), (0x35, 0x44),
    (0xFF, 0x01), (0x31, 0x04), (0x4B, 0x09), (0x4C, 0x05),
    (0x4D, 0x04),
    (0xFF, 0x00), (0x44, 0x00), (0x45, 0x20), (0x47, 0x08),
    (0x48, 0x28), (0x67, 0x00), (0x70, 0x04), (0x71, 0x01),
    (0x72, 0xFE), (0x76, 0x00), (0x77, 0x00),
    (0xFF, 0x01), (0x0D, 0x01),
    (0xFF, 0x00), (0x80, 0x01), (0x01, 0xF8),
    (0xFF, 0x01), (0x8E, 0x01), (0x00, 0x01),
    (0xFF, 0x00), (0x80, 0x00),
)


def _ticks_ms() -> int:
    ticks_ms = getattr(time, "ticks_ms", None)
    if ticks_ms is not None:
        return ticks_ms()
    return int(getattr(time, "monotonic")() * 1000)


def _ticks_add(base: int, delta: int) -> int:
    if hasattr(time, "ticks_add"):
        return time.ticks_add(base, delta)
    return base + delta


def _ticks_diff(finish: int, now: int) -> int:
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(finish, now)
    return finish - now


def _sleep_ms(delay_ms: int):
    sleep_ms = getattr(time, "sleep_ms", None)
    if sleep_ms is not None:
        sleep_ms(delay_ms)
        return
    getattr(time, "sleep")(delay_ms / 1000)


class VL53L0X(SensorBase):
    I2C_ADDR = 0x29
    NAME = "VL53L0X"
    READ_INTERVAL_MS = 100
    TYPE = "Distance"

    def __init__(self, i2c_addr: int | None = None):
        super().__init__(i2c_addr=i2c_addr)
        self._stop_variable = 0

    def _init(self) -> bool:
        who = self._read_u8(_WHO_AM_I_REG)
        if who != _WHO_AM_I_EXPECT:
            if self._logging:
                print(f"S:VL53L0X unexpected ID 0x{who:02X} (expected 0x{_WHO_AM_I_EXPECT:02X})")
            return False

        # The VL53L0X needs a substantial startup sequence before single-shot
        # ranging becomes trustworthy; the earlier minimal init returned a
        # fixed-looking distance even though the result register read was valid.
        self._write_u8(
            _VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
            self._read_u8(_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01,
        )
        self._write_u8(0x88, 0x00)

        self._open_stop_variable_window()
        self._stop_variable = self._read_u8(_STOP_VARIABLE_REG)
        self._close_stop_variable_window()

        self._write_u8(
            _MSRC_CONFIG_CONTROL,
            self._read_u8(_MSRC_CONFIG_CONTROL) | 0x12,
        )
        self._set_signal_rate_limit(0.25)
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xFF)

        spad_info = self._get_spad_info()
        if spad_info is None:
            return False

        spad_count, spad_type_is_aperture = spad_info
        ref_spad_map = bytearray(self._read_reg(_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6))
        self._write_u8(0xFF, 0x01)
        self._write_u8(_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)
        self._write_u8(_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)
        self._write_u8(0xFF, 0x00)
        self._write_u8(_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)

        first_spad_to_enable = 12 if spad_type_is_aperture else 0
        spads_enabled = 0
        for index in range(48):
            if index < first_spad_to_enable or spads_enabled == spad_count:
                ref_spad_map[index // 8] &= ~(1 << (index % 8))
                continue
            if (ref_spad_map[index // 8] >> (index % 8)) & 0x01:
                spads_enabled += 1
        self._write_reg(_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, bytes(ref_spad_map))

        for reg, value in _DEFAULT_TUNING_SETTINGS:
            self._write_u8(reg, value)

        self._write_u8(_SYSTEM_INTERRUPT_CONFIG, 0x04)
        self._write_u8(
            _GPIO_HV_MUX_ACTIVE_HIGH,
            self._read_u8(_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10,
        )
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)

        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0x01)
        if not self._perform_single_ref_calibration(0x40):
            return False
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0x02)
        if not self._perform_single_ref_calibration(0x00):
            return False
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)

        return True

    def _measure(self) -> dict:
        diagnostics_output(1,0)
        self._prepare_single_shot()
        self._write_u8(_SYSRANGE_START, 0x01)

        deadline = _ticks_add(_ticks_ms(), _RANGE_TIMEOUT_MS)
        while self._read_u8(_SYSRANGE_START) & 0x01:
            if _ticks_diff(deadline, _ticks_ms()) <= 0:
                return {"dist_mm": "timeout"}
            _sleep_ms(1)

        if not self._wait_for_interrupt_ready():
            return {"dist_mm": "timeout"}

        # The range value lives 10 bytes into the RESULT_RANGE_STATUS block in
        # ST's register map; this offset matches the reference driver.
        dist_mm = self._read_u16_be(_RESULT_RANGE_STATUS + 10)

        if self._logging:
            print(f"S:VL53L0X measured {dist_mm} mm")

        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        diagnostics_output(1,1)

        return {"dist": f"{dist_mm}"}

    def _open_stop_variable_window(self):
        self._write_u8(0x80, 0x01)
        self._write_u8(0xFF, 0x01)
        self._write_u8(0x00, 0x00)

    def _close_stop_variable_window(self):
        self._write_u8(0x00, 0x01)
        self._write_u8(0xFF, 0x00)
        self._write_u8(0x80, 0x00)

    def _prepare_single_shot(self):
        self._open_stop_variable_window()
        self._write_u8(_STOP_VARIABLE_REG, self._stop_variable)
        self._close_stop_variable_window()

    def _wait_for_interrupt_ready(self) -> bool:
        deadline = _ticks_add(_ticks_ms(), _RANGE_TIMEOUT_MS)
        while (self._read_u8(_RESULT_INTERRUPT_STATUS) & _INTERRUPT_READY_MASK) == 0:
            if _ticks_diff(deadline, _ticks_ms()) <= 0:
                return False
            _sleep_ms(1)
        return True

    def _perform_single_ref_calibration(self, vhv_init_byte: int) -> bool:
        self._write_u8(_SYSRANGE_START, 0x01 | vhv_init_byte)
        if not self._wait_for_interrupt_ready():
            return False
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._write_u8(_SYSRANGE_START, 0x00)
        return True

    def _set_signal_rate_limit(self, limit_mcps: float):
        self._write_u16_be(
            _FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
            int(limit_mcps * (1 << 7)),
        )

    def _get_spad_info(self):
        self._open_stop_variable_window()
        self._write_u8(0xFF, 0x06)
        self._write_u8(_SPAD_POLL_REG, self._read_u8(_SPAD_POLL_REG) | 0x04)
        self._write_u8(0xFF, 0x07)
        self._write_u8(0x81, 0x01)
        self._write_u8(0x80, 0x01)
        self._write_u8(0x94, 0x6B)
        self._write_u8(_SPAD_POLL_REG, 0x00)

        deadline = _ticks_add(_ticks_ms(), _RANGE_TIMEOUT_MS)
        while self._read_u8(_SPAD_POLL_REG) == 0x00:
            if _ticks_diff(deadline, _ticks_ms()) <= 0:
                return None
            _sleep_ms(1)

        self._write_u8(_SPAD_POLL_REG, 0x01)
        spad_info = self._read_u8(_SPAD_INFO_REG)

        self._write_u8(0x81, 0x00)
        self._write_u8(0xFF, 0x06)
        self._write_u8(_SPAD_POLL_REG, self._read_u8(_SPAD_POLL_REG) & ~0x04)
        self._write_u8(0xFF, 0x01)
        self._close_stop_variable_window()

        return spad_info & 0x7F, ((spad_info >> 7) & 0x01) == 1
