"""
OPT4060 RGBW Colour sensor driver.

Default I2C address: 0x44
Texas Instruments OPT4060 — high-speed, high-precision RGBW colour
sensor with four channels (Red, Green, Blue, Clear/White).

Supports up to four devices on a single I2C bus via address-select pin:
  0x44 — ADDR tied to GND
  0x45 — ADDR tied to VCC
  0x46 — ADDR tied to SDA
  0x47 — ADDR tied to SCL

The OPT4060 shares its register map and the same DEVICE_ID value (0x821)
with the OPT4048 — the two devices are distinguished by their channel
content (RGB vs CIE1931 XYZ) rather than a unique identifier register.

Measurements:
  - red   : Red channel (raw ADC code)
  - green : Green channel (raw ADC code)
  - blue  : Blue channel (raw ADC code)
  - w     : Clear / White channel (raw ADC code)
Datasheet: https://www.ti.com/lit/ds/symlink/opt4060.pdf
"""

import time
from .sensor_base import SensorBase


# ── Register addresses (16-bit big-endian) ──────────────────────────────────
_REG_RED_MSB        = 0x00   # Red channel MSB (exponent[15:12] | mantissa_hi[11:0])
_REG_RED_LSB        = 0x01   # Red channel LSB (mantissa_lo[15:8] | counter[7:4] | crc[3:0])
_REG_GREEN_MSB      = 0x02   # Green channel MSB
_REG_GREEN_LSB      = 0x03   # Green channel LSB
_REG_BLUE_MSB       = 0x04   # Blue channel MSB
_REG_BLUE_LSB       = 0x05   # Blue channel LSB
_REG_CLEAR_MSB      = 0x06   # Clear / White channel MSB
_REG_CLEAR_LSB      = 0x07   # Clear / White channel LSB
_REG_THRESH_LO      = 0x08   # Low threshold
_REG_THRESH_HI      = 0x09   # High threshold
_REG_CONFIG         = 0x0A   # Configuration register
_REG_INT_CTRL       = 0x0B   # Interrupt / threshold configuration
_REG_RES_CTRL       = 0x0C   # Result control / status flags
_REG_DEVICE_ID      = 0x11   # Device ID (expect 0x0821, lower 12 bits = 0x821)

# ── Device identification ────────────────────────────────────────────────────
_DEVICE_ID_MASK     = 0x0FFF   # Lower 12 bits contain device ID
_DEVICE_ID_EXPECT   = 0x0821   # Same value as OPT4048 — distinguish by channel content

# ── Data register bit masks (per 16-bit register) ───────────────────────────
# MSB register  [15:12] exponent, [11:0] mantissa_hi
# LSB register  [15:8]  mantissa_lo, [7:4] sample counter, [3:0] CRC
_DATA_EXPONENT_MASK  = 0xF000   # Bits 15:12 of MSB register
_DATA_MSB_MASK       = 0x0FFF   # Bits 11:0  of MSB register (mantissa high)
_DATA_LSB_MASK       = 0xFF00   # Bits 15:8  of LSB register (mantissa low)
_DATA_COUNTER_MASK   = 0x00F0   # Bits 7:4   of LSB register (sample counter)
_DATA_CRC_MASK       = 0x000F   # Bits 3:0   of LSB register (CRC)

# ── CONFIG register (0x0A) bit layout (16-bit big-endian) ────────────────────
# Bit 15      : QWAKE   — quick wake from standby
# Bits 14     : reserved
# Bits 13-10  : RANGE   — 4-bit full-scale range selector
# Bits 9-6    : CONVERSION_TIME — per-channel integration time selector
# Bits 5-4    : OPERATING_MODE  — power/conversion mode
# Bit 3       : INT_LATCH  — 1 = latch interrupt until status is read
# Bit 2       : INT_POL    — interrupt pin polarity (0 = active-low)
# Bits 1-0    : FAULT_COUNT — number of out-of-range results before interrupt
_CFG_QWAKE_MASK      = 0x8000   # Bit 15
_CFG_RANGE_MASK      = 0x3C00   # Bits 13:10
_CFG_CONV_TIME_MASK  = 0x03C0   # Bits 9:6
_CFG_OPER_MODE_MASK  = 0x0030   # Bits 5:4
_CFG_INT_LATCH_MASK  = 0x0008   # Bit 3
_CFG_INT_POL_MASK    = 0x0004   # Bit 2
_CFG_FAULT_CNT_MASK  = 0x0003   # Bits 1:0

# Range constants (RANGE field, bits 13:10)
RANGE_2K        = 0    # ~2.2 klux full scale
RANGE_4K        = 1    # ~4.5 klux
RANGE_9K        = 2    # ~9 klux
RANGE_18K       = 3    # ~18 klux
RANGE_36K       = 4    # ~36 klux
RANGE_72K       = 5    # ~72 klux
RANGE_144K      = 6    # ~144 klux
RANGE_AUTO      = 12   # Automatic range selection

# Conversion time constants (CONVERSION_TIME field, per channel)
CONV_600US      = 0    # 600 µs
CONV_1MS        = 1    # 1 ms
CONV_1_8MS      = 2    # 1.8 ms
CONV_3_4MS      = 3    # 3.4 ms
CONV_6_5MS      = 4    # 6.5 ms
CONV_12_7MS     = 5    # 12.7 ms
CONV_25MS       = 6    # 25 ms
CONV_50MS       = 7    # 50 ms
CONV_100MS      = 8    # 100 ms
CONV_200MS      = 9    # 200 ms
CONV_400MS      = 10   # 400 ms
CONV_800MS      = 11   # 800 ms

# Operating mode constants (OPERATING_MODE field, bits 5:4)
MODE_POWERDOWN  = 0    # Power-down
MODE_FORCED     = 1    # Forced (auto-range one-shot)
MODE_ONE_SHOT   = 2    # Single conversion then power-down
MODE_CONTINUOUS = 3    # Continuous conversion

# Interrupt polarity constants
INT_POL_ACTIVE_LOW  = 0
INT_POL_ACTIVE_HIGH = 1

# Fault count constants (number of faults before interrupt)
FAULT_COUNT_1   = 0
FAULT_COUNT_2   = 1
FAULT_COUNT_4   = 2
FAULT_COUNT_8   = 3

# ── INT_CTRL register (0x0B) bit layout ──────────────────────────────────────
# Bit 15-7    : reserved
# Bits 6-5    : THRESH_SEL  — threshold channel select (0=Red, 1=Green, 2=Blue, 3=Clear)
# Bit 4       : INT_DIR     — interrupt direction (1 = output, 0 = input)
# Bits 3-2    : INT_CFG     — interrupt configuration
# Bit 1-0     : reserved
_INT_CTRL_THRESH_SEL_MASK  = 0x0060   # Bits 6:5
_INT_CTRL_INT_DIR_MASK     = 0x0010   # Bit 4
_INT_CTRL_INT_CFG_MASK     = 0x000C   # Bits 3:2

# INT_CFG values
_INT_CFG_SMBUS    = 0    # SMBUS alert (threshold interrupt disabled for polling)
_INT_CFG_NEXT_CH  = 1    # Interrupt on next channel conversion complete
_INT_CFG_DISABLED = 0    # Alias: effectively disabled for polled usage
_INT_CFG_ALL_READY = 3   # Interrupt when all channels have converted

# INT_DIR values
_INT_DIR_INPUT  = 0   # INT pin is an input (disabled as output)
_INT_DIR_OUTPUT = 1   # INT pin is an output (driven by sensor)

# Threshold channel select values
_THRESH_CH_RED   = 0
_THRESH_CH_GREEN = 1
_THRESH_CH_BLUE  = 2
_THRESH_CH_CLEAR = 3

# ── RES_CTRL register (0x0C) status flags ────────────────────────────────────
# Bits 15-4   : reserved
# Bit 3       : OVERLOAD   — ADC saturation/overflow on any channel
# Bit 2       : CONV_READY — conversion-complete flag (all channels done)
# Bit 1       : FLAG_H     — measurement exceeds high threshold
# Bit 0       : FLAG_L     — measurement below low threshold
_RES_CTRL_OVERLOAD_MASK    = 0x0008   # Bit 3
_RES_CTRL_CONV_READY_MASK  = 0x0004   # Bit 2
_RES_CTRL_FLAG_H_MASK      = 0x0002   # Bit 1
_RES_CTRL_FLAG_L_MASK      = 0x0001   # Bit 0

# Legacy single-bit names (used internally by driver logic)
_FLAG_READY     = _RES_CTRL_CONV_READY_MASK
_FLAG_OVERLOAD  = _RES_CTRL_OVERLOAD_MASK
_FLAG_HIGH      = _RES_CTRL_FLAG_H_MASK
_FLAG_LOW       = _RES_CTRL_FLAG_L_MASK


class OPT4060(SensorBase):
    """Driver for the TI OPT4060 RGBW colour sensor.

    Returns four 20-bit ADC values:
      "red"   — Red channel
      "green" — Green channel
      "blue"  — Blue channel
      "w"     — Clear / White channel
    """

    I2C_ADDR  = 0x44
    I2C_ADDRS = (0x44, 0x45, 0x46, 0x47)
    NAME      = "OPT4060"
    READ_INTERVAL_MS = 10
    TYPE      = "Colour"

    def __init__(self, i2c_addr: int | None = None):
        super().__init__(i2c_addr)
        self._overload = False

    # ── Configuration helpers (public API) ───────────────────────────────────

    def set_range(self, rng: int):
        """Set the measurement range (use RANGE_* constants or RANGE_AUTO)."""
        cfg = self._read_u16_be(_REG_CONFIG)
        cfg = (cfg & ~_CFG_RANGE_MASK) | ((rng & 0x0F) << 10)
        self._write_u16_be(_REG_CONFIG, cfg)

    def get_range(self) -> int:
        """Return the current range setting."""
        return (self._read_u16_be(_REG_CONFIG) >> 10) & 0x0F

    def set_conversion_time(self, ct: int):
        """Set the per-channel conversion time (use CONV_* constants)."""
        cfg = self._read_u16_be(_REG_CONFIG)
        cfg = (cfg & ~_CFG_CONV_TIME_MASK) | ((ct & 0x0F) << 6)
        self._write_u16_be(_REG_CONFIG, cfg)

    def get_conversion_time(self) -> int:
        """Return the current conversion-time setting."""
        return (self._read_u16_be(_REG_CONFIG) >> 6) & 0x0F

    def set_mode(self, mode: int):
        """Set the operating mode (use MODE_* constants)."""
        cfg = self._read_u16_be(_REG_CONFIG)
        cfg = (cfg & ~_CFG_OPER_MODE_MASK) | ((mode & 0x03) << 4)
        self._write_u16_be(_REG_CONFIG, cfg)

    def get_mode(self) -> int:
        """Return the current operating mode."""
        return (self._read_u16_be(_REG_CONFIG) >> 4) & 0x03

    def set_interrupt_enabled(self, enabled: bool):
        """Enable or disable conversion-ready interrupt on the INT pin.

        When enabled the INT pin asserts after all four channels have been
        converted, allowing the host to poll the status register rather than
        busy-waiting.
        """
        tcfg = self._read_u16_be(_REG_INT_CTRL)
        if enabled:
            tcfg = (tcfg & ~_INT_CTRL_INT_CFG_MASK) | (_INT_CFG_ALL_READY << 2)
        else:
            tcfg = (tcfg & ~_INT_CTRL_INT_CFG_MASK) | (_INT_CFG_DISABLED << 2)
        self._write_u16_be(_REG_INT_CTRL, tcfg)

    def get_interrupt_enabled(self) -> bool:
        """Return True if the conversion-ready interrupt is enabled."""
        return ((self._read_u16_be(_REG_INT_CTRL) >> 2) & 0x03) == _INT_CFG_ALL_READY

    def set_latched_interrupt(self, enabled: bool, threshold_ch: int = _THRESH_CH_CLEAR,
                              threshold_low: int = 0x0000, threshold_high: int = 0xFFFF):
        """Enable or disable latched threshold interrupt.

        When enabled the INT pin is held asserted until the status register
        is read, which is more reliable than the 1 µs pulse of the
        non-latched interrupt mode.
        """
        if enabled:
            self._write_u16_be(_REG_THRESH_LO, threshold_low)
            self._write_u16_be(_REG_THRESH_HI, threshold_high)

        cfg = self._read_u16_be(_REG_CONFIG)
        if enabled:
            cfg |= _CFG_INT_LATCH_MASK      # INT_LATCH = 1
        else:
            cfg &= ~_CFG_INT_LATCH_MASK     # INT_LATCH = 0
        self._write_u16_be(_REG_CONFIG, cfg)

        tcfg = self._read_u16_be(_REG_INT_CTRL)
        if enabled:
            # Set threshold channel, INT as output, threshold interrupt config
            tcfg = (tcfg & 0x8001) | ((threshold_ch & 0x03) << 5) | (_INT_DIR_OUTPUT << 4) | (_INT_CFG_SMBUS << 2)
        else:
            tcfg = tcfg & ~_INT_CTRL_INT_DIR_MASK   # make INT pin an input
        self._write_u16_be(_REG_INT_CTRL, tcfg)

    # ── SensorBase interface ─────────────────────────────────────────────────

    def _init(self) -> bool:
        dev_id = self._read_u16_be(_REG_DEVICE_ID) & _DEVICE_ID_MASK
        if dev_id != _DEVICE_ID_EXPECT:
            print(f"S:OPT4060 ID 0x{dev_id:04X} (expected 0x{_DEVICE_ID_EXPECT:04X}) - rejecting")
            return False

        # Configure for fast continuous reads within ~10 ms budget:
        #   Range       : auto (best dynamic range)
        #   Conv time   : 1.8 ms per channel → 4 × 1.8 ms ≈ 7.2 ms total
        #   Mode        : continuous
        #   INT latch   : latched (bit 3 = 1)
        #   INT polarity: active-low (bit 2 = 0)
        #   Fault count : 1 (bits 1:0 = 0)
        cfg = (RANGE_AUTO << 10) | (CONV_1_8MS << 6) | (MODE_CONTINUOUS << 4) | _CFG_INT_LATCH_MASK
        self._write_u16_be(_REG_CONFIG, cfg)

        # Use latched interrupt mode so the CONV_READY flag stays set long enough
        # to be reliably sampled — the non-latched pulse is only ~1 µs wide.
        self.set_latched_interrupt(True, threshold_low=0x8400, threshold_high=0x8400)

        return True

    def _measure(self) -> dict:
        # Poll status for conversion-ready; timeout after READ_INTERVAL_MS
        deadline = time.ticks_add(time.ticks_ms(), self.READ_INTERVAL_MS)
        while True:
            st = self._read_u16_be(_REG_RES_CTRL)
            if st & _FLAG_READY:
                self._overload = bool(st & _FLAG_OVERLOAD)
                break
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return {"Error": "timeout"}
            time.sleep_ms(1)

        # Burst-read all 4 channels (8 registers × 2 bytes = 16 bytes)
        raw = self._i2c.readfrom_mem(self._i2c_addr, _REG_RED_MSB, 16)

        red   = self._decode_channel(raw, 0)
        green = self._decode_channel(raw, 4)
        blue  = self._decode_channel(raw, 8)
        w     = self._decode_channel(raw, 12)
        return {
            "red":   str(red),
            "green": str(green),
            "blue":  str(blue),
            "w":     str(w),
        }

    @staticmethod
    def _decode_channel(buf: bytes, offset: int) -> int:
        """Decode a single channel from a 4-byte (MSB+LSB register) slice.

        Each channel occupies two consecutive 16-bit big-endian registers:
          MSB register: exponent[15:12] | mantissa_hi[11:0]
          LSB register: mantissa_lo[15:8] | counter[7:4] | crc[3:0]

        ADC code = mantissa_20bit << exponent
        """
        msb_hi = buf[offset]
        msb_lo = buf[offset + 1]
        lsb_hi = buf[offset + 2]
        # lsb_lo contains counter + CRC — not needed for the value

        exp      = (msb_hi >> 4) & 0x0F
        mantissa = ((msb_hi & 0x0F) << 16) | (msb_lo << 8) | lsb_hi
        return mantissa << exp

    def _shutdown(self):
        self.set_mode(MODE_POWERDOWN)
