"""
OPT4048 Tristimulus XYZ Colour sensor driver.

Default I2C address: 0x44
Texas Instruments OPT4048 — high-speed, high-precision tristimulus XYZ colour
sensor with four channels (X, Y, Z, W/Clear).

Measurements:
  - x : CIE1931 X channel (raw ADC code)
  - y : CIE1931 Y channel (raw ADC code)
  - z : CIE1931 Z channel (raw ADC code)
  - w : Clear channel (raw ADC code)
Datasheet: https://www.ti.com/lit/ds/symlink/opt4048.pdf
"""

import time
from .sensor_base import SensorBase


# ── Register addresses (16-bit big-endian) ──────────────────────────────────
_REG_CH0_MSB        = 0x00   # X channel MSB (exponent[15:12] | mantissa_hi[11:0])
_REG_CH0_LSB        = 0x01   # X channel LSB (mantissa_lo[15:8] | counter[7:4] | crc[3:0])
_REG_CH1_MSB        = 0x02   # Y channel MSB
_REG_CH1_LSB        = 0x03   # Y channel LSB
_REG_CH2_MSB        = 0x04   # Z channel MSB
_REG_CH2_LSB        = 0x05   # Z channel LSB
_REG_CH3_MSB        = 0x06   # W (Clear) channel MSB
_REG_CH3_LSB        = 0x07   # W (Clear) channel LSB
_REG_THRESH_LO      = 0x08   # Low threshold
_REG_THRESH_HI      = 0x09   # High threshold
_REG_CONFIG         = 0x0A   # Configuration register
_REG_THRESH_CFG     = 0x0B   # Threshold / interrupt configuration
_REG_STATUS         = 0x0C   # Status flags
_REG_DEVICE_ID      = 0x11   # Device ID (expect 0x0821)

# ── Device identification ────────────────────────────────────────────────────
_DEVICE_ID_EXPECT   = 0x0821

# ── CONFIG register (0x0A) bit layout (16-bit big-endian) ────────────────────
# Bit 15      : QWAKE  (quick wake-up)
# Bits 13-10  : RANGE  (4 bits)
# Bits 9-6    : CONVERSION_TIME (4 bits)
# Bits 5-4    : OPERATING_MODE (2 bits)
# Bit 3       : INT_LATCH
# Bit 2       : INT_POL
# Bits 1-0    : FAULT_COUNT

# Range constants
RANGE_2K        = 0   # 2.2 klux full scale
RANGE_4K        = 1   # 4.5 klux
RANGE_9K        = 2   # 9 klux
RANGE_18K       = 3   # 18 klux
RANGE_36K       = 4   # 36 klux
RANGE_72K       = 5   # 72 klux
RANGE_144K      = 6   # 144 klux
RANGE_AUTO      = 12  # Auto-range

# Conversion time constants (per channel)
CONV_600US      = 0   # 600 µs
CONV_1MS        = 1   # 1 ms
CONV_1_8MS      = 2   # 1.8 ms
CONV_3_4MS      = 3   # 3.4 ms
CONV_6_5MS      = 4   # 6.5 ms
CONV_12_7MS     = 5   # 12.7 ms
CONV_25MS       = 6   # 25 ms
CONV_50MS       = 7   # 50 ms
CONV_100MS      = 8   # 100 ms
CONV_200MS      = 9   # 200 ms
CONV_400MS      = 10  # 400 ms
CONV_800MS      = 11  # 800 ms

# Operating mode constants
MODE_POWERDOWN  = 0
MODE_AUTO_ONE   = 1   # Auto-range one-shot
MODE_ONE_SHOT   = 2
MODE_CONTINUOUS = 3

# Interrupt polarity constants
INT_POL_ACTIVE_LOW = 0
INT_POL_ACTIVE_HIGH = 1


# ── STATUS register (0x0C) flags ─────────────────────────────────────────────
_FLAG_LOW       = 0x01  # Measurement < low threshold
_FLAG_HIGH      = 0x02  # Measurement > high threshold
_FLAG_READY     = 0x04  # Conversion ready
_FLAG_OVERLOAD  = 0x08  # ADC overflow

# ── THRESH_CFG register (0x0B) bit layout ────────────────────────────────────
# Bits 6-5 : threshold channel select (0-3)
# Bit 4    : interrupt direction (1 = high threshold active)
_INT_DIR_OUTPUT = 1
_INT_DIR_INPUT  = 0
# Bits 3-2 : interrupt config (0=SMBUS, 1=next-channel ready, 3=all-channels ready)
_INT_CFG_ALL_READY = 3  # Interrupt on all channels ready
_INT_CFG_DISABLED  = 0  # SMBUS alert (effectively disabled for polling)


class OPT4048(SensorBase):
    I2C_ADDR = 0x44
    NAME = "OPT4048"
    READ_INTERVAL_MS = 10
    TYPE = "Colour"

    def __init__(self):
        super().__init__()
        self._overload = False

    # ── Configuration helpers (public API) ───────────────────────────────────

    def set_range(self, rng: int):
        """Set the measurement range (use RANGE_* constants or RANGE_AUTO)."""
        cfg = self._read_u16_be(_REG_CONFIG)
        cfg = (cfg & ~(0x0F << 10)) | ((rng & 0x0F) << 10)
        self._write_u16_be(_REG_CONFIG, cfg)

    def get_range(self) -> int:
        """Return the current range setting."""
        return (self._read_u16_be(_REG_CONFIG) >> 10) & 0x0F

    def set_conversion_time(self, ct: int):
        """Set the per-channel conversion time (use CONV_* constants)."""
        cfg = self._read_u16_be(_REG_CONFIG)
        cfg = (cfg & ~(0x0F << 6)) | ((ct & 0x0F) << 6)
        self._write_u16_be(_REG_CONFIG, cfg)

    def get_conversion_time(self) -> int:
        """Return the current conversion-time setting."""
        return (self._read_u16_be(_REG_CONFIG) >> 6) & 0x0F

    def set_mode(self, mode: int):
        """Set the operating mode (use MODE_* constants)."""
        cfg = self._read_u16_be(_REG_CONFIG)
        cfg = (cfg & ~(0x03 << 4)) | ((mode & 0x03) << 4)
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
        tcfg = self._read_u16_be(_REG_THRESH_CFG)
        if enabled:
            tcfg = (tcfg & ~(0x03 << 2)) | (_INT_CFG_ALL_READY << 2)
        else:
            tcfg = (tcfg & ~(0x03 << 2)) | (_INT_CFG_DISABLED << 2)
        self._write_u16_be(_REG_THRESH_CFG, tcfg)

    def get_interrupt_enabled(self) -> bool:
        """Return True if the conversion-ready interrupt is enabled."""
        return ((self._read_u16_be(_REG_THRESH_CFG) >> 2) & 0x03) == _INT_CFG_ALL_READY


    def set_latched_interrupt(self, enabled: bool, threshold_ch: int = 3, threshold_low: int = 0x0000, threshold_high: int = 0xFFFF):
        """Enable or disable threshold Latched interrupt."""
        if enabled:
            # Setup Threshold
            self._write_u16_be(_REG_THRESH_LO, threshold_low)   # low threshold
            self._write_u16_be(_REG_THRESH_HI, threshold_high)  # high threshold        

        cfg = self._read_u16_be(_REG_CONFIG)
        if enabled:
            cfg |= 1 << 3       # INT_LATCH = 1 (latch interrupt until cleared by reading status)
        else:
            cfg &= ~(1 << 3)    # INT_LATCH = 0 (non-latched interrupt)
        self._write_u16_be(_REG_CONFIG, cfg)

        tcfg = self._read_u16_be(_REG_THRESH_CFG)
        if enabled:
            tcfg = (tcfg & 0x8001) | (threshold_ch << 5) | (_INT_DIR_OUTPUT << 4) | (_INT_CFG_DISABLED << 2)
            # 15-7: 0x80
            # 6-5 THRESHOLD_CH_SEL: 3 = W channel (Clear)
            # 4 INT_DIR: Out = 1
            # 3-2 INT_CFG: SMBUS ALert = 0
        else:
            # Make Int Pin an input
            tcfg = tcfg & ~(1 << 4)
        self._write_u16_be(_REG_THRESH_CFG, tcfg)


    # ── SensorBase interface ─────────────────────────────────────────────────

    def _init(self) -> bool:
        dev_id = self._read_u16_be(_REG_DEVICE_ID)
        if dev_id != _DEVICE_ID_EXPECT:
            print(f"S:OPT4048 ID 0x{dev_id:04X} (expected 0x{_DEVICE_ID_EXPECT:04X}) - rejecting")
            return False

        # Configure for fast continuous reads within ~10 ms budget:
        #   Range       : auto (best dynamic range)
        #   Conv time   : 1.8 ms per channel → 4 × 1.8 ms ≈ 7.2 ms total
        #   Mode        : continuous
        #   INT latch   : latched (bit 3 = 1)
        #   INT polarity: active-low (bit 2 = 0)
        #   Fault count : 1 (bits 1:0 = 0)
        cfg = (RANGE_AUTO << 10) | (CONV_1_8MS << 6) | (MODE_CONTINUOUS << 4) | 0x08
        self._write_u16_be(_REG_CONFIG, cfg)

        # Enable conversion-ready interrupt so status polling works
        #self.set_interrupt_enabled(True)
        # The conversion ready interrupt is only 1us in duration which is too short for the LS pin to 
        # reliably capture, so we enable latching mode and poll the status register for the ready flag instead.
        self.set_latched_interrupt(True, threshold_low = 0x8400, threshold_high = 0x8400)

        #r = self._read_u16_be(_REG_THRESH_LO)
        #print(f"thresh[8]: 0x{r:04X}")
        #r = self._read_u16_be(_REG_THRESH_HI)
        #print(f"thresh[9]: 0x{r:04X}")
        #r = self._read_u16_be(_REG_CONFIG)
        #print(f"config[A]: 0x{r:04X}")
        #r = self._read_u16_be(_REG_THRESH_CFG)
        #print(f"thresh[B]: 0x{r:04X}")
        #r = self._read_u16_be(_REG_STATUS)
        #print(f"status[C]: 0x{r:04X}")

        return True

    def _measure(self) -> dict:
        # Poll status for conversion-ready; timeout after 30 ms
        deadline = time.ticks_add(time.ticks_ms(), self.READ_INTERVAL_MS)
        while True:
            st = self._read_u16_be(_REG_STATUS)
            print(f"OPT4048 status: 0x{st:04X}")
            if st & _FLAG_READY:
                self._overload = bool(st & _FLAG_OVERLOAD)
                break
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return {"Error": "timeout"}
            time.sleep_ms(1)

        # Burst-read all 4 channels (8 registers × 2 bytes = 16 bytes)
        raw = self._i2c.readfrom_mem(self.I2C_ADDR, _REG_CH0_MSB, 16)

        x = self._decode_channel(raw, 0)
        y = self._decode_channel(raw, 4)
        z = self._decode_channel(raw, 8)
        w = self._decode_channel(raw, 12)
        return {
            "x": str(x),
            "y": str(y),
            "z": str(z),
            "w": str(w),
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

        exp = (msb_hi >> 4) & 0x0F
        mantissa = ((msb_hi & 0x0F) << 16) | (msb_lo << 8) | lsb_hi
        return mantissa << exp

    def _shutdown(self):
        self.set_mode(MODE_POWERDOWN)
