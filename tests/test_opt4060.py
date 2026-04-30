"""Tests for the OPT4060 tristimulus XYZ colour sensor driver.

These tests mock the I2C bus to validate register-level behaviour without
real hardware.  They run inside the badge simulator environment set up by
conftest.py.
"""

import sys

import pytest

# Path setup — same as other test files
sys.path.append("../../../")


def _ensure_sim():
    """Lazy simulator init (see conftest.py for rationale)."""
    from conftest import _ensure_sim_initialized
    _ensure_sim_initialized()


# ---------------------------------------------------------------------------
#  Fake I2C bus for unit-testing register reads/writes
# ---------------------------------------------------------------------------

class FakeI2C:
    """Minimal I2C stub that stores 16-bit big-endian registers keyed by
    (addr, reg) and supports readfrom_mem / writeto_mem with arbitrary
    byte counts.
    """

    def __init__(self):
        self._mem = {}  # (addr, reg) -> list[int]  (2-byte chunks)

    def set_reg16(self, addr, reg, value):
        """Pre-load a 16-bit register for subsequent reads."""
        self._mem[(addr, reg)] = [(value >> 8) & 0xFF, value & 0xFF]

    def readfrom_mem(self, addr, reg, nbytes):
        """Read *nbytes* starting from *reg*, spanning consecutive registers."""
        result = bytearray()
        remaining = nbytes
        cur_reg = reg
        while remaining > 0:
            key = (addr, cur_reg)
            if key in self._mem:
                chunk = self._mem[key]
            else:
                chunk = [0x00, 0x00]
            take = min(remaining, len(chunk))
            result.extend(chunk[:take])
            remaining -= take
            cur_reg += 1
        return bytes(result)

    def writeto_mem(self, addr, reg, data):
        """Write *data* bytes into 16-bit register(s) starting at *reg*."""
        pos = 0
        cur_reg = reg
        while pos < len(data):
            chunk_len = min(2, len(data) - pos)
            if chunk_len == 2:
                self._mem[(addr, cur_reg)] = [data[pos], data[pos + 1]]
            else:
                self._mem[(addr, cur_reg)] = [data[pos], 0x00]
            pos += chunk_len
            cur_reg += 1


# ---------------------------------------------------------------------------
#  Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def OPT4060_module():
    """Import and return the OPT4060 module (after sim init)."""
    _ensure_sim()
    import sim.apps.BadgeBot.sensors.OPT4060 as mod
    return mod


@pytest.fixture
def sensor(OPT4060_module):
    """Return an OPT4060 sensor instance wired to a FakeI2C bus that
    has the correct device-ID pre-loaded so begin() succeeds.
    """
    i2c = FakeI2C()
    mod = OPT4060_module
    # Pre-load device ID register with expected value
    i2c.set_reg16(mod.OPT4060.I2C_ADDR, 0x11, 0x0821)
    # Pre-load status as conversion-ready so _measure() doesn't time out
    i2c.set_reg16(mod.OPT4060.I2C_ADDR, 0x0C, 0x0004)  # _FLAG_READY

    s = mod.OPT4060()
    assert s.begin(i2c) is True
    return s


@pytest.fixture
def fake_i2c():
    return FakeI2C()


# ---------------------------------------------------------------------------
#  Import & interface tests
# ---------------------------------------------------------------------------

def test_import_OPT4060(OPT4060_module):
    """Module can be imported and has the expected class."""
    assert hasattr(OPT4060_module, 'OPT4060')


def test_class_attributes(OPT4060_module):
    """Verify class-level constants match the datasheet."""
    cls = OPT4060_module.OPT4060
    assert cls.I2C_ADDR == 0x44
    assert cls.NAME == "OPT4060"
    # check that class constant is between 10 and 100ms, to catch any accidental typos
    assert 10 <= cls.READ_INTERVAL_MS <= 100
    assert cls.TYPE == "Colour"


def test_sensor_base_interface(OPT4060_module):
    """OPT4060 implements the full SensorBase public API."""
    s = OPT4060_module.OPT4060()
    for attr in ('begin', 'read', 'reset', 'is_ready'):
        assert hasattr(s, attr)
    assert s.is_ready is False


# ---------------------------------------------------------------------------
#  Initialisation tests
# ---------------------------------------------------------------------------

def test_begin_sets_continuous_mode(OPT4060_module, fake_i2c):
    """After begin(), the config register should reflect continuous mode,
    auto-range, and 1.8 ms conversion time."""
    mod = OPT4060_module
    fake_i2c.set_reg16(mod.OPT4060.I2C_ADDR, 0x11, 0x0821)
    fake_i2c.set_reg16(mod.OPT4060.I2C_ADDR, 0x0C, 0x0004)

    s = mod.OPT4060()
    assert s.begin(fake_i2c)

    cfg_bytes = fake_i2c.readfrom_mem(mod.OPT4060.I2C_ADDR, 0x0A, 2)
    cfg = (cfg_bytes[0] << 8) | cfg_bytes[1]

    # Range = AUTO (12) in bits 13:10
    assert (cfg >> 10) & 0x0F == mod.RANGE_AUTO
    # Conversion time = CONV_1_8MS (2) in bits 9:6
    assert (cfg >> 6) & 0x0F == mod.CONV_1_8MS
    # Mode = continuous (3) in bits 5:4
    assert (cfg >> 4) & 0x03 == mod.MODE_CONTINUOUS


def test_begin_wrong_id_fails(OPT4060_module, fake_i2c):
    """begin() should reject an unexpected device ID."""
    mod = OPT4060_module
    fake_i2c.set_reg16(mod.OPT4060.I2C_ADDR, 0x11, 0xFFFF)  # wrong ID
    fake_i2c.set_reg16(mod.OPT4060.I2C_ADDR, 0x0C, 0x0004)

    s = mod.OPT4060()
    assert s.begin(fake_i2c) is False


# ---------------------------------------------------------------------------
#  Channel decode tests
# ---------------------------------------------------------------------------

def test_decode_channel_zero(OPT4060_module):
    """Zero mantissa and exponent should produce 0."""
    result = OPT4060_module.OPT4060._decode_channel(bytes([0, 0, 0, 0]), 0)
    assert result == 0


def test_decode_channel_known_value(OPT4060_module):
    """Verify decoding against a manually-calculated example.

    MSB register: exponent=3 (0x3), mantissa_hi=0x123  → byte0=0x31, byte1=0x23
    LSB register: mantissa_lo=0x45                      → byte2=0x45, byte3=0x00
    mantissa_20bit = 0x12345
    ADC code = 0x12345 << 3 = 0x91A28 = 596520
    """
    buf = bytes([0x31, 0x23, 0x45, 0x00])
    result = OPT4060_module.OPT4060._decode_channel(buf, 0)
    assert result == 0x12345 << 3


def test_decode_channel_max_exponent(OPT4060_module):
    """Maximum exponent (15) should shift mantissa left by 15."""
    # exponent=15 (0xF), mantissa_hi=0x000, mantissa_lo=0x01  → mantissa=1
    buf = bytes([0xF0, 0x00, 0x01, 0x00])
    result = OPT4060_module.OPT4060._decode_channel(buf, 0)
    assert result == 1 << 15


def test_decode_channel_with_offset(OPT4060_module):
    """Decoding from a non-zero offset within the buffer should work."""
    # 4 bytes padding + channel data
    padding = bytes([0xFF, 0xFF, 0xFF, 0xFF])
    ch_data = bytes([0x10, 0x00, 0x80, 0x00])  # exp=1, mant_hi=0, mant_lo=0x80
    buf = padding + ch_data
    result = OPT4060_module.OPT4060._decode_channel(buf, 4)
    # mantissa = (0 << 16) | (0 << 8) | 0x80 = 128; code = 128 << 1 = 256
    assert result == 128 << 1


# ---------------------------------------------------------------------------
#  Measurement tests
# ---------------------------------------------------------------------------

def test_measure_returns_xyz(sensor, OPT4060_module):
    """_measure() should return a dict with x, y, z string values."""
    # Set up channel data: all channels have mantissa=100, exponent=0
    i2c = sensor._i2c
    addr = OPT4060_module.OPT4060.I2C_ADDR

    for ch_reg in (0x00, 0x02, 0x04, 0x06):
        # MSB: exp=0, mantissa_hi=0x000
        i2c.set_reg16(addr, ch_reg, 0x0000)
        # LSB: mantissa_lo=100 (0x64), counter=0, crc=0
        i2c.set_reg16(addr, ch_reg + 1, 0x6400)

    result = sensor.read()
    assert 'x' in result
    assert 'y' in result
    assert 'z' in result
    # All channels should decode to mantissa 100 << 0 = 100
    assert result['x'] == '100'
    assert result['y'] == '100'
    assert result['z'] == '100'


def test_measure_timeout(OPT4060_module, fake_i2c):
    """_measure() returns an error dict when status never shows ready."""
    mod = OPT4060_module
    fake_i2c.set_reg16(mod.OPT4060.I2C_ADDR, 0x11, 0x0821)
    # Status: NOT ready (no _FLAG_READY bit set)
    fake_i2c.set_reg16(mod.OPT4060.I2C_ADDR, 0x0C, 0x0000)

    s = mod.OPT4060()
    assert s.begin(fake_i2c)

    result = s.read()
    assert 'Error' in result
    assert 'timeout' in result['Error']


# ---------------------------------------------------------------------------
#  Configuration API tests
# ---------------------------------------------------------------------------

def test_set_get_range(sensor, OPT4060_module):
    """set_range / get_range round-trip."""
    for rng in (OPT4060_module.RANGE_2K, OPT4060_module.RANGE_72K, OPT4060_module.RANGE_AUTO):
        sensor.set_range(rng)
        assert sensor.get_range() == rng


def test_set_get_conversion_time(sensor, OPT4060_module):
    """set_conversion_time / get_conversion_time round-trip."""
    for ct in (OPT4060_module.CONV_600US, OPT4060_module.CONV_1_8MS, OPT4060_module.CONV_800MS):
        sensor.set_conversion_time(ct)
        assert sensor.get_conversion_time() == ct


def test_set_get_mode(sensor, OPT4060_module):
    """set_mode / get_mode round-trip."""
    for mode in (OPT4060_module.MODE_POWERDOWN, OPT4060_module.MODE_CONTINUOUS):
        sensor.set_mode(mode)
        assert sensor.get_mode() == mode


def test_set_get_interrupt(sensor):
    """set_interrupt_enabled / get_interrupt_enabled round-trip."""
    sensor.set_interrupt_enabled(False)
    assert sensor.get_interrupt_enabled() is False
    sensor.set_interrupt_enabled(True)
    assert sensor.get_interrupt_enabled() is True


# ---------------------------------------------------------------------------
#  Shutdown test
# ---------------------------------------------------------------------------

def test_shutdown_powers_down(sensor, OPT4060_module):
    """reset() should set the sensor to power-down mode."""
    sensor.reset()
    assert sensor.get_mode() == OPT4060_module.MODE_POWERDOWN
    assert sensor.is_ready is False


# ---------------------------------------------------------------------------
#  Module-level constant tests
# ---------------------------------------------------------------------------

def test_range_constants(OPT4060_module):
    """Range constants should cover the datasheet values."""
    mod = OPT4060_module
    assert mod.RANGE_2K == 0
    assert mod.RANGE_144K == 6
    assert mod.RANGE_AUTO == 12


def test_conv_time_constants(OPT4060_module):
    """Conversion time constants should span 600 µs to 800 ms."""
    mod = OPT4060_module
    assert mod.CONV_600US == 0
    assert mod.CONV_800MS == 11


def test_mode_constants(OPT4060_module):
    """Operating mode constants should match datasheet."""
    mod = OPT4060_module
    assert mod.MODE_POWERDOWN == 0
    assert mod.MODE_CONTINUOUS == 3
