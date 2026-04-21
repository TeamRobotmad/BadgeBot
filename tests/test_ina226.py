try:
    from sim.apps.BadgeBot.sensors.ina226 import (
        INA226,
        _MASK_CVRF,
        _REG_BUS_VOLTAGE,
        _REG_CALIBRATION,
        _REG_CONFIGURATION,
        _REG_CURRENT,
        _REG_MANUFACTURER_ID,
        _REG_MASK_ENABLE,
        _REG_POWER,
        _DEFAULT_CONFIGURATION,
    )
except ModuleNotFoundError:
    from sensors.ina226 import (
        INA226,
        _MASK_CVRF,
        _REG_BUS_VOLTAGE,
        _REG_CALIBRATION,
        _REG_CONFIGURATION,
        _REG_CURRENT,
        _REG_MANUFACTURER_ID,
        _REG_MASK_ENABLE,
        _REG_POWER,
        _DEFAULT_CONFIGURATION,
    )


def _u16_be(value: int) -> bytes:
    return bytes([(value >> 8) & 0xFF, value & 0xFF])


class _FakeI2C:
    def __init__(self):
        self.reads = {}
        self.writes = []

    def readfrom_mem(self, addr, reg, n):
        value = self.reads[(addr, reg)]
        return value[:n]

    def writeto_mem(self, addr, reg, data):
        self.writes.append((addr, reg, bytes(data)))


def test_ina226_supports_alternative_i2c_addresses():
    assert INA226.I2C_ADDR == 0x40
    assert INA226.I2C_ADDRS[0] == 0x40
    assert INA226.I2C_ADDRS[-1] == 0x4F
    assert len(INA226.I2C_ADDRS) == 16


def test_ina226_init_and_measure_integer_units():
    fake_i2c = _FakeI2C()
    sensor = INA226(i2c_addr=0x45)
    fake_i2c.reads[(0x45, _REG_MANUFACTURER_ID)] = _u16_be(0x5449)
    assert sensor.begin(fake_i2c) is True

    assert (0x45, _REG_CONFIGURATION, _u16_be(_DEFAULT_CONFIGURATION)) in fake_i2c.writes
    assert (0x45, _REG_CALIBRATION, _u16_be(0x0200)) in fake_i2c.writes

    fake_i2c.reads[(0x45, _REG_MASK_ENABLE)] = _u16_be(_MASK_CVRF)
    fake_i2c.reads[(0x45, _REG_BUS_VOLTAGE)] = _u16_be(4000)
    fake_i2c.reads[(0x45, _REG_CURRENT)] = _u16_be(1234)
    fake_i2c.reads[(0x45, _REG_POWER)] = _u16_be(320)

    result = sensor.read()
    assert result["bus_mV"] == "5000"
    assert result["current_mA"] == "123"
    assert result["power_mW"] == "800"


def test_ina226_read_sample_if_ready_none_when_not_ready():
    fake_i2c = _FakeI2C()
    sensor = INA226(i2c_addr=0x40)
    fake_i2c.reads[(0x40, _REG_MANUFACTURER_ID)] = _u16_be(0x5449)
    fake_i2c.reads[(0x40, _REG_MASK_ENABLE)] = _u16_be(0x0000)
    assert sensor.begin(fake_i2c) is True
    assert sensor.read_sample_if_ready() is None
