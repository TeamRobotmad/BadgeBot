"""Register-level tests for the VL53L0X distance sensor driver."""

# pylint: disable=protected-access,redefined-outer-name,unused-import

import sys

import pytest

sys.path.append("../../../")

import sim.run


class FakeI2C:
    def __init__(self):
        self._mem = {}
        self._queued_reads = {}
        self.write_log = []

    def set_reg8(self, addr, reg, value):
        self._mem[(addr, reg)] = value & 0xFF

    def set_reg16(self, addr, reg, value):
        self.set_reg8(addr, reg, value >> 8)
        self.set_reg8(addr, reg + 1, value)

    def set_block(self, addr, reg, data):
        for offset, value in enumerate(data):
            self.set_reg8(addr, reg + offset, value)

    def queue_reads(self, addr, reg, values):
        self._queued_reads.setdefault((addr, reg), []).extend(values)

    def readfrom_mem(self, addr, reg, nbytes):
        result = bytearray()
        for offset in range(nbytes):
            key = (addr, reg + offset)
            queued = self._queued_reads.get(key)
            if queued:
                result.append(queued.pop(0))
            else:
                result.append(self._mem.get(key, 0x00))
        return bytes(result)

    def writeto_mem(self, addr, reg, data):
        payload = bytes(data)
        self.write_log.append((addr, reg, payload))
        for offset, value in enumerate(payload):
            self._mem[(addr, reg + offset)] = value


@pytest.fixture
def vl53l0x_module():
    import sim.apps.BadgeBot.sensors.vl53l0x as mod
    return mod


def _make_sensor_environment(mod):
    i2c = FakeI2C()
    addr = mod.VL53L0X.I2C_ADDR

    i2c.set_reg8(addr, mod._WHO_AM_I_REG, mod._WHO_AM_I_EXPECT)
    i2c.set_reg8(addr, mod._VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 0x00)
    i2c.set_reg8(addr, mod._MSRC_CONFIG_CONTROL, 0x00)
    i2c.set_reg8(addr, mod._GPIO_HV_MUX_ACTIVE_HIGH, 0x10)
    i2c.set_reg8(addr, mod._STOP_VARIABLE_REG, 0xAB)
    i2c.set_reg8(addr, mod._SPAD_INFO_REG, 0x8F)
    i2c.set_block(addr, mod._GLOBAL_CONFIG_SPAD_ENABLES_REF_0, [0xFF] * 6)
    i2c.set_reg16(addr, mod._RESULT_RANGE_STATUS + 10, 345)

    i2c.queue_reads(addr, mod._SPAD_POLL_REG, [0x01, 0x01])
    i2c.queue_reads(addr, mod._RESULT_INTERRUPT_STATUS, [0x01, 0x01, 0x01])
    i2c.queue_reads(addr, mod._SYSRANGE_START, [0x00])

    sensor = mod.VL53L0X()
    return sensor, i2c


def test_begin_captures_stop_variable_and_finishes_calibration(vl53l0x_module):
    sensor, i2c = _make_sensor_environment(vl53l0x_module)

    assert sensor.begin(i2c) is True
    assert sensor._stop_variable == 0xAB
    assert i2c.readfrom_mem(sensor.i2c_addr, vl53l0x_module._SYSTEM_SEQUENCE_CONFIG, 1) == bytes([0xE8])


def test_read_restores_stop_variable_and_returns_range(vl53l0x_module):
    sensor, i2c = _make_sensor_environment(vl53l0x_module)

    assert sensor.begin(i2c) is True
    assert sensor.read() == {"dist_mm": "345"}
    assert any(
        reg == vl53l0x_module._STOP_VARIABLE_REG and payload == b"\xAB"
        for _, reg, payload in i2c.write_log
    )


def test_read_times_out_when_interrupt_never_asserts(monkeypatch, vl53l0x_module):
    sensor, i2c = _make_sensor_environment(vl53l0x_module)

    assert sensor.begin(i2c) is True
    i2c._queued_reads[(sensor.i2c_addr, vl53l0x_module._SYSRANGE_START)] = [0x00]
    i2c._queued_reads[(sensor.i2c_addr, vl53l0x_module._RESULT_INTERRUPT_STATUS)] = [0x00] * 8
    monkeypatch.setattr(vl53l0x_module, "_RANGE_TIMEOUT_MS", 1)
    assert sensor.read() == {"dist_mm": "timeout"}
