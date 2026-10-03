import re
from math import pi, radians
from pathlib import Path

import pytest
from system.hexpansion.config import HexpansionConfig


def _extract_version_from_source(path: Path) -> int:
    content = path.read_text(encoding="utf-8")
    match = re.search(r"^\s*VERSION\s*=\s*(\d+)", content, re.MULTILINE)
    assert match is not None, f"Could not find VERSION in {path}"
    return int(match.group(1))


def test_parse_version_handles_dev_and_git_suffixes():
    from sim.apps.BadgeBot.utils import parse_version

    assert parse_version("v2.2.0") == (2, 2, 0)
    assert parse_version("v2.2.0-dev") == (2, 2, 0)
    assert parse_version("v2.2.0-20-gabcdef") == (2, 2, 0)
    assert parse_version("v2.2.1-rc1") == (2, 2, 1)
    assert parse_version("v2.2.1-rc1+build.7") == (2, 2, 1)


def test_parse_version_works_without_re_findall(monkeypatch):
    import sim.apps.BadgeBot.utils as badge_utils

    class NoFindallModule:
        pass

    monkeypatch.setattr(badge_utils, "re", NoFindallModule())

    assert badge_utils.parse_version("v2.3.4-rc1+build.7") == (2, 3, 4)


def test_ctx_fake_imports_without_wasmtime(monkeypatch):
    import builtins
    import importlib
    import sys

    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "wasmtime":
            raise ModuleNotFoundError("No module named 'wasmtime'")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    sys.modules.pop("sim.fakes.ctx", None)

    mod = importlib.import_module("sim.fakes.ctx")
    assert mod.wasmtime is None
    assert mod._wasm is None
    with pytest.raises(RuntimeError, match="wasmtime"):
        mod._require_wasm()


def test_import_badgebot_app_and_app_export():
    import sim.apps.BadgeBot.app as BadgeBot
    from sim.apps.BadgeBot import BadgeBotApp
    assert BadgeBot.__app_export__ == BadgeBotApp


def test_button_state_get_preserves_direct_and_parent_matching():
    from events.input import Button, Buttons

    parent = Button("DIRECTION", "System")
    up = Button("UP", "System", parent)
    equal_up = Button("UP", "System")
    button_states = Buttons.__new__(Buttons)
    button_states.buttons = {up: False}
    button_states._already_pressed = set()

    assert hash(up) == hash(equal_up)
    assert button_states.get(equal_up) is False
    assert button_states.get(parent) is False

    button_states.buttons[up] = True
    assert button_states.get(equal_up) is True
    assert button_states.get(parent) is True

def test_import_hexdrive_app_and_app_export():
    import sim.apps.BadgeBot.vendor.HexDrive.hexdrive as HexDrive
    from sim.apps.BadgeBot.vendor.HexDrive.hexdrive import HexDriveApp
    assert HexDrive.__app_export__ == HexDriveApp

def test_hexdrive_instance_exposes_version():
    from sim.apps.BadgeBot.vendor.HexDrive.hexdrive import HexDriveApp
    app_instance = HexDriveApp(HexpansionConfig(1))
    assert getattr(app_instance, "VERSION", None) == HexDriveApp.VERSION

def test_badgebot_app_init():
    from sim.apps.BadgeBot import BadgeBotApp
    BadgeBotApp()


def test_motor_controller_send_output_uses_calibration_buffer():
    from sim.apps.BadgeBot.motor_controller import MotorController

    class HexDrive:
        def set_motors(self, outputs):
            self.outputs = outputs

        def set_power(self, enabled):
            self.power_enabled = enabled

    hexdrive = HexDrive()
    controller = MotorController.__new__(MotorController)
    controller._hexdrive = hexdrive
    controller.motor_output = (0, 0)
    controller._calibrated_output_buffer = [0, 0]
    controller._logging = False
    controller._busy = True

    def calibrate(output, output_buffer):
        output_buffer[0] = output[0]
        output_buffer[1] = output[1]

    controller._apply_motor_directions_callback = calibrate
    controller.stop()

    assert hexdrive.outputs is controller._calibrated_output_buffer
    assert hexdrive.outputs == [0, 0]
    assert hexdrive.power_enabled is False
    assert controller._busy is False


def test_line_follow_calibration_reminder_is_shown_once():
    from types import SimpleNamespace
    from sim.apps.BadgeBot.line_follow import LineFollowMgr, STATE_FOLLOWER

    messages = []
    app = SimpleNamespace(
        show_message=lambda *args, **kwargs: messages.append((args, kwargs)),
        button_states={},
    )
    manager = LineFollowMgr(app, logging=False)

    assert manager.update(1) is True
    assert manager.update(1) is True
    assert len(messages) == 1
    assert messages[0][0][0][0] == "Line Follower:"
    assert messages[0][1] == {"return_state": STATE_FOLLOWER, "timeout": 4000}


@pytest.mark.parametrize("output", [0, 1, 127, 512, 32768, 55000, 65535])
@pytest.mark.parametrize("motor_min", [0, 1, 512, 32768, 65535])
def test_motor_output_scaling_matches_16_bit_integer_formula(output, motor_min):
    from sim.apps.BadgeBot.app import _scale_motor_output

    assert _scale_motor_output(output, motor_min) == output * (65536 - motor_min) // 65536


def test_sensor_stats_counts_missed_samples_across_small_int_sequence_wrap():
    from sim.apps.BadgeBot.sensor_test import SensorStats, _SEQUENCE_MASK

    stats = SensorStats("colour")
    stats.new_sample(_SEQUENCE_MASK - 1)
    stats.new_sample(_SEQUENCE_MASK)
    stats.new_sample(0)
    assert stats.missed == 0

    stats.new_sample(2)
    assert stats.missed == 1


def test_sensor_stats_caches_rate_string_until_rate_changes():
    from sim.apps.BadgeBot.sensor_test import SensorStats

    stats = SensorStats("colour", sample_period_ms=1000)
    initial_rate_str = stats.rate_str
    assert stats.rate_str is initial_rate_str

    for _ in range(12):
        stats.new_sample()
    assert stats.update(1000) is True
    twelve_hz = stats.rate_str
    assert twelve_hz == "12.0Hz"

    for _ in range(12):
        stats.new_sample()
    assert stats.update(1000) is True
    assert stats.rate_str is twelve_hz

    for _ in range(13):
        stats.new_sample()
    assert stats.update(1000) is True
    assert stats.rate_str == "13.0Hz"
    assert stats.rate_str is not twelve_hz

    stats.reset()
    assert stats.rate_str == "0.0Hz"
    assert stats.rate_str is initial_rate_str


@pytest.mark.parametrize("error", [-1800, -90, 0, 90, 1800])
def test_line_follow_differential_output_preserves_buffer_and_limits(error):
    from types import SimpleNamespace
    from sim.apps.BadgeBot.line_follow import LineFollowMgr

    manager = LineFollowMgr(SimpleNamespace(max_power=70), logging=False)
    result = [0, 0]
    correction = manager._steering_correction(error, 50)
    expected = (
        max(-70, min(70, manager.line_power + correction)),
        max(-70, min(70, manager.line_power - correction)),
    )
    manager.clear_pid()
    assert manager.compute_differential_output(error, 50, result) is result
    assert tuple(result) == expected
    manager.clear_pid()
    assert manager.compute_differential_output(error, 50) == expected


def test_sensor_polling_reuses_buffers_and_contains_errors():
    from types import SimpleNamespace
    from sim.apps.BadgeBot.sensor_test import SensorTestMgr

    ring = []
    manager = SensorTestMgr(SimpleNamespace(set_ring_colour=lambda *rgb: ring.append(rgb)))
    range_sensor = SimpleNamespace(sequence=1, range=150)
    range_hexdrive = SimpleNamespace(range_sensor=range_sensor)
    range_result = [False, None]
    assert manager.read_range(range_hexdrive, range_result) is range_result
    assert range_result == [True, 150]
    manager.read_range(range_hexdrive, range_result)
    assert range_result == [False, 150]
    manager.read_range(None, range_result)
    assert range_result == [False, None]

    def colour_into(destination):
        destination[:] = [100, 20, 10, 100]
        return destination

    colour_sensor = SimpleNamespace(sequence=1, colour_into=colour_into,
                                    colour_hue=30, colour_saturation=90, colour_name="Red")
    colour_hexdrive = SimpleNamespace(colour_sensor=colour_sensor)
    colour_result = [False, 0, 0, "unknown", None]
    assert manager.read_colour(colour_hexdrive, True, colour_result) is colour_result
    assert colour_result[0] is True
    assert colour_result[4] is manager._colour_raw_buffer
    assert len(ring) == 1
    manager.read_colour(colour_hexdrive, True, colour_result)
    assert colour_result[0] is False
    assert len(ring) == 1

    class BrokenSensor:
        @property
        def sequence(self):
            raise OSError("sensor disconnected")

    manager.read_range(SimpleNamespace(range_sensor=BrokenSensor()), range_result)
    assert range_result == [False, None]
    manager.read_colour(SimpleNamespace(colour_sensor=BrokenSensor()), True, colour_result)
    assert colour_result[0] is False


def test_line_follow_obstacle_stop_preserves_reusable_output():
    from types import SimpleNamespace
    from sim.apps.BadgeBot.line_follow import LineFollowMgr

    def read_range(_hexdrive, result):
        result[0] = True
        result[1] = 50
        return result

    app = SimpleNamespace(sensor_test_mgr=SimpleNamespace(read_range=read_range),
                          performance_mode=False, notification=None)
    manager = LineFollowMgr(app, logging=False)
    manager._colour_hexdrive = object()
    manager._range_hexdrive = SimpleNamespace(config=SimpleNamespace(port=4))
    manager._enable_movement = True
    for _ in range(3):
        assert manager.background_update(50) is manager._output_buffer
        assert manager._output_buffer == [0, 0]
    assert manager._enable_movement is False
    assert app.notification is not None


@pytest.mark.parametrize("source, targets", [
    ("app.py", "update background_update _update_main_application _update_state_transition _update_state_leds _scale_state_leds _scale_state_led _write_state_leds _log_state_led_error _send_motor_output apply_motor_calibration draw _draw_state_ring"),
    ("line_follow.py", "update background_update _obstacle_detected _follow_colour compute_differential_output _steering_correction _differential_output draw draw_tracker _draw_tracker_box _draw_selected_field _draw_tracker_bands _draw_tracker_band_range _draw_tracker_band _tracker_band_hue _draw_tracker_band_line _draw_tracker_reading _draw_tracker_labels _draw_tracker_heading _draw_tracker_gains _draw_tracker_deviation _draw_idle_button_labels _draw_active_button_labels _draw_sensor_rate"),
    ("sensor_test.py", "read_range read_colour _read_range_checked _poll_range _read_colour_checked _poll_colour _count_colour_sample_checked _update_colour_ring_checked _update_colour_ring _store_range_result _store_colour_result"),
    ("vendor/HexDrive2/hexdrive2.py", "background_update _poll_range_background _poll_colour_background _update_keep_alive _stop_timed_out_outputs _stop_pwm_checked _stop_pwm set_motors _set_motor_checked _set_motor _disable_motor_channel _set_pwmoutput _write_pwm_checked _write_pwm read read_into poll _job_poll _read_values colour_into colour_name rgbw_to_str _lookup_colour_math_viper _colour_hsv_into _colour_hue _colour_id apply_white_reference _white_channel _scaled_white_value"),
    ("autodrive.py", "background_update _update_gyro _apply_gyro_sample _update_range_sensor _record_range_sample _log_waiting_for_range _update_plotter _send_plotter_data _set_plot_heading_score _set_scan_plot_heading_score _scan_plot_score _apply_output_ramp _output_ramp_step _ramp_motor_output _clamp_motor_target _update_scan _finish_scan"),
    ("../../../micropython/extmod/asyncio/core.py", "wait_io_event _process_io_event"),
    ("../../../modules/system/notification/app.py", "update _update_notification _advance_notification"),
    ("../../../modules/system/backleds/app.py", "background_update _update_back_led _back_led_colour"),
    ("../../../modules/system/scheduler/__init__.py", "_draw_app _draw_app_safely _handle_app_draw_error _notify_app_draw_crash"),
    ("../../../modules/app_components/tokens.py", "set_color _try_color_function _set_rgb_color _try_rgb_color"),
    ("../../../modules/system/espnow/service.py", "_has_listeners _registry_has_listeners _apply_power_management _sync_power_management _configure_power_management _try_configure_power_management _log_power_management_error _update_radio_awake"),
    ("../../../modules/system/a11y/printer.py", "get_deduped_strings _strings_unchanged _collect_changed_strings _collect_string_entries _should_emit_string _has_transient_strings _string_entry _last_string_text"),
    ("../../../modules/app_components/background.py", "draw _draw_runner _handle_draw_error"),
    ("../../../modules/app_components/menu.py", "draw _draw_info _ensure_focused_item_sizes _update_animation_state _draw_focused_item _draw_focused_label _draw_neighboring_items _draw_previous_items _draw_next_items"),
    ("../../../micropython/lib/micropython-lib/micropython/drivers/led/neopixel/neopixel.py", "set_many _set_many_composed _set_many_string _write_many_segment _write_many_items _ensure_batch_buffer _process_batch _set_many_pixel _correct_batch_pixel _copy_channels _write_many"),
])
def test_line_follow_hot_bytecode_states_fit_stack_cutoff(source, targets, tmp_path):
    import shutil
    import subprocess
    import sys

    compiler = shutil.which("mpy-cross")
    if compiler is None:
        pytest.skip("mpy-cross is required for bytecode frame checks")
    app_root = Path(__file__).resolve().parents[1]
    inspector = app_root.parents[2] / "micropython" / "tools" / "mpy-tool.py"
    if not inspector.exists():
        pytest.skip("MicroPython bytecode inspector is unavailable")
    artifact = tmp_path / "stack-check.mpy"
    subprocess.run([compiler, "-march=xtensawin", "-O2", "-o", str(artifact), str(app_root / source)], check=True, capture_output=True)
    dump = subprocess.run([sys.executable, "-X", "utf8", str(inspector), "-d", str(artifact)], check=True, capture_output=True, encoding="utf-8").stdout
    wanted = set(targets.split())
    measured = []
    function = None
    for line in dump.splitlines():
        if line.startswith("simple_name: "):
            function = line.split(": ", 1)[1]
        match = re.match(r"\s+prelude: \((\d+), (\d+),", line)
        if match and function in wanted:
            measured.append((function, 4 * int(match[1]) + 12 * int(match[2])))
    assert {name for name, _ in measured} == wanted
    assert all(size <= 44 for _, size in measured), measured


def test_eeprom_partition_writes_cross_pages_with_byteslike_buffers(monkeypatch):
    import importlib.util
    import sys
    from sim.apps.BadgeBot.hexpansion_mgr import _EEPROMProgrammingI2C

    root = Path(__file__).resolve().parents[4]
    loaded = {}
    for name, relative in (("bdevice", "lib/bdevice.py"),
                           ("eeprom_i2c", "lib/eeprom_i2c.py"),
                           ("eeprom_partition", "eeprom_partition.py")):
        spec = importlib.util.spec_from_file_location(name, root / "modules" / relative)
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, name, module)
        spec.loader.exec_module(module)
        loaded[name] = module
    monkeypatch.setattr(loaded["eeprom_i2c"].time, "sleep_ms", lambda delay: None, raising=False)

    class StrictI2C:
        def __init__(self):
            self.writes = []
            self.memory = bytearray(32768)

        def scan(self, addresses):
            return [0x50]

        def writeto(self, address, data):
            if not isinstance(data, (bytes, bytearray, memoryview)):
                raise TypeError("a bytes-like object is required")
            if len(data) > 1:
                offset = (data[0] << 8) | data[1]
                self.writeto_mem(address, offset, memoryview(data)[2:], 16)
            return len(data)

        def writeto_mem(self, address, offset, data, addrsize):
            self.writes.append((address, offset, bytes(data), addrsize))
            self.memory[offset:offset + len(data)] = data

        def readfrom_mem_into(self, address, offset, data, addrsize):
            data[:] = self.memory[offset:offset + len(data)]

    i2c = StrictI2C()
    eeprom = loaded["eeprom_i2c"].EEPROM(_EEPROMProgrammingI2C(i2c), chip_size=32768, page_size=64, addrsize=16, verbose=False)
    partition = loaded["eeprom_partition"].EEPROMPartition(eeprom, 64, 32704)
    payload = bytearray(range(100))
    partition.writeblocks(0, memoryview(payload), offset=60)
    assert [(offset, len(data), width) for _, offset, data, width in i2c.writes] == [
        (124, 4, 16), (128, 64, 16), (192, 32, 16),
    ]
    actual = bytearray(100)
    partition.readblocks(0, actual, offset=60)
    assert actual == payload

def test_eeprom_programming_remounts_existing_filesystem(monkeypatch):
    import sim.apps.BadgeBot.hexpansion_mgr as manager

    calls = []
    partition = object()

    def mount(device, path, readonly):
        calls.append(("mount", device, path, readonly))
        if len(calls) == 1:
            raise OSError(1)

    monkeypatch.setattr(manager.vfs, "mount", mount, raising=False)
    monkeypatch.setattr(manager.vfs, "umount", lambda path: calls.append(("umount", path)), raising=False)
    assert manager._mount_eeprom_for_programming(partition, "/hexpansion_4") is True
    assert calls == [
        ("mount", partition, "/hexpansion_4", False),
        ("umount", "/hexpansion_4"),
        ("mount", partition, "/hexpansion_4", False),
    ]

def test_eeprom_programming_identifies_new_mount(monkeypatch):
    import sim.apps.BadgeBot.hexpansion_mgr as manager

    calls = []
    partition = object()
    monkeypatch.setattr(manager.vfs, "mount", lambda *args, **kwargs: calls.append(args), raising=False)
    monkeypatch.setattr(manager.vfs, "umount", lambda path: pytest.fail("New mounts must not be remounted"), raising=False)
    assert manager._mount_eeprom_for_programming(partition, "/hexpansion_4") is False
    assert calls == [(partition, "/hexpansion_4")]


def test_neopixel_dim_correction_reuses_output_buffer(monkeypatch):
    import importlib.util
    import types

    machine_stub = types.ModuleType("machine")
    machine_stub.bitstream = lambda *args: None
    monkeypatch.setitem(__import__("sys").modules, "machine", machine_stub)

    repo_root = Path(__file__).resolve().parents[4]
    neopixel_path = (
        repo_root
        / "micropython"
        / "lib"
        / "micropython-lib"
        / "micropython"
        / "drivers"
        / "led"
        / "neopixel"
        / "neopixel.py"
    )
    spec = importlib.util.spec_from_file_location("_test_neopixel", neopixel_path)
    neopixel = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(neopixel)

    sim_stub = types.ModuleType("_sim")
    sim_stub._sim = types.SimpleNamespace(leds_update=lambda: None)
    monkeypatch.setitem(__import__("sys").modules, "_sim", sim_stub)
    monkeypatch.setitem(__import__("sys").modules, "leds", types.ModuleType("leds"))
    fake_neopixel_path = repo_root / "sim" / "fakes" / "neopixel.py"
    fake_spec = importlib.util.spec_from_file_location(
        "_test_neopixel_fake", fake_neopixel_path
    )
    fake_neopixel = importlib.util.module_from_spec(fake_spec)
    fake_spec.loader.exec_module(fake_neopixel)

    class PixelSink:
        n = 1

        def __init__(self):
            self.value = None

        def __setitem__(self, index, value):
            self.value = tuple(value)

    sink = PixelSink()
    correction = neopixel.DimCorrection(0.5)
    fake_correction = fake_neopixel.DimCorrection(0.5)
    pixels = neopixel.CorrectedNeoPixel(sink, [correction])

    source = tuple(range(256))
    result = [0] * len(source)
    for percentage in range(101):
        correction.amount = percentage / 100
        pixels._apply_into[0](correction, source, result)
        assert correction._amount_percent == percentage
        assert result == [channel * percentage // 100 for channel in source]
        assert correction(source) == result
        fake_correction.amount = percentage / 100
        assert fake_correction(source) == result

    correction.amount = 0.256
    assert correction._amount_percent == 26
    correction.amount = -0.5
    assert correction._amount_percent == 0
    correction.amount = 1.5
    assert correction._amount_percent == 100
    fake_correction.amount = 0.256
    assert fake_correction._amount_percent == 26
    fake_correction.amount = -0.5
    assert fake_correction._amount_percent == 0
    fake_correction.amount = 1.5
    assert fake_correction._amount_percent == 100
    correction.amount = 0.5
    pixels[0] = (100, 40, 3)
    buffer = pixels._correction_buffer
    assert sink.value == (50, 20, 1)

    pixels[0] = (200, 80, 5)
    assert pixels._correction_buffer is buffer
    assert sink.value == (100, 40, 2)

    first_result = correction((100, 40, 3))
    second_result = correction((200, 80, 5))
    assert first_result == [50, 20, 1]
    assert second_result == [100, 40, 2]
    assert first_result is not second_result

    raw_pixels = object.__new__(neopixel.NeoPixel)
    raw_pixels.n = 1
    raw_pixels.bpp = 3
    raw_pixels.buf = bytearray((20, 10, 30))
    raw_result = [0, 0, 0]
    raw_pixels.get_into(0, raw_result)
    assert raw_result == [10, 20, 30]

    composed_pixels = neopixel.ComposedNeoPixel(raw_pixels)
    corrected_pixels = neopixel.CorrectedNeoPixel(
        composed_pixels, [neopixel.DimCorrection(0.5)]
    )
    corrected_pixels.get_into(0, raw_result)
    assert raw_result == [5, 10, 15]

    raw_pixels.n = 2
    raw_pixels.buf = bytearray(6)
    batch_pixels = neopixel.CorrectedNeoPixel(
        neopixel.ComposedNeoPixel(raw_pixels),
        [neopixel.DimCorrection(0.5)] * 2,
    )
    frame = [(100, 40, 3), (200, 80, 5)]
    batch_pixels.set_many(0, frame, 2)
    batch_buffer = batch_pixels._batch_buffer
    assert list(raw_pixels.buf) == [20, 50, 1, 40, 100, 2]

    frame[0] = (20, 60, 8)
    batch_pixels.set_many(0, frame, 2)
    assert batch_pixels._batch_buffer is batch_buffer
    assert list(raw_pixels.buf) == [30, 10, 4, 40, 100, 2]

    offset_pixels = object.__new__(neopixel.NeoPixel)
    offset_pixels.n = 4
    offset_pixels.bpp = 3
    offset_pixels.buf = bytearray(12)
    offset_batch = neopixel.CorrectedNeoPixel(
        neopixel.ComposedNeoPixel(offset_pixels),
        [neopixel.DimCorrection(0.5)] * 4,
    )
    offset_batch.set_many(
        2, [(9, 9, 9), (100, 40, 3), (200, 80, 5)], 2, values_start=1
    )
    assert list(offset_pixels.buf) == [0, 0, 0, 0, 0, 0, 20, 50, 1, 40, 100, 2]
    assert offset_batch._batch_buffer[0] == [50, 20, 1]
    assert offset_batch._batch_buffer[1] == [100, 40, 2]

    first_strip = object.__new__(neopixel.NeoPixel)
    first_strip.n = 3
    first_strip.bpp = 3
    first_strip.buf = bytearray(9)
    second_strip = object.__new__(neopixel.NeoPixel)
    second_strip.n = 3
    second_strip.bpp = 3
    second_strip.buf = bytearray(9)
    composed_strips = neopixel.ComposedNeoPixel(first_strip, 0)
    composed_strips.add_string(second_strip, 1)
    composed_strips.set_many(0, [(1, 2, 3), (4, 5, 6), (7, 8, 9)], 3)
    assert list(first_strip.buf) == [2, 1, 3, 5, 4, 6, 8, 7, 9]
    assert list(second_strip.buf) == [5, 4, 6, 8, 7, 9, 0, 0, 0]

    merged_strip = object.__new__(neopixel.NeoPixel)
    merged_strip.n = 3
    merged_strip.bpp = 3
    merged_strip.buf = bytearray(9)
    merged_pixels = neopixel.MergedNeoPixel(merged_strip, [[0, 2], [1]])
    merged_pixels.set_many(0, [(9, 9, 9), (1, 2, 3), (4, 5, 6)], 2, values_start=1)
    assert list(merged_strip.buf) == [2, 1, 3, 5, 4, 6, 2, 1, 3]

    fake_values = [(0, 0, 0)] * 4
    monkeypatch.setattr(
        fake_neopixel.leds,
        "get_rgb",
        lambda index: fake_values[index],
        raising=False,
    )
    monkeypatch.setattr(
        fake_neopixel.leds,
        "set_rgb",
        lambda index, red, green, blue: fake_values.__setitem__(
            index, (red, green, blue)
        ),
        raising=False,
    )
    fake_strip = fake_neopixel.NeoPixel(None, 4)
    fake_composed = fake_neopixel.ComposedNeoPixel(fake_strip, -1)
    fake_corrected = fake_neopixel.CorrectedNeoPixel(
        fake_composed, [fake_neopixel.DimCorrection(0.5)] * 4
    )
    fake_corrected.set_many(0, [(100, 40, 3), (200, 80, 5)], 2)
    assert fake_values[1:3] == [(50, 20, 1), (100, 40, 2)]

    fake_result = [0, 0, 0]
    fake_values[1] = (100, 40, 3)
    fake_corrected.get_into(0, fake_result)
    assert fake_result == [50, 20, 1]

    fallback = fake_neopixel.CorrectedNeoPixel(
        fake_composed, [lambda colour: [channel + 1 for channel in colour]] * 4
    )
    fallback.get_into(0, fake_result)
    assert fake_result == [101, 41, 4]


@pytest.mark.parametrize(
    "battery_mv,input_mv,should_power_off",
    [
        (3499, 4499, True),
        (3500, 4499, False),
        (3499, 4500, False),
        (3500, 4500, False),
    ],
)
def test_power_manager_uses_millivolt_thresholds(
    monkeypatch, battery_mv, input_mv, should_power_off
):
    import asyncio
    import system.power.app as power_app

    power_off_calls = []
    monkeypatch.setattr(power_app.power, "VbatMilliVolts", lambda: battery_mv)
    monkeypatch.setattr(power_app.power, "VinMilliVolts", lambda: input_mv)
    monkeypatch.setattr(
        power_app.power, "Vbat", lambda: pytest.fail("float battery getter used")
    )
    monkeypatch.setattr(
        power_app.power, "Vin", lambda: pytest.fail("float input getter used")
    )
    monkeypatch.setattr(power_app.power, "Off", lambda: power_off_calls.append(True))

    class StopLoop(Exception):
        pass

    async def stop_after_iteration(_delay):
        raise StopLoop

    monkeypatch.setattr(power_app.asyncio, "sleep", stop_after_iteration)
    loop = asyncio.new_event_loop()
    try:
        with pytest.raises(StopLoop):
            loop.run_until_complete(power_app.PowerManager().background_task())
    finally:
        loop.close()

    assert bool(power_off_calls) is should_power_off


def test_a11y_dedup_avoids_unchanged_normalization_and_preserves_announcements():
    from system.a11y.printer import PrintA11y

    printer = PrintA11y()
    printer.collect_text("steady")
    assert printer.get_deduped_strings() == []
    previous_snapshot = printer.last_strings

    printer.reset()
    printer.collect_text("steady")
    assert printer.get_deduped_strings() is None
    assert printer.last_strings is previous_snapshot

    printer.reset()
    printer.collect_text("changed")
    assert printer.get_deduped_strings() == ["changed"]
    printer.collected[0] = "changed again"
    assert printer.get_deduped_strings() == ["changed again"]

    printer.reset()
    printer.add_alt(None, "changed again")
    assert printer.get_deduped_strings() == []

    printer.reset()
    printer.add_alt(None, "always", always=True)
    printer.add_alt(None, "transient", transient=True)
    printer.add_alt(None, "suppressed change")
    assert printer.get_deduped_strings() == ["always", "transient"]


def test_a11y_reset_reuses_frame_lists_and_all_strings_is_a_snapshot():
    from system.a11y.printer import PrintA11y

    printer = PrintA11y()
    printer.collect_text("collected")
    snapshot = printer.get_all_strings()
    collected = printer.collected
    alts = printer.alts

    printer.reset()

    assert printer.collected is collected
    assert printer.alts is alts
    assert not printer.collected
    assert not printer.alts
    assert snapshot == ["collected"]


def test_a11y_reuses_unchanged_alt_entry_across_frames():
    from system.a11y.printer import PrintA11y

    printer = PrintA11y()
    label = "Menu item"
    printer.add_alt(None, label)
    cached_entry = printer.alts[0]

    printer.reset()
    printer.add_alt(None, label)

    assert printer.alts[0] is cached_entry
    assert printer.get_deduped_strings() == []


def test_a11y_finalise_frame_is_async(capsys):
    import asyncio
    import inspect
    from system.a11y.printer import PrintA11y

    printer = PrintA11y()
    printer.add_alt(None, "steady", always=True)
    assert inspect.iscoroutinefunction(printer.finalise_frame)
    loop = asyncio.new_event_loop()
    try:
        loop.run_until_complete(printer.finalise_frame())
    finally:
        loop.close()
    assert capsys.readouterr().out == "[Screen reader] steady\n"


def test_set_color_dispatches_rgb_tuples_and_callable_gradients(monkeypatch):
    import app_components.tokens as tokens

    calls = []

    class RGB(tuple):
        def __iter__(self):
            raise AssertionError("RGB tuple should be indexed, not expanded")

    def gradient(ctx):
        calls.append(("gradient", ctx))

    class DrawContext:
        def rgb(self, *color):
            calls.append(("rgb", color))
            return self

    context = DrawContext()
    monkeypatch.setattr(tokens, "colors", {})
    monkeypatch.setattr(
        tokens,
        "ui_colors",
        {"rgb": RGB((0.1, 0.2, 0.3)), "gradient": gradient},
    )

    assert tokens.set_color(context, "rgb") is context
    assert tokens.set_color(context, "gradient") is context
    assert calls == [
        ("rgb", (0.1, 0.2, 0.3)),
        ("gradient", context),
    ]


def test_menu_draw_preserves_focused_neighbor_and_accessibility_labels(monkeypatch):
    from types import SimpleNamespace
    import app_components.menu as menu_module

    monkeypatch.setattr(menu_module, "set_color", lambda ctx, color: ctx)

    class DrawContext:
        CENTER = 1
        MIDDLE = 2

        def __init__(self):
            self.labels = []
            self.positions = []
            self.font_sizes = []
            self.a11y = SimpleNamespace(add_alt=lambda app, text: self.labels.append(text))

        @property
        def font_size(self):
            return self._font_size

        @font_size.setter
        def font_size(self, value):
            self._font_size = value
            self.font_sizes.append(value)

        def save(self):
            pass

        def restore(self):
            pass

        def move_to(self, x, y):
            self.positions.append((x, y))
            return self

        def text(self, label):
            self.labels.append(label)

    menu = object.__new__(menu_module.Menu)
    menu.show_info = False
    menu.info_items = []
    menu.focused_item_font_size_arr = [20, 30, 40]
    menu.menu_items = ["Alpha", "Beta", "Gamma"]
    menu.is_animating = "none"
    menu.position = 1
    menu.item_font_size = 10
    menu.focused_item_margin = 20
    menu.item_line_height = 15
    menu._idle_next_item_y = (35, 50)
    menu._menu_draw_y_offset = 0
    context = DrawContext()

    menu.draw(context)

    assert context.labels == ["Beta", "Beta", "Alpha", "Gamma"]
    assert context.positions == [(0, 0), (0, -35), (0, 35)]
    assert context.font_sizes[0] is menu.focused_item_font_size_arr[1]


def test_background_draw_failure_restores_context_and_disables_runner(monkeypatch):
    from types import SimpleNamespace
    import app_components.background as background_module

    class DrawContext:
        def __init__(self):
            self.save_count = 0
            self.restore_count = 0

        def save(self):
            self.save_count += 1

        def restore(self):
            self.restore_count += 1

    manager = object.__new__(background_module._Background)

    def fail_draw(ctx):
        raise RuntimeError("background failed")

    manager.runner = SimpleNamespace(draw=fail_draw)
    manager.selection = ("test", None)
    emitted_events = []
    monkeypatch.setattr(background_module.eventbus, "emit", emitted_events.append)
    context = DrawContext()

    manager.draw(context)

    assert manager.runner is None
    assert context.save_count == context.restore_count == 1
    assert len(emitted_events) == 2


@pytest.mark.parametrize("mirror_pattern", [False, True])
def test_backled_manager_skips_unchanged_pixel_writes(monkeypatch, mirror_pattern):
    import types
    from system.backleds import app as backleds

    class PixelSink:
        def __init__(self):
            self.front = [(10, 20, 30)] * 12
            self.back = {}
            self.set_calls = 0
            self.write_calls = 0
            self.read_buffers = []

        def get_into(self, index, result):
            self.read_buffers.append(id(result))
            colour = self.front[index]
            result[0] = colour[0]
            result[1] = colour[1]
            result[2] = colour[2]
            return result

        def __getitem__(self, index):
            raise AssertionError("mirrored reads must use get_into")

        def __setitem__(self, index, colour):
            self.set_calls += 1
            self.back[index] = tuple(colour)

        def write(self):
            self.write_calls += 1

    pixels = PixelSink()
    active = [False] * 6
    if mirror_pattern:
        active[0] = True

    monkeypatch.setattr(backleds.tildagonos, "leds", pixels)
    monkeypatch.setattr(backleds.tildagonos, "set_led_power", lambda _enabled: None)
    monkeypatch.setattr(backleds.eventbus, "on_async", lambda *args: None)
    monkeypatch.setattr(backleds.settings, "get", lambda key, default=None: (
        mirror_pattern if key == "pattern_mirror_hexpansions" else default
    ))
    monkeypatch.setattr(backleds, "active_back_leds", active)

    manager = backleds.BackLEDManager()
    manager.background_update(50)
    assert pixels.set_calls == 6
    assert pixels.write_calls == 1

    manager.background_update(50)
    assert pixels.set_calls == 6
    assert pixels.write_calls == 1

    if mirror_pattern:
        pixels.front[1] = (40, 50, 60)
        manager.background_update(50)
        assert pixels.back[13] == (40, 50, 60)
        assert len(set(pixels.read_buffers)) == 1
    else:
        active[2] = True
        manager.background_update(50)
        assert pixels.back[15] == backleds.led_colours[2]

    assert pixels.set_calls == 7
    assert pixels.write_calls == 2


def test_notification_wrap_uses_few_width_checks_and_preserves_splits():
    from app_components.notification import Notification

    class TextContext:
        def __init__(self):
            self.width_checks = 0

        def text_width(self, text):
            self.width_checks += 1
            return len(text)

    notification = Notification("", open=False)
    notification.width_limits = [5]
    ctx = TextContext()

    assert notification.get_text_for_line(ctx, "red blue green", 0) == (
        "red", "blue green"
    )
    assert notification.get_text_for_line(ctx, "abcdefgh", 0) == ("abcde", "fgh")
    assert notification.get_text_for_line(ctx, "abcde", 0) == ("abcde", "")

    ctx.width_checks = 0
    notification.width_limits = [512]
    notification.get_text_for_line(ctx, "x" * 1024, 0)
    assert ctx.width_checks <= 12


def test_notification_service_skips_settled_closed_slots():
    from system.notification.app import NotificationService

    class FakeNotification:
        def __init__(self, opened, animation_state):
            self._open = opened
            self._animation_state = animation_state
            self.update_calls = 0
            self.draw_calls = 0

        def update(self, _delta):
            self.update_calls += 1

        def draw(self, _ctx):
            self.draw_calls += 1

    service = NotificationService()
    settled = FakeNotification(False, 0)
    closing = FakeNotification(False, 0.1)
    opening = FakeNotification(True, 0.5)
    service.notifications = [settled, closing, opening]

    assert service.update(10) is True
    assert (settled.update_calls, closing.update_calls, opening.update_calls) == (0, 1, 1)

    service.draw(None)
    assert (settled.draw_calls, closing.draw_calls, opening.draw_calls) == (0, 1, 1)


def test_hexdrive_app_init(port):
    from sim.apps.BadgeBot.vendor.HexDrive.hexdrive import HexDriveApp
    config = HexpansionConfig(port)
    HexDriveApp(config)

def test_app_versions_match():
    import sim.apps.BadgeBot.app as BadgeBot
    from sim.apps.BadgeBot.vendor.HexDrive.hexdrive import HexDriveApp
    assert BadgeBot.HEXDRIVE_APP_VERSION == HexDriveApp.VERSION

def test_hexdrive2_metadata_matches_vendor_source():
    import sim.apps.BadgeBot.app as BadgeBot
    from sim.apps.BadgeBot import BadgeBotApp

    source_version = _extract_version_from_source(
        Path(__file__).resolve().parents[1] / "vendor" / "HexDrive2" / "hexdrive2.py"
    )
    assert BadgeBot.HEXDRIVE2_APP_VERSION == source_version

    app_instance = BadgeBotApp()
    hexdrive2_entries = [
        ht for ht in app_instance.HEXPANSION_TYPES if ht.name == "HexDrive2"
    ]
    assert hexdrive2_entries, "No HexDrive2 entries found in BadgeBot metadata"
    for entry in hexdrive2_entries:
        assert entry.app_mpy_name == "hexdrive2"
        assert entry.app_mpy_version == BadgeBot.HEXDRIVE2_APP_VERSION


def test_hexdrive_type_pids_consistent():
    """Verify HexDriveType PIDs in hexdrive.py are consistent with HexpansionType PIDs in app.py.

    HexDriveType stores a single PID byte (low byte), while HexpansionType
    stores the full 16-bit PID.  For every HexDrive-flavour HexpansionType
    the low byte of its PID must match exactly one HexDriveType entry, and
    the motor/servo capability counts must agree.
    """
    from sim.apps.BadgeBot import BadgeBotApp
    from sim.apps.BadgeBot.vendor.HexDrive.hexdrive import _HEXDRIVE_TYPES

    app_instance = BadgeBotApp()
    hexdrive_hexpansion_types = [
        ht for ht in app_instance.HEXPANSION_TYPES if ht.name == "HexDrive"
    ]

    # Build a lookup from PID byte -> HexDriveType
    # Also verify that PID bytes are unique within _HEXDRIVE_TYPES
    hd_by_pid = {}
    for hdt in _HEXDRIVE_TYPES:
        assert hdt.pid not in hd_by_pid, (
            f"Duplicate HexDriveType PID byte 0x{hdt.pid:02X}: "
            f"'{hd_by_pid[hdt.pid].name}' and '{hdt.name}'"
        )
        hd_by_pid[hdt.pid] = hdt

    for ht in hexdrive_hexpansion_types:
        pid_byte = ht.pid & 0xFF
        assert pid_byte in hd_by_pid, (
            f"HexpansionType PID 0x{ht.pid:04X} low byte 0x{pid_byte:02X} "
            f"has no matching HexDriveType"
        )
        hdt = hd_by_pid[pid_byte]
        assert ht.motors == hdt.motors, (
            f"Motor count mismatch for PID 0x{pid_byte:02X}: "
            f"HexpansionType={ht.motors}, HexDriveType={hdt.motors}"
        )
        assert ht.servos == hdt.servos, (
            f"Servo count mismatch for PID 0x{pid_byte:02X}: "
            f"HexpansionType={ht.servos}, HexDriveType={hdt.servos}"
        )


def test_new_settings_registered():
    """Verify motor direction and front-face base settings are always registered."""
    from sim.apps.BadgeBot import BadgeBotApp
    app_instance = BadgeBotApp()
    for key in ('mtr1_dir', 'mtr2_dir', 'front_face'):
        assert key in app_instance.settings, f"Missing setting: {key}"


def test_autodrive_settings_need_hexpansion():
    """auto_speed/auto_obstacle are hardware-dependent; not present without a HexDrive."""
    from sim.apps.BadgeBot import BadgeBotApp
    app_instance = BadgeBotApp()
    # Without a HexDrive, auto-drive settings are NOT registered
    for key in ('auto_speed', 'auto_obstacle'):
        assert key not in app_instance.settings, (
            f"Setting '{key}' should not be registered without a HexDrive"
        )


def test_autodrive_decide_state_and_turn_transition():
    """Auto-drive should pause to display the scan before entering the turn state."""
    import types
    import sim.apps.BadgeBot.autodrive as autodrive

    class DummyApp:
        def __init__(self):
            self.sensor_test_mgr = None
            self.bluetooth_mgr = None
            self.acceleration = 20000
            self.max_power = 55000
            self.button_states = {}
            self.settings = {
                "auto_speed": types.SimpleNamespace(v=56000),
                "auto_scan_speed": types.SimpleNamespace(v=14000),
                "auto_obstacle": types.SimpleNamespace(v=250),
            }
            self.refresh = False
            self.hexdrive_apps = []
            self.notification = None

        def enable_motors(self, *args, **kwargs):
            return True

        def set_menu(self, *args, **kwargs):
            return None

        def return_to_menu(self, *args, **kwargs):
            return None

        def draw_message(self, *args, **kwargs):
            return None

    mgr = autodrive.AutoDriveMgr(DummyApp(), logging=False)
    mgr._active = True
    mgr.sub_state = autodrive._AUTO_SUB_DECIDE
    mgr.decide_timer = 10
    mgr.quadrant_candidates = [("front", 0.0, 0.0, True)]
    mgr.turn_dir = 1
    mgr.turn_deg = 90.0
    mgr._app.button_states = {}

    mgr.update(100)

    assert mgr.sub_state == autodrive._AUTO_SUB_TURN


def test_front_face_labels_complete():
    """Verify _FRONT_FACE_LABELS has one entry for each valid front_face value (0-11)."""
    import sim.apps.BadgeBot.app as BadgeBot
    front_face_labels = getattr(BadgeBot, '_FRONT_FACE_LABELS', None)
    assert front_face_labels is not None
    assert len(front_face_labels) == 12


def test_autodrive_scan_theta_uses_front_face_rotation():
    """Scan plot angles should rotate with the configured front face, not a fixed 90° offset."""
    import types
    import sim.apps.BadgeBot.autodrive as autodrive

    app = types.SimpleNamespace(
        sensor_test_mgr=None,
        settings={"front_face": types.SimpleNamespace(v=5)},
    )
    mgr = autodrive.AutoDriveMgr(app, logging=False)

    expected = radians(0.0) - (pi / 2.0) + radians(5 * 30.0)
    assert mgr._scan_angle_to_theta(0.0) == pytest.approx(expected)


def test_menu_items_include_sensor_and_auto():
    """Verify the main menu includes Sensor Test and Auto Drive entries."""
    import sim.apps.BadgeBot.app as BadgeBot
    assert "Sensor Test" in BadgeBot.MAIN_MENU_ITEMS
    assert "Auto Drive" in BadgeBot.MAIN_MENU_ITEMS


def test_remote_autodrive_button_3_maps_and_starts():
    """Bluefruit button 3 should trigger Auto Drive start/stop via the same remote command path."""
    import sim.apps.BadgeBot.app as BadgeBot
    from sim.apps.BadgeBot.bluetooth_mgr import _CONTROL_BUTTON_COMMANDS

    app = BadgeBot.BadgeBotApp()
    app.current_state = BadgeBot.STATE_MENU
    app.num_motors = 2
    seen = {}

    class DummyAutoDriveMgr:
        def start(self):
            seen["start"] = True
            return True

        def stop(self):
            seen["stop"] = True

    app._autodrive_mgr = DummyAutoDriveMgr()

    assert _CONTROL_BUTTON_COMMANDS["3"] == BadgeBot.REMOTE_CMD_AUTO_DRIVE_TOGGLE
    app.post_remote_command(BadgeBot.REMOTE_CMD_AUTO_DRIVE_TOGGLE)
    app._process_remote_commands()

    assert app.current_state == BadgeBot.STATE_AUTODRIVE
    assert seen["start"] is True


def test_sensor_base_interface():
    """Verify SensorBase class has the expected interface."""
    from sim.apps.BadgeBot.sensors.sensor_base import SensorBase
    sensor = SensorBase()
    assert hasattr(sensor, 'begin')
    assert hasattr(sensor, 'read')
    assert hasattr(sensor, 'reset')
    assert hasattr(sensor, 'is_ready')
    assert sensor.is_ready is False


def test_legacy_sensor_registry_is_disabled():
    """HexDrive2 owns sensor polling; the old app-level registry stays empty."""
    from sim.apps.BadgeBot.sensors import ALL_SENSOR_CLASSES
    assert ALL_SENSOR_CLASSES == []


def test_scheduler_awaits_async_a11y_and_skips_none(monkeypatch):
    import asyncio
    from contextlib import nullcontext
    from types import SimpleNamespace
    import system.scheduler as scheduler_module

    class StopAfterRender(Exception):
        pass

    class AsyncHandler:
        def __init__(self):
            self.finalised = False
            self.reset_called = False

        async def finalise_frame(self):
            await asyncio.sleep(0)
            self.finalised = True

        def reset(self):
            self.reset_called = True

    context = SimpleNamespace()
    monkeypatch.setattr(scheduler_module.display, "get_ctx", lambda: context)
    monkeypatch.setattr(scheduler_module.display, "end_frame", lambda _ctx: None)
    monkeypatch.setattr(scheduler_module, "_RENDER_PERF_TIMER", nullcontext())

    async def stop_after_render(_delay):
        raise StopAfterRender

    monkeypatch.setattr(scheduler_module, "sleep_ms", stop_after_render)

    async def render_once(handler):
        scheduler = scheduler_module._Scheduler.__new__(scheduler_module._Scheduler)
        scheduler.render_needed = asyncio.Event()
        scheduler.render_needed.set()
        scheduler.foreground_stack = []
        scheduler.on_top_stack = []
        scheduler.a11y_handler = handler
        await scheduler._render_task()

    handler = AsyncHandler()
    loop = asyncio.new_event_loop()
    try:
        with pytest.raises(StopAfterRender):
            loop.run_until_complete(render_once(handler))
        assert handler.finalised is True
        assert handler.reset_called is True

        with pytest.raises(StopAfterRender):
            loop.run_until_complete(render_once(None))
        assert context.a11y is None
    finally:
        loop.close()


def test_badgebot_a11y_suppression_restores_previous_handler(monkeypatch):
    import sim.apps.BadgeBot.app as badgebot

    app = badgebot.BadgeBotApp.__new__(badgebot.BadgeBotApp)
    app._a11y_restore_factory = badgebot._A11yHandlerFactory()
    app._a11y_handler_suppressed = False
    previous_handler = object()
    emitted_events = []

    monkeypatch.setattr(badgebot.scheduler, "a11y_handler", previous_handler)
    monkeypatch.setattr(badgebot.eventbus, "emit", emitted_events.append)

    app._suppress_a11y()
    assert emitted_events[0].klass() is None
    app._suppress_a11y()
    assert len(emitted_events) == 1

    app._restore_a11y()
    assert emitted_events[1].klass() is previous_handler
    app._restore_a11y()
    assert len(emitted_events) == 2
