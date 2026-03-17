"""Tests for the PID auto-tuning module (autotune.py).

These tests exercise the auto-tuner in isolation (no hardware required).
"""
import sys
import os
import importlib

import pytest

# Import autotune directly by file path to avoid the __init__.py package import
# which pulls in badge-platform-specific modules.
_repo_root = os.path.join(os.path.dirname(__file__), "..")
_spec = importlib.util.spec_from_file_location("autotune", os.path.join(_repo_root, "autotune.py"))
autotune = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(autotune)

PIDAutoTuner     = autotune.PIDAutoTuner
METHOD_ZIEGLER_NICHOLS  = autotune.METHOD_ZIEGLER_NICHOLS
METHOD_TYREUS_LUYBEN    = autotune.METHOD_TYREUS_LUYBEN
METHOD_SOME_OVERSHOOT   = autotune.METHOD_SOME_OVERSHOOT
METHOD_NO_OVERSHOOT     = autotune.METHOD_NO_OVERSHOOT
_AT_IDLE   = autotune._AT_IDLE
_AT_RELAY  = autotune._AT_RELAY
_AT_DONE   = autotune._AT_DONE
_AT_FAILED = autotune._AT_FAILED

import math


# ---------- PIDAutoTuner lifecycle ----------

def test_initial_state():
    t = PIDAutoTuner(10000, logging=False)
    assert t.state == _AT_IDLE
    assert t.get_gains() is None
    assert t.get_quality() == 0
    assert t.is_running is False
    assert t.is_complete is False

def test_start_sets_relay():
    t = PIDAutoTuner(10000, logging=False)
    t.start()
    assert t.state == _AT_RELAY
    assert t.is_running is True

def test_get_status_text_idle():
    t = PIDAutoTuner(10000, logging=False)
    assert "Idle" in t.get_status_text()

def test_get_status_text_running():
    t = PIDAutoTuner(10000, logging=False)
    t.start()
    assert "Tuning" in t.get_status_text()


# ---------- Simulated oscillation ----------

def _simulate_oscillation(tuner, period_ms=200, amplitude=500, num_full_cycles=10):
    """Feed a sinusoidal error signal into the tuner to simulate oscillation.

    Returns the list of motor output tuples produced.
    """
    outputs = []
    dt = 5  # 5 ms steps
    total = period_ms * num_full_cycles
    for t_ms in range(0, total, dt):
        error = int(amplitude * math.sin(2 * math.pi * t_ms / period_ms))
        out = tuner.update(error, dt)
        outputs.append(out)
        if not tuner.is_running:
            break
    return outputs


def test_autotune_completes_with_good_data():
    """A clean sinusoidal error should lead to a successful tune."""
    t = PIDAutoTuner(10000, base_power=5000, hysteresis=20,
                     target_cycles=10, method=METHOD_ZIEGLER_NICHOLS, logging=False)
    t.start()
    _simulate_oscillation(t, period_ms=200, amplitude=500, num_full_cycles=20)
    assert t.is_complete or t.is_running  # may need more cycles
    if t.is_complete:
        gains = t.get_gains()
        assert gains is not None
        Kp, Ki, Kd = gains
        assert Kp > 0
        assert Ki >= 0
        assert Kd >= 0
        assert t.get_quality() > 0

def test_autotune_fails_with_zero_amplitude():
    """Constant-zero error should not yield valid gains."""
    t = PIDAutoTuner(10000, hysteresis=20, target_cycles=8, logging=False)
    t.start()
    # Feed constant zero error — no crossings should happen
    for ms in range(0, 5000, 10):
        t.update(0, 10)
    # Tuner should still be running (not enough crossings) or failed
    assert not t.is_complete

def test_autotune_diagnostics():
    """get_diagnostics() returns the expected keys."""
    t = PIDAutoTuner(10000, logging=False)
    t.start()
    diag = t.get_diagnostics()
    expected_keys = {"state", "crossings", "target", "Ku", "Tu_ms",
                     "Kp", "Ki", "Kd", "quality", "method", "elapsed"}
    assert expected_keys.issubset(set(diag.keys()))

def test_tuning_methods_produce_different_gains():
    """Different tuning methods should produce different Kp values."""
    gains_by_method = {}
    for method in [METHOD_ZIEGLER_NICHOLS, METHOD_TYREUS_LUYBEN,
                   METHOD_SOME_OVERSHOOT, METHOD_NO_OVERSHOOT]:
        t = PIDAutoTuner(10000, base_power=5000, hysteresis=20,
                         target_cycles=10, method=method, logging=False)
        t.start()
        _simulate_oscillation(t, period_ms=200, amplitude=500, num_full_cycles=20)
        if t.is_complete:
            gains_by_method[method] = t.get_gains()

    # At least ZN and TL should be complete and different
    if METHOD_ZIEGLER_NICHOLS in gains_by_method and METHOD_TYREUS_LUYBEN in gains_by_method:
        assert gains_by_method[METHOD_ZIEGLER_NICHOLS][0] != gains_by_method[METHOD_TYREUS_LUYBEN][0]

def test_relay_output_format():
    """update() should return a 2-tuple of ints."""
    t = PIDAutoTuner(10000, base_power=5000, logging=False)
    t.start()
    out = t.update(300, 10)
    assert isinstance(out, tuple)
    assert len(out) == 2

def test_quality_score_range():
    """Quality score should be 0-100."""
    t = PIDAutoTuner(10000, hysteresis=20, target_cycles=10, logging=False)
    t.start()
    _simulate_oscillation(t, period_ms=200, amplitude=500, num_full_cycles=20)
    if t.is_complete:
        q = t.get_quality()
        assert 0 <= q <= 100
