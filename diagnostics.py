"""Shared development diagnostics output hooks for BadgeBot."""

_diagnostics_state = {"sink": None}


def set_diagnostics_output(sink):
    """Register a callable that receives diagnostic pin updates."""
    _diagnostics_state["sink"] = sink


def diagnostics_output(index: int, value: int):
    """Emit a diagnostic output update if a sink is registered."""
    sink = _diagnostics_state["sink"]
    if sink is not None:
        sink(index, value)
