# AGENTS.md

Guidance for AI agents and automated tools working in this repository.

## Repository Overview

BadgeBot is a MicroPython application for the EMF Camp Tildagon badge. It controls HexDrive hexpansion boards (brushed DC motors, RC servos) and supports sensor testing, line following, Bluetooth LE control, and persistent settings management.

## Scope

All application source lives at the repository root (not under a subdirectory). Unless explicitly asked, avoid editing files outside the repository root package.

Key files:
- `app.py` — Main app (`BadgeBotApp`), state routing, menus, draw/update loops.
- `hexpansion_mgr.py` — Port scanning, EEPROM prep/program/erase, HexDrive lifecycle.
- `motor_controller.py` — Higher-level movement control and assisted maneuvers.
- `motor_moves.py`, `servo_test.py`, `line_follow.py`, `sensor_test.py` — Mode managers.
- `settings_mgr.py` — Persistent settings, clamping, and defaults.
- `bluetooth_mgr.py` — Bluetooth LE control interface.
- `utils.py` — Shared UI/drawing helpers.
- `vendor/HexDrive/hexdrive.py`, `vendor/HexDrive2/hexdrive2.py` — EEPROM app sources (compiled to `EEPROM/*.mpy`).
- `tests/` — pytest suite.
- `dev/` — Developer tooling (setup scripts, minifier, QR code generator, release builder).

## Development Setup

**WSL (recommended for simulator tests):**
```bash
sh ./dev/setup_wsl_dev_env.sh
```

**Linux/macOS:**
```bash
sh ./dev/setup_dev_env.sh
```

**Windows:**
```powershell
powershell -ExecutionPolicy Bypass -File .\dev\setup_dev_env.ps1
```

The WSL helper uses `uv` to provision Python 3.10 and installs dev and simulator requirements. This is the most reliable environment for running the full test suite.

## Running Tests

Tests must be run from the `tests/` directory:
```bash
cd tests
python -m pytest test_smoke.py test_autotune.py -v
```

If BadgeBot is checked out inside the `badge-2024-software` repo, set `PYTHONPATH` to the parent repo root:
```bash
cd tests
PYTHONPATH=/path/to/badge-2024-software ../.venv-wsl310/bin/python -m pytest test_smoke.py test_autotune.py -v
```

After any change, verify:
1. App imports cleanly.
2. Simulator launches without tracebacks.
3. All relevant tests pass.

## Linting

Run `isort` on in-app Python files and check `pylint` for errors:
```bash
isort *.py
pylint *.py
```

Pylint is configured via `pyproject.toml`. MicroPython/BadgeOS runtime modules are listed under `ignored-modules` to suppress false import errors on desktop.

## Architecture Notes

- `BadgeBotApp` is the orchestration layer. It owns state and dispatch tables and delegates mode behavior to manager classes.
- Prefer adding/changing manager logic rather than embedding mode-specific code in unrelated modules.
- Keep responsibilities separated: UI/rendering, state transitions, motor control, sensor IO.
- Settings are stored via `settings_mgr.py`; on load, `MySetting.clamp()` enforces min/max bounds.

## Simulator vs Hardware

BadgeBot runs in both the badge simulator and on physical hardware.
- Guard hardware-only paths with platform checks where needed.
- Do not assume desktop-only modules exist on-device.
- Keep simulator fakes compatible with call signatures used by app code.

## HexDrive Versioning

When changing `vendor/HexDrive/hexdrive.py` or `vendor/HexDrive2/hexdrive2.py` behavior or interface:
1. Bump `VERSION` in the relevant `hexdrive*.py`.
2. Bump the matching app-side HexDrive version constant used for compatibility checks.
3. Rebuild the `.mpy` artifact: `python dev/minify.py --source vendor/HexDrive/hexdrive.py --artifact EEPROM/hexdrive.mpy`
4. Ensure smoke/version tests still pass.

**Never change only one side of the app/HexDrive version pairing.**

## Minification

EEPROM apps are minified then compiled to `.mpy` to reduce on-badge footprint. This is handled automatically by `dev/download_to_device.py`. To run standalone:
```bash
python dev/minify.py
```

Intermediate build artifacts (`*.min.py`, `*.renamed.py`) are in `.gitignore` and must not be committed.

## Coding Conventions

- Target MicroPython constraints: keep allocations and per-frame overhead low.
- Use relative imports inside the package.
- Keep public API names stable unless a migration plan is included.
- Prefer clear, small changes over broad refactors.
- Maintain existing formatting and style in touched files.
- Do not add comments unless they match the style of existing comments or explain a genuinely complex change.

## Common Pitfalls

- Updating HexDrive logic without syncing the app-side compatibility version constant.
- Mixing simulator-only assumptions into badge runtime paths.
- Editing files outside the repository root package without an explicit requirement.
- Hardcoding line numbers or "currently X" snapshots in documentation.
