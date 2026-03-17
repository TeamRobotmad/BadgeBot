# Copilot Development Instructions for BadgeBot

This document provides essential context for AI assistants (GitHub Copilot, etc.)
working on the BadgeBot codebase.  It was created during a comprehensive review of
code, comments, and documentation consistency.

---

## Project Overview

BadgeBot is a MicroPython robotics control application for the EMF Camp 2024
Tildagon badge.  It drives motors, servos, and stepper motors via the HexDrive
hexpansion, and can follow lines via the HexSense hexpansion with QTRX reflectance
sensors.  The app runs directly on an ESP32-S3 badge (or in a desktop simulator).

**Key facts:**
- Platform: MicroPython on ESP32-S3 (Tildagon badge) / Python 3.10+ simulator
- License: LGPL-3.0-only
- App version: `APP_VERSION` in `app.py` (major.minor format, e.g. "1.3") – this is the
  definitive version.  `tildagon.toml` `version` must always match `APP_VERSION`.
- HexDrive firmware version: `HEXDRIVE_APP_VERSION` in `hexdrive.py` – a separate integer
  versioning the HexDrive public interface, independent of the app version.

---

## Repository Structure

### Runtime Modules (deployed to badge)

| File | Purpose |
|------|---------|
| `__init__.py` | Package init – exports `BadgeBotApp` |
| `app.py` | Main app class (`BadgeBotApp`), state machine, menus, LED control, countdown timers |
| `hexdrive.py` | HexDrive hexpansion firmware – runs from EEPROM; PWM / motor / servo / stepper control |
| `hexpansion_mgr.py` | Hexpansion detection, EEPROM init, firmware programming, upgrade, erasure |
| `motor_moves.py` | Logo/turtle-style motor programming (UP/DOWN/LEFT/RIGHT instruction sequences) |
| `servo_test.py` | Servo tester (position, trim, scanning modes; up to 4 servos) |
| `stepper_test.py` | Stepper motor tester (position and speed modes) |
| `line_follow.py` | Line follower with QTRX sensors and PID control; includes `LineSensor`/`LineSensors` drivers |
| `autotune.py` | PID auto-tuning algorithm (relay feedback / Åström-Hägglund / Ziegler-Nichols) |
| `autotune_mgr.py` | PID auto-tune UI manager (countdown integration, display rendering) |
| `settings_mgr.py` | `MySetting` class (bounded values with persistence) and `SettingsMgr` UI |
| `utils.py` | Helper functions: animated logo drawing, QR code rendering, version parsing, `chain()` |
| `uQR.py` | Micro QR code generator (third-party, for MicroPython) |

### Configuration

| File | Purpose |
|------|---------|
| `tildagon.toml` | Badge app manifest (name, version, license, URL) |
| `metadata.json` | App metadata for badge menu (callable class, name, category) |
| `pyproject.toml` | Pylint configuration |

### Development Tools (`dev/`)

| File | Purpose |
|------|---------|
| `build_release.py` | Compile .py → .mpy and prune non-release files |
| `generate_qr_code.py` | Generate QR code bitfields; optionally update app.py |
| `check_qr_sync.py` | Validate `_QR_CODE` in app.py matches expected URL |
| `download_to_device.py` | Smart incremental deployment to badge (SHA256 change tracking) |
| `setup_dev_env.ps1` | Windows development environment setup |
| `setup_dev_env.sh` | Linux/macOS development environment setup |
| `dev_requirements.txt` | Python dev dependencies |

### Tests (`tests/`)

| File | Purpose |
|------|---------|
| `test_smoke.py` | Integration tests: imports, app init, version match, exports |
| `test_autotune.py` | PID auto-tuner unit tests (17 cases: error computation, lifecycle, tuning methods, quality) |

**Running tests:**
```
cd tests
python -m pytest test_smoke.py test_autotune.py -v
```
Tests must be run from the `tests/` directory.  Running from the repo root causes
import errors because `__init__.py` tries to import badge-platform-specific modules.
The CI workflow (`.github/workflows/tests.yml`) checks out the parent
`badge-2024-software` repo and runs BadgeBot tests as a submodule within that
structure.

### Type Stubs (`typings/`)

Stub `.pyi` files for IDE support (Pylance/mypy) covering badge-specific modules:
`app`, `app_components`, `asyncio`, `display`, `events`, `frontboards`, `machine`,
`ota`, `settings`, `system`, `tildagonos`, `time`, `ure`, `vfs`.

---

## Architecture

### App State Machine

States are defined as module-level constants in `app.py`:

```
STATE_HEXPANSION = -1    Hexpansion detection / setup
STATE_MENU = 0           Main menu
STATE_MESSAGE = 1        Warning / error message display
STATE_LOGO = 2           About screen with animated logo + QR code
STATE_COUNTDOWN = 3      Shared 5-second countdown (Motor Moves & PID AutoTune)
STATE_SETTINGS = 4       Settings editor
STATE_MOTOR_MOVES = 5    Logo-style motor programming
STATE_SERVO = 6          Servo tester
STATE_STEPPER = 7        Stepper tester
STATE_FOLLOWER = 8       Line follower
STATE_AUTOTUNE = 9       PID auto-tuner
```

### Manager Pattern

Each functional area is encapsulated in a manager class with a consistent interface:
- `__init__(app)` – wire up to BadgeBotApp
- `start() -> bool` – enter the mode from the menu
- `update(delta)` – per-tick state machine update
- `draw(ctx)` – render the UI
- `background_update(delta)` – (optional) high-frequency motor control; returns `(int, int)` motor output or `None`

The main app uses dispatch tables (`_state_update_dispatch`, `_state_draw_dispatch`,
`_BG_DISPATCH`) to route to the correct manager based on `current_state`.

### Settings Registration

Each module defines an `init_settings(s, MySetting)` function that registers its own
settings into the shared `settings` dict.  These are called from `BadgeBotApp.__init__()`.

Settings use the `MySetting` class which supports:
- `v` (current value), `d` (default), `_min`, `_max`
- `inc(v, level)` / `dec(v, level)` – level-based magnitude increments
- `persist()` – save to platform storage (deletes if equal to default)
- Types: `bool`, `int`, `float`

### Countdown Mechanism

`STATE_COUNTDOWN` is shared between Motor Moves and PID AutoTune.
`app.countdown_next_state` tracks which state to transition to after the countdown.
After the countdown finishes, `_update_state_countdown()` calls `begin_moves()` or
`begin_tuning()` on the respective manager.

---

## Settings Reference (Comprehensive)

### Motor Moves (registered in `motor_moves.py`)
| Key | Default | Min | Max | Description |
|-----|---------|-----|-----|-------------|
| `acceleration` | 7500 | 1 | 65535 | PWM change limit per tick (prevents jerky motion) |
| `max_power` | 20000 | 1000 | 65535 | Maximum motor power level |
| `drive_step_ms` | 50 | 5 | 200 | Step duration for forward/backward (ms) |
| `turn_step_ms` | 20 | 5 | 200 | Step duration for turning (ms) |

### Servo Test (registered in `servo_test.py`)
| Key | Default | Min | Max | Description |
|-----|---------|-----|-----|-------------|
| `servo_step` | 10 | 1 | 100 | Servo pulse step (µs) |
| `servo_range` | 1000 | 100 | 1400 | Servo motion range ± from centre (µs) |
| `servo_period` | 20 | 5 | 50 | Servo control period (ms) |

### Stepper Test (registered in `stepper_test.py`)
| Key | Default | Min | Max | Description |
|-----|---------|-----|-----|-------------|
| `step_max_pos` | 3100 | 0 | 65535 | Maximum stepper position |

### Line Follower (registered in `line_follow.py`)
| Key | Default | Min | Max | Description |
|-----|---------|-----|-----|-------------|
| `line_threshold` | 500 | 0 | 65535 | Line sensor detection threshold |
| `pid_kp` | 20000 | 0 | 65536 | Proportional gain |
| `pid_ki` | 0 | 0 | 65535 | Integral gain |
| `pid_kd` | 0 | 0 | 65535 | Derivative gain |

### Hexpansion Management (registered in `hexpansion_mgr.py`)
| Key | Default | Min | Max | Description |
|-----|---------|-----|-----|-------------|
| `erase_slot` | 0 | 0 | 6 | Slot to offer EEPROM erase (0 = disabled) |

### General (registered in `app.py`)
| Key | Default | Min | Max | Description |
|-----|---------|-----|-----|-------------|
| `brightness` | 1.0 | 0.1 | 1.0 | LED brightness scale factor |
| `logging` | False | False | True | Enable debug logging output |

---

## Coding Conventions

- **Module headers**: Each `.py` file starts with a comment block describing the module
  purpose and its public interface (functions/methods called by the main app).
- **Settings pattern**: Each module provides `init_settings(s, MySetting)`.
- **Manager pattern**: `__init__(app)`, `start()`, `update(delta)`, `draw(ctx)`,
  optionally `background_update(delta)`.
- **State constants**: Defined in `app.py` and imported by sub-modules via
  `from .app import STATE_*`.
- **Logging**: Use `if app.settings['logging'].v:` guard before `print()` statements.
- **Import style**: Standard library first, then badge-specific, then relative imports.
- **Comments**: Use `#` line comments; docstrings for public classes/methods.
- **MicroPython compatibility**: Avoid features not available in MicroPython (e.g. some
  typing features, `dataclasses`).  Keep memory usage low.

---

## Known Issues to Review

These issues were identified during the consistency review and should be addressed:

### HexDrive types defined in two places

`hexdrive.py` uses `HexDriveType` (for firmware-level type identification from EEPROM
PID byte) while `hexpansion_mgr.py` uses `HexpansionType` (for app-level detection
and firmware management).  Both describe the same hardware variants.

**Constraint:** `hexdrive.mpy` must fit in the 8 KB hexpansion EEPROM, so
`hexdrive.py` must not import from the main app or grow unnecessarily.

**Current approach:** The main app cannot import `HexDriveType` definitions from
`hexdrive.py` without increasing binary size on the EEPROM.  Instead, a test
(`test_hexdrive_type_pids_consistent`) in `test_smoke.py` validates that the PID
bytes and capability counts (motors, servos, steppers) are consistent between the
two definitions.  **Any change to HexDrive variant definitions must update both
files and pass this test.**

In future, `hexsense.py` (for HexSense hexpansions) will follow the same pattern:
it will have its own PID definitions, and `hexpansion_mgr.py` will have corresponding
`HexpansionType` entries.  `hexdrive.py` should not include HexSense PIDs.

### Servo test module: missing `background_update` in public interface comment

The `servo_test.py` module header does not list `background_update` in the public
interface, but the `ServoTestMgr` class does not have a `background_update` method
either (servo updates are handled in `update`).  This is actually consistent – just
note that servo test does not participate in the background dispatch table.  Similarly,
`stepper_test.py` does not have `background_update`.

---

## Development Workflow

1. **Build**: No compile step needed for development – Python source files are used directly.
   For release: `python dev/build_release.py` compiles `.py` → `.mpy`.
2. **Test**: `cd tests && python -m pytest -v`
3. **Lint**: `pylint` (config in `pyproject.toml`)
4. **Format**: `isort` for import ordering
5. **QR Code**: Regenerate with `python dev/generate_qr_code.py --url <URL> --write-app`
6. **Deploy to badge**: `python dev/download_to_device.py`
   - Use `--app-dir :apps/<name>` to deploy to a specific directory on the badge
     (default: `:apps/LineFollower`).  For example, to deploy as the main BadgeBot app:
     `python dev/download_to_device.py --app-dir :apps/TeamRobotmad_BadgeBot`

### Version Management

`APP_VERSION` in `app.py` is the definitive version (major.minor format, e.g. "1.3").
The release process must update `tildagon.toml` `version` to match `APP_VERSION`.
`HEXDRIVE_APP_VERSION` in `hexdrive.py` is a separate integer versioning the HexDrive
firmware public interface and is incremented independently when the HexDrive API changes.

---

## CI/CD

### tests.yml
- Triggers: push to `main`/`ci-test-*`, pull requests, manual dispatch
- Checks out `badge-2024-software` repo, updates BadgeBot submodule, runs pytest

### release.yml
- Manual trigger only, on `main` branch
- Runs `dev/build_release.py` with `-f` flag to compile and prune
