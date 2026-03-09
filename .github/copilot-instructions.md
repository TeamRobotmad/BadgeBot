# BadgeBot — Copilot Instructions

## Project Overview

**BadgeBot** is a MicroPython application for the **Tildagon badge** (EMF Camp 2024). It is the companion app for the **HexDrive** hexpansion — a motor/servo/stepper driver board that plugs into any of the badge's six hexpansion slots.

The app lives at `sim/apps/BadgeBot/` inside the `badge-2024-software` repository and is submoduled there so the **badge simulator** (`sim/run.py`) can run it directly. It also runs on real badge hardware.

- **Author**: Team Robotmad
- **License**: LGPL-3.0-only
- **App version**: defined as `_APP_VERSION` in `app.py` (currently `"1.5"`)
- **HexDrive firmware version**: `CURRENT_APP_VERSION` / `APP_VERSION` in `hexdrive.py` (integer, currently `6`)
- **Repository**: <https://github.com/TeamRobotmad/BadgeBot>

---

## Running the App

### Simulator
```bash
# From the sim directory:
pipenv run python run.py
```
The simulator is detected at runtime via `sys.platform != "esp32"` and stored in `_IS_SIMULATOR`.

### Real Badge
Install via the [Tildagon App Directory](https://apps.badge.emfcamp.org/) or copy the minimal file set onto the badge:
- `app.py`, `tildagon.toml`, `metadata.json`, `utils.py`

---

## File Map

| File / Directory | Purpose |
|---|---|
| `__init__.py` | Re-exports `BadgeBotApp` from `app.py` |
| `app.py` | **Main application** (~2 280 lines). Contains `BadgeBotApp`, the state machine, UI, motor/servo/stepper control, hexpansion management, EEPROM programming, settings, drawing routines, and helper classes (`Stepper`, `HexDriveType`, `MySetting`, `Instruction`, `StepperMode`, `ServoMode`). |
| `hexdrive.py` | **HexDrive EEPROM app** (~548 lines). `HexDriveApp(app.App)` is the firmware stored on the HexDrive's EEPROM and executed by BadgeOS. Manages PWM outputs, motor driving, servo positioning, stepper phases, power control (SMPSU), and a keep-alive watchdog. Also contains a local `HexDriveType` class. Exports `__app_export__ = HexDriveApp`. |
| `linefollower.py` | **Line Follower variant** (~2 611 lines). `LineFollowerApp(app.App)` — extended copy of BadgeBotApp with additional line-following (reflectance sensor) and gesture sensor (GR10-30) support. Has extra states (`STATE_FOLLOWER = 19`) and classes (`LineSensor`). Exports `__app_export__ = LineFollowerApp`. |
| `gr10_30.py` | **DFRobot GR10-30 gesture sensor** I²C driver. Supports directional swipes, rotation, wave, hover, and continuous rotation gestures. |
| `sensor_manager.py` | **SensorManager** — opens an I²C port, scans for known sensors (auto-discovery), and manages the currently selected sensor for the sensor-test UI mode. |
| `sensors/` | Package of I²C sensor drivers, all inheriting from `SensorBase`. |
| `utils.py` | Shared helpers: `roundtext()`, `draw_logo_animated()`, `draw_QRCode()`, `parse_version()`, `chain()`. |
| `uQR.py` | QR code generation library (micro-QR for MicroPython). |
| `metadata.json` | App metadata for BadgeOS loader: `{ "callable": "BadgeBotApp", "name": "BadgeBot" }`. |
| `tildagon.toml` | Tildagon app manifest (name, category, author, license, version, URL, description). |
| `tests/test_smoke.py` | Pytest smoke tests: import checks, `__app_export__` consistency, version parity between `app.py` and `hexdrive.py`. |
| `dev/` | Dev tooling: `build_release.py`, `dev_requirements.txt`. |
| `.github/workflows/` | CI workflow(s). |

---

## Architecture & State Machine

### BadgeBotApp (app.py)

`BadgeBotApp` extends `app.App` (the Tildagon app base class) and uses a **state-machine** pattern:

| Constant | Value | Description |
|---|---|---|
| `STATE_INIT` | -1 | Initial / startup |
| `STATE_WARNING` | 0 | No HexDrive found warning |
| `STATE_MENU` | 1 | Main menu |
| `STATE_HELP` | 2 | Help screen |
| `STATE_RECEIVE_INSTR` | 3 | User entering motor-move sequence |
| `STATE_COUNTDOWN` | 4 | Countdown before executing moves |
| `STATE_RUN` | 5 | Executing motor-move sequence |
| `STATE_DONE` | 6 | Sequence complete |
| `STATE_CHECK` | 7 | Scanning hexpansion ports |
| `STATE_DETECTED` | 8 | Blank EEPROM detected, offer init |
| `STATE_UPGRADE` | 9 | Offer HexDrive firmware upgrade |
| `STATE_ERASE` | 10 | Offer EEPROM erase |
| `STATE_PROGRAMMING` | 11 | EEPROM write in progress |
| `STATE_REMOVED` | 12 | Hexpansion removed |
| `STATE_ERROR` | 13 | Error display |
| `STATE_MESSAGE` | 14 | General message display |
| `STATE_LOGO` | 15 | Animated logo |
| `STATE_SERVO` | 16 | Servo test UI |
| `STATE_STEPPER` | 17 | Stepper test UI |
| `STATE_SETTINGS` | 18 | Settings editor |

State transitions are managed in `update(delta)` → `_update_main_application(delta)` which dispatches to per-state handlers like `_update_state_warning()`, `_update_state_receive_instr()`, etc.

Drawing follows the same pattern: `draw(ctx)` dispatches to `_draw_receive_instr()`, `_draw_state_stepper()`, `_draw_state_servo()`, etc.

### LineFollowerApp (linefollower.py)

Extended variant adding `STATE_FOLLOWER = 19` for line-following mode plus gesture control via the GR10-30. Uses `LineSensor` class for reflectance sensor reading and adds menu items for the line follower.

### Event-Driven Architecture

The app uses the Tildagon **eventbus** for:
- `HexpansionInsertionEvent` / `HexpansionRemovalEvent` — hot-plug hexpansion detection
- `RequestForegroundPushEvent` / `RequestForegroundPopEvent` — focus management
- `PatternDisable` / `PatternEnable` — LED pattern control
- `ButtonUpEvent` — button unpress handling (used in instruction-entry mode)

Button input uses the `Buttons` helper with types: `UP`, `DOWN`, `LEFT`, `RIGHT`, `CANCEL`, `CONFIRM` and supports long-press detection and auto-repeat with acceleration.

---

## Key Classes

### `BadgeBotApp` (app.py, line 209)
Main app class. Key methods:

| Method | Description |
|---|---|
| `update(delta)` | Main update loop tick; routes to state handlers |
| `draw(ctx)` | Main draw; routes to state-specific draw methods |
| `background_update(delta)` | Keep-alive and periodic tasks |
| `_apply_fwd_dir(output)` | Negates all motor outputs when `fwd_dir=1`; called at every `set_motors` callsite so the setting applies to both programmed moves and auto drive |
| `_set_direction_leds(direction)` | Lights the two LEDs corresponding to the given direction, rotated by `front_face` |
| `scan_ports()` | Scan all 6 hexpansion ports for HexDrives |
| `check_port_for_hexdrive(port)` | Check if a specific port has a HexDrive |
| `update_app_in_eeprom(port, addr)` | Write HexDrive firmware to EEPROM |
| `prepare_eeprom(port, addr)` | Initialize blank EEPROM as HexDrive |
| `erase_eeprom(port, addr)` | Erase EEPROM back to 0xFF |
| `find_hexdrive_app(port)` | Find the running HexDriveApp instance for a port |
| `set_menu(menu_name)` | Switch between "main" and "settings" menus |
| `reset_robot()` | Clear instructions and reset motor state |
| `get_current_power_level(delta)` | Ramp motor power with acceleration limiting |
| `finalize_instruction()` | Complete current instruction and advance |
| `reset_servo()` | Reset all servos to defaults |

### `HexDriveApp` (hexdrive.py, line 34)
Runs on the EEPROM per-hexpansion. Key methods:

| Method | Description |
|---|---|
| `initialise()` | Set up pins, detect HexDrive type, init PWM |
| `deinitialise()` | Release PWM, turn off power |
| `set_power(state)` | Enable/disable SMPSU boost converter |
| `get_booster_power()` | Check if external power is present |
| `set_motors(outputs)` | Set motor power levels (tuple of duty cycles) |
| `set_servoposition(channel, position)` | Set servo pulse width in µs |
| `set_servocentre(centre, channel)` | Set servo trim/centre |
| `set_freq(freq, channel)` | Set PWM frequency |
| `set_pwm(duty_cycles)` | Raw PWM duty cycle control |
| `motor_step(phase)` | Drive stepper to specific phase |
| `motor_release()` | De-energise stepper coils |
| `background_update(delta)` | Keep-alive watchdog; zeros outputs on timeout |

### `Stepper` (app.py, line 1929)
Software stepper motor controller using a hardware `Timer` for step timing.

| Method | Description |
|---|---|
| `step(d)` | Single step in direction d (+1/-1) |
| `free_run(d)` | Continuous stepping via timer |
| `track_target()` | Move towards target position via timer |
| `stop()` | Stop timer and halt stepping |
| `speed(sps)` | Set speed in full steps per second |
| `target(t)` | Set target position in half-steps |
| `enable(e)` / `is_enabled()` | Enable/disable stepper |

### `Instruction` (app.py, line 2221)
Represents a single movement command in a user-programmed sequence.

| Method | Description |
|---|---|
| `press_type` | Direction (`UP`/`DOWN`/`LEFT`/`RIGHT`) |
| `inc()` | Increase repetition count |
| `directional_power_tuple(power)` | Get motor power tuple for this direction |
| `directional_duration(settings)` | Get duration based on direction type |
| `make_power_plan(settings)` | Generate time-power schedule for execution |

### `MySetting` (app.py, line 2141)
Persistable setting with min/max bounds and auto-repeat increment/decrement.

### `HexDriveType` (app.py, line 2131)
Describes a HexDrive variant (PID, motor count, servo count, stepper count, name).

### `StepperMode` / `ServoMode` (app.py, lines 164/186)
Enum-like classes for stepper (OFF/POSITION/SPEED) and servo (OFF/TRIM/POSITION/SCANNING) modes.

---

## Sensor Subsystem

### SensorBase (sensors/sensor_base.py)
Abstract base class. Subclasses must implement:
- `I2C_ADDR: int` — 7-bit I²C address
- `NAME: str` — display name
- `_init() -> bool` — hardware init
- `_measure() -> dict` — take measurement, return `{label: value_str}`
- `_shutdown()` — optional power-down

Provides helpers: `_write_reg()`, `_read_reg()`, `_read_u8()`, `_read_u16_le()`, `_read_u16_be()`, `_write_u8()`.

### Supported Sensors

| Driver | Sensor | I²C Addr | Measurements |
|---|---|---|---|
| `vl53l0x.py` | VL53L0X ToF | 0x29 | distance (mm) |
| `vl6180x.py` | VL6180X ToF + ALS | 0x29 | range (mm), lux |
| `bme280.py` | BME280 Env | 0x76 | temp (°C), pressure (hPa), humidity (%RH) |
| `apds9960.py` | APDS-9960 Gesture/Colour | 0x39 | proximity, RGBC |
| `tcs3472.py` | TCS3472 Colour | 0x29 | RGBC, CCT, lux |
| `tcs3439.py` | TCS3439 Colour | 0x39 | RGBW, lux |

### SensorManager (sensor_manager.py)
- `open(port)` — Open I²C, scan, auto-instantiate matching sensor drivers
- `close()` — Shutdown all sensors, release bus
- `read_current()` — Read from selected sensor
- `next_sensor()` / `prev_sensor()` — Cycle through found sensors
- `select_sensor(name)` — Select by name

---

## HexDrive Hexpansion

The HexDrive is a custom hexpansion PCB with:
- **4 high-side PWM outputs** for motors/servos (via badge HS pins)
- **2 low-side pins**: one for SMPSU enable, one for power-detect
- **EEPROM** (8 KB, I²C address 0x50, 16-bit addressing) storing the hexpansion header and `hexdrive.py` (as `app.py`/`.mpy`)
- **SMPSU** boost converter for motor power from USB/battery

### HexDrive Types (Flavours)

| PID | Name | Motors | Servos | Steppers | PWM Channels Used |
|---|---|---|---|---|---|
| 0xCBCA | 2 Motor | 2 | 0 | 0 | 2 (trick: swap active signal on direction change) |
| 0xCBCC | 4 Servo | 0 | 4 | 0 | 4 |
| 0xCBCD | 1 Mot 2 Srvo | 1 | 2 | 0 | 3 |
| 0xCBCE | Stepper | 0 | 0 | 1 | 4 |
| 0xCBCB | Unknown | 2 | 4 | 0 | 4 |

ESP32 has 8 PWM channels total, so max 2 "4-channel" HexDrives or 4 "2 Motor" HexDrives simultaneously.

---

## Settings System

Settings are stored as `MySetting` objects in `self._settings` dict. Each has a default, min, and max. Settings are persisted using the badge `settings` module.

| Key | Default | Description |
|---|---|---|
| `acceleration` | 7500 | Motor power ramp rate per tick |
| `max_power` | 65535 | Maximum PWM duty cycle |
| `drive_step_ms` | 50 | Duration per drive step |
| `turn_step_ms` | 20 | Duration per turn step |
| `servo_step` | 10 | Servo pulse increment (µs) |
| `servo_range` | 1000 | Servo range (±µs from centre) |
| `servo_period` | 20 | Servo PWM period (ms) |
| `brightness` | 1.0 | LED brightness |
| `logging` | False | Console logging |
| `erase_slot` | 0 | Slot to offer erase (0 = none) |
| `step_max_pos` | 3100 | Stepper max position (half-steps) |
| `fwd_dir` | 0 | Motor direction: `0`=Normal (HexDrive faces away from robot front), `1`=Reversed (HexDrive faces toward robot front). Applied by `_apply_fwd_dir()` at every `set_motors` call — affects programmed moves **and** auto drive. Display labels: `Normal` / `Reverse`. |
| `front_face` | 0 | Which physical face of the badge is the robot's front, for LED indicators only (does **not** affect motors). 12 positions clockwise: `0`=BtnA (corner between slot 6 & 1, default top), `1`=Slot 1, `2`=BtnB … `10`=BtnF, `11`=Slot 6. Corners A–F match the badge's physical buttons. |

### Orientation System Design

The two orientation settings are **independent** and affect different subsystems:

- **`fwd_dir`** — a single hardware-polarity switch applied at `set_motors`. Negating every output element correctly handles forward, reverse, and both turn directions simultaneously (since left/right is relative to forward). It also applies to auto drive (cruise, scan spin, turn) without any extra code in those paths. Display: `Normal` / `Reverse`.

- **`front_face`** — a 0–11 ring position (each step = 30° clockwise). `_set_direction_leds()` offsets the lit LED pair from the front position: forward = `front_face`, right = `+2`, backward = `+6`, left = `+8`. Position 0 is the top corner (BtnA, between slot 6 & slot 1). Motor logic is not affected.

---

## Important Constants

```python
_TICK_MS        = 10        # Smallest motor update interval (ms)
_USER_DRIVE_MS  = 50        # Default drive step duration (ms)
_USER_TURN_MS   = 20        # Default turn step duration (ms)
_LONG_PRESS_MS  = 750       # Long-press threshold (ms)
_RUN_COUNTDOWN_MS = 5000    # Countdown before execute (ms)
_AUTO_REPEAT_MS = 200       # Button auto-repeat interval (ms)

_EEPROM_ADDR           = 0x50
_EEPROM_PAGE_SIZE      = 32
_EEPROM_TOTAL_SIZE     = 8192    # 64 Kbit = 8 KB

_MAX_POWER             = 65535   # Full-scale PWM duty
_SERVO_DEFAULT_CENTRE  = 1500    # Servo centre (µs)
_MAX_SERVO_RANGE       = 1400    # Maximum servo range (µs)

# Orientation constants
_FWD_DIR_DEFAULT    = 0
_FWD_DIR_LABELS     = ("Normal", "Reverse")   # display labels for fwd_dir setting
_FRONT_FACE_DEFAULT = 0
_FRONT_FACE_LABELS  = (                        # display labels for front_face setting
    "BtnA",   # 0  - corner between slot 6 & slot 1 (default top)
    "Slot 1",  # 1
    "BtnB",   # 2  - corner between slot 1 & slot 2
    "Slot 2",  # 3
    "BtnC",   # 4
    "Slot 3",  # 5
    "BtnD",   # 6  - corner between slot 3 & slot 4 (bottom)
    "Slot 4",  # 7
    "BtnE",   # 8
    "Slot 5",  # 9
    "BtnF",   # 10
    "Slot 6",  # 11
)
```

---

## Badge Framework APIs Used

The app depends on these Tildagon/BadgeOS APIs:

| Module | Usage |
|---|---|
| `app.App` | Base application class (`update()`, `draw()`, `background_update()`) |
| `events.input` | `Buttons`, `Button`, `ButtonUpEvent`, `BUTTON_TYPES` |
| `app_components` | `Menu`, `Notification`, display tokens |
| `system.eventbus` | Publish/subscribe event system |
| `system.hexpansion.*` | Hexpansion detection, header read/write, block devices |
| `system.scheduler` | App lifecycle events (foreground push/pop, stop) |
| `system.patterndisplay.events` | LED pattern enable/disable |
| `machine` | `I2C`, `PWM`, `Pin`, `Timer` |
| `tildagonos` | LED control (`tildagonos.leds[n]`) |
| `settings` | Persistent key-value storage |
| `ota` | `get_version()` for badge firmware version check |
| `vfs` | Virtual filesystem for EEPROM mounting |
| `ctx` | 2D drawing context (rectangles, text, arcs, etc.) |

---

## Development Notes

### Coding Patterns
- **MicroPython target**: code must run on ESP32 with limited RAM. Avoid large allocations, prefer generators (`chain()`), and use lazy imports where possible.
- **Relative imports**: within the package use `from .utils import ...`, `from .sensors import ...`.
- **`__app_export__`**: every module that can be loaded as an app must set this to the main class at module level.
- **`_IS_SIMULATOR`**: `sys.platform != "esp32"` — use this to guard hardware-only code paths. The Tildagon badge uses an ESP32-S3 so `sys.platform` is `"esp32"` on real hardware.
- **Keep-alive pattern**: `HexDriveApp.background_update()` zeros outputs if no command arrives within `_keep_alive_period` ms — the main app must send updates faster than this.

### Testing
```bash
cd sim/apps/BadgeBot
pytest tests/
```
Smoke tests verify imports, `__app_export__` consistency, and that `CURRENT_APP_VERSION` in `app.py` matches `APP_VERSION` in `hexdrive.py`.

### HexDrive Version Bump (MANDATORY when hexdrive.py changes)

Whenever `hexdrive.py` is modified, you **must** perform all three steps:

1. **Bump `APP_VERSION`** in `hexdrive.py` (integer at the top of the file).
2. **Bump `CURRENT_APP_VERSION`** in `app.py` (and `linefollower.py` if present) to the **same** integer. This is how the app detects that the EEPROM firmware is out-of-date and prompts the user to reprogram.
3. **Rebuild the `.mpy`** by running from the BadgeBot directory:
   ```bash
   mpy-cross -v hexdrive.py
   ```
   This produces `hexdrive.mpy`, which is what actually gets written to the HexDrive EEPROM.

The smoke tests (`tests/test_smoke.py::test_app_versions_match`) will fail if the versions diverge.

### Current Development Branch — Sensor Integration

This branch is adding **sensor capabilities** to BadgeBot:

- **Current approach**: sensors sit on a **separate "dumb" hexpansion** (one without an EEPROM) plugged into another slot. The `SensorManager` and `sensors/` package handle I²C discovery and reading on that port independently of the HexDrive.
- **Future goal**: integrate sensor hardware directly onto a **new revision of the HexDrive PCB**, so a single hexpansion provides both motor driving and sensor input. When that happens, `hexdrive.py` will need new initialisation and reading logic for the on-board sensors, which will trigger a version bump and EEPROM reprogram cycle as described above.
- **Design principle**: keep the sensor abstraction (`SensorBase` / `SensorManager`) cleanly separated from motor control so that the transition from "separate hexpansion" to "on-board sensors" requires minimal refactoring in `app.py`.

### Editing Scope
**Only edit files inside `sim/apps/BadgeBot/`** unless explicitly instructed otherwise. The surrounding `badge-2024-software` repo is the badge firmware — treat it as read-only context.

### Key Relationships
```
BadgeBotApp (app.py)
  ├── finds HexDriveApp instances via scheduler.apps
  ├── programs hexdrive.py onto EEPROM
  ├── sends motor/servo commands to HexDriveApp
  ├── uses SensorManager for sensor test mode
  └── uses utils.py for drawing helpers

HexDriveApp (hexdrive.py)
  ├── runs from EEPROM, managed by BadgeOS
  ├── owns PWM outputs, SMPSU control
  └── implements keep-alive safety timeout

LineFollowerApp (linefollower.py)
  ├── extended fork of BadgeBotApp
  ├── adds LineSensor, gesture sensor (GR10-30)
  └── adds STATE_FOLLOWER for autonomous driving

SensorManager (sensor_manager.py)
  └── auto-discovers sensors via sensors/ package
```
