# BadgeBot Copilot Instructions

This is the canonical Copilot instructions file for BadgeBot.

Keep this file concise and evergreen:
- Do not include hardcoded line numbers.
- Do not include "currently X" snapshots that quickly go stale.
- Prefer naming symbols/files over pinning implementation trivia.

## Scope

These instructions apply to work in this package:
- `sim/apps/BadgeBot/`

Unless explicitly requested, avoid editing files outside this package.

## Project Context

BadgeBot is a MicroPython app for the EMF Camp Tildagon badge.

Core capabilities:
- HexDrive motor/servo control via hexpansion.
- Sensor probing/testing via I2C sensor drivers.
- App UI and mode switching through a manager/state-machine pattern.
- Simulator support for desktop iteration.

## Primary Files

- `app.py`: Main app (`BadgeBotApp`), state routing, menus, draw/update loops.
- `hexdrive.py`: EEPROM app (`HexDriveApp`) controlling PWM/motors/servos and keep-alive safety.
- `hexpansion_mgr.py`: Port scanning, EEPROM prep/program/erase, HexDrive lifecycle.
- `motor_controller.py`: Higher-level movement control and assisted maneuvers.
- `motor_moves.py`, `servo_test.py`, `line_follow.py`, `autotune_mgr.py`, `sensor_test.py`, `autodrive.py`: Mode managers.
- `sensor_manager.py` + `sensors/`: Sensor discovery and sensor driver implementations.
- `utils.py`: Shared UI/drawing and helper utilities.

## Architecture Guidance

`BadgeBotApp` is the orchestration layer:
- It owns state and dispatch tables for `update`, `draw`, and optional background tasks.
- It delegates mode behavior to manager classes.
- It interacts with HexDrive apps discovered/managed through the platform scheduler and hexpansion APIs.

When changing behavior:
- Prefer adding/changing manager logic rather than embedding mode-specific code in unrelated modules.
- Keep responsibilities separated: UI/rendering, state transitions, motor control, sensor IO.

## Simulator vs Hardware

BadgeBot runs in both simulator and on-badge environments.

When editing:
- Guard hardware-only paths with platform checks where needed.
- Do not assume desktop-only modules exist on-device.
- Keep simulator fakes compatible with call signatures used by app code.

## Versioning Rules (HexDrive)

If `hexdrive.py` behavior or interface changes:
1. Bump `VERSION` in `hexdrive.py`.
2. Bump the matching app-side HexDrive version constant used for compatibility checks.
3. Rebuild/update any generated `.mpy` artifact used for EEPROM programming.
4. Ensure related smoke/version tests still pass.

Never change only one side of app/HexDrive version pairing.

## Coding Conventions

- Target MicroPython constraints: keep allocations and per-frame overhead low.
- Use relative imports inside the package.
- Keep public API names stable unless a migration plan is included.
- Prefer clear, small changes over broad refactors.
- Maintain existing formatting and style in touched files.

## Testing and Validation

From `sim/apps/BadgeBot/`:

```bash
pytest tests/
```

For simulator smoke checks (from repo root):

```bash
python sim/run.py --screenshot BadgeBot.BadgeBotApp
```

After changes, verify:
- App imports cleanly.
- Simulator launches without tracebacks.
- Relevant tests pass.

## Common Pitfalls

- Updating HexDrive logic without syncing app-side compatibility version.
- Mixing simulator-only assumptions into badge runtime paths.
- Editing outside `sim/apps/BadgeBot/` without explicit requirement.
- Adding brittle docs tied to exact file lengths or line numbers.

## Documentation Maintenance

When you update architecture or behavior:
- Update this file only for durable guidance.
- Keep operational details short and actionable.
- Remove or rewrite stale statements instead of adding caveats.
