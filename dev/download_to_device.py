"""Incrementally compile and deploy BadgeBot runtime modules to a test device.

This script improves on the old batch file by:
- compiling only modules whose source has changed;
- uploading only artifacts whose content has changed;
- reporting each action with clear status and detailed subprocess errors.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path


DEFAULT_APP_DIR_ON_DEVICE = ":apps/LineFollower"
STATE_DIR = Path(".deploy_state")
STATE_PATH = STATE_DIR / "test_device_download_state.json"


@dataclass(frozen=True)
class ModuleSpec:
    source: Path
    artifact: Path


# Add new runtime modules here as the project grows.
MODULES: tuple[ModuleSpec, ...] = (
    ModuleSpec(Path("hexdrive.py"), Path("hexdrive.mpy")),
    ModuleSpec(Path("app.py"), Path("app.mpy")),
    ModuleSpec(Path("autotune.py"), Path("autotune.mpy")),
    ModuleSpec(Path("autotune_mgr.py"), Path("autotune_mgr.mpy")),
    ModuleSpec(Path("utils.py"), Path("utils.mpy")),
    ModuleSpec(Path("settings_mgr.py"), Path("settings_mgr.mpy")),
    ModuleSpec(Path("hexpansion_mgr.py"), Path("hexpansion_mgr.mpy")),
    ModuleSpec(Path("line_follow.py"), Path("line_follow.mpy")),
    ModuleSpec(Path("motor_moves.py"), Path("motor_moves.mpy")),
    ModuleSpec(Path("servo_test.py"), Path("servo_test.mpy")),
    ModuleSpec(Path("stepper_test.py"), Path("stepper_test.mpy")),
    ModuleSpec(Path("motor_controller.py"), Path("motor_controller.mpy")),
    ModuleSpec(Path("sensor_manager.py"), Path("sensor_manager.mpy")),
    ModuleSpec(Path("autodrive.py"), Path("autodrive.mpy")),
    ModuleSpec(Path("sensors/__init__.py"), Path("sensors/__init__.mpy")),
    ModuleSpec(Path("sensors/sensor_base.py"), Path("sensors/sensor_base.mpy")),
    ModuleSpec(Path("sensors/apds9960.py"), Path("sensors/apds9960.mpy")),
    ModuleSpec(Path("sensors/bme280.py"), Path("sensors/bme280.mpy")),
    ModuleSpec(Path("sensors/tcs3439.py"), Path("sensors/tcs3439.mpy")),
    ModuleSpec(Path("sensors/tcs3472.py"), Path("sensors/tcs3472.mpy")),
    ModuleSpec(Path("sensors/vl53l0x.py"), Path("sensors/vl53l0x.mpy")),
    ModuleSpec(Path("sensors/vl6180x.py"), Path("sensors/vl6180x.mpy")),
)


class CommandFailed(RuntimeError):
    pass


def _log(level: str, message: str) -> None:
    print(f"[{level}] {message}")


def _sha256(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as file:
        while True:
            chunk = file.read(1024 * 1024)
            if not chunk:
                break
            hasher.update(chunk)
    return hasher.hexdigest()


def _load_state(path: Path) -> dict[str, dict[str, str]]:
    if not path.exists():
        return {"compiled": {}, "uploaded": {}}

    try:
        with path.open("r", encoding="utf-8") as file:
            state = json.load(file)
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Could not read state file {path}: {exc}") from exc

    if not isinstance(state, dict):
        raise RuntimeError(f"State file {path} is not a valid JSON object")

    state.setdefault("compiled", {})
    state.setdefault("uploaded", {})
    return state


def _save_state(path: Path, state: dict[str, dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(state, file, indent=2, sort_keys=True)
        file.write("\n")


def _run_command(command: list[str], *, dry_run: bool) -> None:
    quoted = " ".join(f'"{part}"' if " " in part else part for part in command)
    _log("CMD", quoted)

    if dry_run:
        return

    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        raise CommandFailed(
            "\n".join(
                [
                    f"Command failed with exit code {completed.returncode}",
                    f"Command: {quoted}",
                    f"STDOUT:\n{completed.stdout.rstrip() or '<empty>'}",
                    f"STDERR:\n{completed.stderr.rstrip() or '<empty>'}",
                ]
            )
        )

    if completed.stdout.strip():
        _log("OUT", completed.stdout.rstrip())
    if completed.stderr.strip():
        _log("ERR", completed.stderr.rstrip())


def _ensure_repo_root() -> Path:
    repo_root = Path(__file__).resolve().parent.parent
    os.chdir(repo_root)
    return repo_root


def _validate_sources() -> None:
    missing = [str(spec.source) for spec in MODULES if not spec.source.exists()]
    if missing:
        missing_list = ", ".join(sorted(missing))
        raise FileNotFoundError(f"Missing source files: {missing_list}")


def _compile_changed_modules(
    state: dict[str, dict[str, str]],
    *,
    force: bool,
    dry_run: bool,
) -> tuple[int, int]:
    compiled = 0
    skipped = 0

    for spec in MODULES:
        source_key = spec.source.as_posix()
        source_hash = _sha256(spec.source)
        known_hash = state["compiled"].get(source_key)

        needs_compile = force or source_hash != known_hash or not spec.artifact.exists()
        if not needs_compile:
            skipped += 1
            _log("SKP", f"compile {spec.source} (source unchanged)")
            continue

        _log("INFO", f"compile {spec.source} -> {spec.artifact}")
        _run_command(
            ["mpy-cross", "-v", str(spec.source), "-o", str(spec.artifact)],
            dry_run=dry_run,
        )

        if not dry_run and not spec.artifact.exists():
            raise RuntimeError(f"mpy-cross did not produce {spec.artifact}")

        state["compiled"][source_key] = source_hash
        compiled += 1
        _log("OK ", f"compiled {spec.artifact}")

    return compiled, skipped


def _upload_changed_artifacts(
    state: dict[str, dict[str, str]],
    *,
    force: bool,
    dry_run: bool,
    mpremote_args: list[str],
    app_dir: str,
) -> tuple[int, int]:
    uploaded = 0
    skipped = 0

    for spec in MODULES:
        if not spec.artifact.exists() and not dry_run:
            raise FileNotFoundError(f"Artifact not found: {spec.artifact}")

        artifact_key = spec.artifact.as_posix()
        artifact_hash = "DRY-RUN" if dry_run else _sha256(spec.artifact)
        known_hash = state["uploaded"].get(artifact_key)

        needs_upload = force or artifact_hash != known_hash
        if not needs_upload:
            skipped += 1
            _log("SKP", f"upload {spec.artifact} (artifact unchanged)")
            continue

        destination = f"{app_dir}/{spec.artifact.name}"
        _log("INFO", f"upload {spec.artifact} -> {destination}")

        command = ["mpremote", *mpremote_args, "cp", str(spec.artifact), destination]
        _run_command(command, dry_run=dry_run)

        state["uploaded"][artifact_key] = artifact_hash
        uploaded += 1
        _log("OK ", f"uploaded {spec.artifact.name}")

    return uploaded, skipped


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compile and upload only changed modules to the local test badge. "
            "State is tracked in .deploy_state/test_device_download_state.json"
        )
    )
    parser.add_argument(
        "--force-compile",
        action="store_true",
        help="Compile all modules regardless of source hash.",
    )
    parser.add_argument(
        "--force-upload",
        action="store_true",
        help="Upload all artifacts regardless of previous deploy state.",
    )
    parser.add_argument(
        "--clear-state",
        action="store_true",
        help="Remove tracked state before running.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show actions but do not run mpy-cross or mpremote.",
    )
    parser.add_argument(
        "--mpremote-arg",
        action="append",
        default=[],
        help=(
            "Extra argument passed to mpremote before 'cp'. "
            "Can be supplied multiple times, e.g. --mpremote-arg connect --mpremote-arg COM5"
        ),
    )
    parser.add_argument(
        "--app-dir",
        default=DEFAULT_APP_DIR_ON_DEVICE,
        help=(
            "Target directory on the badge for uploaded artifacts. "
            f"Default: {DEFAULT_APP_DIR_ON_DEVICE}"
        ),
    )
    return parser.parse_args()


def main() -> int:
    options = _parse_args()
    repo_root = _ensure_repo_root()
    _log("INF", f"working directory: {repo_root}")

    try:
        _validate_sources()

        if options.clear_state and STATE_PATH.exists():
            _log("INF", f"clearing state file {STATE_PATH}")
            if not options.dry_run:
                STATE_PATH.unlink()

        state = _load_state(STATE_PATH)

        compiled, compile_skipped = _compile_changed_modules(
            state,
            force=options.force_compile,
            dry_run=options.dry_run,
        )
        uploaded, upload_skipped = _upload_changed_artifacts(
            state,
            force=options.force_upload,
            dry_run=options.dry_run,
            mpremote_args=options.mpremote_arg,
            app_dir=options.app_dir,
        )

        if not options.dry_run:
            _save_state(STATE_PATH, state)

        _log(
            "SUMMARY",
            (
                f"compiled: {compiled}, compile-skipped: {compile_skipped}, "
                f"uploaded: {uploaded}, upload-skipped: {upload_skipped}, "
                f"dry-run: {options.dry_run}"
            ),
        )
        return 0

    except CommandFailed as exc:
        _log("ERR", str(exc))
        return 1
    except Exception as exc:  # pylint: disable=broad-except
        _log("ERR", f"{type(exc).__name__}: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
