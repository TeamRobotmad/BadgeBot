"""Incrementally compile and deploy BadgeBot runtime modules to a test device.

This script improves on the old batch file by:
- compiling only modules whose source has changed;
- uploading only artifacts whose content has changed;
- reporting each action with clear status and detailed subprocess errors.

By default only warnings, errors, and the final summary are printed.
Pass ``--verbose`` to see every compile/upload action and subprocess command.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path


DEFAULT_APP_DIR_ON_DEVICE = ":apps/TeamRobotMad_BadgeBot"
STATE_DIR = Path(".deploy_state")
STATE_PATH = STATE_DIR / "test_device_download_state.json"
MPREMOTE_COMMAND_TIMEOUT = 20
MPREMOTE_PROBE_TIMEOUT = 5
MPREMOTE_PROBE_MARKER = "__badgebot_mpremote_ok__"


@dataclass(frozen=True)
class ModuleSpec:
    source: Path
    artifact: Path


# Add new runtime modules here as the project grows.
MODULES: tuple[ModuleSpec, ...] = (
    ModuleSpec(Path("EEPROM/hexdrive.py"), Path("EEPROM/hexdrive.mpy")),
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
    ModuleSpec(Path("sensor_test.py"), Path("sensor_test.mpy")),
    ModuleSpec(Path("autodrive.py"), Path("autodrive.mpy")),
    ModuleSpec(Path("sensors/__init__.py"), Path("sensors/__init__.mpy")),
    ModuleSpec(Path("sensors/sensor_base.py"), Path("sensors/sensor_base.mpy")),
    ModuleSpec(Path("sensors/tcs3430.py"), Path("sensors/tcs3430.mpy")),
    ModuleSpec(Path("sensors/tcs3472.py"), Path("sensors/tcs3472.mpy")),
    ModuleSpec(Path("sensors/vl53l0x.py"), Path("sensors/vl53l0x.mpy")),
    ModuleSpec(Path("sensors/vl6180x.py"), Path("sensors/vl6180x.mpy")),
    ModuleSpec(Path("sensors/opt4048.py"), Path("sensors/opt4048.mpy")),
    ModuleSpec(Path("sensors/ina226.py"), Path("sensors/ina226.mpy")),
)

# Files copied to the device as-is (no compilation).
STATIC_FILES: tuple[Path, ...] = (
    Path("metadata.json"),
    Path("tildagon.toml"),
)


class CommandFailed(RuntimeError):
    pass


# Levels always shown regardless of verbosity.
_ALWAYS_SHOWN = frozenset({"ERR", "WARN", "SUMMARY"})

# Set to True by main() when --verbose is passed.
_verbose: bool = False


def _log(level: str, message: str) -> None:
    if _verbose or level.strip() in _ALWAYS_SHOWN:
        print(f"[{level}] {message}")


def _format_size(size_bytes: int) -> str:
    """Return a human-readable representation of *size_bytes*."""
    if size_bytes < 1024:
        return f"{size_bytes} B"
    if size_bytes < 1024 * 1024:
        return f"{size_bytes / 1024:.1f} KB"
    return f"{size_bytes / (1024 * 1024):.1f} MB"


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


def _format_command(command: list[str]) -> str:
    return " ".join(f'"{part}"' if " " in part else part for part in command)


def _run_command(
    command: list[str],
    *,
    dry_run: bool,
    timeout: int | None = None,
) -> subprocess.CompletedProcess[str] | None:
    quoted = _format_command(command)
    _log("CMD", quoted)

    if dry_run:
        return None

    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as exc:
        stdout = (exc.stdout or "").rstrip() or "<empty>"
        stderr = (exc.stderr or "").rstrip() or "<empty>"
        raise CommandFailed(
            "\n".join(
                [
                    f"Command timed out after {timeout} seconds",
                    f"Command: {quoted}",
                    f"STDOUT:\n{stdout}",
                    f"STDERR:\n{stderr}",
                ]
            )
        ) from exc

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
    return completed


def _find_connect_arg(mpremote_args: list[str]) -> int | None:
    for index, arg in enumerate(mpremote_args[:-1]):
        if arg == "connect":
            return index
    return None


def _list_mpremote_devices() -> list[str]:
    completed = _run_command(
        ["mpremote", "devs"],
        dry_run=False,
        timeout=MPREMOTE_PROBE_TIMEOUT,
    )
    if completed is None:
        return []

    devices: list[str] = []
    for line in completed.stdout.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        first_field = stripped.split()[0]
        if re.match(r"^(COM\d+|/dev/\S+|/tty\S+)$", first_field, re.IGNORECASE):
            devices.append(first_field)
    return devices


def _probe_mpremote_device(port: str) -> bool:
    command = [
        "mpremote",
        "connect",
        port,
        "exec",
        f"print('{MPREMOTE_PROBE_MARKER}')",
    ]
    _log("INFO", f"probing mpremote device on {port}")
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
            timeout=MPREMOTE_PROBE_TIMEOUT,
        )
    except subprocess.TimeoutExpired:
        _log("WARN", f"skipping {port}: mpremote probe timed out after {MPREMOTE_PROBE_TIMEOUT} seconds")
        return False

    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip() or "probe failed"
        _log("WARN", f"skipping {port}: {detail}")
        return False

    if MPREMOTE_PROBE_MARKER not in completed.stdout:
        _log("WARN", f"skipping {port}: probe did not return the expected marker")
        return False

    _log("INFO", f"using verified mpremote device on {port}")
    return True


def _resolve_mpremote_args(mpremote_args: list[str], *, dry_run: bool) -> list[str]:
    if dry_run:
        return list(mpremote_args)

    connect_index = _find_connect_arg(mpremote_args)
    if connect_index is not None:
        if connect_index + 1 >= len(mpremote_args):
            raise RuntimeError("--mpremote-arg connect requires a device argument")
        port = mpremote_args[connect_index + 1]
        if not _probe_mpremote_device(port):
            raise RuntimeError(f"Configured mpremote device {port} did not respond to a probe")
        return list(mpremote_args)

    devices = _list_mpremote_devices()
    if not devices:
        raise RuntimeError("No candidate devices were returned by 'mpremote devs'")

    _log("INFO", f"mpremote reported {len(devices)} candidate device(s): {', '.join(devices)}")
    for port in devices:
        if _probe_mpremote_device(port):
            return ["connect", port, *mpremote_args]

    raise RuntimeError(
        "No responsive MicroPython device was found from 'mpremote devs'; "
        "use --mpremote-arg connect --mpremote-arg <PORT> to force a specific port if needed"
    )


def _ensure_repo_root() -> Path:
    repo_root = Path(__file__).resolve().parent.parent
    os.chdir(repo_root)
    return repo_root


def _ensure_device_dir(dir_path: str, *, mpremote_args: list[str], dry_run: bool) -> None:
    """Create a directory on the device if it does not already exist.

    dir_path uses mpremote ':path' notation; the leading ':' is stripped to
    obtain the on-device absolute path. Uses os.mkdir segment-by-segment
    because MicroPython may not provide os.makedirs.
    """
    _log("INFO", f"ensuring device directory: {dir_path}")
    on_device = dir_path.lstrip(":")
    if not on_device.startswith("/"):
        on_device = "/" + on_device
    safe_path = on_device.replace("'", "\\'")
    exec_code = (
        "import os\n"
        f"p='{safe_path}'\n"
        "cur=''\n"
        "for part in [x for x in p.split('/') if x]:\n"
        "    cur += '/' + part\n"
        "    try:\n"
        "        os.stat(cur)\n"
        "    except OSError:\n"
        "        os.mkdir(cur)"
    )
    _run_command(
        ["mpremote", *mpremote_args, "exec", exec_code],
        dry_run=dry_run,
        timeout=MPREMOTE_COMMAND_TIMEOUT,
    )


def _ensure_device_dirs(app_dir: str, *, mpremote_args: list[str], dry_run: bool) -> None:
    """Ensure every directory needed for uploads exists on the device.

    Collects the app root dir plus any subdirectories implied by artifact and
    static-file paths, de-duplicates, and creates them in sorted order so that
    parent directories are always created before their children.
    """
    dirs: set[str] = {app_dir}
    for spec in MODULES:
        if spec.artifact.parent != Path("."):
            dirs.add(f"{app_dir}/{spec.artifact.parent.as_posix()}")
    for path in STATIC_FILES:
        if path.parent != Path("."):
            dirs.add(f"{app_dir}/{path.parent.as_posix()}")
    for dir_path in sorted(dirs):
        _ensure_device_dir(dir_path, mpremote_args=mpremote_args, dry_run=dry_run)


def _validate_sources() -> None:
    missing = [str(spec.source) for spec in MODULES if not spec.source.exists()]
    missing += [str(f) for f in STATIC_FILES if not f.exists()]
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


def _get_device_files(
    *,
    app_dir: str,
    mpremote_args: list[str],
) -> set[str] | None:
    """Return the set of files present under *app_dir* on the device.

    Returns a set of POSIX paths relative to *app_dir*, e.g.
    ``{'app.mpy', 'sensors/__init__.mpy'}``.  Returns *None* on any error
    (connection failure, mpremote not found, etc.) so the caller can fall back
    to hash-only comparison for that run.

    A single ``mpremote exec`` call walks the directory tree on the device.
    """
    on_device = app_dir.lstrip(":")
    if not on_device.startswith("/"):
        on_device = "/" + on_device

    safe_path = on_device.replace("'", "\\'")
    exec_code = (
        "import os,json\n"
        "def _ls(p):\n"
        "    r=[]\n"
        "    try:\n"
        "        es=os.listdir(p)\n"
        "    except OSError:\n"
        "        return r\n"
        "    for e in es:\n"
        "        fp=p+'/'+e\n"
        "        try:\n"
        "            s=os.stat(fp)\n"
        "            if s[0]&0x4000: r.extend(_ls(fp))\n"
        "            else: r.append(fp)\n"
        "        except OSError: pass\n"
        "    return r\n"
        f"print(json.dumps(_ls('{safe_path}')))"
    )

    command = ["mpremote", *mpremote_args, "exec", exec_code]
    quoted = " ".join(f'"{p}"' if " " in p else p for p in command)
    _log("CMD", quoted)

    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
            timeout=MPREMOTE_COMMAND_TIMEOUT,
        )
    except subprocess.TimeoutExpired:
        _log(
            "WARN",
            "could not list device files because mpremote timed out; will upload all unverified",
        )
        return None
    if completed.returncode != 0:
        _log("WARN", f"could not list device files (will upload all unverified): "
                     f"{completed.stderr.strip() or completed.stdout.strip()}")
        return None

    match = re.search(r"\[.*?\]", completed.stdout, re.DOTALL)
    if not match:
        _log("WARN", "could not parse device file list (will upload all unverified)")
        return None

    try:
        device_paths: list[str] = json.loads(match.group())
    except json.JSONDecodeError as exc:
        _log("WARN", f"could not decode device file list (will upload all unverified): {exc}")
        return None

    prefix = on_device.rstrip("/") + "/"
    result: set[str] = set()
    for p in device_paths:
        p = p.replace("\\", "/")
        if p.startswith(prefix):
            result.add(p[len(prefix):])

    _log("INFO", f"device has {len(result)} file(s) under {app_dir}")
    return result


def _upload_changed_artifacts(
    state: dict[str, dict[str, str]],
    *,
    force: bool,
    dry_run: bool,
    mpremote_args: list[str],
    app_dir: str,
    device_files: set[str] | None,
) -> tuple[int, int, int]:
    uploaded = 0
    skipped = 0
    total_bytes = 0

    for spec in MODULES:
        if not spec.artifact.exists() and not dry_run:
            raise FileNotFoundError(f"Artifact not found: {spec.artifact}")

        artifact_key = spec.artifact.as_posix()
        artifact_hash = "DRY-RUN" if dry_run else _sha256(spec.artifact)
        known_hash = state["uploaded"].get(artifact_key)

        on_device = device_files is None or artifact_key in device_files
        needs_upload = force or artifact_hash != known_hash or not on_device
        if not needs_upload:
            skipped += 1
            _log("SKP", f"upload {spec.artifact} (artifact unchanged, present on device)")
            continue

        if not on_device:
            _log("INFO", f"upload {spec.artifact} (missing from device)")
        else:
            _log("INFO", f"upload {spec.artifact} -> {app_dir}/{spec.artifact.as_posix()}")

        destination = f"{app_dir}/{spec.artifact.as_posix()}"
        command = ["mpremote", *mpremote_args, "cp", str(spec.artifact), destination]
        _run_command(command, dry_run=dry_run, timeout=MPREMOTE_COMMAND_TIMEOUT)

        state["uploaded"][artifact_key] = artifact_hash
        uploaded += 1
        if not dry_run and spec.artifact.exists():
            total_bytes += spec.artifact.stat().st_size
        _log("OK ", f"uploaded {spec.artifact}")

    return uploaded, skipped, total_bytes


def _upload_changed_static_files(
    state: dict[str, dict[str, str]],
    *,
    force: bool,
    dry_run: bool,
    mpremote_args: list[str],
    app_dir: str,
    device_files: set[str] | None,
) -> tuple[int, int, int]:
    """Copy static (non-compiled) files to the device unchanged."""
    uploaded = 0
    skipped = 0
    total_bytes = 0

    for path in STATIC_FILES:
        if not path.exists() and not dry_run:
            raise FileNotFoundError(f"Static file not found: {path}")

        file_key = path.as_posix()
        file_hash = "DRY-RUN" if dry_run else _sha256(path)
        known_hash = state["uploaded"].get(file_key)

        on_device = device_files is None or file_key in device_files
        needs_upload = force or file_hash != known_hash or not on_device
        if not needs_upload:
            skipped += 1
            _log("SKP", f"upload {path} (file unchanged, present on device)")
            continue

        if not on_device:
            _log("INFO", f"upload {path} (missing from device)")
        else:
            _log("INFO", f"upload {path} -> {app_dir}/{path.as_posix()}")

        destination = f"{app_dir}/{path.as_posix()}"
        command = ["mpremote", *mpremote_args, "cp", str(path), destination]
        _run_command(command, dry_run=dry_run, timeout=MPREMOTE_COMMAND_TIMEOUT)

        state["uploaded"][file_key] = file_hash
        uploaded += 1
        if not dry_run and path.exists():
            total_bytes += path.stat().st_size
        _log("OK ", f"uploaded {path.name}")

    return uploaded, skipped, total_bytes


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
        "--verbose",
        action="store_true",
        help=(
            "Show detailed progress: every compile/upload action, skipped files, "
            "and subprocess commands. By default only warnings, errors, and the "
            "final summary are printed."
        ),
    )
    parser.add_argument(
        "--mpremote-arg",
        action="append",
        default=[],
        help=(
            "Extra argument passed to mpremote before 'cp'. "
            "If omitted, the script auto-detects and probes candidate devices from 'mpremote devs'. "
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
    global _verbose  # noqa: PLW0603
    options = _parse_args()
    _verbose = options.verbose
    repo_root = _ensure_repo_root()
    _log("INF", f"working directory: {repo_root}")

    try:
        _validate_sources()

        if options.clear_state and STATE_PATH.exists():
            _log("INF", f"clearing state file {STATE_PATH}")
            if not options.dry_run:
                STATE_PATH.unlink()

        state = _load_state(STATE_PATH)

        resolved_mpremote_args = _resolve_mpremote_args(
            options.mpremote_arg,
            dry_run=options.dry_run,
        )

        _ensure_device_dirs(
            options.app_dir,
            mpremote_args=resolved_mpremote_args,
            dry_run=options.dry_run,
        )
        compiled, compile_skipped = _compile_changed_modules(
            state,
            force=options.force_compile,
            dry_run=options.dry_run,
        )
        device_files = (
            None
            if options.dry_run
            else _get_device_files(
                app_dir=options.app_dir,
                mpremote_args=resolved_mpremote_args,
            )
        )
        uploaded, upload_skipped, artifact_bytes = _upload_changed_artifacts(
            state,
            force=options.force_upload,
            dry_run=options.dry_run,
            mpremote_args=resolved_mpremote_args,
            app_dir=options.app_dir,
            device_files=device_files,
        )
        static_uploaded, static_skipped, static_bytes = _upload_changed_static_files(
            state,
            force=options.force_upload,
            dry_run=options.dry_run,
            mpremote_args=resolved_mpremote_args,
            app_dir=options.app_dir,
            device_files=device_files,
        )

        if not options.dry_run:
            _save_state(STATE_PATH, state)

        total_uploaded = uploaded + static_uploaded
        total_bytes = artifact_bytes + static_bytes
        summary_parts = [
            f"compiled: {compiled}",
            f"compile-skipped: {compile_skipped}",
            f"modules-uploaded: {uploaded}",
            f"modules-skipped: {upload_skipped}",
            f"static-uploaded: {static_uploaded}",
            f"static-skipped: {static_skipped}",
        ]
        if total_uploaded and not options.dry_run:
            summary_parts.append(f"uploaded: {_format_size(total_bytes)}")
        summary_parts.append(f"dry-run: {options.dry_run}")
        _log("SUMMARY", ", ".join(summary_parts))
        return 0

    except CommandFailed as exc:
        _log("ERR", str(exc))
        return 1
    except Exception as exc:  # pylint: disable=broad-except
        _log("ERR", f"{type(exc).__name__}: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
