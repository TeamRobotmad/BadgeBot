"""Minify and compile vendor modules for MicroPython deployment.

Pipeline:
  1. Rename internal instance attributes to short names via AST transform
     (source stays readable; only the build artefact is shrunk)
  2. Strip docstrings via python-minifier
     (--remove-literal-statements --no-hoist-literals)
  3. Compile with mpy-cross -O2

Standalone – minify all configured vendor modules and show size comparison:
    python dev/minify.py

Per-file – used by download_to_device.py for incremental builds:
    python dev/minify.py --source vendor/HexDrive2/hexdrive2.py --artifact EEPROM/hexdrive2.mpy
"""
import argparse
import ast
import string
import subprocess
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parent  # sim/apps/BadgeBot/
MPY_CROSS = (
    ROOT / ".venv" / "Lib" / "site-packages" / "mpy_cross" / "archive" / "v1.20.0" / "mpy-cross.exe"
)


# ── per-file preserve sets ────────────────────────────────────────────────────
# Names that must NOT be renamed – framework hooks and externally visible API.

_PRESERVE_HEXDRIVE2: frozenset[str] = frozenset({
    # BadgeOS app lifecycle
    "background_update",
    "__init__",
    # Public API called by BadgeBot
    "initialise", "get_status", "set_logging", "set_power",
    "set_dist_xshut", "set_sensor_led", "set_keep_alive",
    "set_freq", "set_servoposition", "set_servocentre", "set_motors",
    # Public state
    "config", "VERSION", "PWMOutput",
    # HexDriveType fields accessed externally
    "pid", "name", "motors", "servos", "servo_pin_map",
    "__app_export__",
})

_PRESERVE_HEXDRIVE: frozenset[str] = frozenset({
    # BadgeOS app lifecycle
    "background_update",
    "__init__", "__app_export__",
    # Public API called by BadgeBot
    "initialise", "get_status", "set_logging", "set_power",
    "set_keep_alive", "set_freq", "set_servoposition", "set_servocentre", "set_motors",
    # Public state
    "config", "VERSION", "PWMOutput",
    # HexDriveType fields accessed externally
    "pid", "name", "motors", "servos", "servo_pin_map", "hw_ver",
})

_PRESERVE_HEXTEST: frozenset[str] = frozenset({
    # BadgeOS app lifecycle (uses background_task, not background_update)
    "update", "draw", "background_task",
    "__init__", "__app_export__",
    # Public state accessed by BadgeBot
    "config", "VERSION", "settings", "hexdrive_app", "logging",
    "auto_repeat_level", "refresh", "current_state",
    # Public methods called by BadgeBot
    "update_settings", "set_logging", "deinitialise", "show_message",
    "return_to_menu", "set_menu", "auto_repeat_check", "auto_repeat_clear",
})


@dataclass(frozen=True)
class MinifySpec:
    source: Path        # relative to ROOT
    artifact: Path      # relative to ROOT
    preserve: frozenset[str]


# All vendor modules this script knows how to minify.
MINIFIABLE: tuple[MinifySpec, ...] = (
    MinifySpec(
        ROOT / "vendor" / "HexDrive2" / "hexdrive2.py",
        ROOT / "EEPROM" / "hexdrive2.mpy",
        _PRESERVE_HEXDRIVE2,
    ),
    MinifySpec(
        ROOT / "EEPROM" / "hexdrive.py",
        ROOT / "EEPROM" / "hexdrive.mpy",
        _PRESERVE_HEXDRIVE,
    ),
    MinifySpec(
        ROOT / "EEPROM" / "hextest.py",
        ROOT / "EEPROM" / "hextest.mpy",
        _PRESERVE_HEXTEST,
    ),
)


# ── short-name generator: _a, _b, … _z, _aa, _ab, … ─────────────────────────
def _short_names():
    for c in string.ascii_lowercase:
        yield f"_{c}"
    for c1 in string.ascii_lowercase:
        for c2 in string.ascii_lowercase:
            yield f"_{c1}{c2}"


# ── build rename map ──────────────────────────────────────────────────────────
def _build_rename_map(tree: ast.AST, preserve: frozenset[str]):
    """Return ({old: short}, Counter) for self.xxx attributes worth renaming."""
    counts: Counter = Counter()
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Attribute)
            and isinstance(node.value, ast.Name)
            and node.value.id == "self"
        ):
            counts[node.attr] += 1

    candidates = sorted(
        [(name, cnt) for name, cnt in counts.items()
         if name not in preserve and len(name) > 3],
        key=lambda x: -(len(x[0]) - 2) * x[1],
    )

    gen = _short_names()
    used = set(counts.keys())
    mapping: dict[str, str] = {}

    for name, _cnt in candidates:
        while True:
            short = next(gen)
            if short not in used:
                break
        mapping[name] = short
        used.add(short)

    return mapping, counts


# ── AST transformer ───────────────────────────────────────────────────────────
class _AttrRenamer(ast.NodeTransformer):
    def __init__(self, mapping: dict[str, str]):
        self.mapping = mapping

    def visit_Attribute(self, node: ast.Attribute):
        self.generic_visit(node)
        if (
            isinstance(node.value, ast.Name)
            and node.value.id == "self"
            and node.attr in self.mapping
        ):
            node.attr = self.mapping[node.attr]
        return node

    def visit_FunctionDef(self, node: ast.FunctionDef):
        self.generic_visit(node)
        if node.name in self.mapping:
            node.name = self.mapping[node.name]
        return node

    visit_AsyncFunctionDef = visit_FunctionDef  # type: ignore[assignment]


# ── core pipeline ─────────────────────────────────────────────────────────────
def minify_file(
    source: Path,
    artifact: Path,
    preserve: frozenset[str],
    *,
    verbose: bool = False,
) -> int:
    """AST-rename + minify + mpy-cross compile source → artifact.

    Returns the artifact size in bytes, or -1 on failure.
    Temp files are cleaned up on both success and failure.
    """
    source_text = source.read_text(encoding="utf-8")
    tree = ast.parse(source_text)

    mapping, counts = _build_rename_map(tree, preserve)

    if verbose:
        print(f"Renaming {len(mapping)} attributes/methods in {source.name}:")
        for old, new in sorted(mapping.items(), key=lambda x: -(len(x[0]) - len(x[1])) * counts[x[0]]):
            est = (len(old) - len(new)) * counts[old]
            print(f"  self.{old:35s} -> self.{new:<5s}  (×{counts[old]:3d}, ~{est:+d} chars)")

    renamed_tree = _AttrRenamer(mapping).visit(tree)
    ast.fix_missing_locations(renamed_tree)

    temp_renamed = source.parent / (source.stem + ".renamed.py")
    temp_min = source.parent / (source.stem + ".min.py")

    temp_renamed.write_text(ast.unparse(renamed_tree), encoding="utf-8")
    try:
        cmd = [
            sys.executable, "-m", "python_minifier",
            "--remove-literal-statements", "--no-hoist-literals",
            "--output", str(temp_min), str(temp_renamed),
        ]
        r = subprocess.run(cmd, capture_output=True, text=True)
        #temp_renamed.unlink()
        if r.returncode != 0:
            print(f"[FAIL] python-minifier on {source.name}: {r.stderr}", file=sys.stderr)
            return -1

        artifact.parent.mkdir(parents=True, exist_ok=True)
        #cmd = [str(MPY_CROSS), "-O2", "-o", str(artifact), str(temp_min)]
        cmd = [str(MPY_CROSS), "-O2", "-o", str(artifact), str(source)]

        r = subprocess.run(cmd, capture_output=True, text=True)
        #temp_min.unlink()
        if r.returncode != 0:
            print(f"[FAIL] mpy-cross on {source.name}: {r.stderr}", file=sys.stderr)
            return -1

        return artifact.stat().st_size

    except Exception:
        temp_renamed.unlink(missing_ok=True)
        temp_min.unlink(missing_ok=True)
        raise


# ── entry point ───────────────────────────────────────────────────────────────
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Minify and compile vendor modules for MicroPython deployment.",
    )
    parser.add_argument("--source", type=Path,
                        help="Source .py file to minify (relative to repo root).")
    parser.add_argument("--artifact", type=Path,
                        help="Output .mpy file (relative to repo root).")
    parser.add_argument("--verbose", action="store_true",
                        help="Show attribute rename table.")
    args = parser.parse_args()

    if args.source or args.artifact:
        # ── CLI mode: single file, called by download_to_device.py ──────────
        if not (args.source and args.artifact):
            print("--source and --artifact must be provided together.", file=sys.stderr)
            return 1
        source = ROOT / args.source
        artifact = ROOT / args.artifact
        preserve = next(
            (spec.preserve for spec in MINIFIABLE if spec.source.stem == source.stem),
            frozenset(),
        )
        size = minify_file(source, artifact, preserve, verbose=args.verbose)
        return 0 if size >= 0 else 1

    # ── Standalone mode: minify all configured modules, show comparison ──────
    for spec in MINIFIABLE:
        if not spec.source.exists():
            print(f"  Skipping {spec.source.name}: not found")
            continue

        # Compile original for baseline comparison
        orig_mpy = spec.source.parent / (spec.source.stem + ".orig.mpy")
        cmd = [str(MPY_CROSS), "-O2", "-o", str(orig_mpy), str(spec.source)]
        subprocess.run(cmd, capture_output=True, text=True)
        orig_size = orig_mpy.stat().st_size if orig_mpy.exists() else 0
        orig_mpy.unlink(missing_ok=True)

        min_size = minify_file(spec.source, spec.artifact, spec.preserve, verbose=True)

        if min_size >= 0 and orig_size:
            saving = orig_size - min_size
            print(f"\n  {spec.source.name}:")
            print(f"    original:  {orig_size:6d} bytes")
            print(f"    minified:  {min_size:6d} bytes")
            print(f"    saving:    {saving:+d} bytes ({100 * saving / orig_size:.1f}%)\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
