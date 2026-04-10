"""Generate QR bitfields for app.py and optionally write _QR_CODE automatically."""

from __future__ import annotations

import argparse
import re
import sys
import types
from pathlib import Path

# Ensure repository root (where uQR.py lives) is importable when this script
# is run as `python dev/generate_qr_code.py`.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# uQR imports MicroPython's `ure`. Provide a lightweight desktop shim.
if "ure" not in sys.modules:
    ure_module = types.ModuleType("ure")
    ure_module.compile = re.compile
    sys.modules["ure"] = ure_module

from uQR import QRCode

DEFAULT_QR_URL = "https://robotmad.odoo.com"


def generate_qr_bitfields(url: str) -> list[int]:
    """Return one bitfield per row for a uQR matrix."""
    qr = QRCode(error_correction=1, box_size=10, border=0)
    qr.add_data(url)
    matrix = qr.get_matrix()

    if len(matrix) > 32:
        raise ValueError(f"QR code too big: {len(matrix)}x{len(matrix)} (max 32x32).")

    rows: list[int] = []
    for row in matrix:
        bitfield = 0
        for col, value in enumerate(row):
            # LSBit on the left to match existing BadgeBot convention.
            if value:
                bitfield |= 1 << col
        rows.append(bitfield)
    return rows


def format_qr_block(rows: list[int]) -> str:
    """Format the list of bitfields as a Python block for app.py."""
    # Determine the hex width needed to align all values (based on the widest value)
    max_val = max(rows) if rows else 0
    hex_width = max(1, len(f"{max_val:x}"))
    lines = ["_QR_CODE = ["]
    lines.extend([f"            0x{row:0{hex_width}x}," for row in rows])
    lines.append("]")
    return "\n".join(lines)


def update_app_qr_constant(app_path: Path, qr_block: str) -> None:
    """Replace the _QR_CODE constant in app.py with generated content."""
    source = app_path.read_text(encoding="utf-8")
    pattern = re.compile(r"_QR_CODE\s*=\s*\[(?:.|\n)*?\]\n", re.MULTILINE)

    if not pattern.search(source):
        raise RuntimeError(f"Could not find _QR_CODE constant in {app_path}.")

    updated = pattern.sub(qr_block + "\n", source, count=1)
    app_path.write_text(updated, encoding="utf-8")


def main() -> int:
    """Generate QR code bitfields for a given URL and optionally update app.py."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--url",
        default=DEFAULT_QR_URL,
        help="URL to encode in the QR code.",
    )
    parser.add_argument(
        "--write-app",
        action="store_true",
        help="Write generated _QR_CODE back into app.py.",
    )
    parser.add_argument(
        "--app-path",
        default="app.py",
        help="Path to app.py (used with --write-app).",
    )

    args = parser.parse_args()

    rows = generate_qr_bitfields(args.url)
    block = format_qr_block(rows)
    print(block)

    if args.write_app:
        app_path = Path(args.app_path)
        update_app_qr_constant(app_path, block)
        print(f"\nUpdated {app_path} with generated _QR_CODE for {args.url}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
