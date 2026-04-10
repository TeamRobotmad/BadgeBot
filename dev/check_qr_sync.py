"""Check that app.py _QR_CODE matches generated output for a given URL."""

from __future__ import annotations

import argparse
import ast
import re
import sys
from pathlib import Path

# Allow importing sibling tool module when run as script.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dev.generate_qr_code import DEFAULT_QR_URL, generate_qr_bitfields  # noqa: E402


def read_app_qr_rows(app_path: Path) -> list[int]:
    """Parse the _QR_CODE literal from app.py and return it as a list of ints."""
    source = app_path.read_text(encoding="utf-8")
    match = re.search(r"_QR_CODE\s*=\s*(\[(?:.|\n)*?\])", source, re.MULTILINE)
    if match is None:
        raise RuntimeError(f"Could not find _QR_CODE in {app_path}")

    try:
        rows = ast.literal_eval(match.group(1))
    except (ValueError, SyntaxError) as exc:
        raise RuntimeError(f"Failed to parse _QR_CODE in {app_path}: {exc}") from exc

    if not isinstance(rows, list):
        raise RuntimeError("_QR_CODE is not a list literal.")

    return [int(v) for v in rows]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=DEFAULT_QR_URL, help="URL expected in _QR_CODE")
    parser.add_argument("--app-path", default="app.py", help="Path to app.py")
    args = parser.parse_args()

    app_path = Path(args.app_path)
    expected = generate_qr_bitfields(args.url)
    actual = read_app_qr_rows(app_path)

    if actual != expected:
        print("QR mismatch detected.")
        print(f"  app.py rows: {len(actual)}")
        print(f"  expected rows for '{args.url}': {len(expected)}")
        print("Run:")
        print(f"  {sys.executable} dev/generate_qr_code.py --url {args.url} --write-app")
        return 1

    print(f"QR is in sync for URL: {args.url}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
