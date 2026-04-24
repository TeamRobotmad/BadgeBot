#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_ROOT=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
cd "$REPO_ROOT"

VENV_PATH="${1:-.venv-wsl310}"
PYTHON_VERSION="${PYTHON_VERSION:-3.10}"

ensure_uv() {
    if command -v uv >/dev/null 2>&1; then
        command -v uv
        return
    fi

    if [ -x "$HOME/.local/bin/uv" ]; then
        printf '%s\n' "$HOME/.local/bin/uv"
        return
    fi

    if command -v curl >/dev/null 2>&1; then
        curl -LsSf https://astral.sh/uv/install.sh | sh
    elif command -v wget >/dev/null 2>&1; then
        wget -qO- https://astral.sh/uv/install.sh | sh
    else
        echo "Need curl or wget to install uv." >&2
        exit 1
    fi

    if [ -x "$HOME/.local/bin/uv" ]; then
        printf '%s\n' "$HOME/.local/bin/uv"
        return
    fi

    echo "uv installation failed." >&2
    exit 1
}

UV_BIN="$(ensure_uv)"

"$UV_BIN" python install "$PYTHON_VERSION"
"$UV_BIN" venv "$VENV_PATH" --python "$PYTHON_VERSION"
"$UV_BIN" pip install --python "$VENV_PATH/bin/python" -r "dev/dev_requirements.txt"
"$UV_BIN" pip install --python "$VENV_PATH/bin/python" -r "../../requirements.txt"

echo ""
echo "WSL simulator environment ready."
echo "Activate with: source $VENV_PATH/bin/activate"
echo "If BadgeBot is checked out inside badge-2024-software, run tests with:"
echo "  cd tests"
echo "  PYTHONPATH=/path/to/badge-2024-software ../$VENV_PATH/bin/python -m pytest test_smoke.py test_autotune.py -v"