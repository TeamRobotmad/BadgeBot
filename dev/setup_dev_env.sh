#!/usr/bin/env sh
set -eu

VENV_PATH="${1:-.venv}"

python3 -m venv "$VENV_PATH"

PYTHON_EXE="$VENV_PATH/bin/python"
if [ ! -x "$PYTHON_EXE" ]; then
    echo "Virtual environment python not found at '$PYTHON_EXE'." >&2
    exit 1
fi

"$PYTHON_EXE" -m pip install --upgrade pip
"$PYTHON_EXE" -m pip install -r "dev/dev_requirements.txt"

echo ""
echo "Dev environment ready."
echo "Activate with: source $VENV_PATH/bin/activate"
