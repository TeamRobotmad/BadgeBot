#!/bin/bash
# Incremental compile + upload with detailed logging and error reporting.
cd "$(dirname "$0")"

if [ -x ".venv/bin/python" ]; then
    PYTHON_CMD=".venv/bin/python"
elif [ -x ".venv/Scripts/python.exe" ]; then
    PYTHON_CMD=".venv/Scripts/python.exe"
elif command -v python3 >/dev/null 2>&1; then
    PYTHON_CMD="python3"
elif command -v python >/dev/null 2>&1; then
    PYTHON_CMD="python"
else
    echo
    echo "download failed: could not find Python. Install Python or create .venv."
    exit 1
fi

"$PYTHON_CMD" dev/download_to_device.py "$@"
EXIT_CODE=$?

if [ "$EXIT_CODE" != "0" ]; then
    echo
    echo "download failed with exit code $EXIT_CODE."
fi

exit $EXIT_CODE
