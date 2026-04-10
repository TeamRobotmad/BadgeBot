#!/bin/bash
# Incremental compile + upload with detailed logging and error reporting.
cd "$(dirname "$0")"
python dev/download_to_device.py "$@"
EXIT_CODE=$?

if [ "$EXIT_CODE" != "0" ]; then
    echo
    echo "download failed with exit code $EXIT_CODE."
fi

exit $EXIT_CODE
