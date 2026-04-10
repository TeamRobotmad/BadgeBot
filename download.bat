@echo off
setlocal

REM Incremental compile + upload with detailed logging and error reporting.
python dev\download_to_device.py %*
set "EXIT_CODE=%ERRORLEVEL%"

if not "%EXIT_CODE%"=="0" (
	echo.
	echo download failed with exit code %EXIT_CODE%.
)

exit /b %EXIT_CODE%