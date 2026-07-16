@echo off
setlocal
cd /d "%~dp0"

REM Incremental compile + upload with detailed logging and error reporting.
if exist ".venv\Scripts\python.exe" (
	".venv\Scripts\python.exe" dev\download_to_device.py %*
) else (
	where python >nul 2>nul
	if not errorlevel 1 (
		python dev\download_to_device.py %*
	) else (
		where py >nul 2>nul
		if not errorlevel 1 (
			py -3 dev\download_to_device.py %*
		) else (
			echo.
			echo download failed: could not find Python. Install Python or create .venv.
			exit /b 1
		)
	)
)
set "EXIT_CODE=%ERRORLEVEL%"

if not "%EXIT_CODE%"=="0" (
	echo.
	echo download failed with exit code %EXIT_CODE%.
)

exit /b %EXIT_CODE%