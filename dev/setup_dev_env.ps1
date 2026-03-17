param(
    [string]$VenvPath = ".venv"
)

$ErrorActionPreference = "Stop"

python -m venv $VenvPath

$pythonExe = Join-Path $VenvPath "Scripts\python.exe"
if (-not (Test-Path $pythonExe)) {
    throw "Virtual environment python not found at '$pythonExe'."
}

& $pythonExe -m pip install --upgrade pip
& $pythonExe -m pip install -r "dev/dev_requirements.txt"

Write-Host ""
Write-Host "Dev environment ready."
Write-Host "Activate with: $VenvPath\Scripts\Activate.ps1"
