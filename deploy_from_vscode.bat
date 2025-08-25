@echo off
echo 🚀 HUB Complete System Deployment from VS Code
echo ================================================

REM Add PlatformIO Python to PATH
set PATH=%USERPROFILE%\.platformio\penv\Scripts;%PATH%

REM Run the deployment
python deploy_complete_system.py

REM Keep window open if there's an error
if %errorlevel% neq 0 (
    echo.
    echo ❌ Deployment failed with error code %errorlevel%
    echo Press any key to close...
    pause >nul
)