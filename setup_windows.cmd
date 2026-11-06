@echo off
setlocal DisableDelayedExpansion
rem Double-click or run from PowerShell. No activation script is required.
pushd "%~dp0" >nul
if errorlevel 1 (
    echo Cannot enter the MeArm project folder.
    exit /b 1
)

if defined MEARM_PYTHON (
    "%MEARM_PYTHON%" "scripts\setup_windows.py" %*
    goto :finished
)

py -3.12 -c "import sys" >nul 2>&1
if not errorlevel 1 (
    py -3.12 "scripts\setup_windows.py" %*
    goto :finished
)

if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" -c "import sys" >nul 2>&1
    if not errorlevel 1 (
        ".venv\Scripts\python.exe" "scripts\setup_windows.py" %*
        goto :finished
    )
)

python -c "import sys" >nul 2>&1
if not errorlevel 1 (
    python "scripts\setup_windows.py" %*
    goto :finished
)

echo.
echo Python was not found. Install standard 64-bit Python 3.12 first.
echo See install.md, section 2, then run this file again.
echo You may also set MEARM_PYTHON to the full path of python.exe.
set "MEARM_SETUP_EXIT_CODE=1"
goto :cleanup

:finished
set "MEARM_SETUP_EXIT_CODE=%ERRORLEVEL%"

:cleanup
popd
if not "%MEARM_SETUP_EXIT_CODE%"=="0" if not "%MEARM_SETUP_EXIT_CODE%"=="130" (
    echo.
    echo Setup or Viser stopped with error code %MEARM_SETUP_EXIT_CODE%.
    if not defined MEARM_NO_PAUSE pause
)
exit /b %MEARM_SETUP_EXIT_CODE%
