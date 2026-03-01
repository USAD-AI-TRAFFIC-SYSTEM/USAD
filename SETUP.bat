@echo off
REM USAD Standalone App Builder
REM Double-click this to build USAD.exe — no manual setup needed

echo.
echo ====================================================
echo         USAD Application Builder
echo ====================================================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo Error: Python is not installed or not in PATH
    echo Please install Python from https://www.python.org/
    echo Make sure to check "Add Python to PATH" during installation
    pause
    exit /b 1
)

echo [OK] Python found
echo.

REM Check if we're in the right directory
if not exist "USAD-Model\app.py" (
    echo Error: This script must be run from the USAD root directory
    pause
    exit /b 1
)

echo [OK] USAD directory structure verified
echo.

REM Create venv if it doesn't exist
if not exist "venv\Scripts\activate.bat" (
    echo Creating virtual environment...
    python -m venv venv
    echo [OK] Virtual environment created
    echo.
)

REM Activate venv
call venv\Scripts\activate.bat
echo [OK] Virtual environment activated
echo.

REM Install ALL dependencies (force reinstall to avoid missing packages)
echo Installing all dependencies...
pip install --upgrade pip >nul 2>&1
pip install -r requirements.txt
if errorlevel 1 (
    echo Error: Failed to install dependencies
    pause
    exit /b 1
)

echo.
echo [OK] All dependencies installed
echo.

REM Ensure opencv-python (full, not headless) is installed last
REM Both provide cv2 — must uninstall BOTH then reinstall the full version
echo Fixing OpenCV (ensuring GUI version)...
pip uninstall opencv-python opencv-python-headless -y >nul 2>&1
pip install opencv-python >nul 2>&1

REM Verify cv2 actually works before building
python -c "import cv2; print('[OK] OpenCV version:', cv2.__version__)" 2>nul
if errorlevel 1 (
    echo Error: OpenCV failed to install properly
    echo Trying forced reinstall...
    pip install --force-reinstall opencv-python >nul 2>&1
)

REM Verify all critical packages
python -c "import customtkinter; import cv2; import easyocr; import serial; print('[OK] All critical packages verified')"
if errorlevel 1 (
    echo Error: Some packages are still missing
    pause
    exit /b 1
)
echo.

echo Running build process...
echo.

python build_app.py

if errorlevel 1 (
    echo.
    echo Build failed. Press any key to exit.
    pause
    exit /b 1
)

echo.
echo ====================================================
echo Setup complete!
echo ====================================================
echo.
echo Your app is ready in the 'dist' folder
echo You can now share USAD.exe with users
echo.
pause
