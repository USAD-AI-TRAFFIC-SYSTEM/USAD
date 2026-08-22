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

REM Check if all critical packages are already installed and working
python -c "import customtkinter, cv2, easyocr, serial, fastapi, uvicorn, webview; print('[OK] All dependencies verified')" 2>nul
if errorlevel 1 (
    echo Installing dependencies...
    pip install -r requirements.txt
    if errorlevel 1 (
        echo.
        echo ====================================================
        echo [WARNING] Could not update packages because Python is
        echo currently running and locking cv2.pyd or other files.
        echo Please CLOSE any running USAD / Python app windows,
        echo then run SETUP.bat again.
        echo ====================================================
        echo.
        pause
        exit /b 1
    )
    REM Verify critical packages
    python -c "import customtkinter, cv2, easyocr, serial, fastapi, uvicorn, webview; print('[OK] All critical packages verified')"
    if errorlevel 1 (
        echo Error: Some packages are still missing
        pause
        exit /b 1
    )
) else (
    echo [OK] All dependencies already installed and verified
)
echo.

REM Build the React frontend (USAD-UI)
echo Building React frontend...
where npm >nul 2>&1
if errorlevel 1 (
    echo Error: npm is not installed or not in PATH
    echo Please install Node.js from https://nodejs.org/
    pause
    exit /b 1
)

pushd USAD-UI
if not exist "node_modules" (
    echo Installing frontend dependencies...
    call npm install
    if errorlevel 1 (
        echo Error: Failed to install frontend dependencies
        popd
        pause
        exit /b 1
    )
)
echo Compiling frontend...
call npm run build
if errorlevel 1 (
    echo Error: Frontend build failed
    popd
    pause
    exit /b 1
)
popd
echo [OK] React frontend built (USAD-UI/dist)
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
