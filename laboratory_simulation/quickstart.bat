@echo off
chcp 65001 > nul
echo ========================================
echo MuJoCo Laboratory Simulation - Quick Start
echo ========================================
echo.

echo [1/3] Checking Python environment...
python --version 2>nul
if %errorlevel% neq 0 (
    echo [ERROR] Python not installed or not in PATH
    pause
    exit /b 1
)
echo [OK] Python environment ready
echo.

echo [2/3] Checking dependencies...
python -c "import mujoco; print('[OK] MuJoCo installed - Version:', mujoco.__version__)" 2>nul
if %errorlevel% neq 0 (
    echo [INFO] MuJoCo not installed, installing dependencies...
    pip install -r requirements.txt
    if %errorlevel% neq 0 (
        echo [ERROR] Failed to install dependencies
        pause
        exit /b 1
    )
)
echo.

echo [3/3] Starting simulation...
echo.
echo ========================================
echo Startup Options:
echo ========================================
echo 1. Run model loading test
echo 2. Start visualization simulation
echo 3. Run visual validation program
echo 4. Exit
echo.
set /p choice="Please choose (1-4): "

if "%choice%"=="1" (
    echo.
    echo [INFO] Running model loading test...
    python tests\test_model_loading.py
    pause
) else if "%choice%"=="2" (
    echo.
    echo [INFO] Starting visualization simulation...
    echo [TIP] Press Esc to exit the visualization window
    python controllers\simulation.py
    pause
) else if "%choice%"=="3" (
    echo.
    echo [INFO] Running visual validation program...
    python visual_validation.py
    pause
) else if "%choice%"=="4" (
    echo.
    echo [INFO] Exiting...
    exit /b 0
) else (
    echo.
    echo [ERROR] Invalid choice
    pause
    exit /b 1
)

