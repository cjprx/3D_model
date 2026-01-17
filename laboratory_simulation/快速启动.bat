@echo off
cd /d "%~dp0"

echo ========================================
echo MuJoCo Laboratory Simulation
echo ========================================
echo.

echo [1/3] Checking Python...
python --version
if %errorlevel% neq 0 (
    echo [ERROR] Python not found
    pause
    exit /b 1
)
echo [OK] Python ready
echo.

echo [2/3] Checking MuJoCo...
python -c "import mujoco; print('[OK] MuJoCo version:', mujoco.__version__)"
if %errorlevel% neq 0 (
    echo [INFO] Installing dependencies...
    pip install -r requirements.txt
)
echo.

echo [3/3] Ready to start
echo.
echo ========================================
echo Options:
echo ========================================
echo 1. Run model loading test
echo 2. Start visualization
echo 3. Run visual validation
echo 4. Exit
echo.
set /p choice=Select (1-4): 

if "%choice%"=="1" (
    echo.
    echo [INFO] Running test...
    python tests\test_model_loading.py
    pause
) else if "%choice%"=="2" (
    echo.
    echo [INFO] Starting visualization...
    python controllers\simulation.py
    pause
) else if "%choice%"=="3" (
    echo.
    echo [INFO] Running validation...
    python visual_validation.py
    pause
) else if "%choice%"=="4" (
    echo Bye!
    exit /b 0
) else (
    echo [ERROR] Invalid choice
    pause
)
