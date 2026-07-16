@echo off
setlocal
set PORT=%~1
if "%PORT%"=="" set PORT=COM9
python "%~dp0bridge_stress_test.py" %PORT% --count 100
pause
endlocal
