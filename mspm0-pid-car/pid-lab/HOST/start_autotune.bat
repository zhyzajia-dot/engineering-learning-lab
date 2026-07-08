@echo off
setlocal
python "%~dp0pid_autotune_gui.py"
if errorlevel 1 pause
endlocal
