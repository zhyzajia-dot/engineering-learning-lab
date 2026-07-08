@echo off
setlocal
pushd "%~dp0"
python gimbal_autotune_gui.py
if errorlevel 1 pause
popd
endlocal
