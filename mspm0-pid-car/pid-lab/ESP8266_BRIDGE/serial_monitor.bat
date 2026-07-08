@echo off
setlocal
pushd "%~dp0"
set "PIO=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
"%PIO%" device monitor --baud 115200 --echo
pause
popd
endlocal
