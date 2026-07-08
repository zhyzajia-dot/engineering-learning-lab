@echo off
setlocal
pushd "%~dp0"
set "PIO=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
"%PIO%" run -e pc_bridge -t upload
if errorlevel 1 pause
popd
endlocal
