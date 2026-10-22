@echo off
setlocal
pushd "%~dp0"
cmake --preset debug
if errorlevel 1 goto :error
cmake --build --preset debug
if errorlevel 1 goto :error
popd
exit /b 0

:error
echo Build failed.
popd
pause
exit /b 1
