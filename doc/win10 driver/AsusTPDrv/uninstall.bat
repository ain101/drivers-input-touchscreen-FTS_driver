set _DRV_PATH_PLT=""

for /f "tokens=2*" %%a in ('REG QUERY "HKLM\SYSTEM\CurrentControlSet\Control\Session Manager\Environment" /v PROCESSOR_ARCHITECTURE') do set "ARCHITEC=%%~b"

if %ARCHITEC% == AMD64 goto x64
if %ARCHITEC% == x86 goto x86

:x64
set _DRV_PATH_PLT=%~dp0%x64
goto start_session

:x86
set _DRV_PATH_PLT=%~dp0%x86
goto start_session

:start_session
set _DRV_PATH_=%_DRV_PATH_PLT%



echo "UnInstalling Asus Virtual Touch Driver"
pushd %_DRV_PATH_%
call uninstall.bat
popd


echo "finish uninstall %PROCESSOR_ARCHITECTURE% platform driver"
goto exit

:error
echo "uninstall error

:exit
echo "exit..."

