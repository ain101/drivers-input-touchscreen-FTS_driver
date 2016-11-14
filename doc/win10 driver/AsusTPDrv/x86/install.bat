set _VIRTUAL_TOUCH_DEV="VirtualTouch"
set _MINIDRIVER_DEV="AsusMiniDriver"
set _VIRTUAL_PTP_DEV="VirtualPTP"
set _VPTP_DEV="AsusVirtualPTP"
set _VBUS_DEV="AsusVBusDriver"

set _ELAN_PS2_DEV="Elan_PS2"
set _SYNA_SMB_DEV="SYNA_SMB"
set _ELAN_I2C_DEV="Elan_I2C"
set _FOCALTECH_PS2_DEV="Focaltech_PS2"
set _FOCALTECH_I2C_DEV="Focaltech_I2C"

set _ELAN_PTP_DEV="Elan_PTP"
set _FOCALTECH_PTP_DEV="Focaltech_PTP"

set _DRV_CUR_PATH="%~dp0%"
set _CUR_DRIVER_PATH=%_ELAN_I2C_DEV%

pushd %_DRV_CUR_PATH%
check_hwid.exe
popd
if %ERRORLEVEL% == -1 goto VIRTUAL_TOUCH
if %ERRORLEVEL% ==  0 goto ELAN_PS2
if %ERRORLEVEL% ==  1 goto SYNA_SMB
if %ERRORLEVEL% ==  2 goto ELAN_I2C
if %ERRORLEVEL% ==  3 goto FOCALTECH_I2C
if %ERRORLEVEL% ==  4 goto FOCALTECH_PS2
if %ERRORLEVEL% ==  5 goto ELAN_PTP
if %ERRORLEVEL% ==  6 goto FOCALTECH_PTP

copy /y NUL "%ALLUSERSPROFILE%\DriverInstallFail.txt" >NUL
goto exit

:VIRTUAL_TOUCH
copy /y NUL "%ALLUSERSPROFILE%\DriverInstallFail.txt" >NUL
set _CUR_DRIVER_PATH=%_VIRTUAL_TOUCH_DEV%

pushd %_DRV_CUR_PATH%
cd %_CUR_DRIVER_PATH%\%_VBUS_DEV%
devcon.exe remove root\AsusVBus
devcon.exe install AsusVBus.inf root\AsusVBus

cd ..\%_MINIDRIVER_DEV%
dpinst.exe /f /Q
popd
goto exit

:ELAN_PS2
set _CUR_DRIVER_PATH=%_ELAN_PS2_DEV%

pushd %_DRV_CUR_PATH%
dpinst.exe /path %_CUR_DRIVER_PATH% /f /Q

set /a "_DRV_COUNT = 1"
set /a "INSTALLDRIVER = %ERRORLEVEL% & %_DRV_COUNT%"

if %INSTALLDRIVER% == 1 ATP_Reg.exe 1
popd
goto exit


:SYNA_SMB
set _CUR_DRIVER_PATH=%_SYNA_SMB_DEV%

pushd %_DRV_CUR_PATH%
dpinst.exe /path %_CUR_DRIVER_PATH% /f /Q
ATP_Reg.exe 1
popd
goto exit


:ELAN_I2C
set _CUR_DRIVER_PATH=%_ELAN_I2C_DEV%

pushd %_DRV_CUR_PATH%
dpinst.exe /path %_CUR_DRIVER_PATH% /f /Q
popd
goto exit


:FOCALTECH_PS2
set _CUR_DRIVER_PATH=%_FOCALTECH_PS2_DEV%

pushd %_DRV_CUR_PATH%
dpinst.exe /path %_CUR_DRIVER_PATH% /f /Q
ATP_Reg.exe 1
popd
goto exit


:FOCALTECH_I2C
set _CUR_DRIVER_PATH=%_FOCALTECH_I2C_DEV%

pushd %_DRV_CUR_PATH%
dpinst.exe /path %_CUR_DRIVER_PATH% /f /Q
copy nul %_ELAN_I2C_DEV%\focal
popd
goto exit

:ELAN_PTP
set _CUR_DRIVER_PATH=%_ELAN_PTP_DEV%
pushd %_DRV_CUR_PATH%

dpinst.exe /path %_CUR_DRIVER_PATH% /f /SW
popd
goto exit

:FOCALTECH_PTP
set _CUR_DRIVER_PATH=%_FOCALTECH_PTP_DEV%
pushd %_DRV_CUR_PATH%

dpinst.exe /path %_CUR_DRIVER_PATH% /f /SW
copy nul %_ELAN_PTP_DEV%\focal
popd
goto exit


:exit