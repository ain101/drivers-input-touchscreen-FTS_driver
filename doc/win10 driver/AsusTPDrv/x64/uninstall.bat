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

goto exit

:VIRTUAL_TOUCH
set _CUR_DRIVER_PATH=%_VIRTUAL_TOUCH_DEV%

pushd %_DRV_CUR_PATH%

cd %_CUR_DRIVER_PATH%\%_MINIDRIVER_DEV%
dpinst.exe /u .\AsusTP.inf /S /Q

cd ..\%_VBUS_DEV%
devcon.exe remove root\AsusVBus
popd
goto exit

:ELAN_PS2
pushd %_DRV_CUR_PATH%
dpinst.exe /u %_ELAN_PS2_DEV%\AsusTP.inf /S /Q
ATP_Reg.exe 0
popd
goto exit


:SYNA_SMB
pushd %_DRV_CUR_PATH%
dpinst.exe /u %_SYNA_SMB_DEV%\AsusTP.inf /S /Q
dpinst.exe /u %_SYNA_SMB_DEV%\SmbDrv.inf /S /Q
ATP_Reg.exe 0
popd
goto exit


:ELAN_I2C
pushd %_DRV_CUR_PATH%
if exist "%_ELAN_I2C_DEV%\focal" (
  goto FOCALTECH_I2C
) 
dpinst.exe /u %_ELAN_I2C_DEV%\AsusSGDrv.inf /S /Q
popd
goto exit


:FOCALTECH_PS2
pushd %_DRV_CUR_PATH%
dpinst.exe /u %_FOCALTECH_PS2_DEV%\AsusTP.inf /S /Q
ATP_Reg.exe 0
popd
goto exit


:FOCALTECH_I2C
pushd %_DRV_CUR_PATH%
dpinst.exe /u %_FOCALTECH_I2C_DEV%\AsusSGDrv.inf /S /Q
del %_ELAN_I2C_DEV%\focal
popd
goto exit

:ELAN_PTP
pushd %_DRV_CUR_PATH%
if exist "%_ELAN_PTP_DEV%\focal" (
  goto FOCALTECH_PTP
) 
dpinst.exe /u %_ELAN_PTP_DEV%\AsusPTPDriver.inf /S /Q
dpinst.exe /u %_ELAN_PTP_DEV%\AsusPTPMouse.inf /S /Q
popd
goto exit

:FOCALTECH_PTP
pushd %_DRV_CUR_PATH%
dpinst.exe /u %_FOCALTECH_PTP_DEV%\AsusPTPDriver.inf /S /Q
dpinst.exe /u %_FOCALTECH_PTP_DEV%\AsusPTPMouse.inf /S /Q
del %_ELAN_PTP_DEV%\focal
popd
goto exit


:exit