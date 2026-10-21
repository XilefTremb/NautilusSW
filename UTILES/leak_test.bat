@echo off
title Nautilus HMI

REM Configure X11 forwarding
set DISPLAY=localhost:0.0

REM Start VcXsrv if it is not already running
tasklist /FI "IMAGENAME eq vcxsrv.exe" | find /I "vcxsrv.exe" >nul

if errorlevel 1 (
    start "" "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard
)

REM Connect to Jetson and launch HMI
ssh -Y -t nautilus@192.168.1.134 "bash -ic leak_test"

pause