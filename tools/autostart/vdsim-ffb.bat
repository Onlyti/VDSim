@echo off
REM VDSim wheel force-feedback bridge — Windows autostart launcher.
REM Edit SERVER to your VDSim host, then put a shortcut to this file in the
REM Startup folder (Win+R -> shell:startup), or register a scheduled task:
REM   schtasks /create /tn VDSimFFB /tr "%~f0" /sc onlogon
set SERVER=http://localhost:8100
set VDSIM=%~dp0..\..
python "%VDSIM%\tools\wheel_ffb_sdl.py" --url %SERVER% --gain 0.8 --autocenter 0.15
