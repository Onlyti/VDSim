@echo off
REM VDSim wheel force-feedback bridge — Windows autostart launcher.
REM Edit SERVER/UDPPORT to your VDSim host (UDP port = GUI http port + 1), then put
REM a shortcut to this file in the Startup folder (Win+R -> shell:startup), or:
REM   schtasks /create /tn VDSimFFB /tr "%~f0" /sc onlogon
set SERVER=localhost
set UDPPORT=8091
set VDSIM=%~dp0..\..
python "%VDSIM%\tools\wheel_ffb_sdl.py" --server %SERVER% --udp-port %UDPPORT% --gain 0.8 --autocenter 0.15
