@echo off
setlocal
cd /d "%~dp0"

if "%GCS_UPPER_CAMERA%"=="" set "GCS_UPPER_CAMERA=1"
if "%GCS_CAMERA_BACKEND%"=="" set "GCS_CAMERA_BACKEND=dshow"
if "%GCS_RING_PROCESS_EVERY%"=="" set "GCS_RING_PROCESS_EVERY=1"
if "%GCS_RING_PREVIEW_MODE%"=="" set "GCS_RING_PREVIEW_MODE=legacy"

echo Upper USB camera: %GCS_UPPER_CAMERA%
echo Camera backend: %GCS_CAMERA_BACKEND%
echo Ring process every: %GCS_RING_PROCESS_EVERY% frames
echo Ring detector: %GCS_RING_PREVIEW_MODE%
echo Press q or Esc in the preview window to exit.
echo Focus controls: a=autofocus, [ / ]=focus, - / = zoom.
echo Tune controls: use the Ring Tune V3 sliders.

py -3.10 host_main.py --ring-preview
pause
