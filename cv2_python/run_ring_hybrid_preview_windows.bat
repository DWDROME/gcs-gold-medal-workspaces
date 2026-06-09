@echo off
setlocal
cd /d "%~dp0"

set "GCS_UPPER_CAMERA=1"
set "GCS_CAMERA_BACKEND=dshow"
set "GCS_RING_PREVIEW_MODE=hybrid"
set "GCS_RING_PROCESS_EVERY=1"

echo Upper USB camera: %GCS_UPPER_CAMERA%
echo Camera backend: %GCS_CAMERA_BACKEND%
echo Ring detector: %GCS_RING_PREVIEW_MODE%
echo Ring process every: %GCS_RING_PROCESS_EVERY% frames
echo Press q or Esc in the preview window to exit.
echo Focus controls: a=autofocus, [ / ]=focus, - / = zoom.
echo Hybrid: legacy result first, V3 fallback only when legacy misses.

py -3.10 host_main.py --ring-preview
pause
