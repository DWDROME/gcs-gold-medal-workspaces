@echo off
setlocal
cd /d "%~dp0"

if "%GCS_QR_CAMERA%"=="" set "GCS_QR_CAMERA=1"
if "%GCS_CAMERA_BACKEND%"=="" set "GCS_CAMERA_BACKEND=dshow"

echo QR camera: %GCS_QR_CAMERA%
echo Camera backend: %GCS_CAMERA_BACKEND%
echo Press q or Esc in the preview window to exit.

py -3.10 host_main.py --qr-preview
pause
