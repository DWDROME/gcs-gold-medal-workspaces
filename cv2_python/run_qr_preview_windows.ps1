$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (-not $env:GCS_QR_CAMERA) {
    $env:GCS_QR_CAMERA = "1"
}
if (-not $env:GCS_CAMERA_BACKEND) {
    $env:GCS_CAMERA_BACKEND = "dshow"
}

Write-Host "QR camera: $env:GCS_QR_CAMERA"
Write-Host "Camera backend: $env:GCS_CAMERA_BACKEND"
Write-Host "Press q or Esc in the preview window to exit."

py -3.10 .\host_main.py --qr-preview
