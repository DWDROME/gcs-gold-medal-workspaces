param(
  [string]$Port = "COM11",
  [int]$Baudrate = 9600,
  [int]$BootDelayMs = 1200,
  [int]$WaitMs = 500
)

$ErrorActionPreference = "Stop"

Write-Output "Ports:"
[System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object | ForEach-Object { Write-Output "  $_" }

$p = [System.IO.Ports.SerialPort]::new(
  $Port,
  $Baudrate,
  [System.IO.Ports.Parity]::None,
  8,
  [System.IO.Ports.StopBits]::One
)

$p.ReadTimeout = 400
$p.WriteTimeout = 400

try {
  $p.Open()
  $p.DiscardInBuffer()
  Start-Sleep -Milliseconds $BootDelayMs
  $boot = $p.ReadExisting()

  $pkt = [byte[]](0xFF,0x39,0x38,0x37,0x36,0x35,0x34,0x33,0x32,0x31,0xFE)
  $p.Write($pkt, 0, $pkt.Length)
  Start-Sleep -Milliseconds $WaitMs
  $resp = $p.ReadExisting()

  Write-Output "---BOOT---"
  Write-Output $boot
  Write-Output "---RESP---"
  Write-Output $resp
}
finally {
  if ($p.IsOpen) {
    $p.Close()
  }
}
