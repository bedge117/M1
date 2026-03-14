# Serial monitor that auto-reconnects after USB mode switch
# Usage: powershell -ExecutionPolicy Bypass -File tools\serial_monitor.ps1

$portName = "COM3"
$baud = 115200
$timeout = 120  # seconds

Write-Host "=== Serial Monitor - waiting for $portName ===" -ForegroundColor Cyan
Write-Host "Press Ctrl+C to exit" -ForegroundColor DarkGray
Write-Host ""

$deadline = (Get-Date).AddSeconds($timeout)

while ((Get-Date) -lt $deadline) {
    try {
        $port = New-Object System.IO.Ports.SerialPort $portName, $baud
        $port.ReadTimeout = 1000
        $port.DtrEnable = $true
        $port.Open()
        Write-Host "[CONNECTED to $portName]" -ForegroundColor Green
        $deadline = (Get-Date).AddSeconds($timeout)  # reset timeout on connect

        while ($port.IsOpen) {
            try {
                $line = $port.ReadLine()
                $ts = (Get-Date).ToString("HH:mm:ss.fff")

                # Highlight instrumentation lines
                if ($line -match "INSTRUMENTATION|SendReport|DataIn|Init=|Final:") {
                    Write-Host "[$ts] $line" -ForegroundColor Yellow
                } else {
                    Write-Host "[$ts] $line"
                }
                $deadline = (Get-Date).AddSeconds($timeout)
            }
            catch [TimeoutException] {
                # ReadLine timeout, just loop
            }
            catch {
                Write-Host "[DISCONNECTED]" -ForegroundColor Red
                break
            }
        }
        $port.Close()
    }
    catch {
        # Port not available, retry
        Start-Sleep -Milliseconds 500
    }
}
Write-Host "`n[Monitor timeout after ${timeout}s of inactivity]" -ForegroundColor DarkGray
