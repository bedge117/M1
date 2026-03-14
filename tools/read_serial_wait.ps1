$port = New-Object System.IO.Ports.SerialPort 'COM3',115200,'None',8,'One'
$port.ReadTimeout = 30000
$port.Open()
Write-Host "Waiting for serial data on COM3 (power cycle the M1 now)..."
$lines = 0
try {
    while($lines -lt 200) {
        $line = $port.ReadLine()
        Write-Host $line
        $lines++
    }
} catch {
    Write-Host "(Timeout or read complete after $lines lines)"
}
$port.Close()
