$port = New-Object System.IO.Ports.SerialPort 'COM3',115200,'None',8,'One'
$port.ReadTimeout = 5000
$port.Open()
Start-Sleep -Milliseconds 500
$lines = 0
while($port.BytesToRead -gt 0 -and $lines -lt 100) {
    Write-Host $port.ReadLine()
    $lines++
}
$port.Close()
