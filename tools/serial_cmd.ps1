param([string]$cmd = "help")
$port = New-Object System.IO.Ports.SerialPort 'COM3',115200,'None',8,'One'
$port.ReadTimeout = 2000
$port.Open()
Start-Sleep -Milliseconds 100
# Flush any buffered data
while($port.BytesToRead -gt 0) { $null = $port.ReadExisting() }
# Send command
$port.Write("$cmd`r`n")
Start-Sleep -Milliseconds 500
$lines = 0
try {
    while($lines -lt 100) {
        $line = $port.ReadLine()
        Write-Host $line
        $lines++
    }
} catch {
    # timeout = done reading
}
$port.Close()
