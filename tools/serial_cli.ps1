$port = New-Object System.IO.Ports.SerialPort 'COM3',115200,'None',8,'One'
$port.ReadTimeout = 3000
$port.Open()
Start-Sleep -Milliseconds 200
# Send a newline to get a prompt, then 'help' command
$port.Write("`r`n")
Start-Sleep -Milliseconds 500
$port.Write("help`r`n")
Start-Sleep -Milliseconds 1000
$lines = 0
try {
    while($port.BytesToRead -gt 0 -and $lines -lt 50) {
        $line = $port.ReadLine()
        Write-Host $line
        $lines++
    }
} catch {
    Write-Host "(Read complete after $lines lines)"
}
$port.Close()
