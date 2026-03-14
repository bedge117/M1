$port = New-Object System.IO.Ports.SerialPort 'COM3',115200,'None',8,'One'
$port.ReadTimeout = 2000
$port.Open()
Start-Sleep -Milliseconds 100

# Flush
while($port.BytesToRead -gt 0) { $null = $port.ReadExisting() }

# Send help
$port.WriteLine("help")
Start-Sleep -Milliseconds 1500

# Read everything available
$output = $port.ReadExisting()
Write-Host $output

$port.Close()
