Write-Host "=== USB HID DIAGNOSTIC - Comprehensive device monitoring ==="
Write-Host "=== Polling every 500ms for 120 seconds ==="
Write-Host "=== Monitoring: USB, HIDClass, Keyboard, Mouse ==="
Write-Host "=== Trigger BadUSB now! ==="
Write-Host ""

# Classes to monitor
$classes = @('USB', 'HIDClass', 'Keyboard', 'Mouse')

# Capture initial state for each class
$prevState = @{}
foreach ($cls in $classes) {
    $devs = Get-PnpDevice -Class $cls -ErrorAction SilentlyContinue | Where-Object { $_.Status -eq 'OK' -or $_.Status -eq 'Error' -or $_.Status -eq 'Degraded' -or $_.Status -eq 'Unknown' }
    $prevState[$cls] = $devs
}

# Also capture initial event log marker
$startTime = Get-Date

for ($i = 0; $i -lt 240; $i++) {
    Start-Sleep -Milliseconds 500

    $ts = Get-Date -Format 'HH:mm:ss.fff'

    foreach ($cls in $classes) {
        $curDevs = Get-PnpDevice -Class $cls -ErrorAction SilentlyContinue | Where-Object { $_.Status -eq 'OK' -or $_.Status -eq 'Error' -or $_.Status -eq 'Degraded' -or $_.Status -eq 'Unknown' }
        $prevDevs = $prevState[$cls]

        # Check for removed devices
        if ($prevDevs) {
            foreach ($d in $prevDevs) {
                if ($curDevs) {
                    $found = $curDevs | Where-Object { $_.InstanceId -eq $d.InstanceId }
                } else {
                    $found = $null
                }
                if (-not $found) {
                    Write-Host "[$ts] $cls GONE:  $($d.InstanceId)  [$($d.FriendlyName)]  Status=$($d.Status)" -ForegroundColor Red
                }
            }
        }

        # Check for new devices
        if ($curDevs) {
            foreach ($d in $curDevs) {
                if ($prevDevs) {
                    $found = $prevDevs | Where-Object { $_.InstanceId -eq $d.InstanceId }
                } else {
                    $found = $null
                }
                if (-not $found) {
                    # Get extra details
                    $problem = ""
                    try {
                        $prop = Get-PnpDeviceProperty -InstanceId $d.InstanceId -KeyName 'DEVPKEY_Device_ProblemCode' -ErrorAction SilentlyContinue
                        if ($prop -and $prop.Data) {
                            $problem = " Problem=$($prop.Data)"
                        }
                    } catch {}

                    $driver = ""
                    try {
                        $prop2 = Get-PnpDeviceProperty -InstanceId $d.InstanceId -KeyName 'DEVPKEY_Device_DriverDesc' -ErrorAction SilentlyContinue
                        if ($prop2 -and $prop2.Data) {
                            $driver = " Driver=$($prop2.Data)"
                        }
                    } catch {}

                    $hwids = ""
                    try {
                        $prop3 = Get-PnpDeviceProperty -InstanceId $d.InstanceId -KeyName 'DEVPKEY_Device_HardwareIds' -ErrorAction SilentlyContinue
                        if ($prop3 -and $prop3.Data) {
                            $hwids = " HWIDs=$($prop3.Data -join ',')"
                        }
                    } catch {}

                    Write-Host "[$ts] $cls NEW:   $($d.InstanceId)  [$($d.FriendlyName)]  Status=$($d.Status)$problem$driver$hwids" -ForegroundColor Green
                }
            }
        }

        $prevState[$cls] = $curDevs
    }
}

Write-Host ""
Write-Host "=== Monitor done ==="
Write-Host ""

# Check Event Viewer for device setup events during the monitoring period
Write-Host "=== Checking Device Setup Events ==="
try {
    $events = Get-WinEvent -LogName 'Microsoft-Windows-Kernel-PnP/Device Configuration' -ErrorAction SilentlyContinue |
        Where-Object { $_.TimeCreated -ge $startTime } |
        Select-Object -First 20

    if ($events) {
        foreach ($evt in $events) {
            $evtTs = $evt.TimeCreated.ToString('HH:mm:ss.fff')
            Write-Host "[$evtTs] EventID=$($evt.Id) Level=$($evt.LevelDisplayName): $($evt.Message.Substring(0, [Math]::Min(200, $evt.Message.Length)))"
        }
    } else {
        Write-Host "(No device configuration events found)"
    }
} catch {
    Write-Host "(Could not read device configuration log: $($_.Exception.Message))"
}

Write-Host ""

# Also check general PnP events
Write-Host "=== Checking System PnP Events ==="
try {
    $sysEvents = Get-WinEvent -FilterHashtable @{LogName='System'; ProviderName='Microsoft-Windows-Kernel-PnP'; StartTime=$startTime} -ErrorAction SilentlyContinue |
        Select-Object -First 20

    if ($sysEvents) {
        foreach ($evt in $sysEvents) {
            $evtTs = $evt.TimeCreated.ToString('HH:mm:ss.fff')
            Write-Host "[$evtTs] EventID=$($evt.Id): $($evt.Message.Substring(0, [Math]::Min(200, $evt.Message.Length)))"
        }
    } else {
        Write-Host "(No system PnP events found)"
    }
} catch {
    Write-Host "(Could not read system PnP log: $($_.Exception.Message))"
}

Write-Host ""
Write-Host "=== Diagnostic complete ==="
