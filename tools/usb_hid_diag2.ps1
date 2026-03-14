Write-Host "=== USB HID DIAGNOSTIC v2 - Fast polling ==="
Write-Host "=== Trigger BadUSB now! ==="
Write-Host ""

# Capture initial snapshots as hashtables for fast lookup
$prevUSB = @{}
Get-PnpDevice -Class USB -Status OK -ErrorAction SilentlyContinue | ForEach-Object { $prevUSB[$_.InstanceId] = $_.FriendlyName }

$prevHID = @{}
Get-PnpDevice -Class HIDClass -Status OK -ErrorAction SilentlyContinue | ForEach-Object { $prevHID[$_.InstanceId] = $_.FriendlyName }

$prevKB = @{}
Get-PnpDevice -Class Keyboard -Status OK -ErrorAction SilentlyContinue | ForEach-Object { $prevKB[$_.InstanceId] = $_.FriendlyName }

$startTime = Get-Date
$pollCount = 0

for ($i = 0; $i -lt 240; $i++) {
    Start-Sleep -Milliseconds 500
    $pollCount++
    $ts = Get-Date -Format 'HH:mm:ss.fff'

    # Check USB class
    $curUSB = @{}
    Get-PnpDevice -Class USB -Status OK -ErrorAction SilentlyContinue | ForEach-Object { $curUSB[$_.InstanceId] = $_.FriendlyName }

    foreach ($id in $prevUSB.Keys) {
        if (-not $curUSB.ContainsKey($id)) {
            Write-Host "[$ts] USB GONE:  $id  [$($prevUSB[$id])]" -ForegroundColor Red
        }
    }
    foreach ($id in $curUSB.Keys) {
        if (-not $prevUSB.ContainsKey($id)) {
            Write-Host "[$ts] USB NEW:   $id  [$($curUSB[$id])]" -ForegroundColor Green
        }
    }
    $prevUSB = $curUSB

    # Check HIDClass
    $curHID = @{}
    Get-PnpDevice -Class HIDClass -Status OK -ErrorAction SilentlyContinue | ForEach-Object { $curHID[$_.InstanceId] = $_.FriendlyName }

    foreach ($id in $prevHID.Keys) {
        if (-not $curHID.ContainsKey($id)) {
            Write-Host "[$ts] HID GONE:  $id  [$($prevHID[$id])]" -ForegroundColor Red
        }
    }
    foreach ($id in $curHID.Keys) {
        if (-not $prevHID.ContainsKey($id)) {
            # Get extra details for new HID devices
            $extra = ""
            try {
                $drv = Get-PnpDeviceProperty -InstanceId $id -KeyName 'DEVPKEY_Device_DriverDesc' -ErrorAction SilentlyContinue
                if ($drv -and $drv.Data) { $extra += " Driver=$($drv.Data)" }
            } catch {}
            try {
                $prob = Get-PnpDeviceProperty -InstanceId $id -KeyName 'DEVPKEY_Device_ProblemCode' -ErrorAction SilentlyContinue
                if ($prob -and $prob.Data -and $prob.Data -ne 0) { $extra += " Problem=$($prob.Data)" }
            } catch {}
            Write-Host "[$ts] HID NEW:   $id  [$($curHID[$id])]$extra" -ForegroundColor Green
        }
    }
    $prevHID = $curHID

    # Check Keyboard class
    $curKB = @{}
    Get-PnpDevice -Class Keyboard -Status OK -ErrorAction SilentlyContinue | ForEach-Object { $curKB[$_.InstanceId] = $_.FriendlyName }

    foreach ($id in $prevKB.Keys) {
        if (-not $curKB.ContainsKey($id)) {
            Write-Host "[$ts] KB GONE:   $id  [$($prevKB[$id])]" -ForegroundColor Red
        }
    }
    foreach ($id in $curKB.Keys) {
        if (-not $prevKB.ContainsKey($id)) {
            Write-Host "[$ts] KB NEW:    $id  [$($curKB[$id])]" -ForegroundColor Green
        }
    }
    $prevKB = $curKB

    # Heartbeat every 30 polls (15s)
    if ($pollCount % 30 -eq 0) {
        Write-Host "[$ts] ... polling ($pollCount polls, $([int]((Get-Date) - $startTime).TotalSeconds)s elapsed)"
    }
}

Write-Host ""
Write-Host "=== Monitor done ($pollCount polls) ==="

# Check Event Viewer
Write-Host ""
Write-Host "=== Device Setup Events ==="
try {
    $events = Get-WinEvent -LogName 'Microsoft-Windows-Kernel-PnP/Device Configuration' -ErrorAction SilentlyContinue |
        Where-Object { $_.TimeCreated -ge $startTime } | Select-Object -First 10
    if ($events) {
        foreach ($evt in $events) {
            Write-Host "[$($evt.TimeCreated.ToString('HH:mm:ss.fff'))] ID=$($evt.Id): $($evt.Message.Substring(0, [Math]::Min(200, $evt.Message.Length)))"
        }
    } else {
        Write-Host "(none)"
    }
} catch {
    Write-Host "(log not available)"
}
