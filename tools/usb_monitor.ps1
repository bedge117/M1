Write-Host "=== USB/HID MONITOR - Polling every 1 second for 90 seconds ==="
Write-Host "=== Trigger BadUSB now! ==="
Write-Host ""

$prev_usb = Get-PnpDevice -Class USB -ErrorAction SilentlyContinue | Where-Object { $_.Status -eq 'OK' -or $_.Status -eq 'Error' }
$prev_hid = Get-PnpDevice -Class HIDClass -ErrorAction SilentlyContinue | Where-Object { $_.Status -eq 'OK' -or $_.Status -eq 'Error' }

for ($i = 0; $i -lt 90; $i++) {
    Start-Sleep -Seconds 1

    $cur_usb = Get-PnpDevice -Class USB -ErrorAction SilentlyContinue | Where-Object { $_.Status -eq 'OK' -or $_.Status -eq 'Error' }
    $cur_hid = Get-PnpDevice -Class HIDClass -ErrorAction SilentlyContinue | Where-Object { $_.Status -eq 'OK' -or $_.Status -eq 'Error' }

    $ts = Get-Date -Format 'HH:mm:ss'

    # USB changes
    foreach ($d in $prev_usb) {
        if ($d.InstanceId -notin $cur_usb.InstanceId) {
            Write-Host "[$ts] USB GONE:  $($d.InstanceId)  [$($d.FriendlyName)]" -ForegroundColor Red
        }
    }
    foreach ($d in $cur_usb) {
        if ($d.InstanceId -notin $prev_usb.InstanceId) {
            Write-Host "[$ts] USB NEW:   $($d.InstanceId)  [$($d.FriendlyName)]  Status=$($d.Status)" -ForegroundColor Green
        }
    }

    # HID changes
    foreach ($d in $prev_hid) {
        if ($d.InstanceId -notin $cur_hid.InstanceId) {
            Write-Host "[$ts] HID GONE:  $($d.InstanceId)  [$($d.FriendlyName)]" -ForegroundColor Red
        }
    }
    foreach ($d in $cur_hid) {
        if ($d.InstanceId -notin $prev_hid.InstanceId) {
            Write-Host "[$ts] HID NEW:   $($d.InstanceId)  [$($d.FriendlyName)]  Status=$($d.Status)" -ForegroundColor Green
        }
    }

    $prev_usb = $cur_usb
    $prev_hid = $cur_hid
}
Write-Host "=== Monitor done ==="
