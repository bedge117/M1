Write-Host "=== Current USB devices (VID 0483) ==="
Get-PnpDevice -Class USB -ErrorAction SilentlyContinue | Where-Object { $_.InstanceId -like '*0483*' } | Format-Table InstanceId, FriendlyName, Status -AutoSize

Write-Host "=== Current HIDClass devices ==="
Get-PnpDevice -Class HIDClass -ErrorAction SilentlyContinue | Where-Object { $_.Status -eq 'OK' } | Format-Table InstanceId, FriendlyName, Status -AutoSize

Write-Host "=== Current Keyboard devices ==="
Get-PnpDevice -Class Keyboard -ErrorAction SilentlyContinue | Where-Object { $_.Status -eq 'OK' } | Format-Table InstanceId, FriendlyName, Status -AutoSize

Write-Host "=== All USB devices with Status OK ==="
Get-PnpDevice -Class USB -Status OK -ErrorAction SilentlyContinue | Format-Table InstanceId, FriendlyName -AutoSize
