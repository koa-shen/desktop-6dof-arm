param(
  [string]$PortName = "COM3",
  [int]$BaudRate = 115200,
  [double]$VrefVolts = 1.25,
  [int]$TimeoutMinutes = 25
)

$projectRoot = Split-Path -Parent $PSScriptRoot
$resultsDirectory = Join-Path $projectRoot "docs\test-results"
$timestamp = (Get-Date).ToUniversalTime().ToString("yyyyMMdd-HHmmss'Z'")
$csvPath = Join-Path $resultsDirectory "speed-sweep-$timestamp.csv"
$serialLogPath = Join-Path $resultsDirectory "speed-sweep-$timestamp-serial.log"
$invariantCulture = [System.Globalization.CultureInfo]::InvariantCulture
$serialLines = [System.Collections.Generic.List[string]]::new()
$rows = [System.Collections.Generic.List[object]]::new()
$port = [System.IO.Ports.SerialPort]::new(
  $PortName,
  $BaudRate,
  [System.IO.Ports.Parity]::None,
  8,
  [System.IO.Ports.StopBits]::One
)
$port.ReadTimeout = 1000

try {
  $port.Open()
  $port.DtrEnable = $false
  $port.DtrEnable = $true

  $ready = $false
  $readyDeadline = [DateTime]::UtcNow.AddSeconds(10)
  while ([DateTime]::UtcNow -lt $readyDeadline) {
    try {
      $line = $port.ReadLine().Trim()
      $serialLines.Add($line)
      if ($line -like "READY,*") {
        $ready = $true
        break
      }
      if ($line -like "ABORT,*") {
        throw "Firmware aborted: $line"
      }
    } catch [System.TimeoutException] {
    }
  }
  if (-not $ready) {
    throw "Did not receive READY from the speed-sweep firmware on $PortName."
  }

  $port.WriteLine("START")
  $deadline = [DateTime]::UtcNow.AddMinutes($TimeoutMinutes)
  $completed = $false
  while ([DateTime]::UtcNow -lt $deadline) {
    try {
      $line = $port.ReadLine().Trim()
      $serialLines.Add($line)
      if ($line -like "RESULT,*" -and -not $line.StartsWith("RESULT,speed_pps,")) {
        $parts = $line.Split(',')
        if ($parts.Count -ne 10) {
          throw "Malformed result line: $line"
        }

        $speed = [int]$parts[1]
  $measuredDelta = [int]$parts[5]
  $expectedDelta = [int]$parts[6]
        if ($speed -eq 0) {
          $errorRaw = $null
          $errorDeg = $null
          $errorPercent = $null
          $intendedAngleDeg = $null
        } else {
          $errorRaw = $measuredDelta - $expectedDelta
          $errorDeg = $errorRaw * 360.0 / 4096.0
          $errorPercent = [Math]::Abs($errorRaw) * 100.0 / [Math]::Abs($expectedDelta)
          $intendedAngleDeg = [Math]::Abs($expectedDelta) * 360.0 / 4096.0
        }

        $rows.Add([PSCustomObject]@{
          run_utc = (Get-Date).ToUniversalTime().ToString("o")
          vref_volts = $VrefVolts.ToString("F2", $invariantCulture)
          target_pulses_per_second = $speed
          run_number = [int]$parts[2]
          measure_steps_at_target_speed = if ($speed -eq 0) { "" } else { "400" }
          intended_travel_deg = if ($null -eq $intendedAngleDeg) { "" } else { $intendedAngleDeg.ToString("F3", $invariantCulture) }
          start_raw = [int]$parts[3]
          end_raw = [int]$parts[4]
          measured_delta_raw = $measuredDelta
          expected_delta_raw = $expectedDelta
          error_raw = if ($null -eq $errorRaw) { "" } else { $errorRaw }
          error_deg = if ($null -eq $errorDeg) { "" } else { $errorDeg.ToString("F3", $invariantCulture) }
          error_percent = if ($null -eq $errorPercent) { "" } else { $errorPercent.ToString("F3", $invariantCulture) }
          status_start = $parts[7]
          status_end = $parts[8]
          outcome = $parts[9]
        })
      }
      if ($line -eq "SWEEP_COMPLETE" -or $line -eq "STALL_DETECTED") {
        $completed = $true
        break
      }
    } catch [System.TimeoutException] {
    }
  }
  if (-not $completed) {
    throw "Speed sweep timed out after $TimeoutMinutes minutes."
  }
} finally {
  if ($port.IsOpen) {
    $port.Close()
  }
  $port.Dispose()
  [System.IO.File]::WriteAllLines($serialLogPath, $serialLines)
}

if ($rows.Count -eq 0) {
  throw "The sweep completed without result rows. See $serialLogPath"
}

$rows | Export-Csv -Path $csvPath -NoTypeInformation -Encoding utf8
$rows | Format-Table -AutoSize
Write-Host "Saved results: $csvPath"
Write-Host "Saved serial log: $serialLogPath"
