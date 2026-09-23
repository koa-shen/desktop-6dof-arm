param(
  [string]$PortName = "COM3",
  [int]$BaudRate = 115200,
  [double]$VrefVolts = 1.25,
  [string]$UnitId = "reducer-01",
  [int]$TimeoutMinutes = 20
)

$projectRoot = Split-Path -Parent $PSScriptRoot
$resultsDirectory = Join-Path $projectRoot "docs\test-results"
$timestamp = (Get-Date).ToUniversalTime().ToString("yyyyMMdd-HHmmss'Z'")
$csvPath = Join-Path $resultsDirectory "backlash-$timestamp.csv"
$serialLogPath = Join-Path $resultsDirectory "backlash-$timestamp-serial.log"
$invariantCulture = [System.Globalization.CultureInfo]::InvariantCulture
$serialLines = [System.Collections.Generic.List[string]]::new()
$rows = [System.Collections.Generic.List[object]]::new()
$controlTrips = [System.Collections.Generic.List[double]]::new()

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
    throw "Did not receive READY from the backlash firmware on $PortName."
  }

  $port.WriteLine("START")
  $deadline = [DateTime]::UtcNow.AddMinutes($TimeoutMinutes)
  $completed = $false
  while ([DateTime]::UtcNow -lt $deadline) {
    try {
      $line = $port.ReadLine().Trim()
      $serialLines.Add($line)

      if ($line -like "CONTROL,*") {
        $parts = $line.Split(',')
        if ($parts.Count -ne 4) { throw "Malformed control line: $line" }
        $controlTrips.Add([double]::Parse($parts[2], $invariantCulture))
      }

      if ($line -like "DEADBAND,*") {
        $parts = $line.Split(',')
        if ($parts.Count -ne 5) { throw "Malformed deadband line: $line" }
        $deltaMotorDeg = [double]::Parse($parts[2], $invariantCulture)
        $outputDeg = [double]::Parse($parts[3], $invariantCulture)
        $rows.Add([PSCustomObject]@{
          run_utc = (Get-Date).ToUniversalTime().ToString("o")
          unit_id = $UnitId
          vref_volts = $VrefVolts.ToString("F2", $invariantCulture)
          cycle = [int]$parts[1]
          delta_motor_deg = $deltaMotorDeg.ToString("F4", $invariantCulture)
          deadband_output_deg = $outputDeg.ToString("F5", $invariantCulture)
          deadband_output_arcmin = ($outputDeg * 60.0).ToString("F3", $invariantCulture)
        })
      }

      if ($line -like "ABORT,*") {
        throw "Firmware aborted: $line"
      }
      if ($line -eq "RUN_COMPLETE") {
        $completed = $true
        break
      }
    } catch [System.TimeoutException] {
    }
  }
  if (-not $completed) {
    throw "Backlash run timed out after $TimeoutMinutes minutes."
  }
} finally {
  if ($port.IsOpen) {
    $port.Close()
  }
  $port.Dispose()
  [System.IO.File]::WriteAllLines($serialLogPath, $serialLines)
}

if ($rows.Count -eq 0) {
  throw "The run completed without dead-band rows. See $serialLogPath"
}

$rows | Export-Csv -Path $csvPath -NoTypeInformation -Encoding utf8
$rows | Format-Table -AutoSize

$arcmin = $rows | ForEach-Object { [double]::Parse($_.deadband_output_arcmin, $invariantCulture) }
$mean = ($arcmin | Measure-Object -Average).Average
$spread = ($arcmin | Measure-Object -Maximum -Minimum)
Write-Host ("Dead band (backlash + switch differential travel): {0:F2} arcmin mean, {1:F2}-{2:F2} range" -f $mean, $spread.Minimum, $spread.Maximum)

if ($controlTrips.Count -gt 1) {
  # Same-direction trip spread, converted to output arcmin: the noise floor the
  # dead-band number has to beat to be a real measurement.
  $controlSpreadDeg = ($controlTrips | Measure-Object -Maximum).Maximum - ($controlTrips | Measure-Object -Minimum).Minimum
  $controlArcmin = $controlSpreadDeg / 15.0 * 60.0
  Write-Host ("Control (same-direction) trip spread: {0:F2} arcmin at the output" -f $controlArcmin)
  if ($controlArcmin -ge $mean * 0.5) {
    Write-Warning "Control spread is a large fraction of the measured dead band. Tighten the fixture or increase settle time before trusting this number."
  }
}

Write-Host "Saved results: $csvPath"
Write-Host "Saved serial log: $serialLogPath"
