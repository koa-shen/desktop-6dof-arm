$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$outputPath = Join-Path $repoRoot 'docs\arm-design-workbook.xlsx'

$excel = $null
$workbook = $null

try {
    $excel = New-Object -ComObject Excel.Application
    $excel.Visible = $false
    $excel.DisplayAlerts = $false
    $workbook = $excel.Workbooks.Add()

    $summary = $workbook.Worksheets.Item(1)
    $summary.Name = 'Read Me'
    $bom = $workbook.Worksheets.Add()
    $bom.Name = 'BOM'
    $targets = $workbook.Worksheets.Add()
    $targets.Name = 'Targets'
    $curve = $workbook.Worksheets.Add()
    $curve.Name = 'Epicycloid'

    $navy = 0x55351F
    $blue = 0xD9E8F5
    $yellow = 0xD9F2FF

    $summary.Cells.Item(1, 1).Value2 = 'Desktop 6-DOF Arm | Design Workbook'
    $summary.Cells.Item(3, 1).Value2 = 'Scope'
    $summary.Cells.Item(3, 2).Value2 = 'Repo-derived snapshot. The project goal is a desktop 6-DOF manipulator; current work is Phase 2 characterization of one 15:1 cycloidal reducer.'
    $summary.Cells.Item(4, 1).Value2 = 'Evidence rule'
    $summary.Cells.Item(4, 2).Value2 = 'Documented values are separated from inferred planning quantities and unresolved targets. TBD values are intentionally not guessed.'
    $summary.Cells.Item(5, 1).Value2 = 'BOM'
    $summary.Cells.Item(5, 2).Value2 = 'Items and statuses are transcribed from repo docs. Six-joint quantities are planning counts only where marked inferred; confirm before purchasing.'
    $summary.Cells.Item(6, 1).Value2 = 'Epicycloid'
    $summary.Cells.Item(6, 2).Value2 = 'The Epicycloid tab implements a standard rolling-circle epicycloid and exports equation strings/coordinates. This is not the cycloidal reducer disk profile.'
    $summary.Cells.Item(7, 1).Value2 = 'Regenerate'
    $summary.Cells.Item(7, 2).Value2 = 'Run scripts\generate_arm_design_workbook.ps1 in Windows PowerShell with Microsoft Excel installed.'
    $summary.Cells.Item(9, 1).Value2 = 'Source documents'
    $summary.Cells.Item(10, 1).Value2 = 'docs/copilot-context.md'
    $summary.Cells.Item(11, 1).Value2 = 'docs/reducer-test-protocol.md'
    $summary.Cells.Item(12, 1).Value2 = 'hardware/pinouts/uno-tmc2209-as5600-tca9548a.md'
    $summary.Cells.Item(13, 1).Value2 = 'src/main.cpp'
    $summary.Range('A1:B1').Merge()
    $summary.Range('A1:B1').Interior.Color = $navy
    $summary.Range('A1:B1').Font.Color = 0xFFFFFF
    $summary.Range('A1:B1').Font.Bold = $true
    $summary.Range('A1:B1').Font.Size = 16
    $summary.Range('A3:A7').Font.Bold = $true
    $summary.Range('A9:A9').Font.Bold = $true
    $summary.Columns.Item(1).ColumnWidth = 24
    $summary.Columns.Item(2).ColumnWidth = 100
    $summary.Rows.Item(3).RowHeight = 38
    $summary.Rows.Item(4).RowHeight = 32
    $summary.Rows.Item(5).RowHeight = 38
    $summary.Rows.Item(6).RowHeight = 38
    $summary.Rows.Item(7).RowHeight = 32
    $summary.Range('A1:B13').WrapText = $true
    $summary.Range('A1:B13').VerticalAlignment = -4160

    $bomHeaders = @('Category', 'Item', 'Specification / role', 'Current bench qty', '6-DOF planning qty', 'Status', 'Evidence / caveat')
    for ($column = 0; $column -lt $bomHeaders.Count; $column++) {
        $bom.Cells.Item(1, $column + 1).Value2 = $bomHeaders[$column]
    }
    $bomRows = @(
        @('Actuation', 'NEMA 17 stepper motor', '1.5 A, 42 N·cm torque; selected architecture', 'TBD', '6 (inferred)', 'Design decision', 'docs/copilot-context.md; exact installed motor count is not stated.'),
        @('Actuation', 'TMC2209 stepper driver', 'Step/dir driver; EN active LOW on typical modules', '1 axis under test', '6 (inferred)', 'In use / planned', 'src/main.cpp; one driver per joint assumed for planning.'),
        @('Sensing', 'AS5600 magnetic encoder', 'Single-turn absolute, 0–360°; currently on motor shaft', '1', '6 (inferred)', 'In use / planned', 'Motor-side sensing does not measure reducer output error or backlash.'),
        @('Sensing', 'Diametric magnet', 'AS5600 shaft magnet', '1', '6 (inferred)', 'Required / count TBD', 'One per encoder; exact magnet specification is not documented.'),
        @('Drivetrain', 'Cycloidal reducer', '15:1 assumed ratio; manufactured units on bench', 'Count TBD', '6 (inferred)', 'Bench characterization', 'docs/copilot-context.md; design dimensions and measured performance TBD.'),
        @('Drivetrain', 'AXK2542 needle thrust bearing', '25 mm bore; base axial/moment support', '1 ordered', '1', 'Ordered', 'docs/copilot-context.md; base joint.'),
        @('Control', 'Arduino Uno', 'Phase 1/2 controller', '1', 'TBD', 'Available / in use', 'Multi-axis controller architecture is not selected.'),
        @('Control', 'TCA9548A I²C mux', 'Deferred until multiple encoders are tested', '0', 'TBD', 'Deferred', 'Single AS5600 is wired directly to Uno A4/A5; mux is not currently used.'),
        @('Test equipment', 'HX711 load-cell amplifier', 'Backlash dead-band test', 'Ordered; qty TBD', '1 test setup', 'Ordered', 'docs/copilot-context.md; not required per joint for the planned bench method.'),
        @('Test equipment', 'Kitchen-scale load cell', '5 kg full scale, about 49 N; use low preload for calibration', '1', '1 test setup', 'Available / test setup', 'docs/copilot-context.md and docs/reducer-test-protocol.md.'),
        @('Test equipment', 'Omron V-156-1C25 switch', 'SPDT snap-action; NO to D7, LOW while touching', '1 required', '1 test setup', 'Required for backlash test', 'hardware/pinouts/uno-tmc2209-as5600-tca9548a.md.'),
        @('Power', 'Bench power supply', '30 V, 10 A', '1', '1 shared', 'Available', 'docs/copilot-context.md; driver VMOT supply.'),
        @('Power', 'VMOT bulk capacitor', '100–470 µF electrolytic near driver', 'TBD', '6 (inferred)', 'Required / qty TBD', 'hardware/pinouts/uno-tmc2209-as5600-tca9548a.md; per-driver recommendation.'),
        @('End effector', '3D-printed rack-and-pinion gripper', 'One gripper; controlled holding force desired', '1 concept', '1', 'Concept / actuator TBD', 'Pneumatic actuation preferred; servo alternative. Finger geometry and force range are not specified.'),
        @('Structure', 'M3 fastener kit', 'General assembly fasteners', '1 kit', 'TBD', 'Available', 'docs/copilot-context.md; detailed fastener schedule not designed.'),
        @('Structure', 'Printed structural parts', 'PETG recommended; avoid PLA near warm load-bearing motor parts', 'TBD', 'TBD', 'Design / fabricate', 'Bambu P1S available; final part count and mass TBD.'),
        @('Structure', 'TPU strain relief / grommets', 'Cable strain relief', 'TBD', 'TBD', 'Optional / design', 'Material guidance in docs/copilot-context.md.'),
        @('Structure', 'Arm links, joints, output hubs, covers', 'Geometry, material, and fastener schedule not specified', 'TBD', 'TBD', 'Not designed', 'Required for full arm; no dimensions or counts in repo.'),
        @('Wiring', 'Motor / sensor wiring and connectors', 'Separate sensor wiring from motor power; shared ground', 'TBD', 'TBD', 'Required / qty TBD', 'Exact wire gauges, lengths, and connector series are not documented.'),
        @('Wiring', 'Arduino Uno / driver logic wiring', 'D2 STEP, D3 DIR, D4 EN; A4/A5 AS5600; D5/D6 HX711; D7 switch', '1 axis', 'TBD', 'In use / expand later', 'Pinout from hardware/pinouts/uno-tmc2209-as5600-tca9548a.md.')
    )
    for ($row = 0; $row -lt $bomRows.Count; $row++) {
        for ($column = 0; $column -lt $bomRows[$row].Count; $column++) {
            $bom.Cells.Item($row + 2, $column + 1).Value2 = $bomRows[$row][$column]
        }
    }
    $bom.Range('A1:G1').Font.Bold = $true
    $bom.Range('A1:G1').Interior.Color = $blue
    $bom.Range("A1:G$($bomRows.Count + 1)").WrapText = $true
    $bom.Range("A1:G$($bomRows.Count + 1)").VerticalAlignment = -4160
    $bom.Columns.Item(1).ColumnWidth = 16
    $bom.Columns.Item(2).ColumnWidth = 32
    $bom.Columns.Item(3).ColumnWidth = 54
    $bom.Columns.Item(4).ColumnWidth = 19
    $bom.Columns.Item(5).ColumnWidth = 21
    $bom.Columns.Item(6).ColumnWidth = 25
    $bom.Columns.Item(7).ColumnWidth = 70
    $bom.Rows.Item(1).AutoFilter() | Out-Null

    $targetHeaders = @('Area', 'Target / parameter', 'Value', 'Unit', 'Status', 'Source / notes')
    for ($column = 0; $column -lt $targetHeaders.Count; $column++) {
        $targets.Cells.Item(1, $column + 1).Value2 = $targetHeaders[$column]
    }
    $targetRows = @(
        @('Mission', 'Degrees of freedom', '6', 'axes', 'Project goal', 'docs/copilot-context.md'),
        @('Arm performance', 'Useful payload capacity', '0.5', 'kg', 'Provisional target', 'Tool/gripper mass excluded from useful payload; include it in joint load calculations. User target; not yet validated.'),
        @('Arm geometry', 'Maximum reach to tool center point', '381', 'mm (about 15 in)', 'Provisional target', 'Measure from base rotation axis to tool center point at full extension; desk workspace layout TBD.'),
        @('Arm performance', 'Opposite-side workspace transfer', '5 max', 's', 'Provisional maximum', 'With full target payload; endpoints/path and inclusion of grasp/release time need confirmation.'),
        @('End effector', 'Gripper concept', '3D-printed rack-and-pinion', '', 'Concept', 'Pneumatic actuation with controlled holding force preferred; servo is an alternative. Not selected or tested.'),
        @('Demo use case', 'Phone / compact camera', 'Aspirational', '', 'Candidate demo', 'Specific device and mount must fit within the 0.5 kg useful payload budget.'),
        @('Arm performance', 'Payload-only static shoulder moment at full reach', '1.87', 'N·m', 'Derived lower bound', '0.5 kg x 9.81 m/s² x 0.381 m; excludes gripper, links, friction, and dynamic margin.'),
        @('Arm performance', 'Whole-arm accuracy / repeatability', 'TBD', 'mm', 'Not specified', 'Needs whole-arm output measurement; current AS5600 is motor-side.'),
        @('Arm performance', 'Output accuracy / repeatability', 'TBD', 'deg or mm', 'Not specified', 'Needs output-side measurement; present AS5600 is motor-side.'),
        @('Joint actuation', 'Motor', 'NEMA 17; 1.5 A; 42', 'A; N·cm', 'Architecture decision', 'docs/copilot-context.md; verify exact purchased motor.'),
        @('Joint drivetrain', 'Reducer ratio', '15:1', 'ratio', 'Assumed in firmware', 'src/main.cpp; verify actual manufactured unit.'),
        @('Motor control', 'Full steps / revolution', '200', 'steps/rev', 'Configured', 'src/main.cpp; corresponds to 1.8° stepper.'),
        @('Motor control', 'Microsteps', '1', 'microsteps/full step', 'Configured; verify hardware', 'src/main.cpp; must match TMC2209 MS configuration.'),
        @('Reducer test', 'Sweep output range', '60', 'deg', 'Configured', 'src/main.cpp; per directional sweep.'),
        @('Reducer test', 'Sweep cycles', '3', 'cycles', 'Configured', 'src/main.cpp.'),
        @('Reducer test', 'Load-cell preload', '1.5', 'N', 'Configured target', 'src/main.cpp; calibrate against known low-range weight.'),
        @('Reducer test', 'Force-zero threshold', '0.05', 'N', 'Placeholder; calibrate', 'src/main.cpp explicitly says set from measured noise floor.'),
        @('Reducer test', 'Load-cell capacity', 'about 49', 'N', 'Known', '5 kg kitchen-scale cell; test preload is only about 1–2 N.'),
        @('Reducer test', 'Backlash and breakaway torque specs', 'TBD', 'arcmin; N·m', 'Measure before setting', 'docs/reducer-test-protocol.md; no acceptance limits documented.'),
        @('Electrical', 'Bench supply', '30 V, 10 A', 'V; A', 'Available', 'docs/copilot-context.md; not a final arm power budget.'),
        @('Electrical', 'Encoder range', '0–360', 'deg', 'AS5600 single-turn', 'Motor-shaft measurement only in current mounting.'),
        @('Phase', 'Current development phase', 'Phase 2', '', 'Reducer characterization', 'Full 6-axis arm build targets remain to be specified.')
    )
    for ($row = 0; $row -lt $targetRows.Count; $row++) {
        for ($column = 0; $column -lt $targetRows[$row].Count; $column++) {
            $targets.Cells.Item($row + 2, $column + 1).Value2 = $targetRows[$row][$column]
        }
    }
    $targets.Range('A1:F1').Font.Bold = $true
    $targets.Range('A1:F1').Interior.Color = $blue
    $targets.Range("A1:F$($targetRows.Count + 1)").WrapText = $true
    $targets.Range("A1:F$($targetRows.Count + 1)").VerticalAlignment = -4160
    $targets.Columns.Item(1).ColumnWidth = 22
    $targets.Columns.Item(2).ColumnWidth = 36
    $targets.Columns.Item(3).ColumnWidth = 24
    $targets.Columns.Item(4).ColumnWidth = 20
    $targets.Columns.Item(5).ColumnWidth = 26
    $targets.Columns.Item(6).ColumnWidth = 74
    $targets.Rows.Item(1).AutoFilter() | Out-Null

    $curve.Cells.Item(1, 1).Value2 = 'Standard Epicycloid Calculator'
    $curve.Range('A1:E1').Merge()
    $curve.Range('A1:E1').Interior.Color = $navy
    $curve.Range('A1:E1').Font.Color = 0xFFFFFF
    $curve.Range('A1:E1').Font.Bold = $true
    $curve.Range('A1:E1').Font.Size = 15
    $curve.Cells.Item(2, 1).Value2 = 'Not the cycloidal reducer disk profile; use only when a standard epicycloid is intended.'
    $curve.Range('A2:H2').Merge()
    $curve.Range('A2:H2').WrapText = $true
    $curve.Cells.Item(3, 1).Value2 = 'Fixed-circle radius R'
    $curve.Cells.Item(3, 2).Value2 = 25
    $curve.Cells.Item(4, 1).Value2 = 'Rolling-circle radius r'
    $curve.Cells.Item(4, 2).Value2 = 5
    $curve.Cells.Item(5, 1).Value2 = 'Length unit'
    $curve.Cells.Item(5, 2).Value2 = 'mm'
    $curve.Cells.Item(6, 1).Value2 = 'Parameter start angle'
    $curve.Cells.Item(6, 2).Value2 = 0
    $curve.Cells.Item(6, 3).Value2 = 'deg'
    $curve.Cells.Item(7, 1).Value2 = 'Parameter end angle'
    $curve.Cells.Item(7, 2).Value2 = 360
    $curve.Cells.Item(7, 3).Value2 = 'deg'
    $curve.Cells.Item(8, 1).Value2 = 'Coordinate sample count'
    $curve.Cells.Item(8, 2).Value2 = 361
    $curve.Cells.Item(9, 1).Value2 = 'Cusps per complete curve (if integer)'
    $curve.Cells.Item(9, 2).Formula = '=IFERROR((B3+B4)/B4,"invalid r")'
    $curve.Cells.Item(10, 1).Value2 = 'SolidWorks parameter t'
    $curve.Cells.Item(10, 2).Value2 = 'Radians; x and y below use t'
    $curve.Cells.Item(11, 1).Value2 = 'SolidWorks X(t)'
    $curve.Cells.Item(11, 2).Formula = '="x(t) = ("&TEXT(B3,"0.000")&"mm+"&TEXT(B4,"0.000")&"mm)*cos(t)-"&TEXT(B4,"0.000")&"mm*cos((("&TEXT(B3,"0.000")&"mm+"&TEXT(B4,"0.000")&"mm)/"&TEXT(B4,"0.000")&"mm)*t)"'
    $curve.Cells.Item(12, 1).Value2 = 'SolidWorks Y(t)'
    $curve.Cells.Item(12, 2).Formula = '="y(t) = ("&TEXT(B3,"0.000")&"mm+"&TEXT(B4,"0.000")&"mm)*sin(t)-"&TEXT(B4,"0.000")&"mm*sin((("&TEXT(B3,"0.000")&"mm+"&TEXT(B4,"0.000")&"mm)/"&TEXT(B4,"0.000")&"mm)*t)"'
    $curve.Range('B3:B8').Interior.Color = $yellow
    $curve.Cells.Item(14, 1).Value2 = 'Sample'
    $curve.Cells.Item(14, 2).Value2 = 'Angle (deg)'
    $curve.Cells.Item(14, 3).Value2 = 't (rad)'
    $curve.Cells.Item(14, 4).Value2 = 'X (mm)'
    $curve.Cells.Item(14, 5).Value2 = 'Y (mm)'
    $curve.Range('A14:E14').Font.Bold = $true
    $curve.Range('A14:E14').Interior.Color = $blue
    for ($row = 15; $row -le 1014; $row++) {
        $curve.Cells.Item($row, 1).Formula = '=ROW()-ROW($A$14)'
        $curve.Cells.Item($row, 2).Formula = '=IF(A' + $row + '<=$B$8,$B$6+(A' + $row + '-1)*($B$7-$B$6)/MAX(1,$B$8-1),NA())'
        $curve.Cells.Item($row, 3).Formula = '=RADIANS(B' + $row + ')'
        $curve.Cells.Item($row, 4).Formula = '=($B$3+$B$4)*COS(C' + $row + ')-$B$4*COS((($B$3+$B$4)/$B$4)*C' + $row + ')'
        $curve.Cells.Item($row, 5).Formula = '=($B$3+$B$4)*SIN(C' + $row + ')-$B$4*SIN((($B$3+$B$4)/$B$4)*C' + $row + ')'
    }
    $curve.Range('A2:E1014').WrapText = $true
    $curve.Columns.Item(1).ColumnWidth = 35
    $curve.Columns.Item(2).ColumnWidth = 92
    $curve.Columns.Item(3).ColumnWidth = 22
    $curve.Columns.Item(4).ColumnWidth = 18
    $curve.Columns.Item(5).ColumnWidth = 18
    $curve.Rows.Item(2).RowHeight = 30
    $curve.Range('A3:A12').Font.Bold = $true

    foreach ($sheet in @($summary, $bom, $targets, $curve)) {
        $sheet.UsedRange.Font.Name = 'Aptos'
        $sheet.UsedRange.Font.Size = 10
    }
    $summary.Range('A1:B1').Font.Name = 'Aptos Display'
    $curve.Range('A1:E1').Font.Name = 'Aptos Display'

    $excel.Calculation = -4105
    $excel.CalculateFull()
    if (Test-Path $outputPath) {
        Remove-Item $outputPath -Force
    }
    $workbook.SaveAs($outputPath, 51)
    $workbook.Close($true)
    $excel.Quit()
    Write-Output "Generated $outputPath"
}
finally {
    if ($workbook -ne $null) {
        try { $workbook.Close($false) } catch { }
    }
    if ($excel -ne $null) {
        try { $excel.Quit() } catch { }
    }
}