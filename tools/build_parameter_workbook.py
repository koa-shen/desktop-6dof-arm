"""Build the human-facing arm parameter, BOM, and test-record workbook.

Run from the repository root:
    python tools/build_parameter_workbook.py

The tracked generator is the audit trail for initialized values. The generated
docs/arm-parameters.xlsx is the working artifact for design reviews and test
records. Update the generator when a confirmed design value changes; enter
measured results directly in the workbook's Test Log sheet.
"""

from __future__ import annotations

from pathlib import Path

from openpyxl import Workbook, load_workbook
from openpyxl.formatting.rule import CellIsRule
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.worksheet.datavalidation import DataValidation

from cycloidal_layout import CycloidalLayout

OUTPUT = Path(__file__).parents[1] / "docs" / "arm-parameters.xlsx"
NAVY = "17365D"
TEAL = "0F6B78"
PALE_BLUE = "D9EAF7"
PALE_YELLOW = "FFF2CC"
PALE_RED = "F4CCCC"
WHITE = "FFFFFF"
THIN = Side(style="thin", color="A6A6A6")


def style_sheet(sheet, widths: dict[str, float]) -> None:
    sheet.freeze_panes = "A2"
    sheet.sheet_view.showGridLines = False
    for column, width in widths.items():
        sheet.column_dimensions[column].width = width


def title(sheet, text: str) -> None:
    sheet.merge_cells("A1:G1")
    cell = sheet["A1"]
    cell.value = text
    cell.font = Font(bold=True, color=WHITE, size=14)
    cell.fill = PatternFill("solid", fgColor=NAVY)
    cell.alignment = Alignment(vertical="center")
    sheet.row_dimensions[1].height = 25


def header(sheet, row: int, values: list[str]) -> None:
    for column, value in enumerate(values, start=1):
        cell = sheet.cell(row, column, value)
        cell.font = Font(bold=True, color=WHITE)
        cell.fill = PatternFill("solid", fgColor=TEAL)
        cell.alignment = Alignment(wrap_text=True, vertical="center")
        cell.border = Border(bottom=THIN)
    sheet.row_dimensions[row].height = 30


def table_rows(sheet, start_row: int, rows: list[list[object]]) -> None:
    for row_number, values in enumerate(rows, start=start_row):
        for column, value in enumerate(values, start=1):
            cell = sheet.cell(row_number, column, value)
            cell.alignment = Alignment(vertical="top", wrap_text=True)
            cell.border = Border(bottom=THIN)
        if row_number % 2 == 0:
            for column in range(1, len(values) + 1):
                sheet.cell(row_number, column).fill = PatternFill("solid", fgColor="F5F9FC")


def build_index(workbook: Workbook) -> None:
    sheet = workbook.active
    sheet.title = "Index"
    style_sheet(sheet, {"A": 28, "B": 75})
    title(sheet, "Desktop 6-DOF Arm - Engineering Workbook")
    table_rows(sheet, 3, [
        ["Purpose", "Working record for confirmed parameters, procurements, CAD inputs, and measured test results."],
        ["Generated", "Run `python tools/build_parameter_workbook.py` from the repository root."],
        ["Source control", "The generator and Markdown documents are reviewable source. The XLSX is the human-facing operational record."],
        ["Update rule", "Change confirmed design values in the generator and regenerate. Add raw observations and measured outputs directly to Test Log; regeneration preserves all existing rows."],
        ["Status meaning", "Confirmed = measured or fixed in CAD; Assumed = model input; Pending = needs measurement; Failing = known constraint violation."],
    ])


def build_parameters(workbook: Workbook, layout: CycloidalLayout) -> None:
    sheet = workbook.create_sheet("Parameters")
    style_sheet(sheet, {"A": 22, "B": 30, "C": 14, "D": 16, "E": 22, "F": 46, "G": 28})
    title(sheet, "Controlled Design Parameters")
    header(sheet, 3, ["Subsystem", "Parameter", "Value", "Unit", "Status", "Source / date", "Notes"])
    rows = [
        ["Reducer", "Ring pin count", layout.pins, "count", "Confirmed", "CAD / 2026-09-15", "Integral housing pins"],
        ["Reducer", "Disc lobe count", layout.lobes, "count", "Confirmed", "CAD / 2026-09-15", "Fixed ring, one-tooth difference"],
        ["Reducer", "Nominal ratio", layout.ratio, ":1", "Confirmed", "CAD / 2026-09-15", "Measure actual ratio during calibration"],
        ["Reducer", "Ring-pin-circle radius", layout.pin_circle_r, "mm", "Confirmed", "CAD / 2026-09-15", "36 mm diameter"],
        ["Reducer", "Ring pin diameter", 2 * layout.pin_r, "mm", "Confirmed", "CAD / 2026-09-15", "PETG, approximately half-embedded in wall"],
        ["Reducer", "Eccentricity", layout.eccentricity, "mm", "Confirmed", "CAD / 2026-09-15", "K = e*N/R"],
        ["Reducer", "Disc thickness", layout.disc_thickness, "mm", "Confirmed", "CAD / 2026-09-15", "Two discs, 180 deg indexed"],
        ["Reducer", "Disc count", layout.disc_count, "count", "Confirmed", "CAD / 2026-09-15", ""],
        ["Reducer", "Output shafts", layout.output_pins, "count", "Confirmed", "CAD / 2026-09-15", "M3 shoulder bolts, 4 mm shoulders"],
        ["Reducer", "Output-shaft circle diameter", 2 * layout.output_circle_r, "mm", "Confirmed", "CAD / 2026-09-15", ""],
        ["Reducer", "PTFE spacer thickness", 0.8, "mm", "Confirmed", "CAD / 2026-09-15", "Three spacers"],
        ["Reducer", "Housing wall", layout.housing_wall, "mm", "Assumed", "Layout model", "Replace with minimum CAD wall thickness"],
        ["Reducer", "Minimum feature web", layout.web, "mm", "Assumed", "Layout model", "Model check only"],
        ["Bearings", "Output flange", "6705-2RS", "part no.", "Confirmed", "Selection / 2026-09-15", "25 mm flange land, axial seating lip"],
        ["Bearings", "Disc center bore", "MR148ZZ", "part no.", "Confirmed", "Selection / 2026-09-15", ""],
        ["Bearings", "Crankshaft", "687ZZ", "part no.", "Confirmed", "Selection / 2026-09-15", "Light press in each output flange"],
        ["Materials", "Reducer components", "PETG", "material", "Confirmed", "Selection / 2026-09-15", "Except encoder housing"],
        ["Materials", "Encoder housing", "PLA", "material", "Confirmed", "Selection / 2026-09-15", "Clip-on mount"],
        ["Materials", "Lubricant", "Super Lube Multi-Purpose Synthetic Grease", "product", "Confirmed", "Selection / 2026-09-15", "PETG stressed-coupon result pending"],
        ["Motor", "First motor", "NEMA 17 x 38 mm", "form factor", "Confirmed", "Selection / 2026-09-15", "Driver current and Vref to verify"],
        ["Encoder", "Joint position sensor", "AS5600", "part no.", "Confirmed", "Selection / 2026-09-15", "Magnet and gap pending"],
    ]
    table_rows(sheet, 4, rows)
    validation = DataValidation(type="list", formula1='"Confirmed,Assumed,Pending,Failing"')
    sheet.add_data_validation(validation)
    validation.add(f"E4:E{3 + len(rows)}")
    sheet.conditional_formatting.add(f"E4:E{3 + len(rows)}", CellIsRule(operator="equal", formula=['"Pending"'], fill=PatternFill("solid", fgColor=PALE_YELLOW)))
    sheet.conditional_formatting.add(f"E4:E{3 + len(rows)}", CellIsRule(operator="equal", formula=['"Failing"'], fill=PatternFill("solid", fgColor=PALE_RED)))


def build_bom(workbook: Workbook) -> None:
    sheet = workbook.create_sheet("BOM")
    style_sheet(sheet, {"A": 16, "B": 30, "C": 16, "D": 12, "E": 16, "F": 42, "G": 28})
    title(sheet, "Phase 1 Reducer and Control Electronics BOM")
    header(sheet, 3, ["Subsystem", "Item", "Part / specification", "Qty", "Status", "Purpose", "Supplier / notes"])
    table_rows(sheet, 4, [
        ["Control", "Microcontroller", "Arduino Uno", 1, "On hand", "Phase 1 step generation and I2C control", "Elegoo starter kit"],
        ["Motor control", "Stepper driver", "TMC2209 StepStick module", 1, "On hand", "NEMA 17 step/dir drive", "Heatsink fitted; Vref/current to verify"],
        ["Sensing", "Absolute encoder", "AS5600 magnetic encoder breakout", 1, "On hand", "Output-side joint angle", "First test wires directly to Uno I2C"],
        ["Sensing", "I2C mux", "TCA9548A breakout, address 0x70", 1, "On hand", "Separates multiple AS5600 encoders with shared address 0x36", "Not needed for one direct-wired encoder; required from joint 2"],
        ["Power", "DC-DC converter", "5 V buck converter", "TBD", "On hand", "Regulated logic/encoder supply when required", "Record selected module, input, and output voltage before use"],
        ["Output", "Main bearing", "6705-2RS", 2, "Selected", "Supports output flanges", "Flange land 25 mm; axial lip"],
        ["Cycloidal", "Disc-center bearing", "MR148ZZ", 2, "Selected", "Supports cycloidal disc center bore", ""],
        ["Input", "Crankshaft bearing", "687ZZ", 2, "Selected", "Supports PETG crankshaft in output flanges", "Light press"],
        ["Cycloidal", "Cycloidal discs", "PETG, 15 lobes, 4 mm", 2, "Print", "15:1 stage; 180 deg indexed", ""],
        ["Cycloidal", "Integral ring pins", "PETG, 3.5 mm diameter", 16, "Print", "Fixed ring; side-embedded in housing", "CAD root geometry pending"],
        ["Output", "Output shafts", "M3 shoulder bolt, 4 mm shoulder", 6, "Procure", "Transfers disc motion; retains core", "23 mm diameter circle"],
        ["Output", "PTFE spacer", "0.8 mm", 3, "Procure", "Disc-to-flange separation", ""],
        ["Interface", "Link inserts", "M3 heat-set insert", 6, "Procure", "Linkage connection", "16 mm diameter circle"],
        ["Fasteners", "Internal fasteners", "M3", "TBD", "Procure", "Reducer assembly", "Lengths pending flange design"],
        ["Input", "Crankshaft", "PETG, NEMA 17 shaft-flat drive", 1, "Print", "Eccentric input", ""],
        ["Encoder", "Encoder", "AS5600 absolute magnetic", 1, "On hand", "Output-side joint angle", "Magnet and gap pending"],
        ["Lubrication", "Grease", "Super Lube Multi-Purpose Synthetic", 1, "On hand", "Run-in and friction reduction", "Coupon compatibility test pending"],
    ])


def build_curve(workbook: Workbook, layout: CycloidalLayout) -> None:
    sheet = workbook.create_sheet("Cycloidal Curve")
    style_sheet(sheet, {"A": 18, "B": 18, "C": 18, "D": 18, "E": 18, "F": 42, "G": 18})
    title(sheet, "Cycloidal Profile - SolidWorks Equation Calculator")
    header(sheet, 3, ["Input", "Value", "Unit", "Formula / interpretation", "", "", ""])
    table_rows(sheet, 4, [
        ["N", layout.pins, "count", "Ring pins"],
        ["R", layout.pin_circle_r, "mm", "Ring-pin-circle radius"],
        ["Rr", layout.pin_r, "mm", "Ring pin radius"],
        ["e", layout.eccentricity, "mm", "Eccentricity"],
        ["Lobes", "=B4-1", "count", "N - 1"],
        ["Ratio", "=B8", ":1", "N - 1 : 1"],
        ["K", "=B7*B4/B5", "", "e*N/R; practical target 0.4 to 0.8"],
    ])
    sheet["A12"] = "SolidWorks equations (t in radians)"
    sheet["A12"].font = Font(bold=True, color=WHITE)
    sheet["A12"].fill = PatternFill("solid", fgColor=TEAL)
    sheet.merge_cells("A12:F12")
    equations = [
        ["psi(t)", "atan2(sin((1-N)*t), R/(e*N)-cos((1-N)*t))"],
        ["x(t)", "R*cos(t)-Rr*cos(t+psi)-e*cos(N*t)"],
        ["y(t)", "-R*sin(t)+Rr*sin(t+psi)+e*sin(N*t)"],
    ]
    table_rows(sheet, 13, equations)
    header(sheet, 18, ["t (deg)", "t (rad)", "psi (rad)", "x (mm)", "y (mm)", "Use", ""])
    for row in range(19, 380):
        previous = row - 1
        sheet.cell(row, 1, 0 if row == 19 else f"=A{previous}+1")
        sheet.cell(row, 2, f"=RADIANS(A{row})")
        sheet.cell(row, 3, f"=ATAN2(SIN((1-$B$4)*B{row}),$B$5/($B$7*$B$4)-COS((1-$B$4)*B{row}))")
        sheet.cell(row, 4, f"=$B$5*COS(B{row})-$B$6*COS(B{row}+C{row})-$B$7*COS($B$4*B{row})")
        sheet.cell(row, 5, f"=-$B$5*SIN(B{row})+$B$6*SIN(B{row}+C{row})+$B$7*SIN($B$4*B{row})")
        sheet.cell(row, 6, "Copy x(t), y(t) into SolidWorks Equation Driven Curve")


def build_test_plan(workbook: Workbook) -> None:
    sheet = workbook.create_sheet("Test Plan")
    style_sheet(sheet, {"A": 14, "B": 28, "C": 40, "D": 40, "E": 26, "F": 16, "G": 28})
    title(sheet, "Phase 1B Test Plan and Acceptance Criteria")
    header(sheet, 3, ["Test ID", "Test", "Controlled inputs", "Record outputs", "Acceptance / decision", "Status", "Notes"])
    table_rows(sheet, 4, [
        ["P1B-01", "Hand-turn inspection", "Unpowered, before then after light grease", "Tight-spot angle and direction", "No bind through full input revolution", "Planned", "Repeat whenever assembly changes"],
        ["P1B-02", "Output encoder by hand", "Magnet gap and output rotation", "AGC, magnet state, 360 deg continuity", "No status loss or angle jumps", "Pending", "Run app_01_encoder_test before motor power"],
        ["P1B-03", "Direct-step reducer inspection", "app_10_reducer_bench; 160 then 1600 motor-step jogs", "Output direction, AS5600 motion, noise, I2C errors", "No binding, heat, or encoder fault", "Planned", "GEAR_RATIO intentionally unused"],
        ["P1B-04", "Unloaded run-in", "Verified driver current, input RPM, duration, direction", "Temperature, current, noise, resistance before/after", "10-15 min each direction; stop on binding or rapid heat", "Planned", "Wipe plastic debris before final grease"],
        ["P1B-05", "Actual ratio", "Mark input/output; count input turns", "Input turns per output turn", "Measured value becomes firmware GEAR_RATIO", "Planned", "Do not use nominal 15:1 without this"],
        ["P1B-06", "Backlash", "Output vertical or counterbalanced; 100 mm lever", "Dial displacement and AS5600 angle, both directions", "Report degrees and repeatability", "Planned", "Avoid gravity preload masking backlash"],
        ["P1B-07", "Static stiffness", "Known mass, lever radius, direction", "Torque, AS5600 deflection, residual after unload", "Calculate k = torque / deflection", "Planned", "Inspect integrated pin-wall opening"],
        ["P1B-08", "Efficiency / pullout", "Current, motor speed, lever radius, fish-scale force", "Output torque at stall, motor temperature", "Compare to motor torque x ratio", "Planned", "Start unloaded; test at conservative current"],
        ["P1B-09", "PETG grease coupon", "Bent PETG coupon, Super Lube exposure", "Crazing, crack, stiffness change after 12 h", "No visible craze or embrittlement", "Planned", "Test at disc print settings"],
    ])
    validation = DataValidation(type="list", formula1='"Planned,Running,Pass,Fail,Blocked,Pending"')
    sheet.add_data_validation(validation)
    validation.add("F4:F12")


def initial_test_log_rows() -> list[tuple[object, ...]]:
    return [
        ("2026-08-31", "P1A-01", "Uno + TMC2209 + NEMA 17; 8x microstep; no encoder or reducer",
         "12 V PSU, about 1 A limit; 900 / 1066 / 1600 steps/s",
         "Smooth at all tested rates after current adjustment; motor and driver cold",
         "Pass", "docs/test-results/2026-08-31-open-loop-motor-bench.txt"),
    ]


def existing_test_log_rows() -> list[tuple[object, ...]]:
    if not OUTPUT.exists():
        return []
    old_workbook = load_workbook(OUTPUT, read_only=True, data_only=False)
    if "Test Log" not in old_workbook.sheetnames:
        return []
    sheet = old_workbook["Test Log"]
    return [row for row in sheet.iter_rows(min_row=5, max_col=7, values_only=True)
            if any(value is not None for value in row)]


def build_test_log(workbook: Workbook, prior_rows: list[tuple[object, ...]]) -> None:
    sheet = workbook.create_sheet("Test Log")
    style_sheet(sheet, {"A": 14, "B": 14, "C": 14, "D": 16, "E": 18, "F": 20, "G": 20})
    title(sheet, "Test Log - Append One Row Per Run")
    header(sheet, 3, ["Date", "Test ID", "Configuration", "Input / setpoint", "Measured output", "Pass / fail", "Raw data path / notes"])
    sheet["A4"] = "Enter a new row for each run; keep serial CSV files in docs/test-results/."
    sheet.merge_cells("A4:G4")
    sheet["A4"].fill = PatternFill("solid", fgColor=PALE_YELLOW)
    sheet["A4"].alignment = Alignment(wrap_text=True)
    table_rows(sheet, 5, [list(row) for row in prior_rows])


def build_open_items(workbook: Workbook) -> None:
    sheet = workbook.create_sheet("CAD and Open Items")
    style_sheet(sheet, {"A": 14, "B": 32, "C": 46, "D": 22, "E": 20, "F": 20, "G": 26})
    title(sheet, "Open CAD Inputs and Model Constraints")
    header(sheet, 3, ["ID", "Item", "Required measurement / decision", "Why it matters", "Status", "Owner", "Resolution"])
    table_rows(sheet, 4, [
        ["CAD-01", "Integral ring-pin geometry", "Axial engagement length; radial embed depth or exposed arc angle", "Required to model housing opening and pin-root load path", "Pending", "CAD review", ""],
        ["CAD-02", "Housing behind pin", "Minimum back-wall thickness and root fillet radius", "Controls local bending, peel, and creep", "Pending", "CAD review", ""],
        ["CAD-03", "Housing stack", "Axial side-wall thickness and overall housing diameter", "Confirms stiffness and package envelope", "Pending", "CAD review", ""],
        ["CAD-04", "Output-shaft clearance", "Actual MR148, 4 mm shafts, and 23 mm-circle clearance in CAD", "Current conservative model has +0.35 mm radial overrun", "Failing", "CAD review", ""],
        ["CAD-05", "NEMA 17 envelope", "Actual external housing diameter/wall", "Current 2 mm-wall model reaches 43.5 mm diameter", "Pending", "CAD review", ""],
        ["ELEC-01", "Motor setup", "Motor part number, PSU voltage, TMC2209 Vref/current", "Sets safe initial run-in speed and load", "Pending", "Bench", ""],
        ["ENC-01", "Encoder mechanics", "Magnet size, material, gap, and retention", "Needed before encoder motion validation", "Pending", "CAD review", ""],
    ])
    validation = DataValidation(type="list", formula1='"Pending,In progress,Confirmed,Failing,Closed"')
    sheet.add_data_validation(validation)
    validation.add("E4:E10")


def main() -> None:
    layout = CycloidalLayout()
    prior_test_log_rows = existing_test_log_rows()
    initial_keys = {(row[0], row[1]) for row in initial_test_log_rows()}
    prior_test_log_rows = initial_test_log_rows() + [
        row for row in prior_test_log_rows if (row[0], row[1]) not in initial_keys
    ]
    workbook = Workbook()
    workbook.properties.title = "Desktop 6-DOF Arm Engineering Record"
    workbook.properties.creator = "Desktop 6-DOF Arm"
    build_index(workbook)
    build_parameters(workbook, layout)
    build_bom(workbook)
    build_curve(workbook, layout)
    build_test_plan(workbook)
    build_test_log(workbook, prior_test_log_rows)
    build_open_items(workbook)
    workbook.save(OUTPUT)
    print(f"Wrote {OUTPUT}")


if __name__ == "__main__":
    main()