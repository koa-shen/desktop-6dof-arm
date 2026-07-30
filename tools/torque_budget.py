"""Where does this arm's reach/payload envelope actually land?

Two hard ceilings decide it, and neither is the motor:

  1. The printed cycloidal gearbox. A 42x42x26 mm printed drive is good for
     roughly 4-5 N.m (Sweep Dynamics' published rating for that envelope).
  2. The print bed. A link printed lying flat has its layer planes parallel to
     the bending load; printed standing up it has them perpendicular, at
     40-70 % of the strength. So single-piece link length is capped by the bed
     footprint, not the build height.

Run:  python tools/torque_budget.py
"""

from __future__ import annotations

import math

from arm_model import G, default_arm

# --- ceilings -------------------------------------------------------------
BED = 0.256                     # Bambu P1S, one axis
BED_USABLE = BED - 0.016        # brim, skirt, corner exclusion
BED_DIAGONAL = BED_USABLE * math.sqrt(2)

GEARBOX_RATED = 5.0             # N.m, printed cycloidal in a NEMA 17 envelope
GEARBOX_HIGH = 8.0              # the "high torque" variant of the same envelope
GEARBOX_EFF = 0.80
DESIGN_FACTOR = 1.5             # static demand x this must fit the rating

MOTORS = {"23 mm": 0.13, "40 mm": 0.42, "60 mm": 0.68}
RATIOS = (20, 26)


def motor_ceiling(holding_nm: float, ratio: int) -> float:
    """Output torque, capped by whichever of motor or gearbox gives out first."""
    return min(holding_nm * ratio * GEARBOX_EFF, GEARBOX_RATED)


def evaluate(reach: float, payload: float) -> dict:
    arm = default_arm(reach, payload)
    shoulder, cfg = arm.worst_case_torque(1)
    elbow, _ = arm.worst_case_torque(2)
    longest = max(l.length for l in arm.links)
    return {
        "reach": reach,
        "payload": payload,
        "shoulder": shoulder,
        "elbow": elbow,
        "arm_mass": arm.moving_mass,
        "longest_link": longest,
        "printable_flat": longest <= BED_USABLE,
        "printable_diag": longest <= BED_DIAGONAL,
        "deflection_mm": arm.tip_deflection(1) * 1000,
        "config": cfg,
    }


def main() -> None:
    print(f"print bed usable: {BED_USABLE*1000:.0f} mm flat, "
          f"{BED_DIAGONAL*1000:.0f} mm on the diagonal")
    print(f"gearbox ceiling:  {GEARBOX_RATED:.1f} N.m, design factor {DESIGN_FACTOR:.1f}x "
          f"-> {GEARBOX_RATED/DESIGN_FACTOR:.1f} N.m allowable\n")

    print("available output torque (min of motor and gearbox):")
    for name, nm in MOTORS.items():
        row = "  ".join(f"{r}:1 -> {motor_ceiling(nm, r):5.2f}" for r in RATIOS)
        print(f"  {name:6s} {nm:.2f} N.m   {row}")

    allow = GEARBOX_RATED / DESIGN_FACTOR
    print(f"\n{'reach':>6} {'payload':>8} {'shoulder':>9} {'req.box':>8} {'elbow':>7} "
          f"{'mass':>6} {'link':>6} {'defl':>7}  verdict")
    print("-" * 78)

    for reach in (0.350, 0.400, 0.450, 0.475, 0.550):
        for payload in (0.25, 0.50, 0.75):
            r = evaluate(reach, payload)
            required = r["shoulder"] * DESIGN_FACTOR
            if not r["printable_diag"]:
                verdict = "LINK TOO LONG"
            elif required <= GEARBOX_RATED:
                verdict = "OK on a 5 N.m box"
            elif required <= GEARBOX_HIGH:
                verdict = "needs the HT shoulder"
            else:
                verdict = f"no printed box ({required:.1f} N.m)"
            if r["printable_diag"] and not r["printable_flat"]:
                verdict += ", diagonal print"
            print(f"{r['reach']*1000:5.0f} {payload*1000:7.0f}g "
                  f"{r['shoulder']:8.2f} {required:7.1f} {r['elbow']:7.2f} "
                  f"{r['arm_mass']:5.2f} {r['longest_link']*1000:5.0f} "
                  f"{r['deflection_mm']:6.2f}  {verdict}")

    print("\nunits: N.m, kg, mm. 'req.box' is the shoulder gearbox rating needed at")
    print(f"a {DESIGN_FACTOR:.1f}x design factor. 'defl' is upper-arm tip deflection,")
    print("links only - joint compliance is NOT included and usually dominates.")
    print(f"For scale: one AS5600 count at {0.45*1000:.0f} mm reach is "
          f"{math.radians(360/4096)*0.45*1000:.2f} mm of tool error.")


if __name__ == "__main__":
    main()
