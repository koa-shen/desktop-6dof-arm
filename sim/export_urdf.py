"""Generate arm.urdf from tools/arm_model.py. Never hand-edit the output.

This is stage S0 of docs/simulation-plan.md and the concrete form of D14: the
model format is the interchange, so Drake, MuJoCo, PyBullet, RViz and MoveIt all
load the same arm and the engine choice stops being architectural.

THE CONVERSION, AND WHY IT IS NOT OBVIOUS
-----------------------------------------
Standard (distal) DH composes a joint as

    T_i = Rz(theta_i) . Tz(d_i) . Tx(a_i) . Rx(alpha_i)

with the variable rotation FIRST. URDF composes a joint as

    T_i = origin_i . Rz(q_i)

with the fixed part first. These are not the same order, so you cannot drop the
DH row into a URDF <origin> and expect the arm to be right. Doing exactly that
is the single most common way a generated URDF ends up subtly wrong - it will
look plausible in RViz and disagree with the real arm by centimetres.

The fix is to shift the fixed part by one joint. Give URDF joint i the fixed
part of DH row i-1:

    frame after URDF joint 1 = Rz(q1)
    frame after URDF joint 2 = Rz(q1) . [Tz(d1) Tx(a1) Rx(alpha1)] . Rz(q2)
                             = T_01 . Rz(q2)

which telescopes correctly for the whole chain. A final fixed joint carries the
last row's leftover so the flange frame matches DH frame 6 exactly.

The fixed part itself collapses pleasantly. Translations commute, so

    Tz(d) . Tx(a) . Rx(alpha)  ==  xyz = (a, 0, d),  rpy = (alpha, 0, 0)

which is why every <origin> below has zero pitch and yaw.

None of this is taken on faith: _validate() below compares the generated chain
against tools.kinematics.fk() over random configurations and fails loudly.

Run from this directory:  python export_urdf.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

from arm_model import DH, DHParams, JOINT_LIMITS_DEG, default_arm  # noqa: E402
from kinematics import dh_transform, fk  # noqa: E402

OUT = Path(__file__).resolve().parent / "arm.urdf"

# Motion limits at the output, from joint_config.h. Kept here rather than
# imported because the C++ header is the authority for firmware and this is the
# authority for simulation; the CI job asserts they agree.
VEL_LIMIT_DEG_S = 15.0
EFFORT_LIMIT_NM = 8.0


# ------------------------------------------------------------ conversion --

def urdf_joint_frames(dh: DHParams = DH):
    """(xyz, rpy) for each URDF joint origin, plus the final flange offset.

    Returns (origins, flange) where origins has one entry per revolute joint.
    """
    rows = dh.rows()  # (d, a, alpha) per joint
    origins = []
    for i in range(6):
        if i == 0:
            origins.append(((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
        else:
            d, a, alpha = rows[i - 1]
            origins.append(((a, 0.0, d), (alpha, 0.0, 0.0)))
    d, a, alpha = rows[5]
    flange = ((a, 0.0, d), (alpha, 0.0, 0.0))
    return origins, flange


def _homog(xyz, rpy) -> np.ndarray:
    """URDF origin -> 4x4. URDF rpy is FIXED-axis XYZ, i.e. R = Rz.Ry.Rx."""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = xyz
    return T


def _rotz(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return T


def urdf_fk(q, dh: DHParams = DH) -> np.ndarray:
    """FK by walking the generated URDF chain. Must equal kinematics.fk()."""
    origins, flange = urdf_joint_frames(dh)
    T = np.eye(4)
    for i in range(6):
        xyz, rpy = origins[i]
        T = T @ _homog(xyz, rpy) @ _rotz(q[i])
    xyz, rpy = flange
    return T @ _homog(xyz, rpy)


# ------------------------------------------------------------- inertials --

def box_inertia(mass: float, x: float, y: float, z: float) -> tuple:
    """Principal inertia of a solid box about its centre."""
    k = mass / 12.0
    return (k * (y * y + z * z), k * (x * x + z * z), k * (x * x + y * y))


def link_masses(arm) -> list[float]:
    """Mass for each of the six URDF links.

    The static model in arm_model has four lumped links; the URDF needs six.
    The wrist mass is split across the three wrist links because putting it all
    on one produces a wrong inertia tensor even though the total mass matches -
    and inertia is what a dynamic simulation actually integrates.
    """
    wrist = arm.links[3].mass
    return [
        arm.links[0].motor_mass,   # shoulder housing
        arm.links[1].mass,         # upper arm
        arm.links[2].mass,         # forearm
        wrist * 0.4,
        wrist * 0.4,
        wrist * 0.2,
    ]


# ------------------------------------------------------------------ xml --

def _add_inertial(link_el, mass: float, dims: tuple, com):
    inertial = ET.SubElement(link_el, "inertial")
    ET.SubElement(inertial, "origin", xyz=f"{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}",
                  rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{mass:.6f}")
    ixx, iyy, izz = box_inertia(mass, *dims)
    ET.SubElement(inertial, "inertia",
                  ixx=f"{ixx:.8f}", ixy="0", ixz="0",
                  iyy=f"{iyy:.8f}", iyz="0", izz=f"{izz:.8f}")


def _add_box_visual(link_el, dims: tuple, com, material: str):
    for tag in ("visual", "collision"):
        el = ET.SubElement(link_el, tag)
        ET.SubElement(el, "origin",
                      xyz=f"{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}", rpy="0 0 0")
        geom = ET.SubElement(el, "geometry")
        ET.SubElement(geom, "box",
                      size=f"{dims[0]:.6f} {dims[1]:.6f} {dims[2]:.6f}")
        if tag == "visual":
            mat = ET.SubElement(el, "material", name=material)
            ET.SubElement(mat, "color", rgba="0.25 0.55 0.85 1.0")


def build_urdf(arm, dh: DHParams = DH) -> ET.Element:
    origins, flange = urdf_joint_frames(dh)
    masses = link_masses(arm)
    sec = arm.section

    robot = ET.Element("robot", name="desktop_6dof_arm")
    ET.SubElement(robot, "material", name="arm_blue")

    base = ET.SubElement(robot, "link", name="base_link")
    _add_inertial(base, 0.8, (0.12, 0.12, 0.04), (0, 0, 0.02))
    _add_box_visual(base, (0.12, 0.12, 0.04), (0, 0, 0.02), "arm_blue")

    parent = "base_link"
    for i in range(6):
        name = f"link_{i + 1}"
        link = ET.SubElement(robot, "link", name=name)

        # The visual spans from this link's frame toward the next joint's
        # origin, which is where the structure physically is.
        if i + 1 < 6:
            nxyz, _ = origins[i + 1]
        else:
            nxyz, _ = flange
        span = np.array(nxyz, dtype=float)
        length = float(np.linalg.norm(span))
        com = (span / 2.0).tolist()

        if length < 1e-6:
            dims = (sec.width, sec.width, sec.width)
            com = [0.0, 0.0, 0.0]
        else:
            # Long axis along the dominant direction of the offset.
            axis = int(np.argmax(np.abs(span)))
            dims = [sec.width, sec.width, sec.width]
            dims[axis] = max(length, sec.width)
            dims = tuple(dims)

        _add_inertial(link, masses[i], dims, com)
        _add_box_visual(link, dims, com, "arm_blue")

        joint = ET.SubElement(robot, "joint", name=f"joint_{i + 1}",
                              type="revolute")
        xyz, rpy = origins[i]
        ET.SubElement(joint, "origin",
                      xyz=f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}",
                      rpy=f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=name)
        ET.SubElement(joint, "axis", xyz="0 0 1")
        lo, hi = JOINT_LIMITS_DEG[i]
        ET.SubElement(joint, "limit",
                      lower=f"{math.radians(lo):.6f}",
                      upper=f"{math.radians(hi):.6f}",
                      effort=f"{EFFORT_LIMIT_NM:.2f}",
                      velocity=f"{math.radians(VEL_LIMIT_DEG_S):.6f}")
        # Printed cycloidal reducers are not frictionless and the sim should
        # not pretend otherwise. Replace with measured values after Phase 1B -
        # these are placeholders and D15 says so.
        ET.SubElement(joint, "dynamics", damping="0.05", friction="0.02")
        parent = name

    tool = ET.SubElement(robot, "link", name="tool0")
    _add_inertial(tool, arm.tool_mass, (0.04, 0.04, 0.06), (0, 0, 0.03))
    _add_box_visual(tool, (0.04, 0.04, 0.06), (0, 0, 0.03), "arm_blue")

    fj = ET.SubElement(robot, "joint", name="flange", type="fixed")
    xyz, rpy = flange
    ET.SubElement(fj, "origin",
                  xyz=f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}",
                  rpy=f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}")
    ET.SubElement(fj, "parent", link=parent)
    ET.SubElement(fj, "child", link="tool0")

    return robot


# ------------------------------------------------------------ validation --

def _validate(n: int = 500, seed: int = 0) -> float:
    """Generated chain vs the authoritative FK. Returns worst position error."""
    rng = np.random.default_rng(seed)
    lo = np.radians([a for a, _ in JOINT_LIMITS_DEG])
    hi = np.radians([b for _, b in JOINT_LIMITS_DEG])
    worst = 0.0
    for _ in range(n):
        q = rng.uniform(lo, hi)
        a = urdf_fk(q)
        b = fk(q)
        worst = max(worst, float(np.max(np.abs(a - b))))
    return worst


def _self_check() -> None:
    # The fixed-part collapse: Tz(d).Tx(a).Rx(alpha) == xyz(a,0,d) rpy(alpha,0,0)
    for d, a, alpha in ((0.11, 0.0, math.pi / 2), (0.0, -0.168, 0.0),
                        (0.048, 0.03, -math.pi / 2)):
        lhs = dh_transform(0.0, d, a, alpha)
        rhs = _homog((a, 0.0, d), (alpha, 0.0, 0.0))
        assert np.allclose(lhs, rhs, atol=1e-12), (d, a, alpha)

    worst = _validate()
    assert worst < 1e-9, f"URDF chain disagrees with fk() by {worst:.2e}"
    print(f"[1] fixed-part collapse identity              : ok")
    print(f"[2] URDF chain vs kinematics.fk(), 500 poses  : {worst:.2e} m")


def main() -> None:
    _self_check()
    arm = default_arm(reach=0.400, payload=0.5)
    robot = build_urdf(arm)
    xml = minidom.parseString(ET.tostring(robot)).toprettyxml(indent="  ")
    header = ("<!-- GENERATED by sim/export_urdf.py - DO NOT EDIT.\n"
              "     Source of truth is tools/arm_model.py (D14).\n"
              "     Regenerate after any geometry or mass change. -->\n")
    lines = xml.split("\n")
    OUT.write_text(lines[0] + "\n" + header + "\n".join(lines[1:]),
                   encoding="utf-8")
    total = sum(link_masses(arm)) + arm.tool_mass
    print(f"[3] wrote {OUT.name}: 6 revolute + 1 fixed, {total:.3f} kg moving")


if __name__ == "__main__":
    main()
