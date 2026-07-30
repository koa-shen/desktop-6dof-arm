"""Single source of truth for the arm's geometry and mass.

Every calculation in tools/ reads from here. When a link mass is measured on a
scale, change it in this file only - the torque budget, the deflection check,
and (later) the URDF export all follow.

Units are SI throughout: metres, kilograms, newton-metres, radians.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

G = 9.81

# PETG on an FDM printer. The nominal datasheet modulus is ~2.0 GPa for molded
# bar; printed parts come in lower and anisotropic. This is an in-plane value
# with a knockdown already applied. Measure a coupon and replace it.
PETG_E = 1.5e9
PETG_DENSITY = 1270.0  # kg/m^3 solid


@dataclass
class Link:
    """One rigid segment plus the motor and gearbox mounted at its inboard end.

    The motor sits at the *proximal* joint, so its mass acts at r=0 for that
    joint and at the full link offset for every joint inboard of it.
    """

    name: str
    length: float          # joint axis to next joint axis
    struct_mass: float     # printed structure only
    motor_mass: float      # motor + gearbox at the proximal end
    com_frac: float = 0.5  # structural CoM as a fraction of length

    @property
    def mass(self) -> float:
        return self.struct_mass + self.motor_mass

    @property
    def com(self) -> float:
        """Combined CoM offset from the proximal joint axis."""
        if self.mass == 0:
            return 0.0
        return self.struct_mass * self.com_frac * self.length / self.mass


@dataclass
class Section:
    """Hollow rectangular cross-section for a printed link."""

    width: float
    height: float
    wall: float

    @property
    def second_moment(self) -> float:
        """I about the bending (horizontal) axis."""
        bi = self.width - 2 * self.wall
        hi = self.height - 2 * self.wall
        if bi <= 0 or hi <= 0:
            return self.width * self.height ** 3 / 12
        return (self.width * self.height ** 3 - bi * hi ** 3) / 12

    def mass_per_length(self, infill_density: float = PETG_DENSITY) -> float:
        bi = max(self.width - 2 * self.wall, 0.0)
        hi = max(self.height - 2 * self.wall, 0.0)
        area = self.width * self.height - bi * hi
        return area * infill_density


@dataclass
class ArmModel:
    """A UR-style 6R arm, described only as far as statics needs."""

    links: list[Link]
    payload: float
    section: Section
    name: str = "desktop-6dof"
    tool_mass: float = 0.0  # gripper, at the flange

    @property
    def reach(self) -> float:
        """Base axis to flange, arm horizontal and fully extended."""
        return sum(l.length for l in self.links)

    @property
    def moving_mass(self) -> float:
        return sum(l.mass for l in self.links)

    def gravity_torque(self, joint_index: int, angles_deg: list[float]) -> float:
        """Static gravity torque at one joint, arm in the given configuration.

        Only pitch joints are considered. For a UR-style arm, J1 rotates about a
        vertical axis and J4/J6 are roll axes along the arm, so gravity produces
        no moment about them in the nominal upright mounting.
        """
        torque = 0.0
        cum_angle = 0.0
        x = 0.0  # horizontal distance from the joint under consideration

        for i, link in enumerate(self.links):
            cum_angle += math.radians(angles_deg[i]) if i < len(angles_deg) else 0.0
            if i >= joint_index:
                horiz_com = x + link.com * math.cos(cum_angle)
                torque += link.mass * G * horiz_com
            x += link.length * math.cos(cum_angle) if i >= joint_index else 0.0

        tip = self.payload + self.tool_mass
        if tip:
            torque += tip * G * x
        return torque

    def worst_case_torque(self, joint_index: int, step_deg: int = 15) -> tuple:
        """Sweep reachable configurations and return (max torque, angles)."""
        worst = (0.0, [])
        rng = range(-90, 91, step_deg)
        for a2 in rng:
            for a3 in rng:
                angles = [0.0] * len(self.links)
                if len(angles) > 1:
                    angles[1] = a2
                if len(angles) > 2:
                    angles[2] = a3
                t = self.gravity_torque(joint_index, angles)
                if t > worst[0]:
                    worst = (t, list(angles))
        return worst

    def tip_deflection(self, link_index: int) -> float:
        """Cantilever tip deflection of one link under the outboard load.

        Treats the link as a fixed-free beam with the entire outboard mass and
        payload as a point load at its tip. Pessimistic on distribution,
        optimistic in that it ignores joint compliance - see the note in D8.
        """
        link = self.links[link_index]
        outboard = sum(l.mass for l in self.links[link_index + 1:])
        load = (outboard + self.payload + self.tool_mass) * G
        ei = PETG_E * self.section.second_moment
        return load * link.length ** 3 / (3 * ei)


def default_arm(reach: float, payload: float) -> ArmModel:
    """Build a plausible arm scaled to a target reach.

    Link proportions follow the UR pattern: upper arm slightly longer than the
    forearm, with a short wrist stack. Masses are estimates until parts exist.
    """
    upper = 0.42 * reach
    fore = 0.37 * reach
    wrist = reach - upper - fore

    # Sized for stiffness, then cut back until deflection approaches the
    # encoder resolution. Anything stiffer is mass the shoulder pays for.
    section = Section(width=0.038, height=0.045, wall=0.0022)
    struct = section.mass_per_length()

    return ArmModel(
        links=[
            # J1 base yaw: contributes no gravity torque, length is the shoulder offset
            Link("base", 0.0, 0.00, 0.65),
            Link("upper_arm", upper, struct * upper, 0.34, com_frac=0.45),
            Link("forearm", fore, struct * fore, 0.34, com_frac=0.45),
            Link("wrist", wrist, struct * wrist, 0.30, com_frac=0.5),
        ],
        payload=payload,
        section=section,
        tool_mass=0.12,  # printed gripper + 9 g servo
    )
