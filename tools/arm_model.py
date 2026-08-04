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

        `angles_deg[0]` is J1 (yaw) and is deliberately excluded from the
        pitch accumulation rather than merely assumed to be zero - it used to be
        summed in, which gave silently wrong torques the moment anyone swept the
        base. Every caller happened to pass 0.0, so nothing caught it.
        """
        torque = 0.0
        cum_angle = 0.0
        x = 0.0  # horizontal distance from the joint under consideration

        for i, link in enumerate(self.links):
            if i > 0 and i < len(angles_deg):
                cum_angle += math.radians(angles_deg[i])
            if i >= joint_index:
                horiz_com = x + link.com * math.cos(cum_angle)
                torque += link.mass * G * horiz_com
            x += link.length * math.cos(cum_angle) if i >= joint_index else 0.0

        tip = self.payload + self.tool_mass
        if tip:
            torque += tip * G * x
        return torque

    def potential_energy(self, angles_deg: list[float]) -> float:
        """Gravitational PE of the whole chain, J. Only used to check torques.

        Virtual work says tau_i = dU/dtheta_i, so differentiating this
        numerically is a derivation-independent test of gravity_torque().
        """
        u = 0.0
        cum_angle = 0.0
        h = 0.0
        for i, link in enumerate(self.links):
            if i > 0 and i < len(angles_deg):
                cum_angle += math.radians(angles_deg[i])
            u += link.mass * G * (h + link.com * math.sin(cum_angle))
            h += link.length * math.sin(cum_angle)
        tip = self.payload + self.tool_mass
        if tip:
            u += tip * G * h
        return u

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


@dataclass(frozen=True)
class DHParams:
    """Standard (distal) Denavit-Hartenberg table for a UR-style 6R arm.

    Convention: T_i = Rz(theta_i) . Tz(d_i) . Tx(a_i) . Rx(alpha_i)

    a2 and a3 are negative, matching the UR sign convention, so that positive
    joint angles fold the arm the way the physical machine does.

    d4 is the wrist offset. It is what makes this a NON-spherical wrist: the
    last three axes do not meet at a point. Closed-form IK survives anyway
    because J2, J3 and J4 are parallel - Pieper's parallel-axis branch. See D1.
    """

    d1: float = 0.110   # base plate to shoulder axis
    a2: float = -0.168  # upper arm, shoulder to elbow
    a3: float = -0.148  # forearm, elbow to wrist 1
    d4: float = 0.048   # wrist offset (the non-spherical term)
    d5: float = 0.048   # wrist 1 to wrist 2
    d6: float = 0.036   # wrist 3 to tool flange

    @property
    def horizontal_reach(self) -> float:
        return abs(self.a2) + abs(self.a3) + self.d5 + self.d6

    def rows(self) -> list[tuple[float, float, float]]:
        """(d, a, alpha) per joint, in order. theta is the variable."""
        return [
            (self.d1, 0.0, math.pi / 2),
            (0.0, self.a2, 0.0),
            (0.0, self.a3, 0.0),
            (self.d4, 0.0, math.pi / 2),
            (self.d5, 0.0, -math.pi / 2),
            (self.d6, 0.0, 0.0),
        ]


# Joint travel limits, degrees. J6 is continuous but clamped for cable routing.
JOINT_LIMITS_DEG = [
    (-180.0, 180.0),
    (-135.0, 135.0),
    (-150.0, 150.0),
    (-180.0, 180.0),
    (-120.0, 120.0),
    (-180.0, 180.0),
]

DH = DHParams()


def arm_from_links(upper: float, fore: float, wrist: float,
                   payload: float) -> ArmModel:
    """Build an arm from explicit link lengths, with mass following length.

    Use this rather than mutating an existing model's lengths: structural mass
    is derived from length, so changing one without the other silently yields a
    model that weighs the wrong amount.
    """
    # Sized for stiffness, then cut back until deflection approaches the
    # encoder resolution. Anything stiffer is mass the shoulder pays for.
    section = Section(width=0.038, height=0.045, wall=0.0022)
    struct = section.mass_per_length()

    return ArmModel(
        links=[
            # J1 base yaw carries no gravity torque; length is the shoulder offset
            Link("base", 0.0, 0.00, 0.65),
            Link("upper_arm", upper, struct * upper, 0.34, com_frac=0.45),
            Link("forearm", fore, struct * fore, 0.34, com_frac=0.45),
            Link("wrist", wrist, struct * wrist, 0.30, com_frac=0.5),
        ],
        payload=payload,
        section=section,
        tool_mass=0.12,  # printed gripper + 9 g servo
    )


def default_arm(reach: float, payload: float) -> ArmModel:
    """Build a plausible arm scaled to a target reach.

    Link proportions follow the UR pattern: upper arm slightly longer than the
    forearm, with a short wrist stack. Masses are estimates until parts exist.
    """
    upper = 0.42 * reach
    fore = 0.37 * reach
    return arm_from_links(upper, fore, reach - upper - fore, payload)


def _self_check() -> None:
    arm = default_arm(reach=0.400, payload=0.5)

    # Virtual work: tau_i = dU/dtheta_i. gravity_torque() is derived from
    # horizontal moment arms and potential_energy() from vertical heights, so
    # agreement between them is a real check and not a restatement.
    for joint in (1, 2, 3):
        for angles in ([0, 0, 0, 0], [0, -30, 45, 10], [0, 20, -60, 30],
                       [0, -90, 90, 0]):
            eps = 1e-6
            hi = list(angles)
            lo = list(angles)
            hi[joint] += math.degrees(eps)
            lo[joint] -= math.degrees(eps)
            numeric = (arm.potential_energy(hi) - arm.potential_energy(lo)) / (2 * eps)
            analytic = arm.gravity_torque(joint, angles)
            assert abs(numeric - analytic) < 1e-4, (
                f"joint {joint} at {angles}: {analytic:.6f} vs {numeric:.6f}")

    # Yaw must not change any pitch-joint torque. This is the bug that was
    # hiding behind every caller passing angles[0] = 0.
    for yaw in (0.0, 37.0, -90.0, 180.0):
        base = arm.gravity_torque(1, [0.0, -30.0, 45.0, 0.0])
        rotated = arm.gravity_torque(1, [yaw, -30.0, 45.0, 0.0])
        assert abs(base - rotated) < 1e-12, f"yaw {yaw} changed torque"

    # Hanging straight down is a zero-torque configuration; horizontal is peak.
    assert abs(arm.gravity_torque(1, [0, -90, 0, 0])) < 1e-9
    assert arm.gravity_torque(1, [0, 0, 0, 0]) > arm.gravity_torque(1, [0, -45, 0, 0])

    # Outboard joints can never carry more gravity load than inboard ones.
    angles = [0, -20, 40, 15]
    torques = [arm.gravity_torque(i, angles) for i in (1, 2, 3)]
    assert torques[0] >= torques[1] >= torques[2], torques

    print("arm_model self-checks passed")
    print(f"  worst-case shoulder torque : "
          f"{arm.worst_case_torque(1)[0]:.3f} N.m")
    print(f"  moving mass                : {arm.moving_mass:.3f} kg")


if __name__ == "__main__":
    _self_check()
