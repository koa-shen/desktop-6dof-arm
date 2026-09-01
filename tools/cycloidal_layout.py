"""Cycloidal reducer layout: the P1-a footprint feasibility gate, in numbers.

Run:  python tools/cycloidal_layout.py

WHY THIS EXISTS
---------------
D19 says the ratio is an OUTPUT of the feasible layout, not an independent
target: everything has to fit inside the 42 x 42 mm NEMA 17 face. That makes
the first design question arithmetic, not CAD - and arithmetic that is cheaper
to get wrong here than after a print. This file closes the loop between the
layout, the torque budget, and the D17 error budget:

  ratio          <- lobe count, which is set by the pin circle that fits
  contact stress <- output torque / (pin circle radius x engaged pins)
  backlash       <- printed clearances, which is R-4 and the largest term in
                    the D17 accuracy budget after windup

The profile is a curtate hypocycloid offset by the ring-pin radius:

    psi(t) = atan2( sin((1-N)t), R/(e N) - cos((1-N)t) )
    x(t)   =  R cos t - Rr cos(t + psi) - e cos(N t)
    y(t)   = -R sin t + Rr sin(t + psi) + e sin(N t)

with N ring pins, N-1 disc lobes, pin circle radius R, pin radius Rr,
eccentricity e. Reduction ratio for a fixed ring with N-1 lobes is N-1:1.

THE ONE PARAMETER THAT DECIDES VALIDITY
---------------------------------------
K = e N / R, the eccentricity ratio. Below ~0.5 the lobes are shallow and the
drive is compliant; at K >= 1 the parametric curve loops back on itself and
the "profile" is not a curve a printer can make. `check()` catches that
numerically (the polar angle stops advancing) rather than trusting the
inequality, because the practical limit is lower than 1 and depends on Rr.

Nothing here is a substitute for CAD. It is the filter that stops you opening
CAD on a layout that cannot fit, and the place the resulting numbers get
written down.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# mm, N, MPa throughout.
NEMA17_FACE = 42.0
STEEL_E, STEEL_NU = 200_000.0, 0.30
PETG_E, PETG_NU = 2_000.0, 0.40
PETG_YIELD = 50.0                 # tensile; contact allowable is softer than this
PETG_LAYER_FACTOR = 0.55          # strength across layer lines, mid of the 40-70 % band
ENGAGED_FRACTION = 0.35           # printed parts share load worse than steel
STEEL_DENSITY = 7.85e-3           # g/mm^3

MATERIALS = {"steel": (STEEL_E, STEEL_NU), "petg": (PETG_E, PETG_NU)}

# Friction coefficients, sliding. The spread between dry and greased plastic is
# the single largest lever on efficiency in this design.
MU_DRY_PLASTIC = 0.35             # PETG on PETG, dry - also stick-slips
MU_DRY_STEEL_PLASTIC = 0.25
MU_GREASED = 0.08
MU_NEEDLE_ROLLER = 0.005          # rolling element, not sliding
# Rolling resistance from viscoelastic hysteresis. Steel is ~0.001; a polymer
# raceway is an order of magnitude worse, and it does not go away with grease.
MU_ROLLING_PETG = 0.015


@dataclass(frozen=True)
class CycloidalLayout:
    pins: int = 16                # N ring pins -> N-1 lobes -> N-1 : 1
    pin_circle_r: float = 17.3    # R
    pin_r: float = 1.5            # Rr, half a 3 mm dowel
    eccentricity: float = 0.65    # e
    disc_thickness: float = 5.0
    disc_count: int = 2
    housing_wall: float = 2.0
    bearing_od: float = 16.0      # eccentric bearing, e.g. 688
    output_pins: int = 6
    output_pin_r: float = 2.0     # M3 shoulder bolt, 4 mm shoulder
    output_circle_r: float = 11.3
    web: float = 1.5              # minimum printed material between features
    ring_pin_material: str = "steel"
    ring_pin_fixed: bool = False  # True = printed into the housing, cannot rotate
    ring_pin_supported_both_ends: bool = True
    ring_pin_sleeve: bool = False  # rotating sleeve over a printed post
    sleeve_wall: float = 0.5

    # ------------------------------------------------------------ geometry --
    @property
    def lobes(self) -> int:
        return self.pins - 1

    @property
    def ratio(self) -> int:
        return self.lobes

    @property
    def k(self) -> float:
        """Eccentricity ratio e*N/R. The profile's single validity parameter."""
        return self.eccentricity * self.pins / self.pin_circle_r

    @property
    def tip_r(self) -> float:
        return self.pin_circle_r + self.eccentricity - self.pin_r

    @property
    def root_r(self) -> float:
        return self.pin_circle_r - self.eccentricity - self.pin_r

    @property
    def housing_r(self) -> float:
        return self.pin_circle_r + self.pin_r + self.housing_wall

    @property
    def bore_r(self) -> float:
        """Eccentric bearing bore in the disc, swept by the eccentricity."""
        return self.bearing_od / 2.0 + self.eccentricity

    @property
    def output_hole_r(self) -> float:
        return self.output_pin_r + self.eccentricity

    def profile(self, samples: int = 2000) -> tuple[np.ndarray, np.ndarray]:
        t = np.linspace(0.0, 2.0 * math.pi, samples, endpoint=False)
        n, r, rr, e = self.pins, self.pin_circle_r, self.pin_r, self.eccentricity
        psi = np.arctan2(np.sin((1 - n) * t),
                         r / (e * n) - np.cos((1 - n) * t))
        x = r * np.cos(t) - rr * np.cos(t + psi) - e * np.cos(n * t)
        y = -r * np.sin(t) + rr * np.sin(t + psi) + e * np.sin(n * t)
        return x, y

    # -------------------------------------------------------------- loading --
    def ring_pin_force(self, output_nm: float) -> float:
        """Tangential force on one engaged ring pin, N."""
        engaged = max(1.0, self.pins * ENGAGED_FRACTION)
        return output_nm * 1000.0 / (self.pin_circle_r * engaged)

    def output_pin_force(self, output_nm: float) -> float:
        engaged = max(1.0, self.output_pins * 0.5)
        return output_nm * 1000.0 / (self.output_circle_r * engaged)

    def hertz_pressure(self, force_n: float, pin_r: float,
                       pin_material: str = "steel") -> float:
        """Peak line-contact pressure on a PETG flank, MPa.

        Conservative: the flank is treated as flat, so the relative radius is
        the pin's. A real cycloidal flank is concave and slightly kinder.
        A printed pin gives a LOWER peak pressure than a steel one - the softer
        pair spreads the contact - which is the strongest argument for printing.
        """
        e_pin, nu_pin = MATERIALS[pin_material]
        e_star = 1.0 / ((1 - nu_pin ** 2) / e_pin + (1 - PETG_NU ** 2) / PETG_E)
        length = self.disc_thickness * self.disc_count
        return math.sqrt((force_n / length) * e_star / (math.pi * pin_r))

    @property
    def post_r(self) -> float:
        """Load-bearing radius of the printed post. The sleeve wall eats into it."""
        return self.pin_r - self.sleeve_wall if self.ring_pin_sleeve else self.pin_r

    def pin_bending_stress(self, output_nm: float) -> float:
        """Peak bending stress at the ring-pin root, MPa.

        Irrelevant for a steel dowel and decisive for a printed one. A pin
        printed standing on the housing floor is loaded transverse to its layer
        lines, so compare against PETG_YIELD * PETG_LAYER_FACTOR, not the
        tensile figure. With a sleeve it is the POST that bends, and bending
        goes as 1/d^3, so the wall thickness is expensive.
        """
        f = self.ring_pin_force(output_nm)
        span = self.disc_thickness * self.disc_count
        moment = f * span / 4.0 if self.ring_pin_supported_both_ends else f * span
        d = 2.0 * self.post_r
        return 32.0 * moment / (math.pi * d ** 3)

    def min_printed_pin_diameter(self, output_nm: float) -> float:
        """Post diameter that brings bending stress to the layer-line allowable."""
        allow = PETG_YIELD * PETG_LAYER_FACTOR
        return 2.0 * self.post_r * (self.pin_bending_stress(output_nm) / allow) ** (1 / 3)

    @property
    def efficiency(self) -> float:
        return EFFICIENCY_SLIDING if self.ring_pin_fixed else EFFICIENCY_ROLLING

    # ---------------------------------------------------------- efficiency --
    #
    # The common normal at a meshing contact passes through the pitch point
    # (the fundamental law of gearing), so the ENTIRE relative velocity at a
    # ring-pin contact is tangential - it is all sliding - and its magnitude is
    #
    #     v_slide = omega_disc * |P - IC|
    #
    # i.e. proportional to the contact's distance from the instantaneous
    # centre. That is the key number: sliding is ZERO at the pitch point and
    # grows with distance from it, exactly like an involute gear tooth. The
    # heavily loaded pins sit near the pitch point, so the load-weighted
    # sliding velocity is far lower than "every pin slides at surface speed".
    #
    # Note K = e*N/R = R_pitch / R_pin. A low-K (low eccentricity) drive puts
    # the pins far OUTSIDE the pitch circle, which is what creates the sliding.

    @property
    def pitch_radius(self) -> float:
        """Rolling-circle radius of the ring, e*N. K is exactly R_pitch/R_pin."""
        return self.eccentricity * self.pins

    def ring_sliding_speed(self, input_rpm: float, samples: int = 720) -> float:
        """Load-weighted mean sliding speed at the ring-pin contacts, m/s."""
        w_in = 2.0 * math.pi * input_rpm / 60.0
        w_disc = w_in / self.lobes
        theta = np.arange(self.pins) * 2.0 * math.pi / self.pins
        total_fv, total_f = 0.0, 0.0
        for phi in np.linspace(0.0, 2.0 * math.pi, samples, endpoint=False):
            d = np.hypot(self.pin_circle_r * np.cos(theta) - self.pitch_radius * math.cos(phi),
                         self.pin_circle_r * np.sin(theta) - self.pitch_radius * math.sin(phi))
            psi = np.mod(theta - phi, 2.0 * math.pi)
            load = np.where(psi < math.pi, np.sin(psi), 0.0)   # loaded half only
            total_fv += float(np.sum(load * w_disc * d / 1000.0))
            total_f += float(np.sum(load))
        return total_fv / total_f

    def output_sliding_speed(self, input_rpm: float) -> float:
        """Sliding speed at the output-pin bores, m/s.

        Different in kind from the ring contacts: the bore ORBITS the pin at
        radius e once per INPUT revolution, so the contact circulates rather
        than oscillating, and every engaged pin slides at the same speed. This
        interface sees the input speed, not the output speed.
        """
        w_in = 2.0 * math.pi * input_rpm / 60.0
        return self.eccentricity * w_in / 1000.0

    def ring_loss_factor(self, mu_contact: float, mu_bore: float) -> float:
        """Ring-side friction loss as a multiple of plain sliding at mu_contact.

        A rotating SLEEVE is the one case where a free roller genuinely helps,
        and it is worth being precise about why. For a loose dowel in a pocket
        the sliding radius equals the contact radius, so the radii cancel and
        the loss is unchanged (see pocket_loss_equivalent). A sleeve breaks
        that: the contact happens at the sleeve OD and the sliding happens at
        the post OD, so the loss picks up a geometric lever

            factor = mu_bore/mu_contact * post_r/pin_r  +  mu_rolling/mu_contact

        The lever is a ratio of radii, so it buys tens of percent - not the
        order of magnitude that an actual rolling element buys by changing mu.
        """
        if not self.ring_pin_sleeve:
            return 1.0
        return (mu_bore / mu_contact) * (self.post_r / self.pin_r) \
            + MU_ROLLING_PETG / mu_contact

    def sleeve_rotates(self, mu_contact: float, mu_bore: float) -> bool:
        """A sleeve that cannot overcome its own bore friction just slides."""
        if not self.ring_pin_sleeve:
            return False
        return mu_contact * self.pin_r > mu_bore * self.post_r

    def friction_loss_w(self, output_nm: float, input_rpm: float,
                        mu_ring: float, mu_output: float,
                        mu_bore: float | None = None) -> tuple[float, float]:
        """(ring loss, output-pin loss) in watts."""
        f_ring = output_nm * 1000.0 / self.pin_circle_r      # total tangential
        f_out = output_nm * 1000.0 / self.output_circle_r
        factor = self.ring_loss_factor(mu_ring, mu_bore if mu_bore is not None
                                       else mu_ring)
        return (factor * mu_ring * f_ring * self.ring_sliding_speed(input_rpm),
                mu_output * f_out * self.output_sliding_speed(input_rpm))

    def efficiency(self, output_nm: float = 4.26, input_rpm: float = 300.0,
                   mu_ring: float = MU_GREASED,
                   mu_output: float = MU_GREASED,
                   mu_bore: float | None = None) -> float:
        """Computed, not assumed. Excludes bearing drag and face rub."""
        w_out = 2.0 * math.pi * input_rpm / 60.0 / self.lobes
        p_out = output_nm * w_out
        ring, out = self.friction_loss_w(output_nm, input_rpm, mu_ring,
                                         mu_output, mu_bore)
        return p_out / (p_out + ring + out)

    def pocket_loss_equivalent(self, output_nm: float, input_rpm: float,
                               mu: float) -> tuple[float, float]:
        """Loss with a pin FIXED vs the same pin FREE in a plain pocket, watts.

        The result people find surprising: they are equal. A free pin rolls at
        the contact, but then it must rotate inside its pocket, and the pocket
        radius IS the pin radius - so the sliding is not eliminated, only moved.
        omega_pin = v_slide / Rr and the pocket friction torque is mu*F*Rr, so
        the product is mu*F*v_slide either way. Only a ROLLING element in the
        pocket (needle rollers) actually removes the loss.
        """
        f = output_nm * 1000.0 / self.pin_circle_r
        v = self.ring_sliding_speed(input_rpm)
        fixed = mu * f * v
        omega_pin = v / (self.pin_r / 1000.0)
        free = mu * f * (self.pin_r / 1000.0) * omega_pin
        return fixed, free

    def radial_overrun(self) -> float:
        """How far the disc's inner features overrun the root circle, mm.

        Positive means the bearing sweep, output holes and their webs do not
        fit between the bore and the root. This is what fights the contact
        stress fix: bigger output pins lower the stress and raise this number.
        """
        need = self.bore_r + self.web + 2 * self.output_hole_r + self.web
        return need - self.root_r

    def max_output_torque(self) -> float:
        """Torque ceiling set by output-pin contact stress, N.m.

        Peak pressure goes as sqrt(T), so this inverts the Hertz check. It is
        the binding constraint on the whole layout - not the ratio, not the
        motor, not the ring pins.
        """
        p = self.hertz_pressure(self.output_pin_force(1.0), self.output_pin_r)
        return (PETG_YIELD / p) ** 2

    @property
    def steel_pin_mass_g(self) -> float:
        vol = math.pi * self.pin_r ** 2 * self.disc_thickness * self.disc_count
        return vol * self.pins * STEEL_DENSITY

    # ------------------------------------------------------------ backlash --
    def backlash_deg(self, ring_clearance: float, output_clearance: float) -> float:
        """First-order output lost motion from printed clearances, degrees.

        Ring-side clearance rotates the disc by c/R; the disc drives the output
        1:1 through the roller followers, so that lands directly on the output
        and does not divide by the ratio. This is why a printed cycloidal is a
        backlash problem and not a resolution problem (D17).
        """
        rad = 2.0 * (ring_clearance / self.pin_circle_r
                     + output_clearance / self.output_circle_r)
        return math.degrees(rad)

    # --------------------------------------------------------------- checks --
    def check(self, output_nm: float = 5.0) -> list[tuple[str, bool, str]]:
        out: list[tuple[str, bool, str]] = []

        def add(name, ok, msg):
            out.append((name, bool(ok), msg))

        add("profile is a valid curve", self.profile_is_simple(),
            f"K = {self.k:.2f} (undercuts near K ~ 0.95 at this pin radius)")
        add("K in the practical band", 0.4 <= self.k <= 0.8,
            f"K = {self.k:.2f}; below 0.4 is compliant, above 0.8 undercuts")
        add("housing inside the NEMA 17 face", self.housing_r <= NEMA17_FACE / 2,
            f"housing radius {self.housing_r:.2f} mm vs {NEMA17_FACE/2:.1f} mm")
        add("output holes clear the eccentric bearing",
            self.output_circle_r - self.output_hole_r >= self.bore_r + self.web,
            f"hole inner edge {self.output_circle_r - self.output_hole_r:.2f} mm "
            f"vs bearing sweep {self.bore_r:.2f} + web {self.web:.1f}")
        add("output holes clear the root circle",
            self.output_circle_r + self.output_hole_r <= self.root_r - self.web,
            f"hole outer edge {self.output_circle_r + self.output_hole_r:.2f} mm "
            f"vs root {self.root_r:.2f} - web {self.web:.1f}")
        pitch = 2 * math.pi * self.output_circle_r / self.output_pins
        add("web between output holes", pitch - 2 * self.output_hole_r >= self.web,
            f"{pitch - 2*self.output_hole_r:.2f} mm between holes")
        ring_pitch = 2 * math.pi * self.pin_circle_r / self.pins
        add("web between ring pins", ring_pitch - 2 * self.pin_r >= self.web,
            f"{ring_pitch - 2*self.pin_r:.2f} mm between pins")
        p_ring = self.hertz_pressure(self.ring_pin_force(output_nm), self.pin_r,
                                     self.ring_pin_material)
        add(f"ring pin contact stress at {output_nm:.1f} N.m", p_ring <= PETG_YIELD,
            f"{p_ring:.0f} MPa peak vs PETG ~{PETG_YIELD:.0f} MPa "
            f"({self.ring_pin_force(output_nm):.0f} N/pin, {self.ring_pin_material})")
        if self.ring_pin_material == "petg":
            allow = PETG_YIELD * PETG_LAYER_FACTOR
            sigma = self.pin_bending_stress(output_nm)
            support = "both ends" if self.ring_pin_supported_both_ends else "cantilever"
            add("printed ring pin bending", sigma <= allow,
                f"{sigma:.0f} MPa vs {allow:.0f} MPa across layers ({support}); "
                f"needs d >= {self.min_printed_pin_diameter(output_nm):.2f} mm")
        p_out = self.hertz_pressure(self.output_pin_force(output_nm),
                                    self.output_pin_r)
        add("output pin contact stress", p_out <= PETG_YIELD,
            f"{p_out:.0f} MPa peak ({self.output_pin_force(output_nm):.0f} N/pin)")
        return out

    def profile_is_simple(self, samples: int = 4000) -> bool:
        """True if the profile never doubles back - the numerical form of K < 1."""
        x, y = self.profile(samples)
        d = np.diff(np.unwrap(np.arctan2(y, x)))
        return bool(np.all(d > 0)) or bool(np.all(d < 0))

    def lobe_count(self, samples: int = 8000) -> int:
        x, y = self.profile(samples)
        r = np.hypot(x, y)
        prev, nxt = np.roll(r, 1), np.roll(r, -1)
        return int(np.sum((r > prev) & (r >= nxt)))

    def report(self, output_nm: float = 5.0) -> str:
        lines = [
            f"{self.ratio}:1, {self.pins} ring pins / {self.lobes} lobes, "
            f"R = {self.pin_circle_r:.2f} mm, e = {self.eccentricity:.2f} mm, "
            f"K = {self.k:.2f}",
            f"disc  tip {self.tip_r:.2f} / root {self.root_r:.2f} mm, "
            f"{self.disc_count} x {self.disc_thickness:.1f} mm",
            f"housing radius {self.housing_r:.2f} mm "
            f"(NEMA 17 face allows {NEMA17_FACE/2:.1f})",
            "",
        ]
        for name, ok, msg in self.check(output_nm):
            lines.append(f"  [{'PASS' if ok else 'FAIL'}] {name:38s} {msg}")
        bl = self.backlash_deg(0.10, 0.10)
        lines += [
            "",
            f"ring pins: {self.ring_pin_material}, "
            f"{'fixed' if self.ring_pin_fixed else 'free'}, "
            f"pitch radius {self.pitch_radius:.2f} mm of a {self.pin_circle_r:.2f} mm "
            f"pin circle",
            f"computed efficiency at 300 rpm in: "
            f"{100*self.efficiency(output_nm, 300.0, MU_DRY_PLASTIC, MU_DRY_PLASTIC):.0f} % dry, "
            f"{100*self.efficiency(output_nm, 300.0):.0f} % greased "
            f"(excludes bearing drag and face rub)",
            f"predicted backlash at 0.10 mm printed clearance: {bl:.2f} deg "
            f"(R-4 allows 1.0 deg)",
            f"clearance budget to stay under 1.0 deg: "
            f"{0.10 / bl:.2f} mm equivalent per interface",
            f"OUTPUT-PIN TORQUE CEILING: {self.max_output_torque():.2f} N.m "
            f"(radial overrun {self.radial_overrun():+.2f} mm)",
        ]
        return "\n".join(lines)

    def export_profile_csv(self, path: str | Path, samples: int = 2000) -> Path:
        x, y = self.profile(samples)
        p = Path(path)
        p.write_text("# x_mm,y_mm - cycloidal disc profile, import as a spline\n"
                     + "\n".join(f"{a:.6f},{b:.6f}" for a, b in zip(x, y)) + "\n")
        return p


# ------------------------------------------------------------- self-check ---

def _self_check() -> None:
    ref = CycloidalLayout()

    print("[1] profile has exactly N-1 lobes, which is the reduction ratio")
    assert ref.lobe_count() == ref.lobes, (ref.lobe_count(), ref.lobes)
    assert ref.ratio == 15
    print(f"    counted {ref.lobe_count()} lobes for {ref.pins} pins -> "
          f"{ref.ratio}:1")

    print("[2] profile is closed and stays between root and tip radii")
    x, y = ref.profile(4000)
    r = np.hypot(x, y)
    assert abs(r.max() - ref.tip_r) < 0.05, (r.max(), ref.tip_r)
    assert abs(r.min() - ref.root_r) < 0.05, (r.min(), ref.root_r)
    assert math.hypot(x[0] - x[-1], y[0] - y[-1]) < 2 * math.pi * ref.tip_r / 1000
    print(f"    radius {r.min():.3f}-{r.max():.3f} mm vs predicted "
          f"{ref.root_r:.3f}-{ref.tip_r:.3f}")

    print("[3] the validity limit on K is found numerically, and it is below 1")
    assert ref.profile_is_simple()

    def k_limit(pin_r: float) -> float:
        lo, hi = 0.1, 3.0
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if CycloidalLayout(eccentricity=mid, pin_r=pin_r).profile_is_simple():
                lo = mid
            else:
                hi = mid
        return CycloidalLayout(eccentricity=lo, pin_r=pin_r).k

    limits = {rr: k_limit(rr) for rr in (1.0, 1.5, 2.0, 2.5)}
    assert all(v < 1.0 for v in limits.values())
    assert limits[1.0] > limits[1.5] > limits[2.0] > limits[2.5]
    assert not CycloidalLayout(eccentricity=1.3).profile_is_simple()
    print("    K limit vs pin radius: "
          + ", ".join(f"Rr={rr} -> {v:.3f}" for rr, v in limits.items())
          + "\n    fatter pins undercut sooner; the textbook K < 1 is optimistic")

    print("[4] backlash is NOT divided by the ratio")
    bl = ref.backlash_deg(0.10, 0.10)
    naive = math.degrees(2 * 0.10 / ref.pin_circle_r) / ref.ratio
    assert bl > 20 * naive
    assert 0.2 < bl < 3.0, bl
    print(f"    0.10 mm clearance -> {bl:.2f} deg out "
          f"(dividing by the ratio would claim {naive:.3f} deg)")

    print("[5] contact stress scales as sqrt(torque) and flags the PETG limit")
    p1 = ref.hertz_pressure(ref.ring_pin_force(1.0), ref.pin_r)
    p4 = ref.hertz_pressure(ref.ring_pin_force(4.0), ref.pin_r)
    assert abs(p4 / p1 - 2.0) < 1e-9
    print(f"    1 N.m -> {p1:.0f} MPa, 4 N.m -> {p4:.0f} MPa, "
          f"PETG ~{PETG_YIELD:.0f} MPa")

    print("[6] printed pins LOWER contact stress but move the failure to bending")
    steel = ref.hertz_pressure(ref.ring_pin_force(4.26), ref.pin_r, "steel")
    petg = ref.hertz_pressure(ref.ring_pin_force(4.26), ref.pin_r, "petg")
    assert petg < steel, "softer pair must spread the contact"
    printed = CycloidalLayout(ring_pin_material="petg", ring_pin_fixed=True)
    sigma = printed.pin_bending_stress(4.26)
    allow = PETG_YIELD * PETG_LAYER_FACTOR
    assert sigma > allow, "3 mm printed pins should fail bending at the shoulder"
    cant = CycloidalLayout(ring_pin_material="petg", ring_pin_fixed=True,
                           ring_pin_supported_both_ends=False)
    assert abs(cant.pin_bending_stress(4.26) / sigma - 4.0) < 1e-9
    print(f"    contact {steel:.0f} -> {petg:.0f} MPa (-{100*(1-petg/steel):.0f} %), "
          f"but bending {sigma:.0f} MPa vs {allow:.0f} allowable")
    print(f"    cantilevered pins are 4x worse; supporting both ends is mandatory")

    print("[7] the bending fix is a diameter, and it is a small one")
    need_d = printed.min_printed_pin_diameter(4.26)
    fixed = CycloidalLayout(ring_pin_material="petg", ring_pin_fixed=True,
                            pin_r=need_d / 2)
    assert abs(fixed.pin_bending_stress(4.26) - allow) < 1e-6
    pitch = 2 * math.pi * fixed.pin_circle_r / fixed.pins
    assert pitch - need_d >= fixed.web, "pins must still fit side by side"
    print(f"    d = {need_d:.2f} mm passes bending, leaves "
          f"{pitch - need_d:.2f} mm web between pins")

    print("[8] a FREE pin in a plain pocket saves nothing - the loss just moves")
    ref_rpm, ref_t = 300.0, 4.26
    fixed, free = ref.pocket_loss_equivalent(ref_t, ref_rpm, MU_DRY_PLASTIC)
    assert abs(fixed - free) < 1e-12, (fixed, free)
    print(f"    fixed {fixed:.3f} W vs free-in-plain-pocket {free:.3f} W - identical.")
    print("    omega_pin = v/Rr and the pocket torque is mu*F*Rr, so Rr cancels.")
    print("    Only a ROLLING element in the pocket removes it.")

    print("[9] both interfaces matter - neither one dominates")
    v_ring = ref.ring_sliding_speed(ref_rpm)
    v_out = ref.output_sliding_speed(ref_rpm)
    ring_w, out_w = ref.friction_loss_w(ref_t, ref_rpm, MU_DRY_PLASTIC,
                                        MU_DRY_PLASTIC)
    share = out_w / (ring_w + out_w)
    assert 0.25 < share < 0.75, share
    print(f"    ring contacts slide {1000*v_ring:.1f} mm/s -> {ring_w:.2f} W "
          f"({100*(1-share):.0f} %)")
    print(f"    output bores slide {1000*v_out:.1f} mm/s -> {out_w:.2f} W "
          f"({100*share:.0f} %)")

    print("[10] efficiency lands in the published band, and GREASE is the lever")
    dry = ref.efficiency(ref_t, ref_rpm, MU_DRY_PLASTIC, MU_DRY_PLASTIC)
    greased = ref.efficiency(ref_t, ref_rpm, MU_GREASED, MU_GREASED)
    needle = ref.efficiency(ref_t, ref_rpm, MU_NEEDLE_ROLLER, MU_NEEDLE_ROLLER)
    assert 0.50 < dry < greased < needle < 1.0
    assert 0.85 < greased < 0.99, greased
    print(f"    dry {100*dry:.0f} %, greased {100*greased:.0f} %, "
          f"needle rollers {100*needle:.0f} % "
          f"(commercial cycloidals publish 85-93 %)")
    print(f"    grease is worth {100*(greased-dry):.0f} points - far more than "
          "any pin material choice")

    print("[11] a rotating SLEEVE does help - the radii no longer cancel")
    sleeve = CycloidalLayout(ring_pin_material="petg", ring_pin_sleeve=True,
                             pin_r=2.25, sleeve_wall=0.5)
    assert sleeve.post_r == 1.75
    assert sleeve.sleeve_rotates(MU_GREASED, MU_GREASED), "must not seize"
    factor = sleeve.ring_loss_factor(MU_GREASED, MU_GREASED)
    assert 0.5 < factor < 1.0, factor
    print(f"    post {2*sleeve.post_r:.1f} mm in a {2*sleeve.pin_r:.1f} mm sleeve "
          f"-> geometric lever {sleeve.post_r/sleeve.pin_r:.2f}, but the polymer "
          f"raceway's\n    rolling resistance eats most of it: net x{factor:.2f} "
          f"when greased")

    print("[12] the sleeve and the grease are SUBSTITUTES, not complements")
    plain = CycloidalLayout(ring_pin_material="petg", ring_pin_fixed=True,
                            pin_r=2.25)
    dry_gain = (sleeve.efficiency(ref_t, ref_rpm, MU_DRY_PLASTIC,
                                  MU_DRY_PLASTIC, MU_GREASED)
                - plain.efficiency(ref_t, ref_rpm, MU_DRY_PLASTIC, MU_DRY_PLASTIC))
    wet_gain = (sleeve.efficiency(ref_t, ref_rpm)
                - plain.efficiency(ref_t, ref_rpm))
    assert dry_gain > 5 * wet_gain, "sleeve should pay off mainly when dry"
    assert sleeve.efficiency(ref_t, ref_rpm) < 0.95, "sleeves fall short of 95 %"
    assert sleeve.efficiency(ref_t, ref_rpm, MU_NEEDLE_ROLLER,
                             MU_NEEDLE_ROLLER, MU_NEEDLE_ROLLER) > 0.95
    print(f"    sleeve is worth {100*dry_gain:+.0f} points DRY but only "
          f"{100*wet_gain:+.0f} points GREASED")
    print("    both attack the same term, so they do not stack. A sleeve is")
    print("    insurance against losing the grease, not a bonus on top of it.")
    print("    95 % needs a rolling ELEMENT (mu 16x lower), not a radius ratio.")

    print("[13] the sleeve wall is paid for in POST BENDING, as 1/d^3")
    solid = CycloidalLayout(ring_pin_material="petg", ring_pin_fixed=True,
                            pin_r=2.25)
    ratio = sleeve.pin_bending_stress(ref_t) / solid.pin_bending_stress(ref_t)
    assert abs(ratio - (2.25 / 1.75) ** 3) < 1e-9
    print(f"    same 4.5 mm envelope: solid post "
          f"{solid.pin_bending_stress(ref_t):.0f} MPa vs sleeved "
          f"{sleeve.pin_bending_stress(ref_t):.0f} MPa ({ratio:.2f}x)")
    need = sleeve.min_printed_pin_diameter(ref_t)
    print(f"    post must be >= {need:.2f} mm, so sleeve OD >= "
          f"{need + 2*sleeve.sleeve_wall:.2f} mm")

    print("[14] the sleeve also puts flank contact stress back up to steel")
    p_petg = solid.hertz_pressure(solid.ring_pin_force(ref_t), solid.pin_r, "petg")
    p_steel = sleeve.hertz_pressure(sleeve.ring_pin_force(ref_t), sleeve.pin_r,
                                    "steel")
    assert p_steel > p_petg, "steel sleeve is a stiffer contact pair"
    print(f"    printed post {p_petg:.0f} MPa vs steel sleeve {p_steel:.0f} MPa "
          f"at the same {2*sleeve.pin_r:.1f} mm OD")

    print("[15] the output pins, not the ratio, are what caps this envelope")
    ceiling = ref.max_output_torque()
    assert abs(ref.hertz_pressure(ref.output_pin_force(ceiling),
                                  ref.output_pin_r) - PETG_YIELD) < 1e-9
    assert ceiling < 4.26, "shoulder demand should not fit"
    bigger = CycloidalLayout(output_pin_r=3.0)
    assert bigger.max_output_torque() > ceiling
    assert bigger.radial_overrun() > ref.radial_overrun(), \
        "bigger pins must make the packing worse - that is the bind"
    print(f"    ceiling {ceiling:.2f} N.m vs 4.26 N.m shoulder demand "
          f"({4.26/ceiling:.1f}x over)")
    print(f"    raising pin r 2.0 -> 3.0 lifts it to {bigger.max_output_torque():.2f} "
          f"N.m but overrun {ref.radial_overrun():+.2f} -> "
          f"{bigger.radial_overrun():+.2f} mm")
    small_brg = CycloidalLayout(bearing_od=12.0)
    assert small_brg.radial_overrun() < ref.radial_overrun()
    print(f"    a 12 mm eccentric bearing buys back "
          f"{ref.radial_overrun() - small_brg.radial_overrun():.2f} mm of it")

    print("[16] the envelope check actually bites")
    fat = CycloidalLayout(pin_circle_r=22.0)
    assert any(not ok for name, ok, _ in fat.check() if "NEMA" in name)
    print(f"    R = 22 mm -> housing {fat.housing_r:.1f} mm, rejected")

    print("\nall cycloidal_layout self-checks passed")


def main() -> None:
    _self_check()

    print("\n" + "=" * 72)
    print("D19 reference layout (15:1-class, e = 0.65 mm, 42 x 42 mm face)\n")
    print(CycloidalLayout().report(output_nm=4.26))   # shoulder demand, D8

    print("\n" + "=" * 72)
    print("steel dowels vs pins printed into the housing (shoulder, 4.26 N.m)\n")
    shoulder, rpm = 4.26, 300.0
    steel = CycloidalLayout()
    printed = CycloidalLayout(ring_pin_material="petg", ring_pin_fixed=True,
                              pin_r=2.0)
    for name, lay, mu in (("steel dowel", steel, MU_DRY_STEEL_PLASTIC),
                          ("printed, d=4 mm", printed, MU_DRY_PLASTIC)):
        p = lay.hertz_pressure(lay.ring_pin_force(shoulder), lay.pin_r,
                               lay.ring_pin_material)
        sig = lay.pin_bending_stress(shoulder)
        print(f"  {name:16s} contact {p:5.0f} MPa  bending {sig:5.0f} MPa  "
              f"eta_dry {100*lay.efficiency(shoulder, rpm, mu, mu):4.0f} %  "
              f"eta_greased {100*lay.efficiency(shoulder, rpm, MU_GREASED, MU_GREASED):4.0f} %")
    print(f"\n  Mass saved by printing: {steel.steel_pin_mass_g:.1f} g of a 1806 g "
          f"moving arm = {100*steel.steel_pin_mass_g/1806:.2f} % per joint.")
    ring_w, out_w = steel.friction_loss_w(shoulder, rpm, MU_GREASED, MU_GREASED)
    print(f"  Where the loss lives (greased): ring {ring_w:.2f} W, "
          f"output pins {out_w:.2f} W -> neither dominates, so a fix applied to "
          f"only\n  one interface is capped at "
          f"{100*max(ring_w, out_w)/(ring_w+out_w):.0f} % of the available gain.")
    print("  The pin MATERIAL barely moves efficiency. The friction coefficient "
          "does.")

    print("\n" + "=" * 72)
    print("ratio sweep at the envelope limit - what the 42 mm face will hold\n")
    base = CycloidalLayout()
    r_max = NEMA17_FACE / 2 - base.pin_r - base.housing_wall
    print(f"pin circle pinned to the face limit: R = {r_max:.2f} mm, K = 0.60\n")
    print(f"{'pins':>5} {'ratio':>6} {'e':>6} {'ring MPa':>9} {'out MPa':>8} "
          f"  first failure")
    print("-" * 62)
    for pins in (10, 12, 14, 16, 18, 20, 24):
        lay = CycloidalLayout(pins=pins, pin_circle_r=r_max,
                              eccentricity=0.60 * r_max / pins)
        checks = lay.check(4.26)
        first = next((n for n, ok, _ in checks if not ok), "-")
        print(f"{pins:5d} {lay.ratio:6d} {lay.eccentricity:6.2f} "
              f"{lay.hertz_pressure(lay.ring_pin_force(4.26), lay.pin_r):9.0f} "
              f"{lay.hertz_pressure(lay.output_pin_force(4.26), lay.output_pin_r):8.0f} "
              f"  {first}")
    print("\nRatio is nearly free here - it is set by pin count, and pin count "
          "barely\nmoves the stress. The binding constraints are the output-pin "
          "packing and\nthe contact stress on PETG, which is why D19 makes ratio "
          "an OUTPUT of the\nlayout. Fix those before choosing a number.")


if __name__ == "__main__":
    main()
