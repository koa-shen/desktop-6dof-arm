"""Single-joint plant model, and the firmware's control law running against it.

WHY THIS FILE EXISTS
The expensive way to discover that a control law hunts is to flash it, hear the
joint buzz, and spend an evening guessing which gain to change. The cheap way
is to model the plant well enough that the failure shows up in a table of
numbers, decide what to do about it, and only then touch hardware. This file is
the cheap way, and it runs with no hardware attached.

WHAT IS MODELLED, AND WHY EACH PIECE EARNS ITS PLACE
  * stepper as a rate source with an acceleration limit - the driver's actual
    behaviour, and the reason the controller output is a velocity not a torque
  * a torsional spring between motor and output - a printed cycloidal reducer
    is not rigid, and its compliance is what turns a stiff loop into a ringing
    one
  * backlash as a deadband inside that spring - the reason integral action is
    dangerous here
  * encoder quantization at 0.088 deg - the single most important term in the
    whole model, because it is 8 microsteps wide at 20:1 and it is what makes a
    naive PID limit-cycle
  * step loss when demanded torque exceeds what the motor can hold - the
    failure mode an open-loop stepper has and a servo does not

WHAT IS NOT MODELLED: motor electrical dynamics, resonance of the rotor in its
detent well, thermal derating, and structural modes of the links. The first
three do not change the conclusions below; the fourth is why the jerk limiter
exists, and it is a reason to keep TAPS > 1 rather than a reason to model it.

Run it:
    python joint_sim.py            # the three experiments and a gain sweep
    python joint_sim.py --plot     # same, with figures (needs matplotlib)
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass, field

from arm_model import default_arm
from trajectory import MotionController

# ---------------------------------------------------------------- constants --
DEG_PER_COUNT = 360.0 / 4096.0  # AS5600, 12-bit
MOTOR_FULL_STEPS_PER_REV = 200.0


def deadband(value: float, width: float) -> float:
    if value > width:
        return value - width
    if value < -width:
        return value + width
    return 0.0


def inertia_about_joint(arm, joint_index: int) -> float:
    """Rotational inertia seen by one joint, arm straight out (worst case).

    Point-mass approximation: every outboard link contributes m*r^2 about this
    joint's axis. Optimistic by the links' own second moments, which is fine -
    the numbers that matter here are within a factor of two and the conclusions
    are not sensitive to them.
    """
    j = 0.0
    x = 0.0
    for i, link in enumerate(arm.links):
        if i >= joint_index:
            j += link.mass * (x + link.com) ** 2
            x += link.length
    j += (arm.payload + arm.tool_mass) * x ** 2
    return j


# -------------------------------------------------------------------- plant --
@dataclass
class JointPlant:
    """One geared joint, seen from the output side.

    Stiffness and damping are the two numbers here that are guesses rather than
    derivations. Measure them: hang a known mass off the link, read the encoder
    deflection for stiffness; tap the link and count the ring-down for damping.
    Until then, treat conclusions that hinge on them as provisional.
    """

    gear_ratio: float = 20.0
    microsteps: float = 8.0
    inertia: float = 0.05           # kg m^2 at the output
    stiffness: float = 300.0        # Nm/rad, motor-to-output through the reducer
    damping_ratio: float = 0.15     # of critical; printed gearboxes are lossy
    backlash_deg: float = 0.5       # total, at the output
    coulomb_nm: float = 0.15
    viscous_nms: float = 0.05
    motor_holding_nm: float = 0.42  # 40 mm NEMA 17
    gearbox_eff: float = 0.70
    gravity_nm: float = 1.5         # constant; worst case is what matters here
    accel_steps_s2: float = 4000.0
    encoder_noise_counts: float = 0.5  # AS5600 dither + magnet runout + I2C
    seed: int = 1

    theta_m: float = 0.0   # motor angle referred to the output, deg
    omega_m: float = 0.0
    theta_o: float = 0.0   # true output angle, deg
    omega_o: float = 0.0
    slips: int = 0
    _rng: random.Random = field(default=None, repr=False)

    @property
    def steps_per_output_deg(self) -> float:
        return MOTOR_FULL_STEPS_PER_REV * self.microsteps * self.gear_ratio / 360.0

    @property
    def stall_torque_out(self) -> float:
        return self.motor_holding_nm * self.gear_ratio * self.gearbox_eff

    @property
    def damping(self) -> float:
        return self.damping_ratio * 2.0 * math.sqrt(self.stiffness * self.inertia)

    @property
    def natural_hz(self) -> float:
        return math.sqrt(self.stiffness / self.inertia) / (2.0 * math.pi)

    def reset(self, angle_deg: float = 0.0) -> None:
        self.theta_o = angle_deg
        self.omega_m = self.omega_o = 0.0
        self.slips = 0
        self._rng = random.Random(self.seed)
        # Start at STATIC EQUILIBRIUM, not at zero twist. Starting untwisted
        # means every run begins with the arm falling under gravity until the
        # reducer winds up, which is a transient of the initial condition and
        # not of the controller.
        sag = math.degrees(self.gravity_nm / self.stiffness)
        self.theta_m = angle_deg + sag + math.copysign(
            self.backlash_deg * 0.5, sag if sag else 1.0)

    def step(self, dt: float, cmd_step_rate: float) -> None:
        # --- motor side: a rate source with an acceleration limit ------------
        v_cmd = cmd_step_rate / self.steps_per_output_deg
        a_max = self.accel_steps_s2 / self.steps_per_output_deg
        dv = max(-a_max * dt, min(a_max * dt, v_cmd - self.omega_m))
        self.omega_m += dv
        self.theta_m += self.omega_m * dt

        # --- transmission: spring with a backlash deadband -------------------
        twist = self.theta_m - self.theta_o
        engaged = deadband(twist, self.backlash_deg * 0.5)
        if engaged != 0.0:
            tau = (self.stiffness * math.radians(engaged)
                   + self.damping * math.radians(self.omega_m - self.omega_o))
        else:
            tau = 0.0  # free flight inside the backlash: nothing is connected

        # --- step loss: the open-loop stepper's characteristic failure -------
        if abs(tau) > self.stall_torque_out:
            tau = math.copysign(self.stall_torque_out, tau)
            hold = math.degrees(tau / self.stiffness)
            self.theta_m = self.theta_o + hold + math.copysign(
                self.backlash_deg * 0.5, hold)
            self.omega_m = self.omega_o
            self.slips += 1

        # --- output side -----------------------------------------------------
        w_rad = math.radians(self.omega_o)
        friction = self.coulomb_nm * math.tanh(w_rad / 0.01) + self.viscous_nms * w_rad
        alpha = (tau - self.gravity_nm - friction) / self.inertia
        self.omega_o += math.degrees(alpha) * dt
        self.theta_o += self.omega_o * dt

    def encoder_counts(self) -> int:
        noise = 0.0
        if self.encoder_noise_counts and self._rng is not None:
            noise = self._rng.gauss(0.0, self.encoder_noise_counts)
        return int(round(self.theta_o / DEG_PER_COUNT + noise))


# --------------------------------------------------------------- controller --
@dataclass
class JointLoop:
    """Faithful replica of JointController's control law.

    Deliberately written to mirror the C++ line for line rather than to be
    idiomatic Python. If the two ever disagree, this file is wrong.
    """

    kp: float = 8.0              # 1/s, counts domain
    ki: float = 0.0
    kd: float = 0.0
    steps_per_count: float = 7.81
    deadband_counts: int = 2
    vel_ff_scale: float = 1.0
    max_step_rate: float = 1600.0
    integral_limit: float = 50.0
    use_feedforward: bool = True

    integral: float = 0.0
    last_measurement: float = 0.0
    primed: bool = False

    def reset(self) -> None:
        self.integral = 0.0
        self.primed = False

    def update(self, setpoint_counts: float, vel_ff_counts_s: float,
               measured_counts: float, dt: float) -> float:
        steps = 0.0
        if self.use_feedforward:
            steps = vel_ff_counts_s * self.steps_per_count * self.vel_ff_scale

        err = setpoint_counts - measured_counts
        if abs(err) <= self.deadband_counts:
            self.last_measurement = measured_counts
            self.primed = True
            return max(-self.max_step_rate, min(self.max_step_rate, steps))

        out_limit = self.max_step_rate / self.steps_per_count
        p = self.kp * err
        self.integral += err * dt
        self.integral = max(-self.integral_limit,
                            min(self.integral_limit, self.integral))
        i = self.ki * self.integral
        d = 0.0
        if self.primed:
            d = -self.kd * (measured_counts - self.last_measurement) / dt
        self.last_measurement = measured_counts
        self.primed = True

        out = p + i + d
        if out > out_limit:
            out = out_limit
            if self.ki and err > 0:
                self.integral -= err * dt
        elif out < -out_limit:
            out = -out_limit
            if self.ki and err < 0:
                self.integral -= err * dt

        steps += out * self.steps_per_count
        return max(-self.max_step_rate, min(self.max_step_rate, steps))


# ----------------------------------------------------------------- harness --
@dataclass
class Result:
    t: list[float] = field(default_factory=list)
    setpoint_deg: list[float] = field(default_factory=list)
    measured_deg: list[float] = field(default_factory=list)
    true_deg: list[float] = field(default_factory=list)
    cmd_step_rate: list[float] = field(default_factory=list)
    slips: int = 0


def simulate(plant: JointPlant, loop: JointLoop, target_deg: float,
             duration: float = 4.0, control_dt: float = 0.005,
             physics_dt: float = 5e-5, profiled: bool = False,
             traj_vel_deg_s: float = 15.0, traj_acc_deg_s2: float = 40.0,
             taps: int = 4) -> Result:
    """Run one move. `profiled` selects a streamed trajectory over a step."""
    plant.reset(0.0)
    loop.reset()
    res = Result()

    target_counts = target_deg / DEG_PER_COUNT
    mc = None
    if profiled:
        mc = MotionController(1, traj_vel_deg_s / DEG_PER_COUNT,
                              traj_acc_deg_s2 / DEG_PER_COUNT, taps)
        mc.reset_to([0.0])
        mc.move_to([target_counts], [0.0])

    t = 0.0
    cmd = 0.0
    # One control period of measurement delay: the encoder value the loop acts
    # on is always the previous cycle's. Ignoring this is how simulations
    # predict stability the hardware does not have.
    measured = 0.0

    while t < duration:
        if mc is not None:
            mc.update(control_dt)
            sp, vff = mc.setpoint[0], mc.velocity[0]
        else:
            sp, vff = target_counts, 0.0

        cmd = loop.update(sp, vff, measured, control_dt)
        measured = plant.encoder_counts()

        sub = int(round(control_dt / physics_dt))
        for _ in range(sub):
            plant.step(physics_dt, cmd)

        res.t.append(t)
        res.setpoint_deg.append(sp * DEG_PER_COUNT)
        res.measured_deg.append(measured * DEG_PER_COUNT)
        res.true_deg.append(plant.theta_o)
        res.cmd_step_rate.append(cmd)
        t += control_dt

    res.slips = plant.slips
    return res


def metrics(res: Result, target_deg: float, tol_deg: float = 0.2,
            tail_s: float = 1.0) -> dict:
    """Step-response numbers, plus the two that matter for a stepper."""
    n = len(res.t)
    dt = res.t[1] - res.t[0]
    tail = max(1, int(tail_s / dt))
    y = res.true_deg

    rise = float("nan")
    lo = hi = None
    for i, v in enumerate(y):
        if lo is None and v >= 0.1 * target_deg:
            lo = res.t[i]
        if hi is None and v >= 0.9 * target_deg:
            hi = res.t[i]
            break
    if lo is not None and hi is not None:
        rise = hi - lo

    peak = max(y) if target_deg > 0 else min(y)
    overshoot = 100.0 * (peak - target_deg) / abs(target_deg)

    settle = float("nan")
    for i in range(n - 1, -1, -1):
        if abs(y[i] - target_deg) > tol_deg:
            if i + 1 < n:
                settle = res.t[i + 1]
            break
    else:
        settle = 0.0

    tail_y = y[-tail:]
    tail_cmd = res.cmd_step_rate[-tail:]
    reversals = sum(
        1 for a, b in zip(tail_cmd, tail_cmd[1:])
        if a != 0.0 and b != 0.0 and (a > 0) != (b > 0)
    )

    return {
        "rise_s": rise,
        "overshoot_pct": overshoot,
        "settle_s": settle,
        "steady_err_deg": y[-1] - target_deg,
        # The two numbers a stepper loop lives or dies by:
        "hunt_pp_deg": max(tail_y) - min(tail_y),
        "reversals_per_s": reversals / tail_s,
        "slips": res.slips,
    }


def _row(name: str, m: dict) -> str:
    return (f"{name:<34} {m['rise_s']:>7.3f} {m['overshoot_pct']:>9.1f} "
            f"{m['settle_s']:>8.3f} {m['steady_err_deg']:>10.3f} "
            f"{m['hunt_pp_deg']:>9.3f} {m['reversals_per_s']:>10.1f} "
            f"{m['slips']:>6d}")


HEADER = (f"{'case':<34} {'rise_s':>7} {'over_%':>9} {'settle_s':>8} "
          f"{'ss_err':>10} {'hunt_pp':>9} {'revs/s':>10} {'slips':>6}")


def build_default_plant(joint_index: int = 2) -> JointPlant:
    """Plant parameters traceable to tools/arm_model.py where possible.

    Defaults to J3, the elbow: 40 mm motor at 20:1, per D5's torque tiering.
    The shoulder (index 1) is the harder case and gets a 60 mm motor and a
    higher ratio, which is why it is worth running both.
    """
    arm = default_arm(reach=0.400, payload=0.500)
    inertia = inertia_about_joint(arm, joint_index)
    gravity = arm.gravity_torque(joint_index, [0, 0, 0, 0])
    holding, ratio = (0.68, 26.0) if joint_index <= 1 else (0.42, 20.0)
    return JointPlant(inertia=inertia, gravity_nm=gravity,
                      motor_holding_nm=holding, gear_ratio=ratio)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--target", type=float, default=10.0, help="step size, deg")
    ap.add_argument("--joint", type=int, default=2,
                    help="arm_model link index: 1 = shoulder, 2 = elbow")
    args = ap.parse_args()

    plant = build_default_plant(args.joint)
    steps_per_count = plant.steps_per_output_deg * DEG_PER_COUNT

    print(f"Plant: joint {args.joint}, J = {plant.inertia:.4f} kg m^2 at the "
          f"output, gravity = {plant.gravity_nm:.2f} Nm, "
          f"stall = {plant.stall_torque_out:.1f} Nm "
          f"({plant.stall_torque_out / plant.gravity_nm:.1f}x margin)")
    print(f"       {plant.steps_per_output_deg:.1f} steps/output-deg, "
          f"{steps_per_count:.2f} steps per encoder count, "
          f"backlash {plant.backlash_deg:.2f} deg")
    print(f"       gravity sag through the reducer = "
          f"{math.degrees(plant.gravity_nm / plant.stiffness):.3f} deg, "
          f"first torsional mode ~{plant.natural_hz:.1f} Hz")
    print(f"       encoder noise {plant.encoder_noise_counts:.1f} counts rms "
          f"= {plant.encoder_noise_counts * DEG_PER_COUNT:.3f} deg")

    # The ceiling that decides whether a trajectory is even askable for. Plan a
    # move faster than this and the loop saturates, following error explodes,
    # and it reads exactly like a badly tuned PID.
    max_deg_s = 1600.0 / plant.steps_per_output_deg
    max_deg_s2 = 4000.0 / plant.steps_per_output_deg
    print(f"       driver ceiling: {max_deg_s:.1f} deg/s, {max_deg_s2:.0f} "
          f"deg/s^2 - plan below these or the loop saturates\n")

    cases = []

    naive = JointLoop(steps_per_count=steps_per_count, deadband_counts=0,
                      use_feedforward=False)
    cases.append(("A  step + PID, no deadband, no FF",
                  simulate(plant, naive, args.target)))

    banded = JointLoop(steps_per_count=steps_per_count, deadband_counts=2,
                       use_feedforward=False)
    cases.append(("B  step + PID + 2-count deadband",
                  simulate(plant, banded, args.target)))

    full = JointLoop(steps_per_count=steps_per_count, deadband_counts=2,
                     use_feedforward=True)
    cases.append(("C  profiled + FF + deadband",
                  simulate(plant, full, args.target, profiled=True)))

    integral = JointLoop(steps_per_count=steps_per_count, deadband_counts=2,
                         ki=4.0, use_feedforward=True)
    cases.append(("D  as C, but with Ki = 4",
                  simulate(plant, integral, args.target, profiled=True)))

    print(HEADER)
    print("-" * len(HEADER))
    results = {}
    for name, res in cases:
        m = metrics(res, args.target)
        results[name] = (res, m)
        print(_row(name, m))

    print("\nKp sweep (case C configuration, profiled + FF + deadband)")
    print(f"{'kp':>6} {'peak_follow_err_deg':>20} {'hunt_pp':>9} {'revs/s':>10}")
    for kp in (2.0, 4.0, 8.0, 16.0, 32.0, 64.0):
        loop = JointLoop(kp=kp, steps_per_count=steps_per_count,
                         deadband_counts=2, use_feedforward=True)
        r = simulate(plant, loop, args.target, profiled=True)
        follow = max(abs(s - y) for s, y in zip(r.setpoint_deg, r.true_deg))
        m = metrics(r, args.target)
        print(f"{kp:>6.1f} {follow:>20.3f} {m['hunt_pp_deg']:>9.3f} "
              f"{m['reversals_per_s']:>10.1f}")

    print("\nDeadband sweep (case C configuration)")
    print(f"{'counts':>6} {'deg':>7} {'hunt_pp':>9} {'revs/s':>10} {'ss_err':>9}")
    for db in (0, 1, 2, 3, 5):
        loop = JointLoop(steps_per_count=steps_per_count, deadband_counts=db,
                         use_feedforward=True)
        r = simulate(plant, loop, args.target, profiled=True)
        m = metrics(r, args.target)
        print(f"{db:>6d} {db * DEG_PER_COUNT:>7.3f} {m['hunt_pp_deg']:>9.3f} "
              f"{m['reversals_per_s']:>10.1f} {m['steady_err_deg']:>9.3f}")

    if args.plot:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(len(cases), 1, sharex=True,
                                 figsize=(9, 2.4 * len(cases)))
        for ax, (name, _) in zip(axes, cases):
            res, _m = results[name]
            ax.plot(res.t, res.setpoint_deg, label="setpoint")
            ax.plot(res.t, res.true_deg, label="true")
            ax.plot(res.t, res.measured_deg, label="encoder", alpha=0.5)
            ax.set_title(name, fontsize=9)
            ax.set_ylabel("deg")
            ax.grid(alpha=0.3)
        axes[0].legend(fontsize=8)
        axes[-1].set_xlabel("s")
        fig.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
