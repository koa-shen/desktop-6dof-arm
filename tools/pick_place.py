"""Layer 2: the pick-and-place task, running on the host.

This is the top of D7's three-layer stack. It owns the *task* - where to pick,
where to place, what order to do things in - and nothing else. It asks
kinematics.py where the joints need to be, hands that to trajectory.py to turn
into a timed stream of setpoints, and pushes that stream at the arm through
joint_link.py. The Uno running app_08_host_link never learns that a
pick-and-place is happening; it only ever hears "be at this count, moving at
this rate".

That separation is the point. An ATmega328P cannot do IK, and a host cannot
close a servo loop over USB with any timing guarantee. Putting each job where
it can actually be done is the difference between an arm and a demo.

    python pick_place.py                     # dry run, no hardware
    python pick_place.py --port COM5         # for real
    python pick_place.py --dry-run --verbose # print every waypoint's plan

WHAT THIS DOES NOT DO, DELIBERATELY
  * No collision checking. The workspace is a desk and the operator is the
    collision checker until there is a reason for it not to be.
  * No grasp verification. The gripper is open loop (D11); "gripped" here means
    "the servo was commanded and we waited", not "an object is held".
  * No Cartesian straight lines. Moves are joint-space, so the tool traces an
    arc between waypoints. For pick-and-place with clearance that is fine, and
    it is one less thing that can produce a singularity mid-move.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass, field

import numpy as np

import joint_link as jl
from arm_model import JOINT_LIMITS_DEG
from kinematics import fk, ik
from trajectory import MotionController

TAU = 2 * math.pi

# What a joint's motion costs when choosing between IK branches. The big three
# carry the arm's mass and dominate cycle time; the wrist is comparatively free
# to reorient, so letting it flip to avoid a shoulder swing is a good trade.
BRANCH_WEIGHTS = (1.0, 1.0, 1.0, 0.3, 0.3, 0.3)

# The subset of joints that physically exist today. IK still solves all six -
# planning for the arm you are building, not the arm you have, is what stops
# the host stack from needing a rewrite when J4-J6 arrive.
DEFAULT_ACTUATED = 3

CONTROL_HZ = 200.0
TRAJ_MAX_VEL_DEG_S = 15.0   # must match include/joint_config.h
TRAJ_MAX_ACC_DEG_S2 = 40.0
JERK_FILTER_TAPS = 4
MAX_BRANCH_JUMP_DEG = 120.0

GRIPPER_SETTLE_S = 0.25     # GRIPPER_SETTLE_MS
GRIPPER_SLEW_DEG_S = 120.0
GRIPPER_TRAVEL_DEG = 80.0   # open -> grip


def tool_down_pose(x: float, y: float, z: float, yaw_deg: float = 0.0) -> np.ndarray:
    """Flange pose with the approach axis pointing straight down.

    Tool +Z is the approach direction, so top-down picking means +Z = -world Z.
    `yaw` spins the tool about that axis, which is how the gripper's jaws get
    lined up with the object.
    """
    yaw = math.radians(yaw_deg)
    z_t = np.array([0.0, 0.0, -1.0])
    x_t = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    y_t = np.cross(z_t, x_t)
    pose = np.eye(4)
    pose[:3, 0] = x_t
    pose[:3, 1] = y_t
    pose[:3, 2] = z_t
    pose[:3, 3] = (x, y, z)
    return pose


def branches(pose: np.ndarray, q_now) -> list[np.ndarray]:
    """Every IK branch this arm can actually reach from where it is, cheapest
    first.

    kinematics.best_solution folds every answer into (-pi, pi] and measures
    distance the same way, which quietly assumes a joint can wrap. These joints
    cannot: J1 stops at +/-180 and J2 at +/-135. A branch that is "49 deg away"
    through a wrap the hardware cannot perform is not 49 deg away, it is 311,
    and choosing it is how a transit move turns into a full-speed swing through
    the table.

    So: unwrap each joint onto the value nearest its current angle that is still
    inside its limits, discard branches where any joint has no such value, and
    sort what is left by weighted travel.
    """
    q_now = np.asarray(q_now, dtype=float)
    scored = []
    for sol in ik(pose):
        cand = np.zeros(6)
        feasible = True
        for j in range(6):
            lo, hi = JOINT_LIMITS_DEG[j]
            options = [sol[j] + k * TAU for k in (-1, 0, 1)
                       if lo <= math.degrees(sol[j] + k * TAU) <= hi]
            if not options:
                feasible = False
                break
            cand[j] = min(options, key=lambda a: abs(a - q_now[j]))
        if feasible:
            scored.append((_travel_cost(cand, q_now), cand))
    scored.sort(key=lambda t: t[0])
    return [c for _, c in scored]


def _travel_cost(q: np.ndarray, q_now: np.ndarray) -> float:
    return sum(w * abs(q[j] - q_now[j]) for j, w in enumerate(BRANCH_WEIGHTS))


@dataclass
class Waypoint:
    name: str
    pose: np.ndarray
    gripper_percent: int | None = None  # applied AFTER arriving; None = leave it
    speed_scale: float = 1.0


@dataclass
class TaskConfig:
    # Defaults chosen by sweeping the planner, not by eyeballing the workspace
    # plot: a point can be inside the reachable volume and still have no branch
    # that reaches the whole eight-waypoint task without a reconfiguration. At
    # z = 0.05 m these points put J2 on its -135 deg limit and the plan fails.
    pick: tuple[float, float, float] = (0.25, -0.08, 0.10)
    place: tuple[float, float, float] = (0.25, 0.08, 0.10)
    approach_height: float = 0.08   # clearance above the object, m
    lift_height: float = 0.10       # how high to carry it
    pick_yaw_deg: float = 0.0
    place_yaw_deg: float = 0.0
    open_percent: int = 0
    grip_percent: int = 100
    transit_speed: float = 1.0
    approach_speed: float = 0.35    # slow near the object; this is where
                                    # following error turns into a crash


def build_task(cfg: TaskConfig) -> list[Waypoint]:
    """The classic eight-step pick and place.

    The two DESCEND/RETREAT pairs exist so the tool always enters and leaves a
    grasp along its own approach axis. Diving at an object from an arbitrary
    direction is how a gripper knocks it over before it closes.
    """
    px, py, pz = cfg.pick
    qx, qy, qz = cfg.place
    ap = cfg.approach_height
    lift = cfg.lift_height
    return [
        Waypoint("approach pick", tool_down_pose(px, py, pz + ap, cfg.pick_yaw_deg),
                 gripper_percent=cfg.open_percent, speed_scale=cfg.transit_speed),
        Waypoint("descend to pick", tool_down_pose(px, py, pz, cfg.pick_yaw_deg),
                 speed_scale=cfg.approach_speed),
        Waypoint("grip", tool_down_pose(px, py, pz, cfg.pick_yaw_deg),
                 gripper_percent=cfg.grip_percent, speed_scale=cfg.approach_speed),
        Waypoint("lift", tool_down_pose(px, py, pz + lift, cfg.pick_yaw_deg),
                 speed_scale=cfg.approach_speed),
        Waypoint("transit", tool_down_pose(qx, qy, qz + lift, cfg.place_yaw_deg),
                 speed_scale=cfg.transit_speed),
        Waypoint("descend to place", tool_down_pose(qx, qy, qz, cfg.place_yaw_deg),
                 speed_scale=cfg.approach_speed),
        Waypoint("release", tool_down_pose(qx, qy, qz, cfg.place_yaw_deg),
                 gripper_percent=cfg.open_percent, speed_scale=cfg.approach_speed),
        Waypoint("retreat", tool_down_pose(qx, qy, qz + ap, cfg.place_yaw_deg),
                 speed_scale=cfg.approach_speed),
    ]


# ------------------------------------------------------------- back ends --
class Backend:
    """What layer 2 needs from the world below it. Two implementations:
    a real link, and a perfect-tracking stand-in for planning without hardware.
    """

    def positions_counts(self) -> list[float]: ...
    def send_track(self, counts: list[float], vels: list[float]) -> None: ...
    def gripper(self, percent: int) -> None: ...
    def poll(self) -> None: ...
    def healthy(self) -> bool: return True
    def in_position(self) -> bool: return True
    def stop(self) -> None: ...


class DryRunBackend(Backend):
    """Tracks setpoints exactly.

    This validates sequencing, reachability, timing and branch continuity - the
    things that are actually wrong the first time. It cannot tell you anything
    about following error or hunting; that is joint_sim.py's job, and it uses a
    real plant model to do it.
    """

    def __init__(self, n: int, start_counts: list[float]) -> None:
        self.n = n
        self._pos = list(start_counts)

    def positions_counts(self) -> list[float]:
        return list(self._pos)

    def send_track(self, counts: list[float], vels: list[float]) -> None:
        self._pos = [round(c) for c in counts]

    def gripper(self, percent: int) -> None:
        pass


class LinkBackend(Backend):
    def __init__(self, link: jl.ArmLink, n: int) -> None:
        self.link = link
        self.n = n

    def positions_counts(self) -> list[float]:
        return self.link.positions_counts()[:self.n]

    def send_track(self, counts: list[float], vels: list[float]) -> None:
        for j in range(self.n):
            self.link.send_track(j, counts[j], vels[j])

    def gripper(self, percent: int) -> None:
        self.link.gripper_percent(percent)

    def poll(self) -> None:
        self.link.poll()

    def healthy(self) -> bool:
        return all(s.healthy for s in self.link.state[:self.n])

    def in_position(self) -> bool:
        return all(s.in_position for s in self.link.state[:self.n])

    def stop(self) -> None:
        self.link.estop()


# ------------------------------------------------------------- the runner --
class TaskFailed(RuntimeError):
    pass


@dataclass
class Runner:
    backend: Backend
    actuated: int = DEFAULT_ACTUATED
    control_hz: float = CONTROL_HZ
    realtime: bool = True
    verbose: bool = False
    q: np.ndarray = field(default_factory=lambda: np.zeros(6))
    elapsed: float = 0.0

    def __post_init__(self) -> None:
        max_vel = TRAJ_MAX_VEL_DEG_S / jl.DEG_PER_COUNT
        max_acc = TRAJ_MAX_ACC_DEG_S2 / jl.DEG_PER_COUNT
        self.mc = MotionController(self.actuated, max_vel, max_acc,
                                   JERK_FILTER_TAPS)
        start = self.backend.positions_counts()
        self.mc.reset_to(start)
        # Seed the IK reference from where the arm actually is. Seeding it from
        # zeros makes best_solution pick the branch nearest a pose the arm is
        # not in, which is how the first move ends up being 150 deg long.
        self.q = np.zeros(6)
        for j in range(self.actuated):
            self.q[j] = math.radians(start[j] * jl.DEG_PER_COUNT)

    # -- planning ---------------------------------------------------------
    def plan(self, waypoints: list[Waypoint]) -> list[np.ndarray]:
        """Choose joint angles for the whole task at once.

        Choosing greedily - nearest branch to where the arm is right now, one
        waypoint at a time - is what produces a plan that starts cheaply and
        then discovers halfway through that the family it committed to cannot
        reach the place point without wrapping J1. Branch choice is a property
        of the whole path, so it gets decided over the whole path: try every
        feasible branch of the first waypoint as a seed, chain greedily from
        each, and keep the chain with no flips and the least travel.
        """
        seeds = branches(waypoints[0].pose, self.q)
        if not seeds:
            raise TaskFailed(self._unreachable(waypoints[0]))

        best = None      # (worst_jump, cost, path)
        for seed in seeds:
            q = seed
            path = [seed]
            cost = _travel_cost(seed, self.q)
            worst = 0.0  # the first move is from a standstill; it cannot flip
            for wp in waypoints[1:]:
                opts = branches(wp.pose, q)
                if not opts:
                    path = None
                    break
                nxt = opts[0]
                worst = max(worst, float(np.max(np.abs(np.degrees(
                    nxt[:self.actuated] - q[:self.actuated])))))
                cost += _travel_cost(nxt, q)
                q = nxt
                path.append(nxt)
            if path is None:
                continue
            key = (worst > MAX_BRANCH_JUMP_DEG, cost)
            if best is None or key < best[0]:
                best = (key, worst, path)

        if best is None:
            raise TaskFailed(
                "no branch of the first waypoint leads to a chain that reaches "
                "every later waypoint - check the points with workspace.py")
        _flipped, worst, path = best
        if worst > MAX_BRANCH_JUMP_DEG:
            raise TaskFailed(
                f"the best plan still needs a {worst:.0f} deg reconfiguration "
                f"mid-task; add an intermediate waypoint or move the points")
        if self.verbose:
            print(f"planned {len(path)} waypoints, largest single-joint move "
                  f"{worst:.0f} deg\n")
        return path

    def _unreachable(self, wp: Waypoint) -> str:
        return (f"'{wp.name}' at {np.round(wp.pose[:3, 3], 3).tolist()} m has no "
                f"solution inside the joint limits - move the pick/place point "
                f"or check reach with workspace.py")

    # -- execution --------------------------------------------------------
    def goto(self, wp: Waypoint, q: np.ndarray) -> float:
        target = [math.degrees(q[j]) / jl.DEG_PER_COUNT
                  for j in range(self.actuated)]
        current = self.backend.positions_counts()
        duration = self.mc.move_to(target, current, wp.speed_scale)

        if self.verbose:
            deltas = ", ".join(
                f"J{j+1} {(target[j]-current[j])*jl.DEG_PER_COUNT:+7.2f}"
                for j in range(self.actuated))
            print(f"    plan: {duration:5.2f} s  [{deltas}] deg")

        dt = 1.0 / self.control_hz
        deadline = time.monotonic()
        # Streaming continues past the profile's end until the nodes report
        # in-position: the profile finishing means the SETPOINT arrived, not
        # the joint. Confusing the two is how a move gets cut short.
        settle_budget = 2.0
        settled = 0.0
        while self.mc.moving or settled < settle_budget:
            self.mc.update(dt)
            self.backend.send_track(self.mc.setpoint, self.mc.velocity)
            self.backend.poll()
            if not self.backend.healthy():
                raise TaskFailed(f"joint fault during '{wp.name}'")
            if not self.mc.moving:
                if self.backend.in_position():
                    break
                settled += dt
            self.elapsed += dt
            if self.realtime:
                deadline += dt
                slack = deadline - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
                else:
                    deadline = time.monotonic()  # we are late; do not spiral
        else:
            raise TaskFailed(f"'{wp.name}' never reported in-position")

        self.q = q
        return duration

    def actuate_gripper(self, percent: int) -> None:
        self.backend.gripper(percent)
        wait = GRIPPER_TRAVEL_DEG / GRIPPER_SLEW_DEG_S + GRIPPER_SETTLE_S
        # Open loop: the only thing we can do is command it and wait long
        # enough that it must have finished. There is no feedback to check.
        end = self.elapsed + wait
        dt = 1.0 / self.control_hz
        while self.elapsed < end:
            self.backend.send_track(self.mc.setpoint, self.mc.velocity)
            self.backend.poll()
            self.elapsed += dt
            if self.realtime:
                time.sleep(dt)

    def run(self, waypoints: list[Waypoint]) -> None:
        path = self.plan(waypoints)
        for i, (wp, q) in enumerate(zip(waypoints, path), 1):
            print(f"[{i}/{len(waypoints)}] {wp.name}")
            self.goto(wp, q)
            if wp.gripper_percent is not None:
                print(f"    gripper -> {wp.gripper_percent}%")
                self.actuate_gripper(wp.gripper_percent)
        pos = fk(self.q)[:3, 3]
        print(f"\ndone in {self.elapsed:.1f} s of motion, tool at "
              f"{np.round(pos, 3).tolist()} m")


# ------------------------------------------------------------------- main --
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="serial port; omit for a dry run")
    ap.add_argument("--dry-run", action="store_true",
                    help="plan and simulate without hardware (the default)")
    ap.add_argument("--joints", type=int, default=DEFAULT_ACTUATED)
    ap.add_argument("--pick", nargs=3, type=float, metavar=("X", "Y", "Z"))
    ap.add_argument("--place", nargs=3, type=float, metavar=("X", "Y", "Z"))
    ap.add_argument("--cycles", type=int, default=1)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    if args.dry_run and args.port:
        ap.error("--dry-run and --port are mutually exclusive")

    cfg = TaskConfig()
    if args.pick:
        cfg.pick = tuple(args.pick)
    if args.place:
        cfg.place = tuple(args.place)
    waypoints = build_task(cfg)

    print(f"pick  {cfg.pick}\nplace {cfg.place}\n"
          f"{args.joints} actuated joints, {CONTROL_HZ:.0f} Hz setpoint stream\n")

    link = None
    if args.port:
        link = jl.ArmLink(args.port, args.joints)
        print(f"waiting for state from {args.port}...")
        if not link.wait_for_state(3.0):
            raise SystemExit("no state frames - is app_08_host_link running?")
        if not link.all_healthy():
            faults = ", ".join(jl.fault_str(s.fault) for s in link.state)
            print(f"faults present ({faults}); clearing")
            link.clear_faults()
            time.sleep(0.2)
            link.poll()
            if not link.all_healthy():
                raise SystemExit("faults did not clear; fix the hardware first")
        backend = LinkBackend(link, args.joints)
    else:
        # A plausible parked pose in the same configuration family the task
        # uses. Starting from all zeros means the arm is fully extended, which
        # is both the worst place for the Jacobian and a different IK branch
        # from everything the task needs.
        start = [math.degrees(a) / jl.DEG_PER_COUNT
                 for a in (0.0, -1.90, -1.70)]
        backend = DryRunBackend(args.joints, start[:args.joints])
        print("DRY RUN: perfect tracking, no hardware, no realtime pacing\n")

    runner = Runner(backend, actuated=args.joints, realtime=bool(args.port),
                    verbose=args.verbose)
    try:
        for c in range(args.cycles):
            if args.cycles > 1:
                print(f"--- cycle {c + 1}/{args.cycles} ---")
            runner.run(waypoints)
    except (TaskFailed, KeyboardInterrupt) as exc:
        backend.stop()
        raise SystemExit(f"\nSTOPPED: {exc}")
    finally:
        if link is not None:
            link.close()


if __name__ == "__main__":
    main()
