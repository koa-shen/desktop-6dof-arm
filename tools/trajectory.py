"""Trajectory generation - the Python mirror of lib/ArmMath/Trajectory.h.

Layer 1 of D7 exists on both sides of the wire: the firmware version runs on
the Uno for `app_07_coordinated`, this one runs on the host for
`app_08_host_link`. They are deliberately the same algorithm, so a profile
verified here is the profile the firmware produces.

If you change one, change the other, and make test/test_motion agree.

Run directly for the self-checks:
    python trajectory.py
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass


@dataclass
class MotionSample:
    pos: float
    vel: float
    acc: float


class TrapezoidProfile:
    """Symmetric trapezoidal velocity profile, stretchable to a fixed duration.

    Stretching keeps acceleration at the limit and lowers the cruise velocity,
    rather than scaling both. That distinction is what makes synchronization
    safe: a slowed axis never needs *more* acceleration than it was allowed.
    """

    def __init__(self) -> None:
        self._start = 0.0
        self._goal = 0.0
        self._dist = 0.0
        self._sign = 1.0
        self._accel = 1.0
        self._cruise = 0.0
        self._t_accel = 0.0
        self._t_flat = 0.0
        self._duration = 0.0

    @staticmethod
    def min_duration(distance: float, vmax: float, amax: float) -> float:
        d = abs(distance)
        if d <= 0.0 or vmax <= 0.0 or amax <= 0.0:
            return 0.0
        if d * amax <= vmax * vmax:
            return 2.0 * math.sqrt(d / amax)  # triangular: never reaches vmax
        return vmax / amax + d / vmax

    def plan(self, start: float, goal: float, vmax: float, amax: float) -> None:
        self.plan_for(start, goal, vmax, amax,
                      self.min_duration(goal - start, vmax, amax))

    def plan_for(self, start: float, goal: float, vmax: float, amax: float,
                 duration: float) -> None:
        self._start = start
        self._goal = goal
        delta = goal - start
        self._dist = abs(delta)
        self._sign = -1.0 if delta < 0.0 else 1.0
        self._accel = amax if amax > 0.0 else 1.0

        if self._dist <= 0.0:
            self._cruise = 0.0
            self._duration = max(duration, 0.0)
            self._t_accel = 0.0
            self._t_flat = self._duration
            return

        t_min = self.min_duration(self._dist, vmax, self._accel)
        self._duration = duration if duration > t_min else t_min

        # T = v/a + d/v  ->  v^2 - aTv + ad = 0. The smaller root is the one
        # that respects the acceleration limit.
        a_t = self._accel * self._duration
        disc = max(a_t * a_t - 4.0 * self._accel * self._dist, 0.0)
        self._cruise = 0.5 * (a_t - math.sqrt(disc))
        self._cruise = min(self._cruise, vmax)
        if self._cruise <= 0.0:
            self._cruise = self._dist / self._duration

        self._t_accel = self._cruise / self._accel
        self._t_flat = self._duration - self._t_accel

    def sample(self, t: float) -> MotionSample:
        if self._dist <= 0.0 or t >= self._duration:
            return MotionSample(self._goal, 0.0, 0.0)
        if t <= 0.0:
            return MotionSample(self._start, 0.0, 0.0)

        if t < self._t_accel:
            p = 0.5 * self._accel * t * t
            v = self._accel * t
            a = self._accel
        elif t < self._t_flat:
            p = 0.5 * self._accel * self._t_accel ** 2 + self._cruise * (t - self._t_accel)
            v = self._cruise
            a = 0.0
        else:
            td = self._duration - t
            p = self._dist - 0.5 * self._accel * td * td
            v = self._accel * td
            a = -self._accel
        return MotionSample(self._start + self._sign * p, self._sign * v,
                            self._sign * a)

    @property
    def duration(self) -> float:
        return self._duration

    @property
    def cruise_velocity(self) -> float:
        return self._cruise

    def done(self, t: float) -> bool:
        return t >= self._duration


class BoxcarFilter:
    """Jerk limiter. Mirror of armmath::MovingAverage.

    Convolving the trapezoid with a window of N*dt turns its acceleration steps
    into ramps of jmax = amax / (N*dt) and lengthens the move by exactly N*dt.
    """

    def __init__(self, taps: int) -> None:
        self.taps = max(int(taps), 1)
        self._buf: deque[float] = deque([0.0] * self.taps, maxlen=self.taps)

    def reset(self, value: float = 0.0) -> None:
        self._buf = deque([value] * self.taps, maxlen=self.taps)

    def update(self, sample: float) -> float:
        self._buf.append(sample)
        return sum(self._buf) / self.taps

    @property
    def value(self) -> float:
        return sum(self._buf) / self.taps


class MotionController:
    """Layer 1: plan, synchronize, and stream setpoints for N joints.

    Mirror of lib/Motion/MotionController.h. Units are whatever you feed it;
    the host stack uses encoder counts so the numbers going onto the wire need
    no conversion.
    """

    def __init__(self, n: int, max_vel: float, max_acc: float,
                 taps: int = 4) -> None:
        self.n = n
        self.max_vel = [float(max_vel)] * n
        self.max_acc = [float(max_acc)] * n
        self._profiles = [TrapezoidProfile() for _ in range(n)]
        self._filters = [BoxcarFilter(taps) for _ in range(n)]
        self._last = [0.0] * n
        self._setpoint = [0.0] * n
        self._velocity = [0.0] * n
        self._t = 0.0
        self._duration = 0.0
        self._settle_left = 0
        self._taps = max(int(taps), 1)

    def reset_to(self, current: list[float]) -> None:
        for j in range(self.n):
            self._profiles[j].plan_for(current[j], current[j], self.max_vel[j],
                                       self.max_acc[j], 0.0)
            self._filters[j].reset(current[j])
            self._last[j] = float(current[j])
            self._setpoint[j] = float(current[j])
            self._velocity[j] = 0.0
        self._t = self._duration = 0.0
        self._settle_left = 0

    def move_to(self, target: list[float], current: list[float],
                speed_scale: float = 1.0) -> float:
        """Plan a synchronized move. Returns its duration in seconds."""
        scale = min(max(speed_scale, 1e-3), 1.0)
        longest = max(
            TrapezoidProfile.min_duration(target[j] - current[j],
                                          self.max_vel[j] * scale,
                                          self.max_acc[j] * scale)
            for j in range(self.n)
        )
        for j in range(self.n):
            self._profiles[j].plan_for(current[j], target[j],
                                       self.max_vel[j] * scale,
                                       self.max_acc[j] * scale, longest)
            self._filters[j].reset(current[j])
            self._last[j] = float(current[j])
        self._t = 0.0
        self._duration = longest
        self._settle_left = self._taps
        return longest

    def update(self, dt: float) -> None:
        self._t += dt
        if self._t > self._duration and self._settle_left:
            self._settle_left -= 1
        for j in range(self.n):
            p = self._filters[j].update(self._profiles[j].sample(self._t).pos)
            # The filter is linear, so differentiating its output IS the
            # filtered velocity. One filter, no phase mismatch between the
            # setpoint and the feedforward meant to produce it.
            self._velocity[j] = (p - self._last[j]) / dt if dt > 0 else 0.0
            self._last[j] = p
            self._setpoint[j] = p

    @property
    def setpoint(self) -> list[float]:
        return list(self._setpoint)

    @property
    def velocity(self) -> list[float]:
        return list(self._velocity)

    @property
    def moving(self) -> bool:
        return self._t < self._duration or self._settle_left > 0


def _self_check() -> None:
    ok = True

    def check(name: str, cond: bool) -> None:
        nonlocal ok
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        ok = ok and cond

    print("TrapezoidProfile")
    p = TrapezoidProfile()
    p.plan(0.0, 1.0, 100.0, 4.0)
    check("triangular duration = 2*sqrt(d/a)", abs(p.duration - 1.0) < 1e-6)
    check("apex at midpoint", abs(p.sample(0.5).pos - 0.5) < 1e-6)

    p.plan(0.0, 20.0, 2.0, 2.0)
    check("trapezoidal duration = v/a + d/v", abs(p.duration - 11.0) < 1e-6)
    check("cruises at vmax", abs(p.sample(5.0).vel - 2.0) < 1e-6)

    p.plan_for(0.0, 20.0, 2.0, 2.0, 44.0)
    check("stretched to exact duration", abs(p.duration - 44.0) < 1e-6)
    check("stretched still lands on goal", abs(p.sample(44.0).pos - 20.0) < 1e-6)
    check("stretched cruise is slower", p.cruise_velocity < 2.0)

    p.plan_for(0.0, 20.0, 2.0, 2.0, 0.001)
    check("impossible duration clamps up", abs(p.duration - 11.0) < 1e-6)

    # Numerically integrate the velocity and check it reproduces the position.
    # This is the check that catches an inconsistent profile, which is the bug
    # that makes feedforward fight the loop instead of helping it.
    p.plan(0.0, 20.0, 2.0, 2.0)
    dt = 1e-4
    pos = 0.0
    t = 0.0
    peak_a = 0.0
    while t < p.duration:
        s = p.sample(t)
        pos += s.vel * dt
        peak_a = max(peak_a, abs(s.acc))
        t += dt
    check("integral of vel == pos", abs(pos - 20.0) < 1e-2)
    check("accel never exceeds limit", peak_a <= 2.0 + 1e-9)

    print("MotionController")
    mc = MotionController(3, max_vel=2000.0, max_acc=4000.0, taps=4)
    start = [0.0, 0.0, 0.0]
    target = [100.0, 4000.0, -800.0]
    mc.reset_to(start)
    mc.move_to(target, start)
    dt = 0.005
    arrival = [None, None, None]
    step = 0
    peak_v = 0.0
    while mc.moving and step < 4000:
        mc.update(dt)
        for j in range(3):
            peak_v = max(peak_v, abs(mc.velocity[j]))
            if arrival[j] is None and abs(mc.setpoint[j] - target[j]) < 0.5:
                arrival[j] = step
        step += 1
    check("all axes arrive", all(a is not None for a in arrival))
    spread = max(arrival) - min(arrival)
    check(f"arrival spread {spread} cycles <= taps", spread <= 4)
    check("velocity respects limit", peak_v <= 2000.0 * 1.01)

    print("PASS" if ok else "FAIL")


if __name__ == "__main__":
    _self_check()
