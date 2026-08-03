"""Forward and inverse kinematics for the UR-style 6R arm.

Everything here is cross-checked against an independent method, because the
failure mode of hand-derived kinematics is a sign error that produces plausible
numbers. Specifically:

  * FK by DH is checked against FK by product-of-exponentials
  * The geometric Jacobian is checked against numerical differentiation of FK
  * IK is checked by round-trip: FK(IK(pose)) must reproduce pose

Run `python tools/kinematics.py` to execute all three checks.

Angles are radians internally. Poses are 4x4 homogeneous transforms.
"""

from __future__ import annotations

import math

import numpy as np

from arm_model import DH, JOINT_LIMITS_DEG, DHParams

TAU = 2 * math.pi


# --------------------------------------------------------------- FK by DH --
def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def fk(q, dh: DHParams = DH) -> np.ndarray:
    """Base-to-flange transform for joint angles q (6 radians)."""
    t = np.eye(4)
    for theta, (d, a, alpha) in zip(q, dh.rows()):
        t = t @ dh_transform(theta, d, a, alpha)
    return t


def fk_all(q, dh: DHParams = DH) -> list[np.ndarray]:
    """Cumulative transforms T0_1 .. T0_6, needed for the Jacobian."""
    out, t = [], np.eye(4)
    for theta, (d, a, alpha) in zip(q, dh.rows()):
        t = t @ dh_transform(theta, d, a, alpha)
        out.append(t.copy())
    return out


# ------------------------------------------------- FK by product of exponentials --
def _skew(v) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _screw_exp(screw, theta: float) -> np.ndarray:
    """Matrix exponential of a normalised screw axis [w; v] times theta."""
    w, v = np.asarray(screw[:3], float), np.asarray(screw[3:], float)
    t = np.eye(4)
    if np.linalg.norm(w) < 1e-12:
        t[:3, 3] = v * theta
        return t
    wx = _skew(w)
    r = np.eye(3) + math.sin(theta) * wx + (1 - math.cos(theta)) * (wx @ wx)
    g = (np.eye(3) * theta + (1 - math.cos(theta)) * wx
         + (theta - math.sin(theta)) * (wx @ wx))
    t[:3, :3] = r
    t[:3, 3] = g @ v
    return t


def _poe_model(dh: DHParams = DH):
    """Derive space-frame screw axes and home pose from the DH table.

    Deriving these *from* FK would make the cross-check circular, so instead
    each axis is read off the zero-configuration geometry: the joint axis
    direction and a point on it, both taken from fk_all at q = 0.
    """
    zero = [0.0] * 6
    frames = [np.eye(4)] + fk_all(zero, dh)
    screws = []
    for i in range(6):
        axis = frames[i][:3, 2]          # joint i rotates about z of frame i-1
        point = frames[i][:3, 3]
        screws.append(np.concatenate([axis, -np.cross(axis, point)]))
    return screws, frames[-1]


def fk_poe(q, dh: DHParams = DH) -> np.ndarray:
    """Independent FK via product of exponentials, for cross-checking fk()."""
    screws, home = _poe_model(dh)
    t = np.eye(4)
    for screw, theta in zip(screws, q):
        t = t @ _screw_exp(screw, theta)
    return t @ home


# ------------------------------------------------------------- Jacobian --
def jacobian(q, dh: DHParams = DH) -> np.ndarray:
    """Geometric Jacobian in the base frame: 6x6, [linear; angular]."""
    frames = [np.eye(4)] + fk_all(q, dh)
    p_end = frames[-1][:3, 3]
    j = np.zeros((6, 6))
    for i in range(6):
        z = frames[i][:3, 2]
        p = frames[i][:3, 3]
        j[:3, i] = np.cross(z, p_end - p)
        j[3:, i] = z
    return j


def jacobian_numeric(q, dh: DHParams = DH, eps: float = 1e-7) -> np.ndarray:
    """Finite-difference Jacobian. Only used to validate jacobian()."""
    q = np.asarray(q, float)
    t0 = fk(q, dh)
    j = np.zeros((6, 6))
    for i in range(6):
        dq = q.copy()
        dq[i] += eps
        t1 = fk(dq, dh)
        j[:3, i] = (t1[:3, 3] - t0[:3, 3]) / eps
        dr = (t1[:3, :3] - t0[:3, :3]) / eps @ t0[:3, :3].T
        j[3:, i] = [dr[2, 1], dr[0, 2], dr[1, 0]]
    return j


def manipulability(q, dh: DHParams = DH) -> float:
    """Yoshikawa measure. Approaches zero at a singularity."""
    return float(math.sqrt(abs(np.linalg.det(jacobian(q, dh) @ jacobian(q, dh).T))))


# ------------------------------------------------------------------- IK --
def _clamp(x: float) -> float:
    return max(-1.0, min(1.0, x))


def ik(target: np.ndarray, dh: DHParams = DH) -> list[np.ndarray]:
    """All closed-form solutions for a target flange pose.

    Returns up to 8 branches: shoulder left/right x elbow up/down x wrist
    flip/no-flip. Unreachable or degenerate branches are dropped, so the list
    may be shorter - callers must handle an empty result.
    """
    d1, a2, a3 = dh.d1, dh.a2, dh.a3
    d4, d5, d6 = dh.d4, dh.d5, dh.d6
    t06 = np.asarray(target, float)
    px, py, pz = t06[:3, 3]
    solutions = []

    # --- theta1: from the wrist-2 origin projected on the base plane -------
    p05 = t06 @ np.array([0.0, 0.0, -d6, 1.0])
    r = math.hypot(p05[0], p05[1])
    if r < abs(d4) - 1e-12:
        return []
    phi = math.atan2(p05[1], p05[0])
    psi = math.acos(_clamp(d4 / r))

    for s1 in (1, -1):
        t1 = phi + s1 * psi + math.pi / 2
        c1, sn1 = math.cos(t1), math.sin(t1)

        # --- theta5: how far the tool axis leans out of the shoulder plane --
        arg = (px * sn1 - py * c1 - d4) / d6
        if abs(arg) > 1.0 + 1e-9:
            continue
        for s5 in (1, -1):
            t5 = s5 * math.acos(_clamp(arg))
            if abs(math.sin(t5)) < 1e-9:
                continue  # wrist singularity: theta4 and theta6 are coupled

            # --- theta6: orientation about the tool axis -------------------
            t6 = math.atan2(
                (-t06[0, 1] * sn1 + t06[1, 1] * c1) / math.sin(t5),
                (t06[0, 0] * sn1 - t06[1, 0] * c1) / math.sin(t5),
            )

            # --- reduce to a planar 2R problem for theta2/theta3 -----------
            t01 = dh_transform(t1, d1, 0.0, math.pi / 2)
            t45 = dh_transform(t5, d5, 0.0, -math.pi / 2)
            t56 = dh_transform(t6, d6, 0.0, 0.0)
            t14 = np.linalg.inv(t01) @ t06 @ np.linalg.inv(t45 @ t56)
            p13 = t14 @ np.array([0.0, -d4, 0.0, 1.0])
            planar = math.hypot(p13[0], p13[1])

            cos3 = (planar ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
            if abs(cos3) > 1.0 + 1e-9:
                continue
            for s3 in (1, -1):
                t3 = s3 * math.acos(_clamp(cos3))
                # atan2 form, not the asin form: a2 and a3 are negative in the
                # UR convention and asin silently gets the quadrant wrong.
                t2 = (math.atan2(p13[1], p13[0])
                      - math.atan2(a3 * math.sin(t3), a2 + a3 * math.cos(t3)))
                t12 = dh_transform(t2, 0.0, a2, 0.0)
                t23 = dh_transform(t3, 0.0, a3, 0.0)
                t34 = np.linalg.inv(t12 @ t23) @ t14
                t4 = math.atan2(t34[1, 0], t34[0, 0])
                solutions.append(np.array([t1, t2, t3, t4, t5, t6]))

    return [wrap(s) for s in solutions]


def wrap(q) -> np.ndarray:
    """Fold angles into (-pi, pi]."""
    return np.array([(a + math.pi) % TAU - math.pi for a in q])


def within_limits(q) -> bool:
    return all(lo <= math.degrees(a) <= hi
               for a, (lo, hi) in zip(q, JOINT_LIMITS_DEG))


def best_solution(target: np.ndarray, q_now, dh: DHParams = DH):
    """The reachable branch closest to the current configuration.

    Picking the nearest branch is what stops the arm from flipping through a
    different elbow configuration between two nearby waypoints.
    """
    feasible = [s for s in ik(target, dh) if within_limits(s)]
    if not feasible:
        return None
    return min(feasible, key=lambda s: np.linalg.norm(wrap(s - np.asarray(q_now))))


# ------------------------------------------------------------ self-check --
def _pose_error(a: np.ndarray, b: np.ndarray) -> float:
    pos = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    rot = float(np.linalg.norm(a[:3, :3] - b[:3, :3]))
    return max(pos, rot)


def main() -> None:
    rng = np.random.default_rng(0)
    print(f"reach (horizontal, extended): {DH.horizontal_reach*1000:.0f} mm\n")

    worst = 0.0
    for _ in range(500):
        q = rng.uniform(-math.pi, math.pi, 6)
        worst = max(worst, _pose_error(fk(q), fk_poe(q)))
    print(f"[1] FK by DH vs FK by product-of-exponentials : {worst:.2e}")

    worst = 0.0
    for _ in range(200):
        q = rng.uniform(-math.pi, math.pi, 6)
        worst = max(worst, float(np.max(np.abs(jacobian(q) - jacobian_numeric(q)))))
    print(f"[2] analytic Jacobian vs finite difference    : {worst:.2e}")

    tested = solved = 0
    worst = 0.0
    for _ in range(500):
        q = np.array([rng.uniform(math.radians(lo), math.radians(hi))
                      for lo, hi in JOINT_LIMITS_DEG])
        target = fk(q)
        sols = ik(target)
        if not sols:
            continue
        tested += 1
        errs = [_pose_error(fk(s), target) for s in sols]
        if min(errs) < 1e-6:
            solved += 1
        worst = max(worst, min(errs))
    print(f"[3] IK round-trip FK(IK(pose)) == pose        : "
          f"{solved}/{tested} exact, worst {worst:.2e}")

    q_sing = np.zeros(6)
    print(f"\nmanipulability at q=0            : {manipulability(q_sing):.4e}")
    print(f"manipulability at a folded pose  : "
          f"{manipulability([0, -0.9, 1.4, -0.5, 1.2, 0]):.4e}")
    print(f"branches for a typical pose      : "
          f"{len(ik(fk([0.3, -0.8, 1.1, -0.4, 1.0, 0.2])))}")


if __name__ == "__main__":
    main()
