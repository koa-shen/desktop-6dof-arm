#!/usr/bin/env python3
"""Plot and characterize a telemetry log from the arm firmware.

    python scripts/analyze_log.py docs/test-results/phase1_20260801-101500.csv

Handles both log formats:
  phase1_bench : ms,cycle,cmd_steps,cmd_deg,enc_deg,err_deg,i2c_err
  closed_loop  : ms,target_deg,enc_deg,err_deg,cmd_vel,steps

For closed-loop logs it also extracts step-response metrics (rise time,
overshoot, settling time, steady-state error) for each setpoint change. Those
four numbers are the language controls engineers speak in - learn to quote them.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd
except ImportError:
    sys.exit("Missing deps. Run: pip install -r scripts/requirements.txt")


def load(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, comment="#")
    df.columns = [c.strip() for c in df.columns]
    if "ms" not in df.columns:
        raise SystemExit(f"{path}: no 'ms' column - is this a firmware CSV?")
    df["t"] = (df["ms"] - df["ms"].iloc[0]) / 1000.0
    return df


def step_metrics(t: np.ndarray, y: np.ndarray, y0: float, target: float,
                 tol_frac: float = 0.02) -> dict:
    """Classic second-order step-response descriptors."""
    span = target - y0
    if abs(span) < 1e-6:
        return {}
    band = abs(span) * tol_frac

    def crossing(frac: float) -> float | None:
        level = y0 + frac * span
        hit = np.where((y - level) * np.sign(span) >= 0)[0]
        return float(t[hit[0]]) if hit.size else None

    t10, t90 = crossing(0.10), crossing(0.90)
    peak = float(y.max() if span > 0 else y.min())
    overshoot = 100.0 * (peak - target) / span

    settle = None
    outside = np.where(np.abs(y - target) > band)[0]
    if outside.size and outside[-1] + 1 < len(t):
        settle = float(t[outside[-1] + 1])
    elif not outside.size:
        settle = float(t[0])

    return {
        "target_deg": target,
        "rise_10_90_s": (t90 - t10) if (t10 is not None and t90 is not None) else None,
        "overshoot_pct": overshoot,
        "settling_2pct_s": settle,
        "steady_state_err_deg": float(target - np.mean(y[-max(5, len(y) // 10):])),
    }


def analyze_closed_loop(df: pd.DataFrame) -> None:
    print("\n== step response ==")
    changes = df.index[df["target_deg"].diff().abs() > 1e-6].tolist()
    if not changes:
        print("  no setpoint changes found")
        return
    bounds = changes + [len(df)]
    for i in range(len(bounds) - 1):
        seg = df.iloc[bounds[i]:bounds[i + 1]]
        if len(seg) < 10:
            continue
        y0 = float(df["enc_deg"].iloc[max(0, bounds[i] - 1)])
        m = step_metrics(seg["t"].to_numpy() - seg["t"].iloc[0],
                         seg["enc_deg"].to_numpy(), y0,
                         float(seg["target_deg"].iloc[0]))
        if not m:
            continue
        print(f"  step -> {m['target_deg']:7.2f} deg | "
              f"rise {fmt(m['rise_10_90_s'])} s | "
              f"overshoot {m['overshoot_pct']:6.1f} % | "
              f"settle {fmt(m['settling_2pct_s'])} s | "
              f"ss err {m['steady_state_err_deg']:6.3f} deg")


def fmt(v) -> str:
    return "  n/a" if v is None else f"{v:5.3f}"


def analyze_tracking(df: pd.DataFrame) -> None:
    err = df["err_deg"].to_numpy()
    print("\n== tracking error ==")
    print(f"  samples      : {len(err)}")
    print(f"  mean         : {err.mean():.3f} deg")
    print(f"  std dev      : {err.std():.3f} deg")
    print(f"  max abs      : {np.abs(err).max():.3f} deg")
    print(f"  RMS          : {np.sqrt((err ** 2).mean()):.3f} deg")
    if "i2c_err" in df.columns:
        print(f"  i2c errors   : {int(df['i2c_err'].iloc[-1])}")


def analyze_repeatability(df: pd.DataFrame) -> None:
    if "cycle" not in df.columns:
        return
    homes = []
    for c, seg in df.groupby("cycle"):
        at_home = seg[seg["cmd_steps"].abs() < 1]
        if len(at_home) > 3:
            homes.append(float(at_home["enc_deg"].iloc[-1]))
    if len(homes) < 2:
        return
    h = np.array(homes)
    print("\n== return-to-home repeatability ==")
    print(f"  cycles       : {len(h)}")
    print(f"  mean         : {h.mean():.3f} deg")
    print(f"  std dev      : {h.std():.3f} deg   <- repeatability")
    print(f"  spread       : {h.max() - h.min():.3f} deg")


def plot(df: pd.DataFrame, path: Path, show: bool) -> None:
    cmd_col = "cmd_deg" if "cmd_deg" in df.columns else "target_deg"
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 7))

    ax[0].plot(df["t"], df[cmd_col], label="commanded", linewidth=1.2)
    ax[0].plot(df["t"], df["enc_deg"], label="encoder", linewidth=1.2)
    ax[0].set_ylabel("angle [deg]")
    ax[0].set_title(path.name)
    ax[0].legend()
    ax[0].grid(alpha=0.3)

    ax[1].plot(df["t"], df["err_deg"], color="tab:red", linewidth=1.0)
    ax[1].axhline(0, color="k", linewidth=0.6)
    ax[1].set_ylabel("error [deg]")
    ax[1].set_xlabel("time [s]")
    ax[1].grid(alpha=0.3)

    fig.tight_layout()
    out = path.with_suffix(".png")
    fig.savefig(out, dpi=130)
    print(f"\nplot -> {out}")
    if show:
        plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--no-show", action="store_true", help="save plot only")
    args = ap.parse_args()

    df = load(args.csv)
    print(f"loaded {len(df)} rows, {df['t'].iloc[-1]:.2f} s, "
          f"{len(df) / max(df['t'].iloc[-1], 1e-9):.1f} Hz")

    if "err_deg" in df.columns:
        analyze_tracking(df)
    if "target_deg" in df.columns:
        analyze_closed_loop(df)
    analyze_repeatability(df)
    plot(df, args.csv, not args.no_show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
