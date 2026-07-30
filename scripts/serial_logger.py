#!/usr/bin/env python3
"""Capture a CSV telemetry stream from the arm firmware to docs/test-results/.

The firmware prints comment lines starting with '#' and CSV data lines. This
saves the CSV to a timestamped file, echoes everything so you can watch the run,
and keeps the comment lines in a sidecar .log for context.

    python scripts/serial_logger.py --port COM3 --name phase1
    python scripts/serial_logger.py --list

Ctrl+C to stop.
"""
from __future__ import annotations

import argparse
import datetime as dt
import sys
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial missing. Run: pip install -r scripts/requirements.txt")

RESULTS_DIR = Path(__file__).resolve().parent.parent / "docs" / "test-results"


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        print(f"{p.device}\t{p.description}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="e.g. COM3 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--name", default="run", help="log file name prefix")
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl+C")
    ap.add_argument("--list", action="store_true", help="list ports and exit")
    args = ap.parse_args()

    if args.list:
        list_serial_ports()
        return 0
    if not args.port:
        ap.error("--port is required (use --list to find it)")

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    csv_path = RESULTS_DIR / f"{args.name}_{stamp}.csv"
    log_path = csv_path.with_suffix(".log")

    print(f"port   : {args.port} @ {args.baud}")
    print(f"csv    : {csv_path}")
    print(f"notes  : {log_path}")
    print("-" * 60)

    start = dt.datetime.now()
    rows = 0
    with serial.Serial(args.port, args.baud, timeout=1) as ser, \
            csv_path.open("w", newline="", encoding="utf-8") as csv_f, \
            log_path.open("w", encoding="utf-8") as log_f:
        try:
            while True:
                if args.seconds and (dt.datetime.now() - start).total_seconds() > args.seconds:
                    break
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                print(line)
                if line.startswith("#") or line.startswith("==="):
                    log_f.write(line + "\n")
                    log_f.flush()
                else:
                    csv_f.write(line + "\n")
                    rows += 1
                    if rows % 50 == 0:
                        csv_f.flush()
        except KeyboardInterrupt:
            print("\nstopped by user")

    print(f"\n{rows} CSV lines -> {csv_path}")
    print(f"Next: python scripts/analyze_log.py \"{csv_path}\"")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
