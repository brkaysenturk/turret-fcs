"""Webots telemetrisinden P kontrolcu performans ozeti.

Gercek hedef aci hatasi, balonun gt_x/gt_y konumundan ve taretin gercek
pan acisindan hesaplanir. Boylece kontrolcunun kendi filtrelenmis hatasina
degil, fiziksel nisan hatasina bakilir.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from turret_fcs.config import CONFIG

FX = CONFIG.optics.compute_fx(CONFIG.optics.default_width)


def load(path):
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def f(row, key, default=0.0):
    try:
        return float(row.get(key, default) or default)
    except (TypeError, ValueError):
        return float(default)


def evaluate(path, limit=None):
    rows = load(path)
    det = [r for r in rows if int(f(r, "target_detected")) == 1 and f(r, "box_cx") > 0]
    if limit is not None:
        det = det[:limit]
    if len(det) < 30:
        print(f"{path.name}: yetersiz tespit ({len(det)})")
        return

    true_pan = []
    cmd_pan = []
    for row in det:
        gx, gy = f(row, "gt_x"), f(row, "gt_y")
        bearing = math.degrees(math.atan2(gx, gy))
        camera = f(row, "turret_pan_deg")
        true_pan.append(bearing - camera)
        cmd_pan.append(f(row, "cmd_pan_deg"))

    pan = np.array(true_pan)
    cmd = np.array(cmd_pan)
    err_px = np.abs(pan) * math.pi / 180.0 * FX
    jerk = np.abs(np.diff(cmd))

    print(f"\n{path.name} (first {len(det)} frames)")
    print(
        f"  pan true error: mean={np.mean(np.abs(pan)):.3f} deg "
        f"rms={np.sqrt(np.mean(pan**2)):.3f} p90={np.percentile(np.abs(pan),90):.3f} "
        f"max={np.max(np.abs(pan)):.2f}"
    )
    print(
        f"  pixel error: mean={np.mean(err_px):.1f}px rms={np.sqrt(np.mean(err_px**2)):.1f} "
        f"p90={np.percentile(err_px,90):.1f} lock<30px={np.mean(err_px<30)*100:.1f}%"
    )
    print(
        f"  command: |step| mean={np.mean(np.abs(jerk)):.3f} p95={np.percentile(np.abs(jerk),95):.3f} "
        f"|pan cmd| mean={np.mean(np.abs(cmd)):.3f} "
        f"locked={np.mean([int(f(r,'is_locked')) for r in det])*100:.1f}%"
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="*")
    parser.add_argument("--limit", type=int, default=600)
    args = parser.parse_args()
    paths = [Path(arg) for arg in args.paths] if args.paths else sorted(
        (ROOT / "webots" / "logs").glob("webots_telemetry_*.csv"))[-3:]
    for path in paths:
        evaluate(path, limit=args.limit)
