"""Webots uzerinde P kazanc taramasi: her kazanc icin ayni profil ve sure."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

WEBOTS = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
WORLD = ROOT / "webots" / "worlds" / "webots_turret_fcs.wbt"
LOG_DIR = ROOT / "webots" / "logs"


def run_once(kp_pan, kp_tilt, profile, seconds, label):
    tuning = {
        "label": label,
        "kp_pan": kp_pan,
        "kp_tilt": kp_tilt,
        "max_output_deg": 1.50,
    }
    tmp = Path(tempfile.gettempdir()) / f"turret_tuning_{label}.json"
    tmp.write_text(json.dumps(tuning), encoding="utf-8")

    before = set(LOG_DIR.glob("webots_telemetry_*.csv"))
    env = dict(os.environ)
    env["WEBOTS_BALLOON_PROFILE"] = str(profile)
    env["WEBOTS_TUNING_JSON"] = str(tmp)

    proc = subprocess.Popen(
        [WEBOTS, "--batch", "--mode=fast", "--stdout", "--stderr", str(WORLD)],
        env=env,
        cwd=str(ROOT),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        proc.wait(timeout=seconds)
    except subprocess.TimeoutExpired:
        subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"], capture_output=True)
    time.sleep(4)

    after = sorted(LOG_DIR.glob("webots_telemetry_*.csv"), key=lambda p: p.stat().st_mtime)
    new = [p for p in after if p not in before]
    return new[-1] if new else None


def summarize(path, limit=600):
    import csv

    with open(path, newline="", encoding="utf-8") as fh:
        rows = [r for r in csv.DictReader(fh) if int(float(r.get("target_detected") or 0)) == 1][:limit]
    if len(rows) < 100:
        return None
    fx = 1411.0
    pan = []
    cmd = []
    for row in rows:
        gx = float(row.get("gt_x") or 0.0)
        gy = float(row.get("gt_y") or 0.0)
        bearing = np.degrees(np.arctan2(gx, gy))
        pan.append(bearing - float(row.get("turret_pan_deg") or 0.0))
        cmd.append(float(row.get("cmd_pan_deg") or 0.0))
    pan = np.array(pan)
    cmd = np.array(cmd)
    tilt = np.array([abs(float(r.get("angular_err_tilt") or 0.0)) for r in rows])
    pixel = np.abs(pan) * np.pi / 180.0 * fx
    step = np.abs(np.diff(cmd))
    return {
        "pan_rms": float(np.sqrt(np.mean(pan**2))),
        "pan_p90": float(np.percentile(np.abs(pan), 90)),
        "pixel_mean": float(pixel.mean()),
        "lock30": float(np.mean(pixel < 30.0) * 100.0),
        "tilt_rms": float(np.sqrt(np.mean(tilt**2))),
        "cmd_step": float(step.mean()),
        "cmd_p95": float(np.percentile(step, 95)),
        "cmd_mean": float(np.abs(cmd).mean()),
        "locked": float(np.mean([int(float(r.get("is_locked") or 0)) for r in rows]) * 100.0),
    }


if __name__ == "__main__":
    profile = sys.argv[1] if len(sys.argv) > 1 else "2"
    seconds = int(sys.argv[2]) if len(sys.argv) > 2 else 90
    grids = sys.argv[3:] or ["0.20,0.12", "0.28,0.12", "0.35,0.12", "0.42,0.12"]
    print(f"profile={profile} seconds={seconds}")
    for grid in grids:
        kp_pan, kp_tilt = (float(x) for x in grid.split(","))
        label = f"scan_{kp_pan:.2f}_{kp_tilt:.2f}".replace(".", "p")
        path = run_once(kp_pan, kp_tilt, profile, seconds, label)
        if path is None:
            print(f"{grid}: log yok")
            continue
        stats = summarize(path)
        if stats is None:
            print(f"{grid}: yetersiz veri ({path.name})")
            continue
        print(
            f"{grid}: pan_rms={stats['pan_rms']:.3f} p90={stats['pan_p90']:.3f} "
            f"pixel={stats['pixel_mean']:.1f} lock30={stats['lock30']:.1f}% "
            f"tilt_rms={stats['tilt_rms']:.2f} step={stats['cmd_step']:.3f} "
            f"step95={stats['cmd_p95']:.3f} cmd={stats['cmd_mean']:.3f} "
            f"locked={stats['locked']:.1f}% [{path.name}]"
        )
