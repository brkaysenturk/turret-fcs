"""Kazanc x profil matrisinde Webots P taramasi ve kararlilik analizi.

Her (profil, Kp) cifti icin ayni sureli kosu yapilir; log dosyalari etiketlenir
ve limit cycle gostergeleri (komut yon degistirme, doyum, gercek hata) raporlanir.
"""

from __future__ import annotations

import csv
import json
import os
import subprocess
import tempfile
import time
from pathlib import Path

import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from turret_fcs.config import CONFIG

WEBOTS = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
WORLD = ROOT / "webots" / "worlds" / "webots_turret_fcs.wbt"
LOG_DIR = ROOT / "webots" / "logs"
RESULTS = ROOT / "webots" / "logs" / "p_sweep_results.csv"
FX = CONFIG.optics.compute_fx(CONFIG.optics.default_width)

PROFILES = {1: "sine_slow", 2: "sine_std", 3: "sine_fast", 4: "step_hold", 5: "zigzag", 7: "figure8"}
GAINS = [0.20, 0.24, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38]


def run_one(profile, kp_pan, kp_tilt, seconds, label):
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

    new = [p for p in LOG_DIR.glob("webots_telemetry_*.csv") if p not in before]
    return sorted(new, key=lambda p: p.stat().st_mtime)[-1] if new else None


def analyze(path):
    with open(path, newline="", encoding="utf-8") as fh:
        rows = [r for r in csv.DictReader(fh) if int(float(r.get("target_detected") or 0)) == 1]
    if len(rows) < 150:
        return None
    pan = []
    for row in rows:
        gx = float(row.get("gt_x") or 0.0)
        gy = float(row.get("gt_y") or 0.0)
        pan.append(np.degrees(np.arctan2(gx, gy)) - float(row.get("turret_pan_deg") or 0.0))
    pan = np.array(pan)
    cmd = np.array([float(row.get("cmd_pan_deg") or 0.0) for row in rows])
    diff = np.diff(cmd)
    moves = diff[np.abs(diff) > 1e-6]
    rev = np.mean(np.sign(moves[1:]) != np.sign(moves[:-1])) * 100.0 if moves.size > 2 else 0.0
    pixel = np.abs(pan) * np.pi / 180.0 * FX
    return {
        "pan_rms": float(np.sqrt(np.mean(pan**2))),
        "pixel_mean": float(pixel.mean()),
        "lock30": float(np.mean(pixel < 30.0) * 100.0),
        "rev": float(rev),
        "sat": float(np.mean(np.abs(diff) > 1.35) * 100.0),
        "step95": float(np.percentile(np.abs(diff), 95)),
        "locked": float(np.mean([int(float(r.get("is_locked") or 0)) for r in rows]) * 100.0),
        "frames": len(rows),
    }


def append_result(row):
    header = ["profile", "profile_name", "kp_pan", "kp_tilt", "pan_rms", "pixel_mean",
              "lock30", "rev_pct", "sat_pct", "step95", "locked_pct", "frames", "log"]
    new_file = not RESULTS.exists()
    with open(RESULTS, "a", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        if new_file:
            writer.writerow(header)
        writer.writerow(row)


if __name__ == "__main__":
    import sys

    seconds = int(sys.argv[1]) if len(sys.argv) > 1 else 75
    profiles = [int(x) for x in sys.argv[2].split(",")] if len(sys.argv) > 2 else list(PROFILES)
    gains = [float(x) for x in sys.argv[3].split(",")] if len(sys.argv) > 3 else GAINS
    tilt = 0.13

    print(f"{'prof':>4} {'kp':>5} {'pan_rms':>8} {'pixel':>6} {'lock30':>7} {'rev%':>6} {'sat%':>6} {'step95':>7} {'locked':>7}")
    for profile in profiles:
        name = PROFILES.get(profile, str(profile))
        for kp in gains:
            label = f"psw_{profile}_{int(round(kp * 100)):03d}"
            path = run_one(profile, kp, tilt, seconds, label)
            stats = analyze(path) if path else None
            if stats is None:
                print(f"{profile:>4} {kp:5.2f} veri yok")
                continue
            print(
                f"{profile:>4} {kp:5.2f} {stats['pan_rms']:8.3f} {stats['pixel_mean']:6.1f} "
                f"{stats['lock30']:6.1f}% {stats['rev']:5.1f} {stats['sat']:5.1f} "
                f"{stats['step95']:7.3f} {stats['locked']:6.1f}%",
                flush=True,
            )
            append_result([profile, name, kp, tilt, f"{stats['pan_rms']:.4f}",
                           f"{stats['pixel_mean']:.2f}", f"{stats['lock30']:.1f}",
                           f"{stats['rev']:.2f}", f"{stats['sat']:.2f}",
                           f"{stats['step95']:.4f}", f"{stats['locked']:.1f}",
                           stats["frames"], path.name])
