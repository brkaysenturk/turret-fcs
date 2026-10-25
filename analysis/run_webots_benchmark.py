#!/usr/bin/env python3
"""
================================================================================
MODUL: WEBOTS DIJITAL IKIZ BASARIM KARSILASTIRMA TESTI (P vs RL BENCHMARK)
================================================================================
Bu betik; yeni optimize edilmis sistem parametreleriyle (backlash=1.61 deg,
ego_tau=0.0682 s, Kalman sigma_a=478.19, RL sarmalayici taper/damping)
Webots simulasyonunu Figure-8 profilinde 20 saniye kosturur ve sonuclari
karsilastirir.
================================================================================
"""

import csv
import os
import subprocess
import sys
import time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

WEBOTS = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
WORLD = ROOT / "webots" / "worlds" / "webots_turret_fcs.wbt"
LOG_DIR = ROOT / "webots" / "logs"


def run_simulation(control_mode="P", profile=7, duration_sec=25):
    print(f"\n[*] Webots baslatiliyor: Mod={control_mode}, Profil={profile}, Sure={duration_sec}s...")
    before = set(LOG_DIR.glob("webots_telemetry_*.csv"))

    env = dict(os.environ)
    env["WEBOTS_CONTROL_MODE"] = str(control_mode)
    env["WEBOTS_BALLOON_PROFILE"] = str(profile)

    proc = subprocess.Popen(
        [WEBOTS, "--batch", "--mode=fast", "--stdout", "--stderr", str(WORLD)],
        env=env,
        cwd=str(ROOT),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    try:
        proc.wait(timeout=duration_sec)
    except subprocess.TimeoutExpired:
        subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"], capture_output=True)

    time.sleep(3)

    new_logs = [p for p in LOG_DIR.glob("webots_telemetry_*.csv") if p not in before]
    if not new_logs:
        print("[!] Yeni telemetri logu bulunamadi.")
        return None

    latest_log = sorted(new_logs, key=lambda p: p.stat().st_mtime)[-1]
    print(f"[+] Kosu tamamlandi: {latest_log.name}")
    return latest_log


def analyze_log(csv_path: Path):
    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    det_rows = [r for r in rows if r.get("target_detected") in ("1", "True", "1.0")]
    if len(det_rows) < 30:
        return None

    # Error columns in webots telemetry
    # Let's inspect headers if possible
    headers = list(det_rows[0].keys())

    # Pixel errors or angular errors
    if "angular_err_pan" in headers and "angular_err_tilt" in headers:
        err_p = np.array([float(r["angular_err_pan"]) for r in det_rows])
        err_t = np.array([float(r["angular_err_tilt"]) for r in det_rows])
    elif "last_pan_err_deg" in headers and "last_tilt_err_deg" in headers:
        err_p = np.array([float(r["last_pan_err_deg"]) for r in det_rows])
        err_t = np.array([float(r["last_tilt_err_deg"]) for r in det_rows])
    else:
        # Ground truth pan / tilt
        err_p = np.array([float(r.get("err_pan_deg") or 0.0) for r in det_rows])
        err_t = np.array([float(r.get("err_tilt_deg") or 0.0) for r in det_rows])

    cmd_p = np.array([float(r.get("cmd_pan_deg") or r.get("cmd_pan") or 0.0) for r in det_rows])

    # Lock ratio
    is_locked = np.array([1 if r.get("is_locked") in ("1", "True", "1.0") else 0 for r in det_rows])
    lock_pct = float(np.mean(is_locked) * 100.0)

    # Reversal flip rate
    diff_cmd = np.diff(cmd_p)
    moves = diff_cmd[np.abs(diff_cmd) > 1e-4]
    flip_rate = float(np.mean(np.sign(moves[1:]) != np.sign(moves[:-1])) * 100.0) if len(moves) > 2 else 0.0

    # Jerk
    jerk = float(np.mean(np.abs(diff_cmd))) if len(diff_cmd) > 0 else 0.0

    rms_p = float(np.sqrt(np.mean(err_p ** 2)))
    rms_t = float(np.sqrt(np.mean(err_t ** 2)))
    rms_total = float(np.sqrt(np.mean(err_p ** 2 + err_t ** 2)))

    return {
        "frames": len(det_rows),
        "rms_pan_deg": rms_p,
        "rms_tilt_deg": rms_t,
        "rms_total_deg": rms_total,
        "lock_pct": lock_pct,
        "flip_rate": flip_rate,
        "jerk": jerk,
    }


def main():
    print("=" * 70)
    print("  TURRET FCS // WEBOTS DIGITAL TWIN BENCHMARK (P & RL)")
    print("=" * 70)

    # 1. P Kontrolcu Testi (Figure-8)
    p_log = run_simulation(control_mode="P", profile=7, duration_sec=25)
    p_metrics = analyze_log(p_log) if p_log else None

    # 2. RL Kontrolcu Testi (Figure-8)
    rl_log = run_simulation(control_mode="RL", profile=7, duration_sec=25)
    rl_metrics = analyze_log(rl_log) if rl_log else None

    print("\n" + "=" * 70)
    print("  WEBOTS TEST SONUCLARI KARSILASTIRMASI (FIGURE-8 MANEVRASI)")
    print("=" * 70)
    print(f"{'Metrik':<30} | {'P Kontrolcu':<18} | {'RL Kontrolcu':<18}")
    print("-" * 70)

    if p_metrics and rl_metrics:
        print(f"{'Analiz Edilen Kare':<30} | {p_metrics['frames']:<18} | {rl_metrics['frames']:<18}")
        print(f"{'Pan RMS Hatasi (derece)':<30} | {p_metrics['rms_pan_deg']:<18.2f} | {rl_metrics['rms_pan_deg']:<18.2f}")
        print(f"{'Tilt RMS Hatasi (derece)':<30} | {p_metrics['rms_tilt_deg']:<18.2f} | {rl_metrics['rms_tilt_deg']:<18.2f}")
        print(f"{'Bileske RMS (derece)':<30} | {p_metrics['rms_total_deg']:<18.2f} | {rl_metrics['rms_total_deg']:<18.2f}")
        print(f"{'Hedef Kilit Orani (%)':<30} | %{p_metrics['lock_pct']:<17.1f} | %{rl_metrics['lock_pct']:<17.1f}")
        print(f"{'Komut Yon Degistirme (%)':<30} | %{p_metrics['flip_rate']:<17.2f} | %{rl_metrics['flip_rate']:<17.2f}")
        print(f"{'Komut Jerk':<30} | {p_metrics['jerk']:<18.4f} | {rl_metrics['jerk']:<18.4f}")
    else:
        print(f"P Metrics : {p_metrics}")
        print(f"RL Metrics: {rl_metrics}")
    print("=" * 70)


if __name__ == "__main__":
    main()
