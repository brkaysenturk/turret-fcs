#!/usr/bin/env python3
"""
================================================================================
MODUL: WEBOTS DISTURBANCE OBSERVER (DOB) 24-ADIM OPTUNA TPE OPTIMIZASYONU
================================================================================
Bu betik; hareketli hedef takip gecikmesini sifirlayan Disturbance Observer (DOB)
katsayilarini (dob_gain, dob_tau_s, max_dob_speed_deg_s) Webots uzerinde
Profil 2 (Sinus) ve Profil 5 (Zigzag) rotalarinda 24 adimlik Bayesyen Optuna (TPE)
ile optimize eder.
================================================================================
"""

from __future__ import annotations

import csv
import json
import os
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
import optuna

optuna.logging.set_verbosity(optuna.logging.WARNING)

ROOT = Path(__file__).resolve().parents[4]
WEBOTS = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
WORLD = ROOT / "webots" / "worlds" / "turret_fcs.wbt"
OUT_JSON = ROOT / "turret_fcs" / "analysis" / "controllers" / "p" / "dob_optuna_results.json"


def run_subscenario(params: dict, profile: int, trial_label: str, sim_time_sec: float = 11.0, port: int = 1236) -> dict | None:
    tmp_dir = Path(tempfile.gettempdir()) / "webots_dob_optuna"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    tuning_file = tmp_dir / f"tune_{trial_label}_p{profile}.json"
    csv_out = tmp_dir / f"tlm_{trial_label}_p{profile}.csv"
    if csv_out.exists():
        csv_out.unlink()

    payload = {
        "label": f"{trial_label}_p{profile}",
        "dob_enabled": 1.0,
        "dob_gain": float(params["dob_gain"]),
        "dob_tau_s": float(params["dob_tau_s"]),
        "max_dob_speed_deg_s": float(params["max_dob_speed_deg_s"]),
    }
    tuning_file.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    env = os.environ.copy()
    env["PATH"] = r"C:\Program Files\Webots\msys64\mingw64\bin;" + env.get("PATH", "")
    env.update(
        PYTHONUTF8="1",
        WEBOTS_CONTROL_MODE="P",
        WEBOTS_BALLOON_PROFILE=str(profile),
        WEBOTS_TUNING_JSON=str(tuning_file),
        WEBOTS_TELEMETRY_PATH=str(csv_out),
        WEBOTS_HEADLESS="1",
        WEBOTS_MAX_SIM_TIME=str(sim_time_sec),
        WEBOTS_IGNORE_PROFILE_LIMITS="1",
    )

    proc = subprocess.Popen(
        [
            WEBOTS,
            f"--port={port}",
            "--batch",
            "--minimize",
            "--mode=fast",
            "--stdout",
            "--stderr",
            str(WORLD),
        ],
        cwd=str(ROOT),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    try:
        proc.wait(timeout=28)
    except subprocess.TimeoutExpired:
        subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"], capture_output=True)

    time.sleep(1.5)
    if not csv_out.exists() or csv_out.stat().st_size < 1000:
        return None

    with open(csv_out, newline="", encoding="utf-8") as f:
        rows = [r for r in csv.DictReader(f) if r.get("target_detected") == "1"]

    if len(rows) < 40:
        return None

    err_p = np.array([float(r["angular_err_pan"]) for r in rows])
    err_t = np.array([float(r["angular_err_tilt"]) for r in rows])
    cmd_p = np.array([float(r.get("cmd_pan_deg") or 0.0) for r in rows])
    is_locked = np.array([1 if r["is_locked"] == "1" else 0 for r in rows])

    diff_cmd = np.diff(cmd_p)
    moves = diff_cmd[np.abs(diff_cmd) > 1e-4]
    flip_rate = float(np.mean(np.sign(moves[1:]) != np.sign(moves[:-1])) * 100.0) if len(moves) > 2 else 0.0

    return {
        "rms_pan": float(np.sqrt(np.mean(err_p**2))),
        "mae_pan": float(np.mean(np.abs(err_p))),
        "max_pan": float(np.max(np.abs(err_p))),
        "rms_tilt": float(np.sqrt(np.mean(err_t**2))),
        "lock_pct": float(np.mean(is_locked) * 100.0),
        "flip_rate": flip_rate,
        "frames": len(rows),
    }


def evaluate_trial(params: dict, trial_label: str) -> tuple[float, dict]:
    res_p2 = run_subscenario(params, profile=2, trial_label=trial_label, sim_time_sec=11.0)
    res_p5 = run_subscenario(params, profile=5, trial_label=trial_label, sim_time_sec=11.0)

    if res_p2 is None or res_p5 is None:
        return 999.0, {}

    # P2 Maliyeti: Sinus takip rampa hatasi ve kilit
    cost_p2 = res_p2["rms_pan"] + 0.5 * res_p2["mae_pan"] - 0.015 * res_p2["lock_pct"] + max(0.0, res_p2["flip_rate"] - 1.5) * 0.4
    # P5 Maliyeti: Zigzag yon degisim asimi, rampa hatasi ve titreti cezasi
    cost_p5 = res_p5["rms_pan"] + 0.5 * res_p5["mae_pan"] - 0.015 * res_p5["lock_pct"] + max(0.0, res_p5["flip_rate"] - 2.5) * 0.5

    total_cost = cost_p2 + 0.70 * cost_p5
    metrics = {"p2": res_p2, "p5": res_p5}
    return float(total_cost), metrics


def main():
    print("=" * 70, flush=True)
    print("DISTURBANCE OBSERVER (DOB) 24-ADIM OPTUNA TPE OPTIMIZASYONU", flush=True)
    print("Senaryolar: Profil 2 (Sinus) + Profil 5 (Zigzag)", flush=True)
    print("=" * 70, flush=True)

    sampler = optuna.samplers.TPESampler(seed=42)
    study = optuna.create_study(direction="minimize", sampler=sampler)

    # Trial 0: Mevcut el ile secilen baslangic degeri
    baseline = {
        "dob_gain": 0.70,
        "dob_tau_s": 0.15,
        "max_dob_speed_deg_s": 20.0,
    }
    study.enqueue_trial(baseline)

    records = []
    n_trials = 24

    for step in range(n_trials):
        trial = study.ask()
        if step == 0:
            params = baseline
        else:
            params = {
                "dob_gain": trial.suggest_float("dob_gain", 0.35, 1.25),
                "dob_tau_s": trial.suggest_float("dob_tau_s", 0.06, 0.32),
                "max_dob_speed_deg_s": trial.suggest_float("max_dob_speed_deg_s", 12.0, 28.0),
            }

        t0 = time.time()
        cost, metrics = evaluate_trial(params, f"trial_{step:03d}")
        study.tell(trial, cost)

        elapsed = time.time() - t0
        p2 = metrics.get("p2", {})
        p5 = metrics.get("p5", {})

        rec = {
            "trial": step + 1,
            "params": params,
            "cost": round(cost, 3),
            "p2_rms": round(p2.get("rms_pan", 0.0), 3),
            "p2_lock": round(p2.get("lock_pct", 0.0), 1),
            "p5_rms": round(p5.get("rms_pan", 0.0), 3),
            "p5_lock": round(p5.get("lock_pct", 0.0), 1),
            "p5_flip": round(p5.get("flip_rate", 0.0), 1),
            "elapsed_s": round(elapsed, 1),
        }
        records.append(rec)

        print(
            f"  [Trial {step+1:02d}/{n_trials:02d}] ({elapsed:.1f}s) Cost={cost:.3f} | "
            f"Gain={params['dob_gain']:.2f} Tau={params['dob_tau_s']:.3f}s MaxSpd={params['max_dob_speed_deg_s']:.1f} | "
            f"P2: Lock=%{p2.get('lock_pct', 0.0):.1f} RMS={p2.get('rms_pan', 0.0):.2f}° | "
            f"P5: Lock=%{p5.get('lock_pct', 0.0):.1f} Flip=%{p5.get('flip_rate', 0.0):.1f}",
            flush=True,
        )

    best_trial = study.best_trial
    print("\n" + "=" * 70, flush=True)
    print(f"[+] OPTIMIZASYON TAMAMLANDI! En Iyi Trial: #{best_trial.number + 1}", flush=True)
    print(f"    En Dusuk Maliyet: {best_trial.value:.3f}", flush=True)
    print(f"    Parametreler: {best_trial.params or baseline}", flush=True)
    print("=" * 70, flush=True)

    result_data = {
        "best_trial": best_trial.number + 1,
        "best_cost": float(best_trial.value),
        "best_params": best_trial.params or baseline,
        "trials": records,
    }
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(result_data, indent=2), encoding="utf-8")
    print(f"[+] Sonuclar kaydedildi: {OUT_JSON}", flush=True)


if __name__ == "__main__":
    main()
