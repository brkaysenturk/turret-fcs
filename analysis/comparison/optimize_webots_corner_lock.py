#!/usr/bin/env python3
"""
================================================================================
MODUL: DOGRUDAN WEBOTS FIZIGINDE KOSE GIRIS / ILK KILIT OPTUNA TPE OPTIMIZASYONU
================================================================================
self-balancing-robot projesindeki simulation_runner.py + optimize_pid_controller.py
mimarisini temel alir:
- Sahte/eksik Python ortami yerine DOGRUDAN Webots (--batch --minimize --mode=fast)
  ve tam FCSManager boru hattini (YOLOv8 + ByteTrack + CWNA Kalman + 3B Ego-Motion
  + LockManager + P / RL Kontrolcu + Servo Lag + 1.61 deg Backlash) kosturur.
- Ozellikle gercek sistemdeki en kritik sorunu olan Profil 8 (KOSE GIRIS / ILK KILIT:
  4 farkli kadraj kosesinden aniden giren balona ilk yonelme ve osilasyonsuz kilitlenme)
  ve Profil 7 (FIGURE-8 dinamik takip) uzerinde P ve RL kontrolculeri optimize eder.
================================================================================
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import optuna

optuna.logging.set_verbosity(optuna.logging.WARNING)

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from turret_fcs.config import CONFIG

WEBOTS = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
WORLD = ROOT / "webots" / "worlds" / "webots_turret_fcs.wbt"

P_BASELINE = {
    "kp_pan": CONFIG.control.kp_pan,
    "kp_tilt": CONFIG.control.kp_tilt,
    "in_flight_damping": CONFIG.control.in_flight_damping,
    "deadband_pan_deg": CONFIG.control.deadband_pan_deg,
    "deadband_tilt_deg": CONFIG.control.deadband_tilt_deg,
    "cmd_accel_pan_deg_s2": CONFIG.control.cmd_accel_pan_deg_s2,
    "cmd_accel_tilt_deg_s2": CONFIG.control.cmd_accel_tilt_deg_s2,
    "alpha_filter": CONFIG.control.alpha_filter,
}

P_SPACE = {
    "kp_pan": (0.18, 0.30),
    "kp_tilt": (0.11, 0.22),
    "in_flight_damping": (0.20, 0.85),
    "deadband_pan_deg": (0.55, 0.95),
    "deadband_tilt_deg": (0.65, 1.05),
    "cmd_accel_pan_deg_s2": (320.0, 600.0),
    "cmd_accel_tilt_deg_s2": (180.0, 380.0),
    "alpha_filter": (0.78, 0.95),
}

RL_BASELINE = {
    "in_flight_damping": CONFIG.control.in_flight_damping,
    "reversal_damping": CONFIG.rl.reversal_damping,
    "taper_pan_deg": CONFIG.rl.taper_pan_deg,
    "taper_tilt_deg": CONFIG.rl.taper_tilt_deg,
    "deadband_pan_deg": CONFIG.control.deadband_pan_deg,
    "deadband_tilt_deg": CONFIG.control.deadband_tilt_deg,
    "d_err_ema_alpha": CONFIG.rl.d_err_ema_alpha,
}

RL_SPACE = {
    "in_flight_damping": (0.15, 0.80),
    "reversal_damping": (0.25, 0.75),
    "taper_pan_deg": (0.35, 1.65),
    "taper_tilt_deg": (0.35, 1.65),
    "deadband_pan_deg": (0.55, 0.95),
    "deadband_tilt_deg": (0.65, 1.05),
    "d_err_ema_alpha": (0.25, 0.65),
}


def _sign_flip_rate(arr: np.ndarray) -> float:
    moves = arr[np.abs(arr) > 1e-4]
    if moves.size < 3:
        return 0.0
    return float(np.mean(np.sign(moves[1:]) != np.sign(moves[:-1])) * 100.0)


def score_corner_trace(csv_path: Path, profile: int = 8) -> tuple[float, dict]:
    """
    Webots telemetri CSV kaydini analiz eder; ozellikle koseden giris sonrasi
    ilk kilitlenme asimini (overshoot), yerlesme hatasini (tail RMS),
    komut yon degistirme (flip rate) osilasyonunu ve kilit oranini puanlar.
    """
    if not csv_path.exists():
        raise ValueError(f"Telemetry file missing: {csv_path}")

    with csv_path.open(encoding="utf-8", newline="") as fh:
        rows = list(csv.DictReader(fh))

    det_rows = [r for r in rows if int(float(r.get("target_detected") or 0)) == 1]
    if len(det_rows) < 60:
        raise ValueError(f"Insufficient detected frames ({len(det_rows)})")

    sim_t = np.array([float(r["sim_time_sec"]) for r in det_rows])
    err_p = np.array([float(r["angular_err_pan"]) for r in det_rows])
    err_t = np.array([float(r["angular_err_tilt"]) for r in det_rows])
    err_tot = np.hypot(err_p, err_t)
    cmd_p = np.array([float(r["cmd_pan_deg"]) for r in det_rows])
    cmd_t = np.array([float(r["cmd_tilt_deg"]) for r in det_rows])
    locked = np.array([int(float(r["is_locked"])) for r in det_rows])

    step_p = np.diff(cmd_p, prepend=cmd_p[0])
    step_t = np.diff(cmd_t, prepend=cmd_t[0])

    if profile == 8:
        # 5.0 s periyotlu Kose Giris pencereleri:
        # 0.0..3.4 s aktif kose girisi, 1.0..3.4 s kilit yerlesme (settling) penceresi
        local_t = np.mod(sim_t, 5.0)
        active_mask = local_t <= 3.40
        settle_mask = (local_t >= 1.00) & (local_t <= 3.40)
        if np.sum(settle_mask) < 20:
            settle_mask = active_mask

        # Her 5.0 s pencerede ilk hata isaretine gore asim (overshoot) hesabi
        overshoots = []
        n_windows = max(1, int(np.max(sim_t) // 5.0) + 1)
        for w_idx in range(n_windows):
            w_mask = (sim_t >= w_idx * 5.0) & (sim_t < w_idx * 5.0 + 3.40)
            if np.sum(w_mask) < 10:
                continue
            w_err_p = err_p[w_mask]
            init_sign = float(np.sign(w_err_p[0])) if abs(w_err_p[0]) > 2.0 else 0.0
            if init_sign != 0.0:
                opp_excursion = np.maximum(0.0, -init_sign * w_err_p - 0.85)
                overshoots.append(float(np.max(opp_excursion)))

        max_overshoot_deg = float(np.max(overshoots)) if overshoots else 0.0
        tail_rms_deg = float(np.sqrt(np.mean(err_tot[settle_mask] ** 2)))
        overall_rms_deg = float(np.sqrt(np.mean(err_tot[active_mask] ** 2)))
        lock_pct = float(np.mean(locked[settle_mask]) * 100.0)
        flip_pct = 0.5 * (
            _sign_flip_rate(step_p[settle_mask]) + _sign_flip_rate(step_t[settle_mask])
        )
        settling_jerk_deg = float(
            0.5 * (
                np.mean(np.abs(np.diff(step_p[settle_mask])))
                + np.mean(np.abs(np.diff(step_t[settle_mask])))
            )
        )
    else:
        max_overshoot_deg = 0.0
        tail_rms_deg = float(np.sqrt(np.mean(err_tot ** 2)))
        overall_rms_deg = tail_rms_deg
        lock_pct = float(np.mean(locked) * 100.0)
        flip_pct = 0.5 * (_sign_flip_rate(step_p) + _sign_flip_rate(step_t))
        settling_jerk_deg = float(
            0.5 * (np.mean(np.abs(np.diff(step_p))) + np.mean(np.abs(np.diff(step_t))))
        )

    cost = (
        12.0 * tail_rms_deg
        + 4.0 * overall_rms_deg
        + 25.0 * max_overshoot_deg
        + 2.2 * flip_pct
        + 35.0 * settling_jerk_deg
        - 0.60 * lock_pct
    )
    metrics = {
        "frames": len(det_rows),
        "tail_rms_deg": round(tail_rms_deg, 3),
        "overall_rms_deg": round(overall_rms_deg, 3),
        "max_overshoot_deg": round(max_overshoot_deg, 3),
        "lock_pct": round(lock_pct, 2),
        "flip_pct": round(flip_pct, 2),
        "settling_jerk_deg": round(settling_jerk_deg, 4),
        "cost": round(float(cost), 3),
    }
    return float(cost), metrics


def run_webots_eval(
    mode: str,
    params: dict[str, float],
    label: str,
    profiles: tuple[int, ...] = (8,),
    sim_time_sec: float = 15.0,
    port: int = 1295,
) -> tuple[float, dict]:
    """
    Belirtilen kontrol modunu (P veya RL) ve parametreleri dogrudan Webots'ta
    kosturur ve self-balancing-robot robust() maliyetiyle puanlar.
    """
    tmp_dir = Path(tempfile.gettempdir()) / "turret_webots_optuna"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    tuning_file = tmp_dir / f"tuning_{mode}_{label}.json"
    payload = dict(params)
    payload["label"] = label
    tuning_file.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    all_metrics: dict[str, dict] = {}
    costs: list[float] = []

    for prof in profiles:
        csv_out = tmp_dir / f"telemetry_{mode}_{label}_prof{prof}.csv"
        if csv_out.exists():
            csv_out.unlink()

        env = os.environ.copy()
        env.update(
            PYTHONUTF8="1",
            WEBOTS_CONTROL_MODE=mode,
            WEBOTS_BALLOON_PROFILE=str(prof),
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
            proc.wait(timeout=max(35.0, sim_time_sec * 3.0))
        except subprocess.TimeoutExpired:
            subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"], capture_output=True)
            time.sleep(1.0)

        try:
            cost, metrics = score_corner_trace(csv_out, profile=prof)
        except Exception as exc:
            return 1e6, {f"prof_{prof}": {"failure": str(exc)}}

        costs.append(cost)
        all_metrics[f"prof_{prof}"] = metrics

    robust_cost = float(np.mean(costs) + 0.25 * np.max(costs))
    return robust_cost, all_metrics


def optimize_controller(
    mode: str,
    n_trials: int = 12,
    sim_time_sec: float = 15.0,
    port: int = 1295,
) -> dict:
    baseline = dict(P_BASELINE if mode == "P" else RL_BASELINE)
    space = P_SPACE if mode == "P" else RL_SPACE

    print("=" * 78, flush=True)
    print(f"  DOGRUDAN WEBOTS OPTUNA TPE // MOD: {mode} | PROFIL 8 (KOSE ILK KILIT)", flush=True)
    print("=" * 78, flush=True)

    t0 = time.time()
    base_cost, base_metrics = run_webots_eval(
        mode, baseline, f"{mode.lower()}_baseline", profiles=(8,), sim_time_sec=sim_time_sec, port=port
    )
    bm = base_metrics.get("prof_8", {})
    print(
        f"[BASELINE #{mode}] ({time.time() - t0:.1f}s) cost={base_cost:.2f} | "
        f"tail_rms={bm.get('tail_rms_deg')} deg | overshoot={bm.get('max_overshoot_deg')} deg | "
        f"lock=%{bm.get('lock_pct')} | flip=%{bm.get('flip_pct')} | jerk={bm.get('settling_jerk_deg')}",
        flush=True,
    )

    sampler = optuna.samplers.TPESampler(seed=731, n_startup_trials=min(5, max(2, n_trials // 2)), multivariate=True)
    study = optuna.create_study(direction="minimize", sampler=sampler)
    study.enqueue_trial(baseline)

    # Enqueue smart physics-guided candidate for Corner-Entry Slew Braking
    if mode == "P":
        study.enqueue_trial({
            "kp_pan": 0.245,
            "kp_tilt": 0.155,
            "in_flight_damping": 0.55,
            "deadband_pan_deg": 0.75,
            "deadband_tilt_deg": 0.85,
            "cmd_accel_pan_deg_s2": 450.0,
            "cmd_accel_tilt_deg_s2": 250.0,
            "alpha_filter": 0.90,
        })
    else:
        study.enqueue_trial({
            "in_flight_damping": 0.50,
            "reversal_damping": 0.50,
            "taper_pan_deg": 0.70,
            "taper_tilt_deg": 0.70,
            "deadband_pan_deg": 0.75,
            "deadband_tilt_deg": 0.85,
            "d_err_ema_alpha": 0.45,
        })

    records = []

    def objective(trial: optuna.Trial) -> float:
        t_start = time.time()
        params = {k: trial.suggest_float(k, *space[k]) for k in space}
        cost, metrics = run_webots_eval(
            mode, params, f"trial_{trial.number:03d}", profiles=(8,), sim_time_sec=sim_time_sec, port=port
        )
        m8 = metrics.get("prof_8", {})
        rec = {
            "trial": trial.number,
            "cost": cost,
            "params": params,
            "metrics": metrics,
        }
        records.append(rec)
        print(
            f"  [Trial {trial.number + 1:02d}/{n_trials:02d}] ({time.time() - t_start:.1f}s) "
            f"cost={cost:.2f} | tail_rms={m8.get('tail_rms_deg')} deg | "
            f"overshoot={m8.get('max_overshoot_deg')} deg | lock=%{m8.get('lock_pct')} | "
            f"flip=%{m8.get('flip_pct')}",
            flush=True,
        )
        return cost

    study.optimize(objective, n_trials=n_trials)

    best_rec = min(records, key=lambda r: r["cost"])
    accepted = best_rec["cost"] < base_cost

    # Ayrica kazanan adayi Profil 7 (Figure-8) uzerinde de dogrula
    print(f"\n[*] Kazanan {mode} adayi Profil 7 (Figure-8) uzerinde dogrulaniyor...", flush=True)
    _, fig8_base = run_webots_eval(mode, baseline, f"{mode.lower()}_fig8_base", profiles=(7,), sim_time_sec=12.0, port=port)
    _, fig8_best = run_webots_eval(mode, best_rec["params"], f"{mode.lower()}_fig8_best", profiles=(7,), sim_time_sec=12.0, port=port)

    out_dir = ROOT / "analysis" / "comparison"
    out_path = out_dir / f"webots_optuna_{mode.lower()}_results.json"
    summary = {
        "mode": mode,
        "accepted": accepted,
        "baseline_cost": base_cost,
        "best_cost": best_rec["cost"],
        "best_trial": best_rec["trial"],
        "baseline_params": baseline,
        "best_params": best_rec["params"],
        "corner_profile8_baseline": base_metrics.get("prof_8", {}),
        "corner_profile8_best": best_rec["metrics"].get("prof_8", {}),
        "figure8_profile7_baseline": fig8_base.get("prof_7", {}),
        "figure8_profile7_best": fig8_best.get("prof_7", {}),
        "trials": records,
    }
    out_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"[OK] {mode} sonuclari kaydedildi: {out_path}", flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["P", "RL", "BOTH"], default="BOTH")
    parser.add_argument("--trials", type=int, default=8)
    parser.add_argument("--sim-time", type=float, default=15.0)
    parser.add_argument("--port", type=int, default=1295)
    args = parser.parse_args()

    if args.mode in ("P", "BOTH"):
        optimize_controller("P", n_trials=args.trials, sim_time_sec=args.sim_time, port=args.port)
    if args.mode in ("RL", "BOTH"):
        optimize_controller("RL", n_trials=args.trials, sim_time_sec=args.sim_time, port=args.port + 1)


if __name__ == "__main__":
    main()

