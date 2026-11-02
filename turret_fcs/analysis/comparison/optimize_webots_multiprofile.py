#!/usr/bin/env python3
"""
================================================================================
MODUL: COKLU MESAFE VE COKLU PROFILDE WEBOTS OPTUNA TPE OPTIMIZASYONU
================================================================================
Kullanici ihtiyaci dogrultusunda 3 farkli mesafede (1.0m, 2.0m, 3.0m) ve
3 tamamlayici profilde (8-Kose Giris, 7-Figure-8, 2-Standart Sinus) sistemi zorlar:
- 1.0 m (Yakin Mesafe): Yuksek acisal dinamik, YOLO kutu gurultusu ve salinim riski.
- 2.0 m (Nominal Mesafe): Dengeli calisma noktasi.
- 3.0 m (Uzak Mesafe): Dar aci, kucuk piksel boyutu ve balistik kompanzasyon.

Hem P hem RL kontrolcu icin Pan/Tilt kazanclari, ivme limitleri, in-flight damping
ve olu bolgeleri tek bir guclu (robust) maliyet fonksiyonuyla optimize eder:
Robust Cost = Ortalama(Senaryo Maliyetleri) + 0.30 * En_Kotu_Senaryo_Maliyeti
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

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from turret_fcs.config import CONFIG

WEBOTS = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
WORLD = ROOT / "webots" / "worlds" / "turret_fcs.wbt"

# Arama senaryolari: (Profil ID, Mesafe metre, Sure saniye, Senaryo Agirligi)
# 8: Kose Ilk Kilit (asim/salinim), 7: Figure-8 (dinamik lag/ceviklik), 2: Standart Sinus (puruzsuz harmonik)
TUNING_SCENARIOS = [
    # 1.0 Metre (Yakin mesafe: yuksek acisal dinamik, maksimum salinim riski)
    (8, 1.0, 7.0, 1.5),  # 1m Kose giris (salinimi ve asimi bastirmak icin en yuksek agirlik)
    (7, 1.0, 6.0, 1.3),  # 1m Figure-8
    (2, 1.0, 5.0, 1.1),  # 1m Standart sinus

    # 2.0 Metre (Nominal calisma mesafesi)
    (8, 2.0, 7.0, 1.2),  # 2m Kose giris
    (7, 2.0, 6.0, 1.0),  # 2m Figure-8
    (2, 2.0, 5.0, 0.9),  # 2m Standart sinus

    # 3.0 Metre (Uzak mesafe: dar aci, kucuk hedef boyutu ve balistik hassasiyet)
    (8, 3.0, 6.0, 1.0),  # 3m Kose giris
    (7, 3.0, 5.0, 0.9),  # 3m Figure-8
    (2, 3.0, 5.0, 0.8),  # 3m Standart sinus
]

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
    "kp_pan": (0.16, 0.32),
    "kp_tilt": (0.08, 0.22),
    "in_flight_damping": (0.20, 0.70),      # 1m'de salinimi bastirmak icin guclu frenleme
    "deadband_pan_deg": (0.45, 0.90),
    "deadband_tilt_deg": (0.50, 0.95),
    "cmd_accel_pan_deg_s2": (300.0, 650.0),
    "cmd_accel_tilt_deg_s2": (180.0, 400.0),
    "alpha_filter": (0.75, 0.95),
}

RL_BASELINE = {
    "in_flight_damping": CONFIG.control.in_flight_damping,
    "reversal_damping": CONFIG.rl.reversal_damping,
    "taper_pan_deg": CONFIG.rl.taper_pan_deg,
    "taper_tilt_deg": CONFIG.rl.taper_tilt_deg,
    "rl_deadband_pan_deg": CONFIG.rl.deadband_pan_deg,
    "rl_deadband_tilt_deg": CONFIG.rl.deadband_tilt_deg,
    "d_err_ema_alpha": CONFIG.rl.d_err_ema_alpha,
}

RL_SPACE = {
    "in_flight_damping": (0.20, 0.70),
    "reversal_damping": (0.35, 0.85),
    "taper_pan_deg": (0.25, 1.10),
    "taper_tilt_deg": (0.25, 1.10),
    "rl_deadband_pan_deg": (0.45, 0.85),
    "rl_deadband_tilt_deg": (0.50, 0.95),
    "d_err_ema_alpha": (0.20, 0.55),
}


def _sign_flip_rate(arr: np.ndarray) -> float:
    moves = arr[np.abs(arr) > 1e-4]
    if moves.size < 3:
        return 0.0
    return float(np.mean(np.sign(moves[1:]) != np.sign(moves[:-1])) * 100.0)


def score_profile_trace(csv_path: Path, profile: int, distance_m: float = 2.0) -> tuple[float, dict]:
    """Her profilin ve mesafenin dinamigine gore maliyet ve metriklerini hesaplar."""
    if not csv_path.exists():
        raise ValueError(f"Telemetry file missing: {csv_path}")

    with csv_path.open(encoding="utf-8", newline="") as fh:
        rows = list(csv.DictReader(fh))

    det_rows = [r for r in rows if int(float(r.get("target_detected") or 0)) == 1]
    if len(det_rows) < 25:
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
        # Kose giris penceresi
        local_t = np.mod(sim_t, 5.0)
        active_mask = local_t <= 3.40
        settle_mask = (local_t >= 1.00) & (local_t <= 3.40)
        if np.sum(settle_mask) < 15:
            settle_mask = active_mask

        overshoots = []
        n_windows = max(1, int(np.max(sim_t) // 5.0) + 1)
        for w_idx in range(n_windows):
            w_mask = (sim_t >= w_idx * 5.0) & (sim_t < w_idx * 5.0 + 3.40)
            if np.sum(w_mask) < 10:
                continue
            w_err_p = err_p[w_mask]
            init_sign = float(np.sign(w_err_p[0])) if abs(w_err_p[0]) > 2.0 else 0.0
            if init_sign != 0.0:
                opp_excursion = np.maximum(0.0, -init_sign * w_err_p - 0.70)
                overshoots.append(float(np.max(opp_excursion)))

        max_overshoot_deg = float(np.max(overshoots)) if overshoots else 0.0
        tail_rms_deg = float(np.sqrt(np.mean(err_tot[settle_mask] ** 2)))
        overall_rms_deg = float(np.sqrt(np.mean(err_tot[active_mask] ** 2)))
        lock_pct = float(np.mean(locked[settle_mask]) * 100.0)
        flip_pct = 0.5 * (_sign_flip_rate(step_p[settle_mask]) + _sign_flip_rate(step_t[settle_mask]))
        settling_jerk_deg = float(0.5 * (np.mean(np.abs(np.diff(step_p[settle_mask]))) + np.mean(np.abs(np.diff(step_t[settle_mask])))))

        # Asim ve ilk kilit odakli maliyet
        cost = (
            10.0 * tail_rms_deg
            + 3.0 * overall_rms_deg
            + 20.0 * max_overshoot_deg
            + 1.8 * flip_pct
            + 25.0 * settling_jerk_deg
            - 0.50 * lock_pct
        )
    else:
        # Dinamik takip profilleri (Figure-8, Sinus): Takip geriligi (RMS) ve salinim/kilit odakli
        tail_rms_deg = float(np.sqrt(np.mean(err_tot ** 2)))
        overall_rms_deg = tail_rms_deg
        max_overshoot_deg = 0.0
        lock_pct = float(np.mean(locked) * 100.0)
        flip_pct = 0.5 * (_sign_flip_rate(step_p) + _sign_flip_rate(step_t))
        settling_jerk_deg = float(0.5 * (np.mean(np.abs(np.diff(step_p))) + np.mean(np.abs(np.diff(step_t)))))

        # Geride kalmayi (lag) ve salinimi sert cezalandiran maliyet
        cost = (
            18.0 * tail_rms_deg
            + 1.5 * flip_pct
            + 20.0 * settling_jerk_deg
            - 0.80 * lock_pct
        )

    metrics = {
        "profile": profile,
        "dist_m": distance_m,
        "frames": len(det_rows),
        "rms_deg": round(tail_rms_deg, 3),
        "overshoot_deg": round(max_overshoot_deg, 3),
        "lock_pct": round(lock_pct, 2),
        "flip_pct": round(flip_pct, 2),
        "jerk_deg": round(settling_jerk_deg, 4),
        "cost": round(float(cost), 3),
    }
    return float(cost), metrics


def run_single_sim(
    mode: str,
    params: dict[str, float],
    label: str,
    profile: int,
    distance_m: float,
    sim_time_sec: float,
    port: int,
) -> tuple[float, dict]:
    """Tek bir profil ve mesafe icin Webots simulasyonunu calistirir."""
    tmp_dir = Path(tempfile.gettempdir()) / "turret_webots_multiprof"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    d_tag = f"{int(distance_m * 100)}cm"
    tuning_file = tmp_dir / f"tune_{mode}_{label}_p{profile}_{d_tag}.json"
    payload = dict(params)
    payload["label"] = label
    tuning_file.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    csv_out = tmp_dir / f"tlm_{mode}_{label}_p{profile}_{d_tag}.csv"
    if csv_out.exists():
        csv_out.unlink()

    env = os.environ.copy()
    env["PATH"] = r"C:\Program Files\Webots\msys64\mingw64\bin;" + env.get("PATH", "")
    env.update(
        PYTHONUTF8="1",
        WEBOTS_CONTROL_MODE=mode,
        WEBOTS_BALLOON_PROFILE=str(profile),
        WEBOTS_BALLOON_DISTANCE=str(distance_m),
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
        proc.wait(timeout=max(30.0, sim_time_sec * 3.0))
    except subprocess.TimeoutExpired:
        subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"], capture_output=True)
        time.sleep(1.0)

    return score_profile_trace(csv_out, profile=profile, distance_m=distance_m)


def evaluate_multi_scenario(
    mode: str,
    params: dict[str, float],
    label: str,
    port: int = 1295,
) -> tuple[float, dict]:
    """Tum profillerde ve mesafelerde calistirip agirlikli ortalama + en kotu senaryo maliyetini doner."""
    scenario_costs = []
    scenario_metrics = {}

    for prof, dist_m, sim_t, weight in TUNING_SCENARIOS:
        try:
            c, m = run_single_sim(
                mode, params, label, profile=prof, distance_m=dist_m, sim_time_sec=sim_t, port=port
            )
            weighted_c = c * weight
            scenario_costs.append(weighted_c)
            key = f"p{prof}_{int(dist_m * 100)}cm"
            scenario_metrics[key] = m
        except Exception as exc:
            return 1e6, {"failure": str(exc), "prof": prof, "dist_m": dist_m}

    # Robust cost: Agirlikli ortalama + 0.30 * En Kotu Senaryo (ozellikle 1m salinimlarini cezalandirir)
    robust_cost = float(np.mean(scenario_costs) + 0.30 * np.max(scenario_costs))
    return robust_cost, scenario_metrics


def optimize_multi_profile(
    mode: str,
    n_trials: int = 15,
    port: int = 1295,
) -> dict:
    baseline = dict(P_BASELINE if mode == "P" else RL_BASELINE)
    space = P_SPACE if mode == "P" else RL_SPACE

    print("=" * 85, flush=True)
    print(f"  COKLU MESAFE (1m, 2m, 3m) & COKLU PROFIL (8-Kose, 7-Fig8, 2-Sinus) OPTUNA TPE // {mode}", flush=True)
    print("=" * 85, flush=True)

    t0 = time.time()
    base_cost, base_metrics = evaluate_multi_scenario(mode, baseline, f"{mode.lower()}_base", port=port)
    p8_1m = base_metrics.get("p8_100cm", {})
    p7_1m = base_metrics.get("p7_100cm", {})
    p2_1m = base_metrics.get("p2_100cm", {})
    p8_2m = base_metrics.get("p8_200cm", {})
    print(
        f"[BASELINE #{mode}] ({time.time() - t0:.1f}s) Toplam Robust Cost={base_cost:.2f}\n"
        f"  - 1.0m P8(Kose): Lock=%{p8_1m.get('lock_pct')} | Over={p8_1m.get('overshoot_deg')} deg | Flip=%{p8_1m.get('flip_pct')}\n"
        f"  - 1.0m P7(Fig8): Lock=%{p7_1m.get('lock_pct')} | RMS={p7_1m.get('rms_deg')} deg | Flip=%{p7_1m.get('flip_pct')}\n"
        f"  - 1.0m P2(Sin2): Lock=%{p2_1m.get('lock_pct')} | RMS={p2_1m.get('rms_deg')} deg | Flip=%{p2_1m.get('flip_pct')}\n"
        f"  - 2.0m P8(Kose): Lock=%{p8_2m.get('lock_pct')} | RMS={p8_2m.get('rms_deg')} deg",
        flush=True,
    )

    sampler = optuna.samplers.TPESampler(seed=731, n_startup_trials=min(5, max(2, n_trials // 2)), multivariate=True)
    study = optuna.create_study(direction="minimize", sampler=sampler)
    study.enqueue_trial(baseline)

    records = []

    def objective(trial: optuna.Trial) -> float:
        t_start = time.time()
        params = {k: trial.suggest_float(k, *space[k]) for k in space}
        cost, metrics = evaluate_multi_scenario(mode, params, f"trial_{trial.number:03d}", port=port)
        rec = {
            "trial": trial.number,
            "cost": cost,
            "params": params,
            "metrics": metrics,
        }
        records.append(rec)
        m8_1m = metrics.get("p8_100cm", {})
        m7_1m = metrics.get("p7_100cm", {})
        m2_1m = metrics.get("p2_100cm", {})
        print(
            f"  [Trial {trial.number + 1:02d}/{n_trials:02d}] ({time.time() - t_start:.1f}s) "
            f"Cost={cost:.2f} | 1m_P8: Lock=%{m8_1m.get('lock_pct')} Over={m8_1m.get('overshoot_deg')}° Flip=%{m8_1m.get('flip_pct')} | "
            f"1m_P7: Lock=%{m7_1m.get('lock_pct')} RMS={m7_1m.get('rms_deg')}° | "
            f"1m_P2: Lock=%{m2_1m.get('lock_pct')}",
            flush=True,
        )
        return cost

    study.optimize(objective, n_trials=n_trials)

    best_rec = min(records, key=lambda r: r["cost"])
    accepted = best_rec["cost"] < base_cost

    out_dir = ROOT / "turret_fcs" / "analysis" / "comparison"
    out_path = out_dir / f"webots_multiprofile_{mode.lower()}_results.json"
    summary = {
        "mode": mode,
        "accepted": accepted,
        "baseline_cost": base_cost,
        "best_cost": best_rec["cost"],
        "best_trial": best_rec["trial"],
        "baseline_params": baseline,
        "best_params": best_rec["params"],
        "baseline_metrics": base_metrics,
        "best_metrics": best_rec["metrics"],
        "trials": records,
    }
    out_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"\n[OK] {mode} Coklu Mesafe Optimizasyonu Tamamlandi -> {out_path.name}", flush=True)

    if accepted:
        tuning_target = ROOT / "webots" / "tuning_params.json"
        try:
            curr_tune = json.loads(tuning_target.read_text(encoding="utf-8")) if tuning_target.exists() else {}
        except Exception:
            curr_tune = {}
        curr_tune.update(best_rec["params"])
        curr_tune["label"] = f"multidistance_optuna_{mode.lower()}"
        tuning_target.write_text(json.dumps(curr_tune, indent=2), encoding="utf-8")
        print("[GUNCELLEME] webots/tuning_params.json dosyasina en iyi parametreler yazildi!", flush=True)

    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["P", "RL", "BOTH"], default="BOTH")
    parser.add_argument("--trials", type=int, default=12)
    parser.add_argument("--port", type=int, default=1295)
    args = parser.parse_args()

    if args.mode in ("P", "BOTH"):
        optimize_multi_profile("P", n_trials=args.trials, port=args.port)
    if args.mode in ("RL", "BOTH"):
        optimize_multi_profile("RL", n_trials=args.trials, port=args.port + 1)


if __name__ == "__main__":
    main()
