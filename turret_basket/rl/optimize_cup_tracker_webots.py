#!/usr/bin/env python3

"""

================================================================================

BASKET CHALLENGE: WEBOTS OPTUNA TPE KUPA KILITLENME OPTIMIZASYONU (1. MODEL)

================================================================================

Bu betik; 1. Model (CupTracker / RL Kilitlenme Motoru) parametrelerini,

kullanıcının talep ettiği 6 farklı mesafede (100cm, 140cm, 180cm, 220cm, 260cm, 300cm)

Webots simülatöründe koşturarak optimize eder.



ISOLATION GUARANTEE:

- turret_fcs ana konfigürasyonuna ASLA dokunmaz.

- Yalnızca basket/models/cup_tracker_params.json ve cup_tracker_params_webots.json yazar.

- Baseline (Trial 0) olarak Python simülatörünün ürettiği parametreleri enjekte eder.

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



PROJECT_ROOT = Path(__file__).resolve().parents[2]

if str(PROJECT_ROOT) not in sys.path:

    sys.path.insert(0, str(PROJECT_ROOT))



WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

WORLD_PATH = PROJECT_ROOT / "webots" / "worlds" / "turret_basket.wbt"



# Kullanıcının belirlediği 6 senaryo: (Mesafe Y metre, Ofset X metre, Süre saniye, Senaryo Ağırlığı)

SCENARIOS = [

    (1.00, -0.16, 1.8, 1.3),  # 100 cm: Yakın mesafe, geniş açı, aşım/titreşim riski

    (1.40, +0.20, 1.8, 1.1),  # 140 cm: Orta-yakın, sağ ofset

    (1.80, -0.18, 1.8, 1.2),  # 180 cm: Nominal, sol ofset (hunting baskılama)

    (2.20, +0.22, 1.8, 1.0),  # 220 cm: Nominal-uzak, sağ ofset

    (2.60, -0.14, 1.8, 1.2),  # 260 cm: Uzak mesafe, sol ofset (hunting baskılama)

    (3.00, +0.18, 1.8, 1.1),  # 300 cm: Maksimum menzil, dar açı, hassas kilit

]



# Python Simülatöründen gelen Baseline Parametreleri (Trial 0)

BASELINE_PARAMS = {

    "deadband_pan_deg": 0.1948,

    "deadband_tilt_deg": 0.1522,

    "taper_pan_deg": 0.1844,

    "taper_tilt_deg": 0.1222,

    "in_flight_damping": 0.3907,

    "reversal_damping": 0.3351,

    "aim_mouth_ratio": 0.3518,

}



SEARCH_SPACE = {

    "deadband_pan_deg": (0.10, 0.40),

    "deadband_tilt_deg": (0.08, 0.32),

    "taper_pan_deg": (0.10, 0.35),

    "taper_tilt_deg": (0.08, 0.28),

    "in_flight_damping": (0.20, 0.65),

    "reversal_damping": (0.20, 0.60),

    "aim_mouth_ratio": (0.20, 0.50),

}





def score_scenario_csv(csv_path: Path, weight: float = 1.0) -> tuple[float, dict]:

    """Senaryo telemetrisinden kararlılık, RMS ve kilitlenme metriğini çıkarır."""

    if not csv_path.exists():

        return 999.0, {"error": "telemetry_missing"}



    with open(csv_path, "r", encoding="utf-8") as f:

        rows = list(csv.DictReader(f))



    det_rows = [r for r in rows if int(r.get("target_detected", 0)) == 1]

    if len(det_rows) < 15:

        return 999.0, {"error": "insufficient_detections", "count": len(det_rows)}



    sim_t = np.array([float(r["sim_time_sec"]) for r in det_rows])

    err_pan = np.array([float(r["err_pan"]) for r in det_rows])

    err_tilt = np.array([float(r["err_tilt"]) for r in det_rows])

    is_locked = np.array([int(r["is_locked"]) for r in det_rows])

    cmd_pan = np.array([float(r["cmd_pan_deg"]) for r in det_rows])

    cmd_tilt = np.array([float(r["cmd_tilt_deg"]) for r in det_rows])



    err_tot = np.hypot(err_pan, err_tilt)



    # 1. Kilitlenme Süresi (İlk kilitlenme anı)

    locked_idx = np.where(is_locked == 1)[0]

    settling_time = float(sim_t[locked_idx[0]] - sim_t[0]) if len(locked_idx) > 0 else 1.8



    # 2. Kararlı Durum (Son 1.0 saniye) RMS Hatası

    t_end = sim_t[-1]

    tail_mask = sim_t >= (t_end - 1.0)

    tail_rms = float(np.sqrt(np.mean(err_tot[tail_mask] ** 2))) if np.any(tail_mask) else float(np.sqrt(np.mean(err_tot ** 2)))



    # 3. Titreşim / Yön Değiştirme Oranı (Sign Flip Rate)

    diff_p = np.diff(cmd_pan)

    diff_t = np.diff(cmd_tilt)

    moves_p = diff_p[np.abs(diff_p) > 1e-4]

    moves_t = diff_t[np.abs(diff_t) > 1e-4]

    flip_p = float(np.mean(np.sign(moves_p[1:]) != np.sign(moves_p[:-1])) * 100.0) if len(moves_p) > 2 else 0.0

    flip_t = float(np.mean(np.sign(moves_t[1:]) != np.sign(moves_t[:-1])) * 100.0) if len(moves_t) > 2 else 0.0

    flip_rate = 0.5 * (flip_p + flip_t)



    # 4. Kilitli Kalma Oranı (%)

    tail_lock_pct = float(np.mean(is_locked[tail_mask]) * 100.0) if np.any(tail_mask) else 0.0



    # Maliyet Fonksiyonu:

    # Düşük RMS, Hızlı kilitlenme, Sıfır titreşim ve %100 kararlı kilit ödüllendirilir.

    cost = (

        2.5 * tail_rms

        + 1.0 * settling_time

        + 0.04 * flip_rate

        + 3.0 * (1.0 - tail_lock_pct / 100.0)

    ) * weight



    metrics = {

        "settling_time": round(settling_time, 3),

        "tail_rms": round(tail_rms, 3),

        "flip_rate": round(flip_rate, 2),

        "tail_lock_pct": round(tail_lock_pct, 1),

        "cost": round(cost, 3),

    }

    return float(cost), metrics





def run_single_simulation(

    params: dict[str, float],

    target_y: float,

    target_x: float,

    sim_time: float,

    label: str,

    port: int = 1290,

) -> tuple[float, dict]:

    """Tek bir kupa senaryosu için Webots'u headless koşturur."""

    tmp_dir = Path(tempfile.gettempdir()) / "cup_tracker_optuna"

    tmp_dir.mkdir(parents=True, exist_ok=True)



    dist_tag = f"{int(target_y * 100)}cm"

    tuning_file = tmp_dir / f"tune_{label}_{dist_tag}.json"

    tuning_file.write_text(json.dumps(params, indent=2), encoding="utf-8")



    csv_out = tmp_dir / f"tlm_{label}_{dist_tag}.csv"

    if csv_out.exists():

        csv_out.unlink()



    env = os.environ.copy()
    env["PATH"] = r"C:\Program Files\Webots\msys64\mingw64\bin;" + env.get("PATH", "")

    env.update({

        "PYTHONUTF8": "1",

        "CUP_HEADLESS": "1",

        "CUP_MAX_SIM_TIME": str(sim_time),

        "CUP_TARGET_X": str(target_x),

        "CUP_TARGET_Y": str(target_y),

        "CUP_TUNING_JSON": str(tuning_file),

        "CUP_TELEMETRY_CSV": str(csv_out),

    })



    cmd = [

        WEBOTS_EXE,

        f"--port={port}",

        "--batch",

        "--minimize",

        "--mode=fast",

        "--stdout",

        "--stderr",

        str(WORLD_PATH),

    ]



    proc = subprocess.Popen(

        cmd,

        cwd=str(PROJECT_ROOT),

        env=env,

        stdout=subprocess.DEVNULL,

        stderr=subprocess.DEVNULL,

    )



    try:

        proc.wait(timeout=max(25.0, sim_time * 4.0))

    except subprocess.TimeoutExpired:

        subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"], capture_output=True)

        time.sleep(0.5)



    return score_scenario_csv(csv_out)





def evaluate_trial_all_scenarios(params: dict[str, float], trial_idx: int) -> tuple[float, dict]:

    """Tüm 6 senaryoyu koşturur ve robust bileşik maliyeti hesaplar."""

    scenario_costs = []

    scenario_metrics = {}



    for target_y, target_x, sim_t, weight in SCENARIOS:

        dist_tag = f"{int(target_y * 100)}cm"

        cost, metrics = run_single_simulation(

            params=params,

            target_y=target_y,

            target_x=target_x,

            sim_time=sim_t,

            label=f"trial_{trial_idx}",

            port=1291,

        )

        scenario_costs.append(cost * weight)

        scenario_metrics[dist_tag] = metrics



    # Robust Minimax Optuna: Ağırlıklı Ortalama + 0.30 * En Kötü Senaryo

    robust_cost = float(np.mean(scenario_costs) + 0.30 * np.max(scenario_costs))

    return robust_cost, scenario_metrics





def main():

    parser = argparse.ArgumentParser(description="Webots Cup Tracker Optuna Optimization")

    parser.add_argument("--trials", type=int, default=12, help="Number of Optuna trials (default: 12)")

    args = parser.parse_args()



    print("=" * 75)

    print("  WEBOTS OPTUNA TPE: CUP TRACKER (1. MODEL) 12-TRIAL OPTIMIZATION")

    print("=" * 75)

    print(f"  Senaryolar: {[f'{int(s[0]*100)}cm' for s in SCENARIOS]}")

    print(f"  Baseline (Trial 0): Simulator Prior ({BASELINE_PARAMS['deadband_pan_deg']:.3f}° pan db)")

    print(f"  Toplam Deneme Sayısı: {args.trials}")

    print("=" * 75, flush=True)



    study = optuna.create_study(direction="minimize", sampler=optuna.samplers.TPESampler(seed=42))



    # Trial 0 olarak Simülatör Parametrelerini Kuyruğa Ekle

    study.enqueue_trial(BASELINE_PARAMS)



    trial_history = []



    def objective(trial: optuna.Trial) -> float:

        params = {

            "deadband_pan_deg": trial.suggest_float("deadband_pan_deg", *SEARCH_SPACE["deadband_pan_deg"]),

            "deadband_tilt_deg": trial.suggest_float("deadband_tilt_deg", *SEARCH_SPACE["deadband_tilt_deg"]),

            "taper_pan_deg": trial.suggest_float("taper_pan_deg", *SEARCH_SPACE["taper_pan_deg"]),

            "taper_tilt_deg": trial.suggest_float("taper_tilt_deg", *SEARCH_SPACE["taper_tilt_deg"]),

            "in_flight_damping": trial.suggest_float("in_flight_damping", *SEARCH_SPACE["in_flight_damping"]),

            "reversal_damping": trial.suggest_float("reversal_damping", *SEARCH_SPACE["reversal_damping"]),

            "aim_mouth_ratio": trial.suggest_float("aim_mouth_ratio", *SEARCH_SPACE["aim_mouth_ratio"]),

        }



        t_start = time.time()

        cost, metrics = evaluate_trial_all_scenarios(params, trial.number)

        elapsed = time.time() - t_start



        avg_rms = np.mean([m["tail_rms"] for m in metrics.values() if "tail_rms" in m])

        avg_lock = np.mean([m["tail_lock_pct"] for m in metrics.values() if "tail_lock_pct" in m])

        avg_flip = np.mean([m["flip_rate"] for m in metrics.values() if "flip_rate" in m])



        tag = "[BASELINE]" if trial.number == 0 else f"[TRIAL {trial.number:02d}]"

        print(

            f"{tag} Cost: {cost:.3f} | RMS: {avg_rms:.3f}° | Lock: {avg_lock:.1f}% | Flip: {avg_flip:.1f}% | Time: {elapsed:.1f}s",

            flush=True,

        )



        trial_history.append({

            "trial": trial.number,

            "cost": cost,

            "avg_rms": avg_rms,

            "avg_lock": avg_lock,

            "avg_flip": avg_flip,

            "params": params,

            "metrics": metrics,

        })

        return cost



    study.optimize(objective, n_trials=args.trials)



    print("\n" + "=" * 75)

    print("  OPTIMIZASYON TAMAMLANDI!")

    print("=" * 75)

    print(f"  En Iyi Deneme: Trial #{study.best_trial.number}")

    print(f"  En Iyi Robust Maliyet: {study.best_value:.3f}")

    print(f"  Baseline (Trial 0) Maliyeti: {trial_history[0]['cost']:.3f}")

    improvement_pct = ((trial_history[0]['cost'] - study.best_value) / trial_history[0]['cost']) * 100.0

    print(f"  Gelisim Orani: %{improvement_pct:.1f}")

    print("-" * 75)

    print("  En Iyi Parametreler:")

    for k, v in study.best_params.items():

        base_v = BASELINE_PARAMS[k]

        print(f"    - {k:22s}: {v:.4f}  (Baseline: {base_v:.4f})")

    print("=" * 75)



    # Parametreleri kaydet (SADECE basket/models altina, turret-fcs'e ASLA dokunmaz!)

    out_dir = Path(__file__).resolve().parent / "models"

    out_dir.mkdir(parents=True, exist_ok=True)

    best_file = out_dir / "cup_tracker_params_webots.json"

    active_file = out_dir / "cup_tracker_params.json"



    with open(best_file, "w", encoding="utf-8") as f:

        json.dump(study.best_params, f, indent=4)

    with open(active_file, "w", encoding="utf-8") as f:

        json.dump(study.best_params, f, indent=4)



    print("[SAVE] En iyi parametreler kaydedildi:")

    print(f"       1. {best_file.name}")

    print(f"       2. {active_file.name} (Aktif parametreler guncellendi)")



    # Analiz sonuclarini kaydet

    analysis_dir = PROJECT_ROOT / "basket" / "analysis"

    analysis_dir.mkdir(parents=True, exist_ok=True)

    res_file = analysis_dir / "webots_cup_tracker_trials.json"

    with open(res_file, "w", encoding="utf-8") as f:

        json.dump({

            "best_trial": study.best_trial.number,

            "best_value": study.best_value,

            "baseline_value": trial_history[0]["cost"],

            "improvement_pct": improvement_pct,

            "best_params": study.best_params,

            "trials": trial_history,

        }, f, indent=2)

    print(f"[SAVE] Analiz verisi kaydedildi: {res_file.name}")





if __name__ == "__main__":

    main()

