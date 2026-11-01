#!/usr/bin/env python3
"""
Mevcut Baseline ve Yeni Webots Optuna parametrelerini tüm balon profillerinde
(0: Statik, 1: Yavaş Sinüs, 2: Standart Sinüs, 3: Çevik Sinüs, 4: Step/Hold, 5: Zigzag, 7: Figure-8, 8: Köşe Giriş)
çalıştırıp karşılaştırma matrisini çıkaran test betiği.
"""

import csv
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "analysis" / "comparison") not in sys.path:
    sys.path.insert(0, str(ROOT / "analysis" / "comparison"))

import optimize_webots_corner_lock as opt

PROFILES = [0, 1, 2, 3, 4, 5, 7, 8]
SIM_TIME = 12.0  # Her profil için 12 saniye koşu

# Optuna sonuçlarını oku
p_res = json.loads((ROOT / "analysis" / "comparison" / "webots_optuna_p_results.json").read_text(encoding="utf-8"))
rl_res = json.loads((ROOT / "analysis" / "comparison" / "webots_optuna_rl_results.json").read_text(encoding="utf-8"))

p_base = p_res["baseline_params"]
p_opt = p_res["best_params"]

rl_base = rl_res["baseline_params"]
rl_opt = rl_res["best_params"]

def benchmark_all():
    print("=" * 80)
    print("  TURRET FCS // TUM PROFILLERDE DETAYLI KARSILASTIRMA TESTI (WEBOTS)")
    print("=" * 80)
    
    results = []

    configs = [
        ("P", "Baseline", p_base),
        ("P", "Optuna", p_opt),
        ("RL", "Baseline", rl_base),
        ("RL", "Optuna", rl_opt),
    ]

    for mode, tag, params in configs:
        print(f"\n---> Test Ediliyor: Mod={mode} | Versiyon={tag}")
        for prof in PROFILES:
            t0 = time.time()
            cost, metrics = opt.run_webots_eval(
                mode=mode,
                params=params,
                label=f"all_{mode.lower()}_{tag.lower()}_p{prof}",
                profiles=(prof,),
                sim_time_sec=SIM_TIME,
                port=1298
            )
            dur = time.time() - t0
            m = metrics.get(f"prof_{prof}", {})
            row = {
                "Mode": mode,
                "Version": tag,
                "Profile": prof,
                "Cost": m.get("cost", 999.0),
                "Tail_RMS_deg": m.get("tail_rms_deg", 99.0),
                "Lock_Pct": m.get("lock_pct", 0.0),
                "Flip_Pct": m.get("flip_pct", 0.0),
                "Overshoot_deg": m.get("max_overshoot_deg", 0.0),
                "Duration_s": round(dur, 1)
            }
            results.append(row)
            print(f"  [Profil {prof}] ({dur:.1f}s) Cost: {row['Cost']:<6.2f} | RMS: {row['Tail_RMS_deg']:<5.3f} deg | Lock: %{row['Lock_Pct']:<5.1f} | Flip: %{row['Flip_Pct']:<5.2f}")

    out_csv = ROOT / "analysis" / "comparison" / "all_profiles_benchmark.csv"
    if results:
        with open(out_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            writer.writeheader()
            writer.writerows(results)
    print("\n" + "=" * 80)
    print(f"[+] Tum testler tamamlandi ve kaydedildi: {out_csv}")
    print("=" * 80)

if __name__ == "__main__":
    benchmark_all()
