"""
Tam Otomatik RL Egitim ve Benchmark Boru Hatti (14x150K Optuna + 250K Egitim + Disa Aktarim + Benchmark Grafigi)
"""

import sys
import time
from pathlib import Path

# Paths
current_dir = Path(__file__).resolve().parent
project_root = current_dir.parents[1]
for p in (str(current_dir), str(project_root)):
    if p not in sys.path:
        sys.path.insert(0, p)

from turret_fcs.rl_sim.optimize_rl import run_optimization
from turret_fcs.rl_sim.train_rl import train
from analysis.comparison.generate_benchmark_plots import generate_plots


def main():
    t_start = time.time()
    print("==================================================================", flush=True)
    print(">>> ADIM 1: OPTUNA TPE HIPERPARAMETRE TARAMASI (14 x 150.000 ADIM)", flush=True)
    print("==================================================================", flush=True)
    best_params = run_optimization(n_trials=14)
    print(f"\n[Optuna Tamamlandi] En Iyi Parametreler: {best_params}", flush=True)

    print("\n==================================================================", flush=True)
    print(">>> ADIM 2: 250.000 ADIM NIHAI PPO MODELI EGITIMI", flush=True)
    print("==================================================================", flush=True)
    train(timesteps=250_000)
    print("[Egitim Tamamlandi] Model ve NPZ politikalari guncellendi.", flush=True)

    print("\n==================================================================", flush=True)
    print(">>> ADIM 3: P vs RL KARSILASTIRMALI BENCHMARK GRAFIGI URETIMI", flush=True)
    print("==================================================================", flush=True)
    try:
        generate_plots()
        print("[Benchmark Tamamlandi] fig11_p_vs_rl_final_benchmark.png guncellendi.", flush=True)
    except Exception as exc:
        print(f"[Uyari: Benchmark grafigi olusturulurken hata]: {exc}", flush=True)

    total_time = time.time() - t_start
    print("\n==================================================================", flush=True)
    print(f" [TAMAMLANDI] RL BORU HATTI BASARIYLA BITTI! Toplam Sure: {total_time/60:.1f} dakika", flush=True)
    print("==================================================================", flush=True)


if __name__ == "__main__":
    main()
