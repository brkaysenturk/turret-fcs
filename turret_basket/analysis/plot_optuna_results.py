#!/usr/bin/env python3
"""
================================================================================
BASKET CHALLENGE: WEBOTS OPTUNA 12-TRIAL SONUCLARI GRAFIKLEME
================================================================================
Baseline (Trial 0) ile 12 deneme boyunca elde edilen gelisimi gosterir:
1. Maliyet (Robust Cost) Gelisim Egrisi
2. Kararli Durum RMS Hatasi Karsilastirmasi
3. Kilitli Kalma Orani (%)
4. Komut Titresim / Yon Degistirme Orani (%)
================================================================================
"""

import json
from pathlib import Path
import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DATA_FILE = PROJECT_ROOT / "basket" / "analysis" / "webots_cup_tracker_trials.json"
OUTPUT_PNG = PROJECT_ROOT / "basket" / "analysis" / "cup_tracker_trial_comparison.png"


def main():
    if not DATA_FILE.exists():
        print(f"[ERROR] Veri dosyasi bulunamadi: {DATA_FILE}")
        return

    with open(DATA_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)

    trials = data["trials"]
    trial_nums = [t["trial"] for t in trials]
    costs = [t["cost"] for t in trials]
    rms_vals = [t["avg_rms"] for t in trials]
    lock_pcts = [t["avg_lock"] for t in trials]
    flip_pcts = [t["avg_flip"] for t in trials]

    best_idx = data["best_trial"]

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), facecolor="#0e1117")
    fig.suptitle("Webots Cup Tracker (1. Model) - 12 Optuna Denemesi & Sim-to-Real Kazanci", 
                 fontsize=15, fontweight="bold", color="white", y=0.98)

    # 1. Cost Egrisi
    ax1 = axes[0, 0]
    ax1.set_facecolor("#161b22")
    ax1.plot(trial_nums, costs, marker="o", color="#38bdf8", linewidth=2.0, label="Trial Cost")
    ax1.scatter([0], [costs[0]], color="#facc15", s=130, zorder=5, label="Trial 0 (Simulator Baseline)")
    ax1.scatter([best_idx], [costs[best_idx]], color="#4ade80", s=150, zorder=5, label=f"Best Trial #{best_idx}")
    ax1.set_title("Robust Maliyet (Minimax)", color="white", fontsize=12)
    ax1.set_xlabel("Deneme No", color="#cbd5e1")
    ax1.set_ylabel("Maliyet (Dusuk Daha Iyi)", color="#cbd5e1")
    ax1.tick_params(colors="#94a3b8")
    ax1.grid(True, alpha=0.15)
    ax1.legend(facecolor="#1e293b", edgecolor="#334155", labelcolor="white")

    # 2. RMS Hatasi
    ax2 = axes[0, 1]
    ax2.set_facecolor("#161b22")
    ax2.plot(trial_nums, rms_vals, marker="s", color="#f43f5e", linewidth=2.0)
    ax2.scatter([0], [rms_vals[0]], color="#facc15", s=130, zorder=5)
    ax2.scatter([best_idx], [rms_vals[best_idx]], color="#4ade80", s=150, zorder=5)
    ax2.set_title("Kararli Durum Kilitlenme RMS Hatasi (Derece)", color="white", fontsize=12)
    ax2.set_xlabel("Deneme No", color="#cbd5e1")
    ax2.set_ylabel("RMS (Derece)", color="#cbd5e1")
    ax2.tick_params(colors="#94a3b8")
    ax2.grid(True, alpha=0.15)

    # 3. Kilit Orani (%)
    ax3 = axes[1, 0]
    ax3.set_facecolor("#161b22")
    ax3.plot(trial_nums, lock_pcts, marker="^", color="#a855f7", linewidth=2.0)
    ax3.scatter([0], [lock_pcts[0]], color="#facc15", s=130, zorder=5)
    ax3.scatter([best_idx], [lock_pcts[best_idx]], color="#4ade80", s=150, zorder=5)
    ax3.set_title("Kilitli Kalma Orani (%)", color="white", fontsize=12)
    ax3.set_xlabel("Deneme No", color="#cbd5e1")
    ax3.set_ylabel("Kilitli Sure (%)", color="#cbd5e1")
    ax3.tick_params(colors="#94a3b8")
    ax3.grid(True, alpha=0.15)

    # 4. Yon Degistirme / Titresim (%)
    ax4 = axes[1, 1]
    ax4.set_facecolor("#161b22")
    ax4.plot(trial_nums, flip_pcts, marker="d", color="#fb923c", linewidth=2.0)
    ax4.scatter([0], [flip_pcts[0]], color="#facc15", s=130, zorder=5)
    ax4.scatter([best_idx], [flip_pcts[best_idx]], color="#4ade80", s=150, zorder=5)
    ax4.set_title("Komut Yon Degistirme / Titresim Orani (%)", color="white", fontsize=12)
    ax4.set_xlabel("Deneme No", color="#cbd5e1")
    ax4.set_ylabel("Sign Flip Rate (%)", color="#cbd5e1")
    ax4.tick_params(colors="#94a3b8")
    ax4.grid(True, alpha=0.15)

    plt.tight_layout()
    plt.savefig(OUTPUT_PNG, dpi=160, facecolor=fig.get_facecolor())
    print(f"[PLOT] Grafik kaydedildi: {OUTPUT_PNG}")


if __name__ == "__main__":
    main()

