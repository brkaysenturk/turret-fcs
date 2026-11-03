#!/usr/bin/env python3
"""
================================================================================
MODUL: TURRET BASKET WEBOTS CAD 24 OPTUNA DENEMESI YAKINSAMA VE PERFORMANS ANALIZI
================================================================================
Bu betik; webots_cup_tracker_trials.json icindeki 24 denemenin tamamini okuyarak:
  1. python_fig6_optuna_24trials_convergence.png:
     - Minimax Robust Maliyet Egrisi (Trial 0: 3.60 -> Trial 14: 2.99, -%16.9)
     - Kararli Durum RMS Kilitlenme Hatasi (0.580° -> 0.452°, -%22.1)
     - 6 Farkli Mesafede Kilitli Kalma Oranlari (Ozellikle 2.6m: %76.1 -> %100.0)
     - Eyleyici Yon Degistirme / Titresim Orani (% Sign Flip Rate)
================================================================================
"""

import json
import sys
from pathlib import Path
import matplotlib.pyplot as plt

# Font ve stil ayarlari (Akademik yayin formati)
plt.rcParams["font.sans-serif"] = "DejaVu Sans"
plt.rcParams["axes.edgecolor"] = "#333333"
plt.rcParams["axes.linewidth"] = 0.8

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

FIG_DIR = PROJECT_ROOT / "turret_basket" / "analysis" / "figures" / "python"
FIG_DIR.mkdir(parents=True, exist_ok=True)

# Veri dosyasi konumlari (Hem analysis koku hem de comparison ici desteklenir)
DATA_FILE_LOCAL = BASE_DIR / "webots_cup_tracker_trials.json"
DATA_FILE_ROOT = PROJECT_ROOT / "turret_basket" / "analysis" / "webots_cup_tracker_trials.json"
DATA_FILE = DATA_FILE_LOCAL if DATA_FILE_LOCAL.exists() else DATA_FILE_ROOT


def plot_fig6_optuna_convergence():
    """Figure 6: 24-Trial Optuna Hiperparametre Optimizasyonu Yakınsama Grafiği."""
    if not DATA_FILE.exists():
        print(f"[HATA] Veri dosyasi bulunamadi: {DATA_FILE}")
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

    # Mesafelere gore kilitlenme ayrintilari
    lock_260 = [t.get("metrics", {}).get("260cm", {}).get("tail_lock_pct", 80.0) for t in trials]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.patch.set_facecolor("#ffffff")
    fig.suptitle("Turret Basket: Webots CAD Tareti 24 Optuna Denemesi Yakinsama ve Kararlilik Analizi\n"
                 "Baseline (Trial 0) vs Optimal Webots CAD Parametreleri (Trial 14)",
                 fontsize=13, fontweight="bold", y=0.98, color="#0f172a")

    # 1. Minimax Robust Maliyet Egrisi (Sol Ust)
    ax1 = axes[0, 0]
    ax1.set_facecolor("#fcfcfc")
    ax1.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    ax1.plot(trial_nums, costs, marker="o", markersize=5, color="#0284c7", linewidth=2.0, label="Deneme Maliyeti")
    ax1.scatter([0], [costs[0]], color="#dc2626", s=110, zorder=5, label="Trial 0 (Baseline: 3.60)")
    ax1.scatter([best_idx], [costs[best_idx]], color="#16a34a", s=140, zorder=6,
                label=f"Trial {best_idx} (Optimal: 2.99, -%16.9)")

    ax1.annotate(f"Optimal Nokta: Trial {best_idx}\nCost = {costs[best_idx]:.2f}",
                 xy=(best_idx, costs[best_idx]), xytext=(best_idx - 3.5, costs[best_idx] + 0.35),
                 arrowprops=dict(facecolor="#16a34a", shrink=0.08, width=1.5, headwidth=6),
                 fontsize=8.5, fontweight="bold", color="#15803d",
                 bbox=dict(boxstyle="round,pad=0.3", fc="#dcfce7", ec="#16a34a", lw=1.0))

    ax1.set_title("Minimax Robust Maliyet Gelisimi", fontsize=11, fontweight="bold", pad=8)
    ax1.set_xlabel("Optuna Deneme No", fontsize=9.5)
    ax1.set_ylabel("Maliyet [Dusuk Daha Iyi]", fontsize=9.5)
    ax1.legend(loc="upper right", fontsize=8.5, framealpha=0.92, edgecolor="#cbd5e1")

    # 2. RMS Aci Hatasi Gelisimi (Sag Ust)
    ax2 = axes[0, 1]
    ax2.set_facecolor("#fcfcfc")
    ax2.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    ax2.plot(trial_nums, rms_vals, marker="s", markersize=5, color="#ea580c", linewidth=2.0, label="Ortalama RMS Hatasi")
    ax2.scatter([0], [rms_vals[0]], color="#dc2626", s=110, zorder=5, label=f"Trial 0 ({rms_vals[0]:.3f}°)")
    ax2.scatter([best_idx], [rms_vals[best_idx]], color="#16a34a", s=140, zorder=6,
                label=f"Trial {best_idx} ({rms_vals[best_idx]:.3f}°, -%22.1)")

    ax2.set_title("Kararli Durum Kilitlenme RMS Hatasi (Derece)", fontsize=11, fontweight="bold", pad=8)
    ax2.set_xlabel("Optuna Deneme No", fontsize=9.5)
    ax2.set_ylabel("RMS Aci Hatasi [derece]", fontsize=9.5)
    ax2.legend(loc="upper right", fontsize=8.5, framealpha=0.92, edgecolor="#cbd5e1")

    # 3. Kilitli Kalma Orani ve 2.6m Zorlu Mesafe (Sol Alt)
    ax3 = axes[1, 0]
    ax3.set_facecolor("#fcfcfc")
    ax3.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    ax3.plot(trial_nums, lock_pcts, marker="^", markersize=5, color="#7c3aed", linewidth=1.8, label="Genel Ortalama Kilit (%)")
    ax3.plot(trial_nums, lock_260, marker="d", markersize=5, color="#2563eb", linestyle="--", linewidth=1.8,
             label="2.6m Zorlu Mesafe Kilit (%76.1 -> %100)")
    ax3.scatter([0], [lock_260[0]], color="#dc2626", s=90, zorder=5)
    ax3.scatter([best_idx], [lock_260[best_idx]], color="#16a34a", s=120, zorder=6)

    ax3.set_title("Kilitli Kalma Orani ve Mesafe Dayanikliligi", fontsize=11, fontweight="bold", pad=8)
    ax3.set_xlabel("Optuna Deneme No", fontsize=9.5)
    ax3.set_ylabel("Kilitli Sure Orani [%]", fontsize=9.5)
    ax3.set_ylim(70, 105)
    ax3.legend(loc="lower right", fontsize=8.5, framealpha=0.92, edgecolor="#cbd5e1")

    # 4. Eyleyici Yon Degistirme / Titresim Orani (Sag Alt)
    ax4 = axes[1, 1]
    ax4.set_facecolor("#fcfcfc")
    ax4.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    ax4.plot(trial_nums, flip_pcts, marker="v", markersize=5, color="#16a34a", linewidth=2.0, label="Sign Flip Orani (%)")
    ax4.scatter([0], [flip_pcts[0]], color="#dc2626", s=110, zorder=5, label=f"Trial 0 ({flip_pcts[0]:.2f}%)")
    ax4.scatter([best_idx], [flip_pcts[best_idx]], color="#0284c7", s=140, zorder=6,
                label=f"Trial {best_idx} ({flip_pcts[best_idx]:.2f}%)")

    ax4.axhline(2.0, color="#64748b", linestyle=":", label="Kararli Eşik (<= %2.0)")
    ax4.set_title("Komut Yon Degistirme / Mekanik Titresim Orani", fontsize=11, fontweight="bold", pad=8)
    ax4.set_xlabel("Optuna Deneme No", fontsize=9.5)
    ax4.set_ylabel("Sign Flip Rate [%]", fontsize=9.5)
    ax4.set_ylim(0, 4.0)
    ax4.legend(loc="upper right", fontsize=8.5, framealpha=0.92, edgecolor="#cbd5e1")

    plt.tight_layout()
    out_path = FIG_DIR / "python_fig6_optuna_24trials_convergence.png"
    plt.savefig(out_path, dpi=300, facecolor=fig.get_facecolor())
    plt.close()
    print(f"[FIG 6 OK] Kaydedildi: {out_path}")


if __name__ == "__main__":
    plot_fig6_optuna_convergence()
