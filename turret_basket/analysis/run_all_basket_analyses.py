#!/usr/bin/env python3
"""
================================================================================
MODUL: TURRET BASKET TUM ANALIZLERI CALISTIRICI (MASTER RUNNER)
================================================================================
Bu betik; Turret Basket icin tanimlanan tum aerodinamik, balistik, kontrolcu
ve karsilastirma grafik motorlarini sira ile calistirarak 300 DPI akademik
figurlerin tamamini 'analysis/figures/python/' altinda gunceller.
================================================================================
"""

import sys
import subprocess
from pathlib import Path

ANALYSIS_DIR = Path(__file__).resolve().parent

SCRIPTS = [
    ANALYSIS_DIR / "system" / "python" / "plot_ballistics_rk4.py",
    ANALYSIS_DIR / "controllers" / "ballistics" / "plot_rl_ballistics.py",
    ANALYSIS_DIR / "system" / "python" / "plot_engagement_profile.py",
    ANALYSIS_DIR / "controllers" / "tracker" / "plot_cup_tracker.py",
    ANALYSIS_DIR / "comparison" / "plot_optuna_trials.py",
    ANALYSIS_DIR / "comparison" / "plot_baseline_vs_24_basket_table.py",
]


def main():
    print("=" * 80)
    print("TURRET BASKET AKADEMIK GRAFIK VE ANALIZ MOTORU BASLATILIYOR")
    print("=" * 80)

    success_count = 0
    for script in SCRIPTS:
        rel_path = script.relative_to(ANALYSIS_DIR)
        print(f"\n[CALISTIRILIYOR] {rel_path} ...")
        res = subprocess.run([sys.executable, str(script)], capture_output=True, text=True)
        if res.returncode == 0:
            print(f"[BASARILI] {rel_path}")
            if res.stdout.strip():
                for line in res.stdout.strip().splitlines():
                    print(f"  -> {line}")
            success_count += 1
        else:
            print(f"[HATA] {rel_path} basarisiz oldu:")
            print(res.stderr)

    print("\n" + "=" * 80)
    print(f"OZET: {success_count}/{len(SCRIPTS)} analiz betigi basariyla tamamlandi.")
    print(f"Figurler: {ANALYSIS_DIR / 'figures' / 'python'}")
    print("=" * 80)


if __name__ == "__main__":
    main()
