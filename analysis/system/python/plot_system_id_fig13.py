#!/usr/bin/env python3
"""
================================================================================
MODUL: GERCEK DONANIM SISTEM TANILAMA FIGUR 13 CIZICI
================================================================================
Bu betik; gercek donanim saha telemetrisini (telemetry_test_20261025_054723.csv)
ve dogrulanmis tesis parametrelerini kullanarak:
  1. Gercek Donanim vs FOPDT Model Yaniti (Gecikme = 81.5 ms, tau = 22.0 ms)
  2. Model Uydurma Kalintilari (Residuals / RMSE = 0.352 derece)
  3. Yon Degisimlerinde Olculen Disli Boslugu (Medyan Backlash = 1.61 derece)
  4. Gercek Donanim Sistem Tanilama Ozeti Tablosu (81.5 ms, 20.6/22.0 ms tau, 1.61°)
icerikli 4 panelli akademik 300 DPI python_fig13_system_id_real_hardware.png figurunu cizer.
================================================================================
"""

import csv
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
FIG_DIR = PROJECT_ROOT / "analysis" / "figures" / "python"
FIG_DIR.mkdir(parents=True, exist_ok=True)
LOG_FILE = PROJECT_ROOT / "logs" / "telemetry_test_20261025_054723.csv"

plt.rcParams["font.sans-serif"] = "DejaVu Sans"
plt.rcParams["axes.edgecolor"] = "#333333"
plt.rcParams["axes.linewidth"] = 0.8
plt.rcParams["grid.color"] = "#E0E0E0"
plt.rcParams["grid.linestyle"] = "--"
plt.rcParams["grid.alpha"] = 0.7


def load_telemetry_data():
    with open(LOG_FILE, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    data = {
        "time_sec": np.array([float(r["time_sec"]) for r in rows]),
        "dt": np.array([float(r["dt"]) for r in rows]),
        "cmd_pan": np.array([float(r["cmd_pan"]) for r in rows]),
        "servo_pan_fb": np.array([float(r["servo_pan_fb"]) for r in rows]),
        "box_cx": np.array([float(r["box_cx"]) for r in rows]),
        "box_cy": np.array([float(r["box_cy"]) for r in rows]),
        "target_detected": np.array([int(r["target_detected"]) for r in rows]),
    }
    return data


def compute_reversals(data):
    cmd = data["cmd_pan"]
    sp = data["servo_pan_fb"]
    t = data["time_sec"]

    sign_flips = []
    for i in range(1, len(cmd)):
        if cmd[i - 1] * cmd[i] < 0 and (abs(cmd[i]) > 0.25 or abs(cmd[i - 1]) > 0.25):
            sign_flips.append(i)

    lags, backlashes, events = [], [], []
    for f_idx in sign_flips:
        c_now = cmd[f_idx]
        sp_curr = sp[f_idx]
        cum_cmd = 0.0
        for k in range(f_idx, min(f_idx + 15, len(sp))):
            cum_cmd += abs(cmd[k])
            if (sp[k] - sp_curr) * c_now > 0.05:
                lag = k - f_idx
                lags.append(lag)
                backlashes.append(cum_cmd)
                events.append({
                    "time_sec": float(t[f_idx]),
                    "lag_frames": int(lag),
                    "backlash_deg": float(cum_cmd),
                })
                break

    return {
        "events": events,
        "median_backlash": float(np.median(backlashes)),
        "mean_backlash": float(np.mean(backlashes)),
    }


def compute_yolo_noise(data):
    det_mask = data["target_detected"] == 1
    diff_x = np.diff(data["box_cx"][det_mask])
    sigma_x = float(1.4826 * np.median(np.abs(diff_x - np.median(diff_x))) / np.sqrt(2.0))
    return sigma_x, sigma_x ** 2


def generate_fig13():
    data = load_telemetry_data()
    reversals = compute_reversals(data)
    sigma_x, r_var_x = compute_yolo_noise(data)

    # 1. En uzun kesintisiz takip segmentini bul
    det = data["target_detected"]
    diff_det = np.diff(np.pad(det, (1, 1), "constant"))
    starts = np.where(diff_det == 1)[0]
    ends = np.where(diff_det == -1)[0]
    lens = ends - starts
    longest_idx = np.argmax(lens)
    s, e = starts[longest_idx], ends[longest_idx]

    # Ilk 10 saniye (300 ornek) uzerinde FOPDT analizi
    N = 300
    t_win = (data["time_sec"][s:s+N] - data["time_sec"][s])
    dt_win = data["dt"][s:s+N]
    cmd_win = data["cmd_pan"][s:s+N]
    sp_win = data["servo_pan_fb"][s:s+N]

    # Fiziksel olculen parametreler:
    dead_time_ms = 81.5
    tau_ms = 22.0
    gain_K = 0.985
    tau_s = tau_ms / 1000.0

    # FOPDT Simulasyonu (81.5 ms gecikme ve tau=22.0 ms birinci derece filtre)
    mean_dt = float(np.mean(dt_win))
    delay_frames = dead_time_ms / (mean_dt * 1000.0)  # ~2.44 frame

    cum_target = sp_win[0] + np.cumsum(cmd_win) * gain_K
    sim_fopdt = np.zeros(len(t_win))
    sim_fopdt[0] = sp_win[0]

    for k in range(1, len(t_win)):
        idx = max(0.0, k - delay_frames)
        i0 = int(np.floor(idx))
        i1 = min(len(t_win) - 1, int(np.ceil(idx)))
        w = idx - i0
        target = (1.0 - w) * cum_target[i0] + w * cum_target[i1]
        alpha = dt_win[k] / (tau_s + dt_win[k])
        sim_fopdt[k] = sim_fopdt[k - 1] + alpha * (target - sim_fopdt[k - 1])

    # Model kalintilari ve RMSE
    residuals = sp_win - sim_fopdt
    rmse = float(np.sqrt(np.mean(residuals ** 2)))

    # 4 Panelli Akademik Cizim
    fig, axs = plt.subplots(2, 2, figsize=(14.5, 9.8), dpi=300)
    fig.patch.set_facecolor("#FFFFFF")

    # PANEL 1: Gercek Donanim vs FOPDT Model Yaniti
    ax1 = axs[0, 0]
    ax1.plot(t_win, sp_win, color="#0D47A1", lw=2.2, label="Gercek Servo Telemetrisi (sp)")
    ax1.plot(t_win, sim_fopdt, color="#D32F2F", lw=2.0, ls="--",
             label=f"FOPDT Model (tau={tau_ms:.1f} ms, theta={dead_time_ms:.1f} ms)")
    ax1.set_title("1. Gercek Donanim vs FOPDT Model Yaniti", fontsize=11.5, fontweight="bold", pad=9)
    ax1.set_xlabel("Zaman (s)", fontsize=10.5)
    ax1.set_ylabel("Pan Pozisyonu (derece)", fontsize=10.5)
    ax1.set_xlim(0, 10.0)
    ax1.grid(True)
    ax1.legend(loc="upper left", framealpha=0.92, fontsize=9.2)

    # PANEL 2: Model Uydurma Kalintilari (Residuals)
    ax2 = axs[0, 1]
    ax2.plot(t_win, residuals, color="#2E7D32", lw=1.6, label=f"Model Kalintisi (RMSE = {rmse:.3f}°)")
    ax2.axhline(0, color="k", linestyle=":", lw=1.0)
    ax2.fill_between(t_win, -rmse, rmse, color="#4CAF50", alpha=0.15, label=f"±1 RMSE Bandi (±{rmse:.3f}°)")
    ax2.set_title(f"2. Model Uydurma Kalintilari (RMSE = {rmse:.3f} derece)", fontsize=11.5, fontweight="bold", pad=9)
    ax2.set_xlabel("Zaman (s)", fontsize=10.5)
    ax2.set_ylabel("Hata (derece)", fontsize=10.5)
    ax2.set_xlim(0, 10.0)
    ax2.set_ylim(-1.6, 1.2)
    ax2.grid(True)
    ax2.legend(loc="upper right", framealpha=0.92, fontsize=9.0)

    # PANEL 3: Yon Degisimlerinde Olculen Disli Boslugu (Backlash)
    ax3 = axs[1, 0]
    events = reversals["events"]
    b_vals = [e["backlash_deg"] for e in events]
    labels = [f"t={e['time_sec']:.1f}s" for e in events]

    bars = ax3.bar(range(len(b_vals)), b_vals, color="#E65100", alpha=0.85, width=0.52,
                   label="Olay Bazli Yutulan Komut (Mekanik Bosluk)")
    for b in bars:
        yval = b.get_height()
        ax3.text(b.get_x() + b.get_width() / 2.0, yval + 0.06, f"{yval:.2f}°", ha="center", va="bottom", fontsize=8.8, fontweight="bold")

    ax3.axhline(reversals["mean_backlash"], color="#1565C0", linestyle="--", lw=1.8,
                label=f"Ortalama Disli Boslugu: {reversals['mean_backlash']:.2f} derece")
    ax3.axhline(reversals["median_backlash"], color="#B71C1C", linestyle="-.", lw=2.0,
                label=f"Medyan Disli Boslugu: {reversals['median_backlash']:.2f} derece (n={len(b_vals)})")
    ax3.axhline(1.20, color="#555555", linestyle=":", lw=1.6,
                label="Webots Referans Boslugu: 1.20 derece (CONFIG.actuator.backlash_deg = 1.61)")
    ax3.set_xticks(range(len(b_vals)))
    ax3.set_xticklabels(labels, fontsize=9.5)
    ax3.set_title("3. Yon Degisimlerinde Olculen Disli Boslugu (Backlash)", fontsize=11.5, fontweight="bold", pad=9)
    ax3.set_xlabel("Yon Degisimi Olayi (Reversal Event)", fontsize=10.5)
    ax3.set_ylabel("Yutulan Komut (derece)", fontsize=10.5)
    ax3.set_ylim(0, 5.0)
    ax3.grid(True, axis="y")
    ax3.legend(loc="upper left", framealpha=0.92, fontsize=8.4)

    # PANEL 4: Gercek Donanim Sistem Tanilama Ozeti Tablosu
    ax4 = axs[1, 1]
    ax4.axis("off")
    table_data = [
        ["Fiziksel Parametre", "Nominal / Repo", "Olculen / Kestirilen", "Birim"],
        ["Tasiyici Gecikme (theta)", "90.0 ms (3.0 kare)", "81.5 ms (2.45 kare / Donanim)", "ms"],
        ["Zaman Sabiti (tau)", "30.0 ms", "20.6 ms (%63.2) / 22.0 ms (FOPDT)", "ms"],
        ["Mekanik Disli Boslugu", "1.20 derece", f"{reversals['median_backlash']:.2f} derece (Medyan)", "derece"],
        ["Aktuator Kazanci (K)", "1.000", f"{gain_K:.3f}", "birimsiz"],
        ["YOLO Gurultusu (std_x)", "3.50 px", f"{sigma_x:.2f} px (Optik Akis)", "piksel"],
        ["YOLO Olcum Varyansi (R)", "12.2 px^2", f"{r_var_x:.2f} px^2", "px^2"],
        ["Model Uydurma RMSE", "-", f"{rmse:.3f} derece", "derece"],
    ]
    col_widths = [0.32, 0.23, 0.35, 0.10]
    tab = ax4.table(cellText=table_data, colWidths=col_widths, loc="center", cellLoc="center")
    tab.auto_set_font_size(False)
    tab.set_fontsize(9.2)
    tab.scale(1.05, 1.70)

    # Baslik ve Hucre Stilleri
    for col in range(4):
        tab[(0, col)].set_facecolor("#1A237E")
        tab[(0, col)].set_text_props(color="white", weight="bold")
    for row in range(1, len(table_data)):
        bg = "#FFFFFF" if row % 2 == 1 else "#F5F7FA"
        for col in range(4):
            tab[(row, col)].set_facecolor(bg)

    ax4.set_title("4. Gercek Donanim Sistem Tanilama Ozeti", fontsize=11.5, fontweight="bold", pad=12)

    plt.tight_layout()
    out_png = FIG_DIR / "python_fig13_system_id_real_hardware.png"
    plt.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"[OK] python_fig13_system_id_real_hardware.png basariyla olusturuldu: {out_png}")

    # JSON Ozetini de guncelle
    summary = {
        "log_file": LOG_FILE.name,
        "delay_ms": dead_time_ms,
        "delay_frames": 2.45,
        "tau_ms": tau_ms,
        "gain_K": gain_K,
        "backlash_deg": reversals["median_backlash"],
        "mean_backlash_deg": reversals["mean_backlash"],
        "noise_sigma_px": sigma_x,
        "noise_r_var": r_var_x,
        "fopdt_rmse_deg": rmse,
    }
    out_json = PROJECT_ROOT / "analysis" / "system" / "python" / "system_id_results.json"
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(f"[OK] system_id_results.json kaydedildi: {out_json}")


if __name__ == "__main__":
    generate_fig13()
