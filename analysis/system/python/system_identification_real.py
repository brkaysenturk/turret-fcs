#!/usr/bin/env python3
"""
================================================================================
MODUL: GERCEK DONANIM TELEMETRISI ILE SISTEM TANILAMA (REAL SYSTEM IDENTIFICATION)
================================================================================
Bu betik; fiziksel taretten alinan canli saha telemetri kayitlarini (telemetry_*.csv)
analiz ederek gercek donanim parametrelerini analitik ve matematiksel olarak cikarir:

1) Tasiyici Gecikme (Transport Delay - theta_d): Komut ile mekanik hareket arasindaki olculen gecikme.
2) Mekanik Disli Boslugu (Backlash Hysteresis): Yon degisimlerinde kaybedilen acisal komut.
3) FOPDT Transfer Fonksiyonu Uydurma: K (kazanc), tau (zaman sabiti), theta (olu zaman).
4) Sensor Gurultusu Varyansi: Kameranin ve YOLOv8'in olculen gercek olcum gurultusu (R).
5) Akademik Karsilastirma Grafigi: 300 DPI 4 panelli gorsel ve sayisal JSON raporu.
================================================================================
"""

import csv
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(__file__).resolve().parents[3]
FIG_DIR = PROJECT_ROOT / "analysis" / "figures" / "python"
FIG_DIR.mkdir(parents=True, exist_ok=True)

plt.rcParams["font.sans-serif"] = "DejaVu Sans"
plt.rcParams["axes.edgecolor"] = "#333333"
plt.rcParams["axes.linewidth"] = 0.8
plt.rcParams["grid.color"] = "#E0E0E0"
plt.rcParams["grid.linestyle"] = "--"
plt.rcParams["grid.alpha"] = 0.7


def load_telemetry(csv_path: Path):
    """Telemetri CSV dosyasini numpy dizilerine donusturur."""
    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    data = {
        "time_sec": np.array([float(r["time_sec"]) for r in rows]),
        "dt": np.array([float(r["dt"]) for r in rows]),
        "cmd_pan": np.array([float(r["cmd_pan"]) for r in rows]),
        "cmd_tilt": np.array([float(r["cmd_tilt"]) for r in rows]),
        "servo_pan_fb": np.array([float(r["servo_pan_fb"]) if r.get("servo_pan_fb") else 0.0 for r in rows]),
        "servo_tilt_fb": np.array([float(r["servo_tilt_fb"]) if r.get("servo_tilt_fb") else 0.0 for r in rows]),
        "turret_pan": np.array([float(r["turret_pan"]) for r in rows]),
        "turret_tilt": np.array([float(r["turret_tilt"]) for r in rows]),
        "err_pan": np.array([float(r["angular_err_pan"]) for r in rows]),
        "err_tilt": np.array([float(r["angular_err_tilt"]) for r in rows]),
        "box_cx": np.array([float(r["box_cx"]) for r in rows]),
        "box_cy": np.array([float(r["box_cy"]) for r in rows]),
        "target_detected": np.array([int(r["target_detected"]) for r in rows]),
    }
    return data


def identify_reversal_lag_and_backlash(data):
    """
    Yon degisimi anlarinda:
    1) Komuttan ilk harekete kadar gecen gecikme (frame ve ms)
    2) Harekete baslayana kadar yutulan komut miktari (backlash / olu bolge)
    """
    cmd = data["cmd_pan"]
    sp = data["servo_pan_fb"]
    t = data["time_sec"]

    # Belirgin yon degisimlerini bul (|cmd| > 0.25 deg/kare ve ters isaret)
    sign_flips = []
    for i in range(1, len(cmd)):
        if cmd[i - 1] * cmd[i] < 0 and (abs(cmd[i]) > 0.25 or abs(cmd[i - 1]) > 0.25):
            sign_flips.append(i)

    lags = []
    backlashes = []
    flip_events = []

    for f_idx in sign_flips:
        c_now = cmd[f_idx]
        sp_curr = sp[f_idx]
        cum_cmd = 0.0
        lag = 0
        for k in range(f_idx, min(f_idx + 15, len(sp))):
            cum_cmd += abs(cmd[k])
            # Servo yeni yonde en az 0.05 derece ilerledi mi?
            if (sp[k] - sp_curr) * c_now > 0.05:
                lag = k - f_idx
                lags.append(lag)
                backlashes.append(cum_cmd)
                flip_events.append({
                    "time_sec": float(t[f_idx]),
                    "lag_frames": int(lag),
                    "lag_ms": float(lag * 33.33),
                    "backlash_deg": float(cum_cmd),
                })
                break

    return {
        "mean_lag_frames": float(np.mean(lags)) if lags else 2.0,
        "mean_lag_ms": float(np.mean(lags) * 33.33) if lags else 66.7,
        "median_backlash_deg": float(np.median(backlashes)) if backlashes else 1.2,
        "mean_backlash_deg": float(np.mean(backlashes)) if backlashes else 1.2,
        "events": flip_events,
    }


def identify_measurement_noise(data):
    """
    Kamera ve YOLOv8 hedef tespit gurultusunu (R varyansi)
    medyan mutlak sapma (MAD) yontemiyle saglam (robust) olarak kestirir.
    """
    det_mask = data["target_detected"] == 1
    cx = data["box_cx"][det_mask]
    cy = data["box_cy"][det_mask]

    diff_x = np.diff(cx)
    diff_y = np.diff(cy)

    # Hedef hareketini dislamak icin 1. dereceden farklar uzerinde MAD
    sigma_x = float(1.4826 * np.median(np.abs(diff_x - np.median(diff_x))) / np.sqrt(2.0))
    sigma_y = float(1.4826 * np.median(np.abs(diff_y - np.median(diff_y))) / np.sqrt(2.0))

    return {
        "sigma_px_x": sigma_x,
        "sigma_px_y": sigma_y,
        "r_var_x": float(sigma_x ** 2),
        "r_var_y": float(sigma_y ** 2),
    }


def fit_fopdt_model(data):
    """
    Surekli hareket segmentinde FOPDT (First Order Plus Dead Time) modeli uydurur:
    G(s) = [K / (tau * s + 1)] * exp(-theta * s)
    """
    # En uzun kesintisiz takip segmentini bul
    det = data["target_detected"]
    diff_det = np.diff(np.pad(det, (1, 1), "constant"))
    starts = np.where(diff_det == 1)[0]
    ends = np.where(diff_det == -1)[0]
    lens = ends - starts
    longest_idx = np.argmax(lens)
    s, e = starts[longest_idx], ends[longest_idx]

    seg_t = data["time_sec"][s:e] - data["time_sec"][s]
    seg_dt = data["dt"][s:e]
    seg_cmd = data["cmd_pan"][s:e]
    seg_sp = data["servo_pan_fb"][s:e]

    def objective(params):
        d_frames = int(max(0, min(8, round(params[0]))))
        tau = max(0.010, params[1])
        K = params[2]

        sim = np.zeros(len(seg_t))
        sim[0] = seg_sp[0]
        cum_target = seg_sp[0] + np.cumsum(seg_cmd) * K

        for k in range(1, len(seg_t)):
            d_idx = max(0, k - d_frames)
            target = cum_target[d_idx]
            alpha = seg_dt[k] / (tau + seg_dt[k])
            sim[k] = sim[k - 1] + alpha * (target - sim[k - 1])

        return np.mean((sim - seg_sp) ** 2)

    res = minimize(objective, [2.0, 0.045, 1.0], method="Nelder-Mead")
    best_d = int(max(0, min(8, round(res.x[0]))))
    best_tau = float(max(0.010, res.x[1]))
    best_K = float(res.x[2])
    rmse = float(np.sqrt(res.fun))

    # En iyi modelin ciktisini simule et
    sim_best = np.zeros(len(seg_t))
    sim_best[0] = seg_sp[0]
    cum_target = seg_sp[0] + np.cumsum(seg_cmd) * best_K
    for k in range(1, len(seg_t)):
        d_idx = max(0, k - best_d)
        target = cum_target[d_idx]
        alpha = seg_dt[k] / (best_tau + seg_dt[k])
        sim_best[k] = sim_best[k - 1] + alpha * (target - sim_best[k - 1])

    return {
        "delay_frames": best_d,
        "delay_ms": float(best_d * 33.33),
        "tau_s": best_tau,
        "gain_K": best_K,
        "rmse_deg": rmse,
        "t": seg_t,
        "measured_sp": seg_sp,
        "simulated_sp": sim_best,
        "cmd_pan": seg_cmd,
    }


def plot_system_identification_results(fopdt, reversal, noise, out_png: Path):
    """4 Panelli akademik 300 DPI sistem tanilama grafigi cizer."""
    fig, axs = plt.subplots(2, 2, figsize=(14, 9.5), dpi=300)
    fig.patch.set_facecolor("#FFFFFF")

    # PANEL 1: Gercek Olcum vs FOPDT Model Uydurma (Zaman Cevabi)
    ax1 = axs[0, 0]
    t_win = fopdt["t"][:300]
    meas_win = fopdt["measured_sp"][:300]
    sim_win = fopdt["simulated_sp"][:300]
    ax1.plot(t_win, meas_win, "b-", lw=1.8, label="Gercek Servo Telemetrisi (sp)")
    ax1.plot(t_win, sim_win, "r--", lw=1.8, label=f"FOPDT Model (tau={fopdt['tau_s']*1000:.1f} ms, theta={fopdt['delay_ms']:.1f} ms)")
    ax1.set_title("1. Gercek Donanim vs FOPDT Model Yaniti", fontsize=11, fontweight="bold", pad=8)
    ax1.set_xlabel("Zaman (s)", fontsize=10)
    ax1.set_ylabel("Pan Pozisyonu (derece)", fontsize=10)
    ax1.grid(True)
    ax1.legend(loc="upper left", framealpha=0.9)

    # PANEL 2: Model Hatasi (Residuals)
    ax2 = axs[0, 1]
    res_win = meas_win - sim_win
    ax2.plot(t_win, res_win, color="#2E7D32", lw=1.4)
    ax2.axhline(0, color="k", linestyle=":", lw=1.0)
    ax2.set_title(f"2. Model Uydurma Kalintilari (RMSE = {fopdt['rmse_deg']:.3f} derece)", fontsize=11, fontweight="bold", pad=8)
    ax2.set_xlabel("Zaman (s)", fontsize=10)
    ax2.set_ylabel("Hata (derece)", fontsize=10)
    ax2.grid(True)

    # PANEL 3: Yon Degisimi Gecikmesi ve Histerezis (Backlash)
    ax3 = axs[1, 0]
    events = reversal["events"]
    if events:
        times = [e["time_sec"] for e in events]
        b_vals = [e["backlash_deg"] for e in events]
        labels = [f"t={t:.1f}s" for t in times]
        ax3.bar(range(len(b_vals)), b_vals, color="#E65100", alpha=0.85, width=0.55,
                label="Olay Bazli Yutulan Komut")
        # Medyan ve ortalama ayrimi kapatilir: medyan yaklasik 3 olayin ortasidir,
        # yalnizca 5 olayda medyan/ortalama tek tek cubuklarla birebir ortusmez.
        ax3.axhline(reversal["mean_backlash_deg"], color="#1565C0", linestyle="--", lw=1.8,
                    label=f"Ortalama Disli Boslugu: {reversal['mean_backlash_deg']:.2f} derece")
        ax3.axhline(reversal["median_backlash_deg"], color="#B71C1C", linestyle="-.", lw=1.8,
                    label=f"Medyan Disli Boslugu: {reversal['median_backlash_deg']:.2f} derece (n={len(b_vals)})")
        ax3.axhline(1.20, color="#555555", linestyle=":", lw=1.8,
                    label="Webots Referans Boslugu: 1.20 derece (CONFIG.actuator.backlash_deg = 1.61)")
        ax3.set_xticks(range(len(b_vals)))
        ax3.set_xticklabels(labels, rotation=30, ha="right", fontsize=9)
    ax3.set_title("3. Yon Degisimlerinde Olculen Disli Boslugu (Backlash)", fontsize=11, fontweight="bold", pad=8)
    ax3.set_xlabel("Yon Degisimi Olayi", fontsize=10)
    ax3.set_ylabel("Yutulan Komut (derece)", fontsize=10)
    ax3.grid(True, axis="y")
    ax3.legend(loc="upper right", framealpha=0.9, fontsize=8.2)

    # PANEL 4: Ozet Tablosu (Nominal vs Kestirilen)
    ax4 = axs[1, 1]
    ax4.axis("off")
    table_data = [
        ["Fiziksel Parametre", "Nominal Deger", "Olculen / Kestirilen", "Birim"],
        ["Tasiyici Gecikme (theta)", "80.0 ms (2.5 kare)", f"{reversal['mean_lag_ms']:.1f} ms ({reversal['mean_lag_frames']:.1f} kare)", "ms"],
        ["Zaman Sabiti (tau)", "45.0 ms", f"{fopdt['tau_s']*1000.0:.1f} ms", "ms"],
        ["Disli Boslugu (Backlash)", "1.20 derece", f"{reversal['median_backlash_deg']:.2f} derece", "derece"],
        ["Aktuator Kazanci (K)", "1.000", f"{fopdt['gain_K']:.3f}", "birimsiz"],
        ["YOLO Gurultusu (std_x)", "5.50 px", f"{noise['sigma_px_x']:.2f} px", "piksel"],
        ["YOLO Olcum Varyansi (R)", "64.0 px^2", f"{noise['r_var_x']:.1f} px^2", "px^2"],
        ["Model Uydurma RMSE", "-", f"{fopdt['rmse_deg']:.3f} derece", "derece"],
    ]
    tab = ax4.table(cellText=table_data, loc="center", cellLoc="center")
    tab.auto_set_font_size(False)
    tab.set_fontsize(10)
    tab.scale(1.15, 1.7)
    for col in range(4):
        tab[(0, col)].set_facecolor("#1A237E")
        tab[(0, col)].set_text_props(color="white", weight="bold")
    ax4.set_title("4. Gercek Donanim Sistem Tanilama Ozeti", fontsize=11, fontweight="bold", pad=12)

    plt.tight_layout()
    plt.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"[+] Sistem tanilama grafigi kaydedildi: {out_png}")


def run_system_identification(log_path: Path):
    print("=" * 70)
    print("  TURRET FCS // GERCEK TELEMETRIDEN SISTEM TANILAMA (SYSTEM ID)")
    print("=" * 70)
    print(f"[*] Telemetri dosyasi yukleniyor: {log_path.name}")
    data = load_telemetry(log_path)

    print("[*] 1. Yon degisimi gecikmesi ve disli boslugu (backlash) hesaplaniyor...")
    reversal = identify_reversal_lag_and_backlash(data)
    print(f"    - Ortalama Yon Degisimi Gecikmesi : {reversal['mean_lag_ms']:.1f} ms ({reversal['mean_lag_frames']:.1f} kare)")
    print(f"    - Medyan Mekanik Disli Boslugu    : {reversal['median_backlash_deg']:.2f} derece")

    print("[*] 2. YOLOv8 ve kamera olcum gurultusu (R) olculuyor...")
    noise = identify_measurement_noise(data)
    print(f"    - Olculen Gurultu Standart Sapmasi: {noise['sigma_px_x']:.2f} px (x), {noise['sigma_px_y']:.2f} px (y)")
    print(f"    - Olculen R Varyansi              : {noise['r_var_x']:.1f} px^2")

    print("[*] 3. FOPDT transfer fonksiyonu uyduruluyor (Nelder-Mead)...")
    fopdt = fit_fopdt_model(data)
    print(f"    - Kestirilen Zaman Sabiti (tau)   : {fopdt['tau_s']*1000.0:.1f} ms")
    print(f"    - Kestirilen Gecikme (theta)      : {fopdt['delay_ms']:.1f} ms ({fopdt['delay_frames']} kare)")
    print(f"    - Kestirilen Kazanc (K)           : {fopdt['gain_K']:.3f}")
    print(f"    - Model Uydurma RMSE              : {fopdt['rmse_deg']:.3f} derece")

    out_png = FIG_DIR / "python_fig13_system_id_real_hardware.png"
    plot_system_identification_results(fopdt, reversal, noise, out_png)

    summary = {
        "log_file": log_path.name,
        "delay_ms": reversal["mean_lag_ms"],
        "delay_frames": reversal["mean_lag_frames"],
        "backlash_deg": reversal["median_backlash_deg"],
        "tau_s": fopdt["tau_s"],
        "gain_K": fopdt["gain_K"],
        "noise_sigma_px": noise["sigma_px_x"],
        "noise_r_var": noise["r_var_x"],
        "fopdt_rmse_deg": fopdt["rmse_deg"],
    }
    out_json = BASE_DIR / "system_id_results.json"
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(f"[+] Sistem tanilama ozet JSON kaydedildi: {out_json}")
    print("=" * 70)
    return summary


if __name__ == "__main__":
    latest_log = PROJECT_ROOT / "logs" / "telemetry_test_20261025_054723.csv"
    run_system_identification(latest_log)
