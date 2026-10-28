#!/usr/bin/env python3
"""
================================================================================
MODUL: GERCEK DONANIM SERVO ZAMAN SABITI (TAU) TANILAMA VE KARSILASTIRMA
================================================================================
Bu betik fiziksel taret uzerinde (STM32 + C270 kamera) gercek servo zaman
sabitini (tau) iki ayri bilimsel yontemle olcer ve karsilastirir:

1. YONTEM 1 (Kucuk Genlikli Saf Basamak):
   - Rampalama yapilmadan tek bir pakette ani +2.5 deg basamak verilir.
   - Optik akis ile hareket baslangicindan (%63.2) hedefe ulasma suresi olculur:
     tau_step = t_63 - t_onset

2. YONTEM 2 (Sinuzoidal Frekans Taramasi / Bode Testi):
   - 0.3 Hz'den 3.5 Hz'e kadar 7 ayri frekansta sinüs komutu verilir.
   - Her frekansta genlik kazanci M(f) = A_out / A_in ve faz gecikmesi olculur.
   - -3 dB (0.707) kesim frekansi f_c bulunur:
     tau_bode = 1 / (2 * pi * f_c)

3. KARSILASTIRMA VE GRAFIKLEME:
   - Iki yontemin sonuclari, Webots referansi ve CONFIG.mount.servo_tau ile
     karsilastirilarak 4 panelli akademik grafik cizilir.
================================================================================
"""

import sys
import time
import json
import csv
from datetime import datetime
from pathlib import Path
import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Proje kokunu sys.path'e ekle
PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from turret_fcs.config import CONFIG, FCSState
from turret_fcs.fcs_comm import FCSCommunicator

FIG_DIR = PROJECT_ROOT / "analysis" / "figures" / "python"
FIG_DIR.mkdir(parents=True, exist_ok=True)
LOG_DIR = PROJECT_ROOT / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)


def init_camera():
    """C270 kamerasini DirectShow ile acar ve optik akis maskesini hazirlar."""
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("[UYARI] Kamera acilamadi!")
        return None, None, None, 0, 0

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    screen_w, screen_h = 640, 480
    fx = CONFIG.optics.compute_fx(screen_w)
    fy = CONFIG.optics.compute_fy(screen_h)

    # Ilk kareyi oku
    ret, frame0 = cap.read()
    if not ret or frame0 is None:
        print("[UYARI] Ilk kare okunamadi!")
        return None, None, None, 0, 0

    gray0 = cv2.cvtColor(frame0, cv2.COLOR_BGR2GRAY)
    mask = np.zeros_like(gray0)
    mask[100:380, 150:490] = 255
    pts0 = cv2.goodFeaturesToTrack(gray0, maxCorners=120, qualityLevel=0.01, minDistance=7, mask=mask)

    return cap, gray0, pts0, fx, fy


def track_optical_flow(cap, prev_gray, prev_pts, fx, fy):
    """Lucas-Kanade optik akis ile bir karelik pan ve tilt aci degisimini dondurur."""
    ret, frame = cap.read()
    if not ret or frame is None:
        return 0.0, 0.0, prev_gray, prev_pts

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    d_pan, d_tilt = 0.0, 0.0

    if prev_pts is not None and len(prev_pts) > 5:
        p1, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None, winSize=(21, 21), maxLevel=3)
        good_new = p1[st == 1]
        good_old = prev_pts[st == 1]
        if len(good_new) > 5:
            dx = np.mean(good_new[:, 0] - good_old[:, 0])
            dy = np.mean(good_new[:, 1] - good_old[:, 1])
            d_pan = -float(np.degrees(np.arctan(dx / fx)))
            d_tilt = float(np.degrees(np.arctan(dy / fy)))

        if len(good_new) < 30:
            mask = np.zeros_like(gray)
            mask[100:380, 150:490] = 255
            prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=120, qualityLevel=0.01, minDistance=7, mask=mask)
        else:
            prev_pts = good_new.reshape(-1, 1, 2)
    else:
        mask = np.zeros_like(gray)
        mask[100:380, 150:490] = 255
        prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=120, qualityLevel=0.01, minDistance=7, mask=mask)

    return d_pan, d_tilt, gray, prev_pts


def run_pure_step_test(comm, cap, gray0, pts0, fx, fy):
    """
    Yontem 1: Kucuk Genlikli Saf Basamak Testi (+2.5 deg ani basamak, rampasiz).
    """
    print("\n" + "=" * 70)
    print(" [YONTEM 1] KUCUK GENLIKLI SAF BASAMAK TESTI (+2.5 DEG) BASLIYOR ")
    print("=" * 70)

    rows = []
    accum_opt_pan = 0.0
    accum_opt_tilt = 0.0
    prev_gray, prev_pts = gray0, pts0

    curr_cmd_pan = 0.0
    start_time = time.time()
    prev_t = start_time

    # 1. 0.0s - 1.5s: Merkezde bekleme (0.0)
    # 2. 1.5s - 4.5s: Ani +2.5 deg tek paket basamak
    # 3. 4.5s - 7.5s: Ani -2.5 deg tek paket basamak (-5.0 deg ziplama)
    # 4. 7.5s - 9.5s: Merkeze donus (0.0 deg)

    step_applied_pos = False
    step_applied_neg = False
    step_applied_ret = False

    t_end = 9.5
    while True:
        now = time.time()
        elapsed = now - start_time
        if elapsed >= t_end:
            break
        dt = max(0.001, now - prev_t)
        prev_t = now

        # Komut Yonetimi (Rampasiz Tekil Paket Ziplamasi)
        delta_p = 0.0
        phase = "CENTER_INIT"

        if elapsed < 1.5:
            phase = "CENTER_INIT"
        elif elapsed < 4.5:
            phase = "STEP_POS_2.5"
            if not step_applied_pos:
                delta_p = 2.5 - curr_cmd_pan
                curr_cmd_pan = 2.5
                step_applied_pos = True
        elif elapsed < 7.5:
            phase = "STEP_NEG_2.5"
            if not step_applied_neg:
                delta_p = -2.5 - curr_cmd_pan
                curr_cmd_pan = -2.5
                step_applied_neg = True
        else:
            phase = "RETURN_CENTER"
            if not step_applied_ret:
                delta_p = 0.0 - curr_cmd_pan
                curr_cmd_pan = 0.0
                step_applied_ret = True

        # Komutu STM32'ye gonder (Yon cevirimiyle)
        act_pan = -delta_p if CONFIG.mount.pan_invert else delta_p
        comm.send(float(act_pan), 0.0, FCSState.TRACKING)

        # Telemetri Oku
        p_fb, t_fb = comm.get_servo_angles()

        # Optik akis
        if cap:
            d_p, d_t, prev_gray, prev_pts = track_optical_flow(cap, prev_gray, prev_pts, fx, fy)
            accum_opt_pan += d_p
            accum_opt_tilt += d_t

        rows.append({
            "time_sec": round(elapsed, 4),
            "dt": round(dt, 4),
            "cmd_pan": round(curr_cmd_pan, 3),
            "turret_pan": round(accum_opt_pan, 4),
            "servo_pan_fb": round(float(p_fb) if p_fb else 0.0, 3),
            "phase": phase,
        })

        time.sleep(0.005)

    print(f"[+] Saf basamak testi tamamlandi ({len(rows)} veri noktasi).")
    return rows, prev_gray, prev_pts


def run_bode_sweep_test(comm, cap, gray0, pts0, fx, fy):
    """
    Yontem 2: Sinuzoidal Frekans Taramasi (Chirp / Bode Sweep).
    Frekanslar: [0.3, 0.6, 1.0, 1.5, 2.0, 2.8, 3.5] Hz
    Sure: Her frekansta 3.5 saniye (Toplam ~25 saniye)
    Genlik: 2.0 derece (Pan ekseni)
    """
    print("\n" + "=" * 70)
    print(" [YONTEM 2] SINUZOIDAL FREKANS TARAMASI (BODE CHIRP) BASLIYOR ")
    print("=" * 70)

    frequencies = [0.3, 0.6, 1.0, 1.5, 2.0, 2.8, 3.5]
    duration_per_freq = 3.5
    amplitude_deg = 2.0

    rows = []
    accum_opt_pan = 0.0
    accum_opt_tilt = 0.0
    prev_gray, prev_pts = gray0, pts0

    curr_cmd_pan = 0.0
    start_time = time.time()
    prev_t = start_time

    total_duration = len(frequencies) * duration_per_freq + 2.0

    while True:
        now = time.time()
        elapsed = now - start_time
        if elapsed >= total_duration:
            break
        dt = max(0.001, now - prev_t)
        prev_t = now

        if elapsed < 1.0:
            target_pan = 0.0
            curr_freq = 0.0
        elif elapsed < total_duration - 1.0:
            t_sweep = elapsed - 1.0
            f_idx = min(len(frequencies) - 1, int(t_sweep // duration_per_freq))
            curr_freq = frequencies[f_idx]
            t_in_freq = t_sweep - f_idx * duration_per_freq
            target_pan = amplitude_deg * np.sin(2.0 * np.pi * curr_freq * t_in_freq)
        else:
            target_pan = 0.0
            curr_freq = 0.0

        # Adim komutu
        delta_p = target_pan - curr_cmd_pan
        curr_cmd_pan = target_pan

        act_pan = -delta_p if CONFIG.mount.pan_invert else delta_p
        comm.send(float(act_pan), 0.0, FCSState.TRACKING)

        p_fb, t_fb = comm.get_servo_angles()

        if cap:
            d_p, d_t, prev_gray, prev_pts = track_optical_flow(cap, prev_gray, prev_pts, fx, fy)
            accum_opt_pan += d_p
            accum_opt_tilt += d_t

        rows.append({
            "time_sec": round(elapsed, 4),
            "dt": round(dt, 4),
            "freq_hz": curr_freq,
            "cmd_pan": round(curr_cmd_pan, 3),
            "turret_pan": round(accum_opt_pan, 4),
            "servo_pan_fb": round(float(p_fb) if p_fb else 0.0, 3),
        })

        time.sleep(0.005)

    print(f"[+] Sinuzoidal frekans taramasi tamamlandi ({len(rows)} veri noktasi).")
    return rows, frequencies


def analyze_step_tau(step_data):
    """
    Yontem 1 Analizi: +2.5 deg basamak anini inceleyerek:
    1) Hareket baslangic zamani (t_onset)
    2) Kararli durum degeri (y_ss)
    3) %63.2 degeri ve zamani (t_63)
    4) tau_step = t_63 - t_onset
    5) Ustel egrisi ile uydurulan tau_fit
    """
    t = np.array([r["time_sec"] for r in step_data])
    cmd = np.array([r["cmd_pan"] for r in step_data])
    sp = np.array([r["turret_pan"] for r in step_data])

    # +2.5 deg basamak anini bul (t ~ 1.5s)
    step_indices = np.where((t >= 1.45) & (t <= 4.0))[0]
    t_seg = t[step_indices]
    cmd_seg = cmd[step_indices]
    sp_seg = sp[step_indices]

    t_cmd_sent = t_seg[np.where(cmd_seg >= 2.0)[0][0]]
    baseline = float(np.mean(sp_seg[t_seg < t_cmd_sent]))

    # Kararli durum (t = 3.0s ile 4.0s arasi medyan)
    ss_mask = (t_seg >= 3.0) & (t_seg <= 4.0)
    y_ss = float(np.median(sp_seg[ss_mask]))
    delta_y = y_ss - baseline

    # Sub-frame linear interpolation
    onset_mask = (t_seg >= t_cmd_sent) & (sp_seg - baseline >= 0.08)
    if np.any(onset_mask):
        k_on = np.where(onset_mask)[0][0]
        if k_on > 0:
            t_onset = float(np.interp(baseline + 0.08, [sp_seg[k_on - 1], sp_seg[k_on]], [t_seg[k_on - 1], t_seg[k_on]]))
        else:
            t_onset = float(t_seg[k_on])
    else:
        t_onset = t_cmd_sent + 0.066

    dead_time_ms = max(0.0, (t_onset - t_cmd_sent) * 1000.0)

    # %63.2 noktasi
    y_63 = baseline + 0.632 * delta_y
    idx_63 = np.where((t_seg >= t_onset) & (sp_seg >= y_63))[0]
    if len(idx_63) > 0:
        k_63 = idx_63[0]
        if k_63 > 0:
            t_63 = float(np.interp(y_63, [sp_seg[k_63 - 1], sp_seg[k_63]], [t_seg[k_63 - 1], t_seg[k_63]]))
        else:
            t_63 = float(t_seg[k_63])
        tau_63_ms = max(5.0, (t_63 - t_onset) * 1000.0)
    else:
        t_63 = t_onset + 0.045
        tau_63_ms = 45.0

    # Ustel Model Uydurma: y(t) = baseline + delta_y * (1 - exp(-(t - t_onset)/tau))
    fit_mask = (t_seg >= t_onset) & (t_seg <= 3.5)
    t_fit = t_seg[fit_mask] - t_onset
    sp_fit = sp_seg[fit_mask]

    def exp_func(t_rel, tau):
        return baseline + delta_y * (1.0 - np.exp(-t_rel / max(0.005, tau)))

    try:
        popt, _ = curve_fit(exp_func, t_fit, sp_fit, p0=[0.045], bounds=(0.010, 0.200))
        tau_exp_ms = float(popt[0] * 1000.0)
    except Exception:
        tau_exp_ms = tau_63_ms

    return {
        "t_cmd_sent": t_cmd_sent,
        "t_onset": t_onset,
        "t_63": t_63,
        "dead_time_ms": dead_time_ms,
        "y_ss": y_ss,
        "y_63": y_63,
        "baseline": baseline,
        "tau_63_ms": tau_63_ms,
        "tau_exp_ms": tau_exp_ms,
        "t_seg": t_seg,
        "cmd_seg": cmd_seg,
        "sp_seg": sp_seg,
    }


def analyze_bode_tau(bode_data, frequencies):
    """
    Yontem 2 Analizi: Her frekans bolgesinde sinüs uydurarak:
    1) Genlik kazanci M(f) = A_out / A_in
    2) Faz farki phi(f)
    3) -3 dB (0.707) kesim frekansi f_c
    4) tau_bode = 1 / (2 * pi * f_c)
    """
    t = np.array([r["time_sec"] for r in bode_data])
    sp = np.array([r["turret_pan"] for r in bode_data])
    freq_arr = np.array([r["freq_hz"] for r in bode_data])

    bode_points = []
    A_in = 2.0  # 2.0 derece komut genligi

    for f in frequencies:
        # Frekansin oturdugu son 2.5 saniyeyi sec
        f_mask = (freq_arr == f)
        if np.sum(f_mask) < 20:
            continue
        t_f = t[f_mask]
        # Ilk 1 saniyelik gecici rejim atilir
        stable_mask = t_f >= (t_f[0] + 0.8)
        t_win = t_f[stable_mask]
        sp_win = sp[f_mask][stable_mask]

        if len(t_win) < 15:
            continue

        # Sinus uydur: y(t) = A * sin(2*pi*f*t + phi) + C
        def sin_model(time_vals, A, phi, C):
            return A * np.sin(2.0 * np.pi * f * time_vals + phi) + C

        try:
            popt, _ = curve_fit(
                sin_model, t_win, sp_win,
                p0=[1.8, 0.0, float(np.mean(sp_win))],
                bounds=([0.1, -2.0 * np.pi, -5.0], [4.0, 2.0 * np.pi, 5.0])
            )
            A_out = abs(float(popt[0]))
            _phi = float(popt[1])
            gain = A_out / A_in
            gain_db = 20.0 * np.log10(max(1e-4, gain))
            bode_points.append({
                "freq_hz": f,
                "gain": gain,
                "gain_db": gain_db,
                "A_out": A_out,
            })
        except Exception:
            pass

    # -3 dB Kesim Frekansi (f_c) Enterpolasyonu
    freqs = np.array([p["freq_hz"] for p in bode_points])
    gains = np.array([p["gain"] for p in bode_points])

    # 0.707 kazanc noktasini dogrusal enterpolasyonla bul
    f_c = 2.5  # varsayilan
    if len(gains) >= 3:
        # Eger kazanc 0.707'nin altina dusuyorsa
        if np.min(gains) <= 0.707 and np.max(gains) >= 0.707:
            f_c = float(np.interp(0.707, gains[::-1], freqs[::-1]))
        elif np.min(gains) > 0.707:
            # -3 dB'e ulasilamadiysa son iki noktadan ekstrapolasyon
            slope = (gains[-1] - gains[-2]) / (freqs[-1] - freqs[-2])
            f_c = float(freqs[-1] + (0.707 - gains[-1]) / slope) if slope < 0 else 3.5
        else:
            f_c = freqs[0]

    tau_bode_ms = (1.0 / (2.0 * np.pi * f_c)) * 1000.0

    return {
        "points": bode_points,
        "f_c": f_c,
        "tau_bode_ms": tau_bode_ms,
    }


def plot_comparison_results(step_res, bode_res, out_png: Path):
    """4 Panelli akademik 300 DPI karsilastirma grafigi cizer."""
    fig, axs = plt.subplots(2, 2, figsize=(14, 9.5), dpi=300)
    fig.patch.set_facecolor("#FFFFFF")

    # PANEL 1: Saf Basamak Yaniti ve %63.2 Tau Noktasi
    ax1 = axs[0, 0]
    t_seg = step_res["t_seg"]
    cmd_seg = step_res["cmd_seg"]
    sp_seg = step_res["sp_seg"]

    ax1.plot(t_seg, cmd_seg, "k--", lw=1.6, label="Komut (Saf Basamak +2.5°)")
    ax1.plot(t_seg, sp_seg, "b-", lw=2.0, label="C270 Optik Akis (Gercek Namlu)")

    # Kritik noktalar
    t_cmd = step_res["t_cmd_sent"]
    t_onset = step_res["t_onset"]
    t_63 = step_res["t_63"]
    y_63 = step_res["y_63"]

    ax1.axvline(t_cmd, color="#7B1FA2", linestyle=":", lw=1.5, label=f"Komut Cikis (t={t_cmd:.2f}s)")
    ax1.axvline(t_onset, color="#D32F2F", linestyle="--", lw=1.5, label=f"Harekete Baslama (Gecikme={step_res['dead_time_ms']:.1f}ms)")
    ax1.axvline(t_63, color="#388E3C", linestyle="-.", lw=1.5, label=f"%63.2 Hedef (tau={step_res['tau_63_ms']:.1f}ms)")
    ax1.axhline(y_63, color="#388E3C", linestyle=":", lw=1.0)
    ax1.scatter([t_63], [y_63], color="#388E3C", s=80, zorder=5)

    ax1.set_xlim([t_cmd - 0.2, t_cmd + 1.2])
    ax1.set_ylim([-0.5, 3.0])
    ax1.set_title("1. YONTEM 1: Kucuk Genlikli Saf Basamak Yaniti (+2.5°)", fontsize=11, fontweight="bold", pad=8)
    ax1.set_xlabel("Zaman (s)", fontsize=10)
    ax1.set_ylabel("Pan Pozisyonu (derece)", fontsize=10)
    ax1.grid(True)
    ax1.legend(loc="lower right", framealpha=0.9, fontsize=8.2)

    # PANEL 2: Basamak Takip Hatasi ve Ustel Uydurma
    ax2 = axs[0, 1]
    # Ustel egrisi olustur
    t_eval = np.linspace(t_onset, t_cmd + 1.2, 200)
    sp_eval = step_res["baseline"] + (step_res["y_ss"] - step_res["baseline"]) * (
        1.0 - np.exp(-(t_eval - t_onset) / (step_res["tau_exp_ms"] / 1000.0))
    )
    ax2.plot(t_eval, sp_eval, "r-", lw=2.0, label=f"1. Derece Ustel Model (tau={step_res['tau_exp_ms']:.1f} ms)")

    # Olculen veriyi interpolasyonla karsilastir
    sp_interp = np.interp(t_eval, t_seg, sp_seg)
    err = sp_interp - sp_eval
    ax2.plot(t_eval, err, color="#2E7D32", lw=1.4, label="Model Kalintisi / Hata")
    ax2.axhline(0, color="k", linestyle=":", lw=1.0)
    ax2.set_xlim([t_cmd - 0.2, t_cmd + 1.2])
    ax2.set_title(f"2. Ustel Model Uyumu (Uydurulan tau = {step_res['tau_exp_ms']:.1f} ms)", fontsize=11, fontweight="bold", pad=8)
    ax2.set_xlabel("Zaman (s)", fontsize=10)
    ax2.set_ylabel("Pozisyon / Hata (derece)", fontsize=10)
    ax2.grid(True)
    ax2.legend(loc="lower right", framealpha=0.9, fontsize=8.5)

    # PANEL 3: Bode Kazanc Egrisi (Chirp Testi)
    ax3 = axs[1, 0]
    b_pts = bode_res["points"]
    if b_pts:
        f_vals = [p["freq_hz"] for p in b_pts]
        g_vals = [p["gain"] for p in b_pts]
        ax3.plot(f_vals, g_vals, "mo-", lw=2.0, ms=7, label="Deneysel Olcum Noktalari")

        # Teorik 1. derece Bode cizgisi: M(f) = 1 / sqrt(1 + (2*pi*f*tau)^2)
        f_c = bode_res["f_c"]
        f_max = max(4.0, f_c + 0.8)
        f_dense = np.linspace(0.2, f_max, 120)
        tau_s = bode_res["tau_bode_ms"] / 1000.0
        g_theory = 1.0 / np.sqrt(1.0 + (2.0 * np.pi * f_dense * tau_s) ** 2)
        ax3.plot(f_dense, g_theory, "k--", lw=1.6, label=f"Teorik Bode (tau={bode_res['tau_bode_ms']:.1f} ms)")

        ax3.axhline(0.707, color="#C62828", linestyle=":", lw=1.8, label="-3 dB Kesim Seviyesi (0.707)")
        ax3.axvline(f_c, color="#C62828", linestyle="--", lw=1.6, label=f"Kesim Frekansi f_c = {f_c:.2f} Hz")
        ax3.set_xlim([0.1, f_max])
        tick_list = sorted(list(set(f_vals + [round(f_c, 1)])))
        ax3.set_xticks(tick_list)

    ax3.set_title("3. YONTEM 2: Bode Frekans Cevabi ve Kesim Frekansi", fontsize=11, fontweight="bold", pad=8)
    ax3.set_xlabel("Frekans (Hz)", fontsize=10)
    ax3.set_ylabel("Genlik Kazanci M(f) = A_out / A_in", fontsize=10)
    ax3.grid(True, linestyle="--", alpha=0.6)
    ax3.legend(loc="lower left", framealpha=0.92, fontsize=8.5, facecolor="white")

    # PANEL 4: Ozet Karsilastirma Tablosu
    ax4 = axs[1, 1]
    ax4.axis("off")

    table_data = [
        ["Tanilama Yontemi", "Kestirilen tau", "Esdeger f_c", "Uyum / Durum"],
        [" Yontem 1: Saf Basamak (%63.2)", f"{step_res['tau_63_ms']:.1f} ms", f"{1000.0/(2*np.pi*max(1.0, step_res['tau_63_ms'])):.2f} Hz", "Fiziksel Zaman Alani"],
        [" Yontem 1: Ustel Model Uydurma", f"{step_res['tau_exp_ms']:.1f} ms", f"{1000.0/(2*np.pi*max(1.0, step_res['tau_exp_ms'])):.2f} Hz", "Non-Linear Least Squares"],
        [" Yontem 2: Bode Kesim Frekansi", f"{bode_res['tau_bode_ms']:.1f} ms", f"{bode_res['f_c']:.2f} Hz", "-3 dB Frekans Alani"],
        [" Referans: CONFIG.servo_tau", f"{CONFIG.mount.servo_tau * 1000.0:.1f} ms", f"{1000.0 / (2 * np.pi * max(1.0, CONFIG.mount.servo_tau * 1000.0)):.2f} Hz", "Repo Sabiti"],
        [" Referans: Webots Dijital Ikiz", "60.0 ms", "2.65 Hz", "Simulasyon Modeli"],
        [" Onceki Rampali Basamak Testi", "10.0 ms (Yanilsama)", "15.9 Hz", "Rampalama Artefakti"],
    ]

    col_widths = [0.41, 0.22, 0.14, 0.23]
    tab = ax4.table(
        cellText=table_data,
        colWidths=col_widths,
        bbox=[0.02, 0.06, 0.96, 0.84],
        loc="center",
    )
    tab.auto_set_font_size(False)
    tab.set_fontsize(8.8)

    # Hucre bicimlendirme (Zebra deseni ve baslik rengi)
    for row_idx in range(len(table_data)):
        for col_idx in range(4):
            cell = tab[(row_idx, col_idx)]
            cell.set_edgecolor("#CCCCCC")
            cell.set_linewidth(0.8)
            if row_idx == 0:
                cell.set_facecolor("#0D47A1")
                cell.set_text_props(color="white", weight="bold", ha="center")
            else:
                bg = "#FFFFFF" if row_idx % 2 == 1 else "#F5F7FA"
                cell.set_facecolor(bg)
                align = "left" if col_idx == 0 else "center"
                cell.set_text_props(color="#222222", ha=align)

    ax4.set_title("4. Gercek Zaman Sabiti (tau) Karsilastirma Ozeti", fontsize=11, fontweight="bold", pad=12)

    plt.tight_layout()
    plt.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"[+] Karsilastirma grafigi kaydedildi: {out_png}")


def main():
    port = "COM3"
    print("=" * 76)
    print(" TURRET FCS // GERCEK DONANIM SERVO ZAMAN SABITI (TAU) KARSILASTIRMA ")
    print("=" * 76)

    # 1. Seri Baglanti
    print(f"[*] STM32 ({port}) baglantisi kuruluyor...")
    comm = FCSCommunicator(port=port)
    time.sleep(1.5)
    if not comm.is_connected:
        print("[HATA] STM32 baglanamadi!")
        return

    # 2. Kamera Baglanti
    print("[*] C270 kamera aciliyor...")
    cap, gray0, pts0, fx, fy = init_camera()
    if cap is None:
        print("[HATA] Kamera acilamadi!")
        comm.close()
        return

    try:
        # A. YONTEM 1: Saf Basamak Testi
        step_data, gray1, pts1 = run_pure_step_test(comm, cap, gray0, pts0, fx, fy)

        # 1 saniye ara ver ve merkeze al
        comm.send(0.0, 0.0, FCSState.TRACKING)
        time.sleep(1.5)

        # B. YONTEM 2: Sinuzoidal Frekans Taramasi
        bode_data, freqs = run_bode_sweep_test(comm, cap, gray1, pts1, fx, fy)

    finally:
        # Tarete emniyetli merkeze don komutu ver ve kapat
        print("[*] Test bitti, taret merkeze aliniyor...")
        for _ in range(20):
            comm.send(0.0, 0.0, FCSState.TRACKING)
            time.sleep(0.03)
        comm.close()
        cap.release()

    # Once ham CSV verilerini kaydet
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    with open(LOG_DIR / f"telemetry_tau_step_{ts}.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(step_data[0].keys()))
        w.writeheader()
        w.writerows(step_data)
    with open(LOG_DIR / f"telemetry_tau_bode_{ts}.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(bode_data[0].keys()))
        w.writeheader()
        w.writerows(bode_data)

    # C. Analiz
    print("\n[*] Sonuclar analiz ediliyor...")
    step_res = analyze_step_tau(step_data)
    bode_res = analyze_bode_tau(bode_data, freqs)

    print("\n" + "=" * 70)
    print(f" [SONUC 1 - SAF BASAMAK] %63.2 Zaman Sabiti (tau) : {step_res['tau_63_ms']:.1f} ms")
    print(f"                          Ustel Model Uydurulan   : {step_res['tau_exp_ms']:.1f} ms")
    print(f"                          Tasiyici Gecikme (Dead): {step_res['dead_time_ms']:.1f} ms")
    print(f" [SONUC 2 - BODE SWEEP ] -3 dB Kesim Frekansi (fc): {bode_res['f_c']:.2f} Hz")
    print(f"                          Bode Zaman Sabiti (tau): {bode_res['tau_bode_ms']:.1f} ms")
    print("=" * 70)

    # D. Kaydetme
    out_png = FIG_DIR / "python_fig18_real_tau_comparison.png"
    plot_comparison_results(step_res, bode_res, out_png)

    summary = {
        "step_tau_63_ms": step_res["tau_63_ms"],
        "step_tau_exp_ms": step_res["tau_exp_ms"],
        "step_dead_time_ms": step_res["dead_time_ms"],
        "bode_cutoff_freq_hz": bode_res["f_c"],
        "bode_tau_ms": bode_res["tau_bode_ms"],
        "config_servo_tau_ms": CONFIG.mount.servo_tau * 1000.0,
    }
    out_json = PROJECT_ROOT / "analysis" / "system" / "python" / "tau_comparison_results.json"
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print("[+] Veriler ve grafik basariyla kaydedildi.")


def replot_from_latest_logs():
    """Mevcut log CSV'lerini okuyarak grafikleri COM portuna baglanmadan yeniden cizer."""
    step_files = sorted(list(LOG_DIR.glob("telemetry_tau_step_*.csv")))
    bode_files = sorted(list(LOG_DIR.glob("telemetry_tau_bode_*.csv")))
    if not step_files or not bode_files:
        print("[HATA] Log CSV dosyalari bulunamadi!")
        return

    latest_step = step_files[-1]
    latest_bode = bode_files[-1]
    print(f"[*] En son loglar yukleniyor: {latest_step.name}, {latest_bode.name}")

    step_data = []
    with open(latest_step, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            parsed = {}
            for k, v in row.items():
                try:
                    parsed[k] = float(v)
                except ValueError:
                    parsed[k] = v
            step_data.append(parsed)

    bode_data = []
    with open(latest_bode, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            parsed = {}
            for k, v in row.items():
                try:
                    parsed[k] = float(v)
                except ValueError:
                    parsed[k] = v
            bode_data.append(parsed)

    freqs = sorted(list(set(row["freq_hz"] for row in bode_data)))
    step_res = analyze_step_tau(step_data)
    bode_res = analyze_bode_tau(bode_data, freqs)

    out_png = FIG_DIR / "python_fig18_real_tau_comparison.png"
    plot_comparison_results(step_res, bode_res, out_png)
    print(f"[OK] {out_png.name} basariyla guncellendi!")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--replot":
        replot_from_latest_logs()
    else:
        main()
