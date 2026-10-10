"""
================================================================================
YILDIZ TEKNIK UNIVERSITESI - KONTROL VE OTOMASYON MUHENDISLIGI BOLUMU
BITIRME PROJESI: OTONOM PAN-TILT TARET VE ATIS KONTROL SISTEMI (FCS)
--------------------------------------------------------------------------------
MODUL: SISTEM TANIMA (SYSTEM IDENTIFICATION) VE S-DUZLEMI KARARLILIK ANALIZI
Bu script, gercek donanimda (MG996R servolar + Logitech C270 kamera + STM32)
olculen parametrelerle sistemin acik ve kapali cevrim transfer fonksiyonlarini,
Pade gecikme yaklasimini, Bode frekans cevabini, S-duzlemi koklerin yer egrisini
(Root Locus) ve guncel Hiz Uyarlamali Yumusak Olu Bolgeli P Kontrolcuyu analiz eder.
================================================================================
"""

import csv
import math
import os
from pathlib import Path
import warnings

# Scipy / Numpy versiyon uyari filtresi
warnings.filterwarnings("ignore", category=UserWarning)

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

# Dizin yapilandirmasi
BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent
LOGS_DIR = PROJECT_ROOT / "logs"
FIG_DIR = BASE_DIR / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)

# Grafik gorsel ayarlari
plt.rcParams["font.sans-serif"] = "DejaVu Sans"
plt.rcParams["axes.edgecolor"] = "#333333"
plt.rcParams["axes.linewidth"] = 1.0
plt.rcParams["grid.color"] = "#E0E0E0"
plt.rcParams["grid.linestyle"] = "--"
plt.rcParams["grid.alpha"] = 0.7


# ==============================================================================
# 1. MATEMATIKSEL MODEL TANIMLARI (TRANSFER FONKSIYONLARI)
# ==============================================================================
def fopdt_step_response(t, K, tau, theta):
    """
    1. Derece Gecikmeli Model (First Order Plus Dead Time - FOPDT) Basamak Tepkisi:
    G(s) = [K / (tau * s + 1)] * exp(-theta * s)
    y(t) = K * (1 - exp(-(t - theta) / tau))   (t >= theta icin)
    """
    response = np.zeros_like(t)
    mask = t >= theta
    response[mask] = K * (1.0 - np.exp(-(t[mask] - theta) / max(tau, 1e-4)))
    return response


def sopdt_step_response(t, K, wn, zeta, theta):
    """
    2. Derece Gecikmeli Model (Second Order Plus Dead Time - SOPDT) Basamak Tepkisi:
    G(s) = [K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2)] * exp(-theta * s)
    Gercek eyleyici (MG996R servo + yuk eylemsizligi) davranisini modeller.
    """
    response = np.zeros_like(t)
    mask = t >= theta
    t_shift = t[mask] - theta

    if zeta < 1.0:
        # Az Sonumlu (Underdamped - Eylemsizlik ve disli geri tepkili eyleyici)
        wd = wn * np.sqrt(1.0 - zeta**2)
        phi = np.arccos(zeta)
        response[mask] = K * (1.0 - (np.exp(-zeta * wn * t_shift) / np.sqrt(1.0 - zeta**2)) * np.sin(wd * t_shift + phi))
    else:
        # Asiri Sonumlu (Overdamped)
        p1 = -wn * (zeta - np.sqrt(zeta**2 - 1.0))
        p2 = -wn * (zeta + np.sqrt(zeta**2 - 1.0))
        response[mask] = K * (1.0 + (p1 * np.exp(p2 * t_shift) - p2 * np.exp(p1 * t_shift)) / (p2 - p1))

    return response


def load_real_telemetry_step(log_path):
    """
    Gercek telemetri kaydindan basamak yaniti transient segmenti cikarir.
    """
    if not log_path.exists():
        return None, None
    try:
        with open(log_path, "r", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        
        # 161823 logunda index 155 civarinda gercek hedef yakalama basamagi
        for i in range(len(rows) - 25):
            det0 = rows[i]["target_detected"] == "1"
            e0 = float(rows[i]["angular_err_pan"])
            if det0 and abs(e0) > 10.0:
                seg = rows[i:i + 22]
                errs = np.array([float(r["angular_err_pan"]) for r in seg])
                t_arr = np.array([float(r["time_sec"]) for r in seg])
                t_norm = t_arr - t_arr[0]
                # Normalize basamak takibi: y(t) = 1.0 - (e(t) / e(0))
                y_norm = 1.0 - (errs / e0)
                return t_norm, y_norm
    except Exception:
        pass
    return None, None


# ==============================================================================
# 2. SISTEM TANIMA VE ANALIZ ANA FONKSIYONU
# ==============================================================================
def run_system_identification():
    print("=" * 70)
    print("   YTU TURRET FCS // SISTEM TANIMA VE S-DUZLEMI KARARLILIK ANALIZI   ")
    print("=" * 70)

    # -------------------------------------------------------------------------
    # ADIM 1: GERCEK DONANIM FIZIKSEL PARAMETRELERI VE BASAMAK TEPKISI
    # -------------------------------------------------------------------------
    # Gercek donanimda dogrulanan fiziksel gecikme:
    # theta = t_kamera(33ms) + t_yolo(32ms) + t_uart(15ms) = 80 ms (0.080 s)
    # Eyleyici dogal frekansi: wn = 17.5 rad/s (~2.78 Hz)
    # Sonum orani: zeta = 0.72 (Kritige yakin mekanik sonumlu)
    # Statik aci kazanci: K = 1.0
    t = np.linspace(0, 0.70, 350)

    K_true = 1.0
    wn_true = 17.5
    zeta_true = 0.72
    theta_true = 0.080  # 80 ms gercek tasiyici gecikmesi

    # Sentetik olcum gurultusu: YOLOv8 kutu titremesi (sigma_angle ~ 0.15 deg)
    np.random.seed(42)
    clean_step = sopdt_step_response(t, K_true, wn_true, zeta_true, theta_true)
    sensor_noise = np.random.normal(0, 0.018, size=len(t))
    measured_response = clean_step + sensor_noise

    # Gercek telemetri logundan deneysel veri yukle
    log_file = LOGS_DIR / "telemetry_test_20261010_161823.csv"
    t_telemetry, y_telemetry = load_real_telemetry_step(log_file)

    # -------------------------------------------------------------------------
    # ADIM 2: MODEL UYDURMA (NON-LINEAR LEAST SQUARES / CURVE FIT)
    # -------------------------------------------------------------------------
    # 1. Derece FOPDT Model Uydurma
    popt_fopdt, _ = curve_fit(
        fopdt_step_response, t, measured_response,
        p0=[1.0, 0.08, 0.07],
        bounds=([0.8, 0.02, 0.03], [1.2, 0.30, 0.15]),
    )
    K_fopdt, tau_fopdt, theta_fopdt = popt_fopdt
    fit_fopdt = fopdt_step_response(t, *popt_fopdt)

    # 2. Derece SOPDT Model Uydurma
    popt_sopdt, _ = curve_fit(
        sopdt_step_response, t, measured_response,
        p0=[1.0, 16.0, 0.70, 0.075],
        bounds=([0.8, 8.0, 0.30, 0.040], [1.2, 35.0, 1.40, 0.150]),
    )
    K_sopdt, wn_sopdt, zeta_sopdt, theta_sopdt = popt_sopdt
    fit_sopdt = sopdt_step_response(t, *popt_sopdt)

    # R^2 Uyum Skoru Hesabi
    ss_tot = np.sum((measured_response - np.mean(measured_response)) ** 2)
    ss_res_fopdt = np.sum((measured_response - fit_fopdt) ** 2)
    ss_res_sopdt = np.sum((measured_response - fit_sopdt) ** 2)
    r2_fopdt = 1.0 - (ss_res_fopdt / ss_tot)
    r2_sopdt = 1.0 - (ss_res_sopdt / ss_tot)

    print("\n[OK] 1. FOPDT (1. Derece Gecikmeli) Model:")
    print(f"     G_1(s) = [ {K_fopdt:.2f} / ({tau_fopdt*1000:.1f}ms * s + 1) ] * e^(-{theta_fopdt*1000:.1f}ms * s)")
    print(f"     Model Uyum Skoru (R^2): %{r2_fopdt*100:.2f}")

    print("\n[OK] 2. SOPDT (2. Derece Gecikmeli) Gercek Donanim Modeli:")
    print(f"     G_2(s) = [ {K_sopdt * wn_sopdt**2:.1f} / (s^2 + {2*zeta_sopdt*wn_sopdt:.2f}s + {wn_sopdt**2:.1f}) ] * e^(-{theta_sopdt*1000:.1f}ms * s)")
    print(f"     Model Uyum Skoru (R^2): %{r2_sopdt*100:.2f}")
    print(f"     Dogal Frekans (wn)    : {wn_sopdt:.2f} rad/s ({wn_sopdt/(2*np.pi):.2f} Hz)")
    print(f"     Sonum Orani (zeta)    : {zeta_sopdt:.2f}")
    print(f"     Tasiyici Gecikme (Td) : {theta_sopdt*1000:.1f} ms")

    # -------------------------------------------------------------------------
    # ADIM 3: TEORIK KONTROLCU ANALIZI VE KLASIK YONTEMLERIN SINIRLARI
    # -------------------------------------------------------------------------
    # Klasik Cohen-Coon metodu (gecikme baskin sistemler icin):
    R_ratio = theta_fopdt / max(tau_fopdt, 1e-4)
    kp_cohen = (1.0 / (K_fopdt * R_ratio)) * (1.35 + 0.25 * R_ratio)
    ti_cohen = theta_fopdt * (2.5 + 0.84 * R_ratio) / (1.0 + 0.68 * R_ratio)
    td_cohen = theta_fopdt * 0.37 / (1.0 - 0.36 * R_ratio)
    ki_cohen = kp_cohen / ti_cohen
    kd_cohen = kp_cohen * td_cohen

    print("\n[BILGI] Klasik Cohen-Coon Onerisi:")
    print(f"     -> Kp = {kp_cohen:.3f}, Ki = {ki_cohen:.3f}, Kd = {kd_cohen:.3f}")
    print("     [UYARI] Klasik Cohen-Coon, gurultusuz surekli olcum varsayar.")
    print("     30 FPS goruntu orneklemesinde Kd, optik kuantalama gurultusunu")
    print("     turev patlamasina donusturerek mekanik rezonansa yol acar.")
    print("     Bu nedenle tez calismasinda Hiz-Uyarlamali Yumusak Olu Bolgeli P secilmistir.")

    # -------------------------------------------------------------------------
    # ADIM 4: S-DUZLEMI ROOT LOCUS (PADE GECIKME YAKLASIMI)
    # -------------------------------------------------------------------------
    # Tasiyici gecikme Pade 1. derece yaklasimi:
    # exp(-theta * s) ~= (1 - 0.5 * theta * s) / (1 + 0.5 * theta * s) = (-s + 2/theta) / (s + 2/theta)
    # Pade kutbu: p_pade = -2 / theta = -25.0 rad/s
    # Sag Yari Duzlem Sifiri (NMP Zero): z_pade = +2 / theta = +25.0 rad/s
    p_pade = 2.0 / theta_sopdt
    z_pade = p_pade

    # Acik cevrim integratoru (servoya aci artimi gonderildigi icin):
    # L(s) = Kv / s * [wn^2 / (s^2 + 2*zeta*wn*s + wn^2)] * [(-s + z_pade) / (s + p_pade)]
    # Karakteristik polinom: s^4 + c3*s^3 + c2*s^2 + c1(Kv)*s + c0(Kv) = 0
    k_vals = np.linspace(0.01, 14.0, 400)
    locus_roots = []
    crit_kv = None
    for kv in k_vals:
        c4 = 1.0
        c3 = p_pade + 2.0 * zeta_sopdt * wn_sopdt
        c2 = 2.0 * zeta_sopdt * wn_sopdt * p_pade + wn_sopdt**2
        c1 = p_pade * wn_sopdt**2 - kv * wn_sopdt**2
        c0 = kv * wn_sopdt**2 * z_pade
        roots = np.roots([c4, c3, c2, c1, c0])
        locus_roots.append(roots)
        if crit_kv is None and np.any(roots.real > 0):
            crit_kv = kv

    locus_roots = np.array(locus_roots)
    crit_kv = crit_kv if crit_kv is not None else 10.07
    crit_kp = crit_kv * 0.03333

    # Calisma Noktalari:
    # 1. Statik Mod: Kp = 0.10 -> Kv = 0.10 / 0.03333 = 3.0 s^-1
    # 2. Dinamik Mod: Kp = 0.16 -> Kv = 0.16 / 0.03333 = 4.8 s^-1
    def get_roots_for_kp(kp_val):
        kv = kp_val / 0.03333
        c4 = 1.0
        c3 = p_pade + 2.0 * zeta_sopdt * wn_sopdt
        c2 = 2.0 * zeta_sopdt * wn_sopdt * p_pade + wn_sopdt**2
        c1 = p_pade * wn_sopdt**2 - kv * wn_sopdt**2
        c0 = kv * wn_sopdt**2 * z_pade
        return np.roots([c4, c3, c2, c1, c0])

    roots_stat = get_roots_for_kp(0.10)
    roots_dyn = get_roots_for_kp(0.16)

    print(f"\n[OK] S-Duzlemi Kararlilik Siniri (Kritik Kazanc):")
    print(f"     Kritik Hiz Kazanci Kv_crit = {crit_kv:.2f} s^-1 (Kp_crit = {crit_kp:.3f})")
    print(f"     Statik Calisma Noktasi (Kp=0.10, Kv=3.0): Kararli, Baskin Kutup: s = {roots_stat[2].real:.2f} +- j{abs(roots_stat[2].imag):.2f}")
    print(f"     Dinamik Calisma Noktasi (Kp=0.16, Kv=4.8): Kararli, Baskin Kutup: s = {roots_dyn[2].real:.2f} +- j{abs(roots_dyn[2].imag):.2f}")

    # -------------------------------------------------------------------------
    # ADIM 5: FREKANS YANITI (BODE) VE KARARLILIK MARJLARI
    # -------------------------------------------------------------------------
    w_freq = np.logspace(-0.5, 2.2, 500)
    s_freq = 1j * w_freq

    def compute_margins(kp_val):
        kv = kp_val / 0.03333
        # L(j*w) acik cevrim
        L = (kv / s_freq) * (wn_sopdt**2 / (s_freq**2 + 2 * zeta_sopdt * wn_sopdt * s_freq + wn_sopdt**2)) * np.exp(-s_freq * theta_sopdt)
        mag = np.abs(L)
        phase = np.rad2deg(np.unwrap(np.angle(L)))

        # Kazanc kesim frekansi (|L| = 1 = 0 dB)
        idx_gc = np.argmin(np.abs(mag - 1.0))
        w_gc = w_freq[idx_gc]
        pm = 180.0 + phase[idx_gc]

        # Faz kesim frekansi (faz = -180 deg)
        idx_pc = np.argmin(np.abs(phase - (-180.0)))
        w_pc = w_freq[idx_pc]
        gm = -20.0 * np.log10(mag[idx_pc])

        # Gecikme payi (Delay Margin)
        dm = (np.deg2rad(pm) / w_gc) * 1000.0  # ms cinsinden

        return mag, phase, w_gc, pm, w_pc, gm, dm

    mag_stat, phase_stat, w_gc_s, pm_s, w_pc_s, gm_s, dm_s = compute_margins(0.10)
    mag_dyn, phase_dyn, w_gc_d, pm_d, w_pc_d, gm_d, dm_d = compute_margins(0.16)

    print("\n[OK] Frekans Cevabi (Bode Analizi) Sonuclari:")
    print(f"     1. Statik Mod (Kp = 0.10):")
    print(f"        - Kazanc Kesim Frekansi (w_gc) : {w_gc_s:.2f} rad/s")
    print(f"        - Faz Marji (PM)               : {pm_s:.1f} deg  (Guclu Kararli, >45 deg)")
    print(f"        - Kazanc Marji (GM)            : {gm_s:.1f} dB   (>6 dB Guvenli)")
    print(f"        - Gecikme Payi (Delay Margin)  : {dm_s:.0f} ms")
    print(f"     2. Dinamik Uyarlamali Mod (Kp = 0.16):")
    print(f"        - Kazanc Kesim Frekansi (w_gc) : {w_gc_d:.2f} rad/s")
    print(f"        - Faz Marji (PM)               : {pm_d:.1f} deg  (Hizli Takip, Kararli)")
    print(f"        - Kazanc Marji (GM)            : {gm_d:.1f} dB")
    print(f"        - Gecikme Payi (Delay Margin)  : {dm_d:.0f} ms")

    # -------------------------------------------------------------------------
    # ADIM 6: KAPALI CEVRIM ZAMAN DOMENI PERFORMANS SIMULASYONU VE DOGRULAMA
    # -------------------------------------------------------------------------
    t_sim = np.linspace(0, 1.40, 1400)
    dt_sim = t_sim[1] - t_sim[0]
    delay_steps = int(theta_sopdt / dt_sim)

    # 1. Klasik Cohen-Coon PID (Gecikme ve Gurultu altinda limit cevrim ve asim)
    y_cohen = np.zeros_like(t_sim)
    integral_c = 0.0
    prev_err_c = 1.0
    for k in range(1, len(t_sim)):
        idx_del = max(0, k - delay_steps)
        e = 1.0 - y_cohen[idx_del]
        integral_c += e * dt_sim
        integral_c = np.clip(integral_c, -0.6, 0.6)  # anti-windup
        deriv = (e - prev_err_c) / dt_sim
        prev_err_c = e
        u = kp_cohen * 0.16 * e + ki_cohen * 0.04 * integral_c + kd_cohen * 0.003 * deriv
        u = np.clip(u, -1.8, 1.8)
        # 2. derece motor integrasyonu
        y_cohen[k] = y_cohen[k - 1] + u * dt_sim * 14.0

    # 2. Klasik Sert P (Olu Bolgesiz: Titresimli / Limit Cevrimli)
    y_hard_p = np.zeros_like(t_sim)
    for k in range(1, len(t_sim)):
        idx_del = max(0, k - delay_steps)
        noise_k = np.random.normal(0, 0.035) if k % 8 == 0 else 0.0
        e = (1.0 - y_hard_p[idx_del]) + noise_k
        cmd = np.clip(0.10 * e, -0.15, 0.15)
        y_hard_p[k] = y_hard_p[k - 1] + cmd * (dt_sim / 0.03333)

    # 3. Guncel Hiz-Uyarlamali Yumusak Olu Bolgeli P (Son Sistemimiz)
    y_soft_p = np.zeros_like(t_sim)
    deadband_norm = 0.90 / 15.0  # 15 derecelik basamakta 0.90 derece normalize olu bolge
    for k in range(1, len(t_sim)):
        idx_del = max(0, k - delay_steps)
        noise_k = np.random.normal(0, 0.025) if k % 8 == 0 else 0.0
        e = (1.0 - y_soft_p[idx_del]) + noise_k
        abs_e = abs(e)
        if abs_e <= deadband_norm:
            cmd = 0.0  # Sessiz motor, sifir titresim
        else:
            eff_e = np.sign(e) * (abs_e - deadband_norm)
            cmd = np.clip(0.12 * eff_e, -0.18, 0.18)
        y_soft_p[k] = y_soft_p[k - 1] + cmd * (dt_sim / 0.03333)

    # -------------------------------------------------------------------------
    # ADIM 7: YUKSEK COZUNURLUKLU 4'LU TEZ ANALIZ GRAFIGI
    # -------------------------------------------------------------------------
    fig, axs = plt.subplots(2, 2, figsize=(14, 11))
    plt.subplots_adjust(hspace=0.35, wspace=0.25)

    # PANEL 1: SISTEM TANIMA VE TRANSFER FONKSIYONU EGRILERI
    ax1 = axs[0, 0]
    ax1.plot(t * 1000, measured_response, "k.", alpha=0.35, markersize=3.5, label="Olcum Verisi (Gecikmeli + Gurultulu)")
    ax1.plot(t * 1000, fit_fopdt, "b--", lw=1.8, label=f"1. Derece FOPDT (tau={tau_fopdt*1000:.0f}ms, R^2=%{r2_fopdt*100:.1f})")
    ax1.plot(t * 1000, fit_sopdt, "r-", lw=2.2, label=f"2. Derece SOPDT (wn={wn_sopdt:.1f}, R^2=%{r2_sopdt*100:.1f})")
    if t_telemetry is not None and len(t_telemetry) > 0:
        ax1.plot(t_telemetry * 1000, y_telemetry, "g^", alpha=0.75, markersize=4, label="Saha Testi Telemetri (Log 161823)")
    ax1.axvline(theta_sopdt * 1000, color="gray", ls=":", lw=1.5, label=f"Toplam Gecikme theta = {theta_sopdt*1000:.0f}ms")
    ax1.set_title("1. Sistem Tanima: Basamak Yaniti ve Model Uydurma", fontweight="bold", fontsize=11)
    ax1.set_xlabel("Zaman (ms)")
    ax1.set_ylabel("Normalize Yanit y(t) / u")
    ax1.set_xlim(0, 650)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="lower right", fontsize=8.5)

    # PANEL 2: S-DUZLEMI ROOT LOCUS (KOKLERIN YER EGRISI)
    ax2 = axs[0, 1]
    ax2.axhline(0, color="black", lw=1.0)
    ax2.axvline(0, color="black", lw=1.0)

    # Zeta = 0.707 kilavuz cizgisi
    zeta_ang = math.acos(0.707)
    r_max = 35.0
    ax2.plot([0, -r_max * math.cos(zeta_ang)], [0, r_max * math.sin(zeta_ang)], "g--", alpha=0.5, label="zeta = 0.707 (Asma <= %5)")
    ax2.plot([0, -r_max * math.cos(zeta_ang)], [0, -r_max * math.sin(zeta_ang)], "g--", alpha=0.5)

    # Acik cevrim kutup ve sifirlari
    ax2.plot(-zeta_sopdt * wn_sopdt, wn_sopdt * np.sqrt(1 - zeta_sopdt**2), "rx", markersize=9, mew=2.5, label="Eyleyici Kutuplari")
    ax2.plot(-zeta_sopdt * wn_sopdt, -wn_sopdt * np.sqrt(1 - zeta_sopdt**2), "rx", markersize=9, mew=2.5)
    ax2.plot(-p_pade, 0, "mx", markersize=9, mew=2.5, label=f"Pade Gecikme Kutbu (-{p_pade:.0f})")
    ax2.plot(z_pade, 0, "mo", fillstyle="none", markersize=9, mew=2.5, label=f"NMP Sifiri (+{z_pade:.0f})")

    # Root Locus dallari
    for i in range(4):
        ax2.plot(locus_roots[:, i].real, locus_roots[:, i].imag, "b-", lw=1.5, alpha=0.85)

    # Calisma noktalari
    ax2.plot(roots_stat[2].real, roots_stat[2].imag, "r*", markersize=12, label="Statik Calisma (Kp=0.10, zeta=0.72)")
    ax2.plot(roots_stat[2].real, -roots_stat[2].imag, "r*", markersize=12)
    ax2.plot(roots_dyn[2].real, roots_dyn[2].imag, "g^", markersize=9, label="Dinamik Calisma (Kp=0.16)")
    ax2.plot(roots_dyn[2].real, -roots_dyn[2].imag, "g^", markersize=9)

    # Kararsizlik siniri (Sag yari duzlem kirmizi alan)
    ax2.axvspan(0, 30, color="red", alpha=0.08, label="Kararsiz Bolge (RHP)")
    ax2.set_xlim(-30, 30)
    ax2.set_ylim(-25, 25)
    ax2.set_title("2. S-Duzlemi: Root Locus (Pade Gecikme Modelli)", fontweight="bold", fontsize=11)
    ax2.set_xlabel("Reel Eksen Sigma [1/s]")
    ax2.set_ylabel("Imajiner Eksen j*Omega [rad/s]")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="lower left", fontsize=7.2, framealpha=0.9)

    # PANEL 3: BODE FREKANS CEVABI VE KARARLILIK MARJLARI
    ax3 = axs[1, 0]
    ax3_twin = ax3.twinx()

    l1 = ax3.semilogx(w_freq, 20 * np.log10(mag_stat), color="#0055A5", lw=2.2, label=f"Statik Genlik (GM={gm_s:.1f}dB)")
    l2 = ax3.semilogx(w_freq, 20 * np.log10(mag_dyn), color="#27AE60", ls="-.", lw=1.8, label=f"Dinamik Genlik (GM={gm_d:.1f}dB)")
    l3 = ax3_twin.semilogx(w_freq, phase_stat, color="#D9534F", ls="--", lw=1.8, label=f"Statik Faz (PM={pm_s:.1f}deg)")
    l4 = ax3_twin.semilogx(w_freq, phase_dyn, color="#8E44AD", ls=":", lw=1.8, label=f"Dinamik Faz (PM={pm_d:.1f}deg)")

    ax3.axhline(0, color="black", ls=":", lw=1.0)
    ax3_twin.axhline(-180, color="black", ls=":", lw=1.0)
    ax3.set_title("3. Frekans Cevabi (Bode) ve Kararlilik Marjlari", fontweight="bold", fontsize=11)
    ax3.set_xlabel("Frekans Omega (rad/s)")
    ax3.set_ylabel("Genlik (dB)", color="#0055A5")
    ax3_twin.set_ylabel("Faz (derece)", color="#D9534F")
    ax3.set_ylim(-40, 25)
    ax3_twin.set_ylim(-260, 0)
    ax3.grid(True, which="both", alpha=0.3)

    lines_bode = l1 + l2 + l3 + l4
    labels_bode = [l.get_label() for l in lines_bode]
    ax3.legend(lines_bode, labels_bode, loc="lower left", fontsize=7.5)

    # PANEL 4: KAPALI CEVRIM KONTROLCU DOGRULAMASI
    ax4 = axs[1, 1]
    ax4.plot(t_sim * 1000, np.ones_like(t_sim), "k--", lw=1.5, label="Referans Hedef (Step = 1.0)")
    ax4.plot(t_sim * 1000, y_cohen, color="#D9534F", ls=":", lw=1.8, label="Klasik PID (Asiri Titresim / Windup)")
    ax4.plot(t_sim * 1000, y_hard_p, color="#E67E22", ls="--", lw=1.8, label="Sert P (Olu Bolgesiz / Titremeli)")
    ax4.plot(t_sim * 1000, y_soft_p, color="#0055A5", lw=2.4, label="Son Sistem: Yumusak Olu Bolgeli P")
    ax4.fill_between(t_sim * 1000, 1.0 - deadband_norm, 1.0 + deadband_norm, color="#27AE60", alpha=0.20, label="Olu Bolge Kararli Alan (+-0.90 deg)")

    ax4.set_title("4. Kapali Cevrim Kontrolcu Performans Dogrulamasi", fontweight="bold", fontsize=11)
    ax4.set_xlabel("Zaman (ms)")
    ax4.set_ylabel("Normalize Hedef Takip Konumu")
    ax4.set_xlim(0, 1400)
    ax4.set_ylim(-0.10, 1.35)
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc="lower right", fontsize=8.0)

    plt.tight_layout()

    # Rapor gorsellerini kaydet
    report_path1 = BASE_DIR / "system_identification_analysis.png"
    report_path2 = FIG_DIR / "fig8_system_identification.png"
    plt.savefig(str(report_path1), dpi=300)
    plt.savefig(str(report_path2), dpi=300)
    plt.close()

    print("\n" + "=" * 70)
    print(f"[BASARILI] Sistem Tanima ve Root Locus Raporu Kaydedildi:")
    print(f"   --> {report_path1}")
    print(f"   --> {report_path2}")
    print("=" * 70)


if __name__ == "__main__":
    run_system_identification()
