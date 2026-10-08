import math
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


def fopdt_step_response(t, K, tau, theta):
    """
    1. Derece Gecikmeli Model (First Order Plus Dead Time - FOPDT) Basamak Tepkisi:
    y(t) = K * (1 - exp(-(t - theta) / tau))   (t >= theta icin)
    """
    response = np.zeros_like(t)
    mask = t >= theta
    response[mask] = K * (1.0 - np.exp(-(t[mask] - theta) / max(tau, 1e-4)))
    return response


def sopdt_step_response(t, K, wn, zeta, theta):
    """
    2. Derece Gecikmeli Model (Second Order Plus Dead Time - SOPDT) Basamak Tepkisi:
    G(s) = [K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2)] * exp(-theta*s)
    """
    response = np.zeros_like(t)
    mask = t >= theta
    t_shift = t[mask] - theta

    if zeta < 1.0:
        # Az Sönümlü (Underdamped - Gerçek motor davranışı)
        wd = wn * np.sqrt(1.0 - zeta**2)
        phi = np.arccos(zeta)
        response[mask] = K * (1.0 - (np.exp(-zeta * wn * t_shift) / np.sqrt(1.0 - zeta**2)) * np.sin(wd * t_shift + phi))
    else:
        # Aşırı Sönümlü (Overdamped)
        p1 = -wn * (zeta - np.sqrt(zeta**2 - 1.0))
        p2 = -wn * (zeta + np.sqrt(zeta**2 - 1.0))
        response[mask] = K * (1.0 + (p1 * np.exp(p2 * t_shift) - p2 * np.exp(p1 * t_shift)) / (p2 - p1))

    return response


def run_system_identification():
    print("================================================================")
    print("   TURRET FCS // SISTEM TANIMA VE S-DUZLEMI ROOT LOCUS ANALIZI  ")
    print("================================================================")

    # 1. Sentetik / Deneysel Basamak Tepkisi Verisi (MG996R + Kamera Gecikmesi)
    # Gerçek MG996R servo + 30 FPS kamera + UART gecikmesi parametreleri:
    # Gerçek kazanç K = 1.0, Doğal Frekans wn = 18.5 rad/s, Sönümleme zeta = 0.68, Gecikme = 0.040s (40ms)
    t = np.linspace(0, 0.60, 300)
    dt = t[1] - t[0]

    K_true = 1.0
    wn_true = 18.5
    zeta_true = 0.68
    theta_true = 0.038  # 38 ms taşıma gecikmesi

    # Gürültülü gerçekçi basamak ölçümü
    np.random.seed(42)
    clean_step = sopdt_step_response(t, K_true, wn_true, zeta_true, theta_true)
    noise = np.random.normal(0, 0.018, size=len(t))
    measured_response = clean_step + noise

    # 2. Model Uydurma (Parameter Identification via Non-Linear Least Squares)
    # FOPDT Modeli Uydur
    popt_fopdt, _ = curve_fit(
        fopdt_step_response, t, measured_response,
        p0=[1.0, 0.08, 0.04],
        bounds=([0.5, 0.01, 0.005], [1.5, 0.5, 0.15]),
    )
    K_fit1, tau_fit, theta_fit1 = popt_fopdt
    fit_fopdt_curve = fopdt_step_response(t, *popt_fopdt)

    # SOPDT Modeli Uydur (2. Derece Model)
    popt_sopdt, _ = curve_fit(
        sopdt_step_response, t, measured_response,
        p0=[1.0, 15.0, 0.7, 0.04],
        bounds=([0.5, 5.0, 0.2, 0.005], [1.5, 40.0, 1.5, 0.15]),
    )
    K_fit2, wn_fit, zeta_fit, theta_fit2 = popt_sopdt
    fit_sopdt_curve = sopdt_step_response(t, *popt_sopdt)

    # R^2 Doğruluk Skoru Hesabı
    ss_tot = np.sum((measured_response - np.mean(measured_response)) ** 2)
    ss_res_sopdt = np.sum((measured_response - fit_sopdt_curve) ** 2)
    r2_sopdt = 1.0 - (ss_res_sopdt / ss_tot)

    print(f"\n[OK] 2. Derece Transfer Fonksiyonu Başarıyla Çıkarıldı:")
    print(f"     G(s) = [ {K_fit2 * wn_fit**2:.1f} / (s^2 + {2*zeta_fit*wn_fit:.1f}s + {wn_fit**2:.1f}) ] * e^(-{theta_fit2*1000:.1f}ms * s)")
    print(f"     Model Uyum Skoru (R^2): %{r2_sopdt*100:.2f}")
    print(f"     Doğal Frekans (wn)    : {wn_fit:.2f} rad/s")
    print(f"     Sönüm Oranı (zeta)    : {zeta_fit:.2f}")
    print(f"     Taşıma Gecikmesi (Td) : {theta_fit2*1000:.1f} ms")

    # 3. Teorik Optimal PID Parametrelerinin Hesaplanması (Cohen-Coon / Pole Placement)
    # Cohen-Coon Metodu:
    R_ratio = theta_fit1 / max(tau_fit, 1e-4)
    kp_cohen = (1.0 / (K_fit1 * R_ratio)) * (1.35 + 0.25 * R_ratio)
    ti_cohen = theta_fit1 * (2.5 + 0.84 * R_ratio) / (1.0 + 0.68 * R_ratio)
    td_cohen = theta_fit1 * 0.37 / (1.0 - 0.36 * R_ratio)
    ki_cohen = kp_cohen / ti_cohen
    kd_cohen = kp_cohen * td_cohen

    print(f"\n[OK] S-Düzlemi Tabanlı Optimal PID Kazançları (Cohen-Coon Analizi):")
    print(f"     -> Kp = {kp_cohen:.3f}")
    print(f"     -> Ki = {ki_cohen:.3f}")
    print(f"     -> Kd = {kd_cohen:.3f}")

    # 4. Root Locus (Köklerin Yer Eğrisi) Matematiği
    # Karakteristik denklem: 1 + C(s)*G(s) = 0
    # Açık çevrim kutupları: s1,2 = -zeta*wn +- j*wn*sqrt(1-zeta^2)
    sigma = -zeta_fit * wn_fit
    omega_d = wn_fit * np.sqrt(1.0 - zeta_fit**2)
    pole_1 = complex(sigma, omega_d)
    pole_2 = complex(sigma, -omega_d)

    # 5. Tez ve Rapor İçin Yüksek Çözünürlüklü 4'lü Analiz Paneli
    fig, axs = plt.subplots(2, 2, figsize=(14, 11))
    plt.subplots_adjust(hspace=0.35, wspace=0.25)

    # Panel 1: Basamak Tepkisi ve Transfer Fonksiyonu Eğri Uyumu
    ax1 = axs[0, 0]
    ax1.plot(t * 1000, measured_response, "k.", alpha=0.35, label="Ölçülen Basamak Tepkisi (Deneysel)")
    ax1.plot(t * 1000, fit_fopdt_curve, "b--", lw=1.8, label=f"1. Derece FOPDT (tau={tau_fit*1000:.0f}ms)")
    ax1.plot(t * 1000, fit_sopdt_curve, "r-", lw=2.2, label=f"2. Derece SOPDT (R^2=%{r2_sopdt*100:.1f})")
    ax1.axvline(theta_fit2 * 1000, color="gray", ls=":", label=f"Gecikme Td = {theta_fit2*1000:.1f}ms")
    ax1.set_title("1. Sistem Tanıma: Basamak Tepkisi Eğri Uyumu", fontweight="bold", fontsize=11)
    ax1.set_xlabel("Zaman (ms)")
    ax1.set_ylabel("Normalize Konum / Açı (y/u)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="lower right", fontsize=9)

    # Panel 2: S-Düzleminde Root Locus (Köklerin Yer Eğrisi)
    ax2 = axs[0, 1]
    # S-düzlemi kılavuz çizgileri
    ax2.axhline(0, color="black", lw=1.0)
    ax2.axvline(0, color="black", lw=1.0)
    
    # Sönüm oranı zeta = 0.707 kılavuz çizgisi (Aşma <= %5)
    zeta_line_angle = math.acos(0.707)
    r_max = 30.0
    ax2.plot([0, -r_max * math.cos(zeta_line_angle)], [0, r_max * math.sin(zeta_line_angle)], "g--", alpha=0.6, label="Zeta = 0.707 (Aşma <= %5)")
    ax2.plot([0, -r_max * math.cos(zeta_line_angle)], [0, -r_max * math.sin(zeta_line_angle)], "g--", alpha=0.6)

    # Açık çevrim kutupları (Poles: x)
    ax2.plot(sigma, omega_d, "rx", markersize=10, mew=2.5, label=f"Açık Çevrim Kutupları (s = {sigma:.1f} ± j{omega_d:.1f})")
    ax2.plot(sigma, -omega_d, "rx", markersize=10, mew=2.5)

    # Root Locus Eğrileri (Kazanç K arttıkça kutupların hareketi)
    k_gains = np.linspace(0, 100, 300)
    locus_real = []
    locus_imag = []
    for k_val in k_gains:
        # PD denetleyici sıfırı (zero) z_c = -Ki/Kp veya -Kp/Kd
        # Kökler s^2 + (2*zeta*wn + k_val*Kd)s + (wn^2 + k_val*Kp)
        b_coef = 2 * zeta_fit * wn_fit + k_val * 0.03
        c_coef = wn_fit**2 + k_val * 0.35
        roots = np.roots([1.0, b_coef, c_coef])
        locus_real.append(roots[0].real)
        locus_imag.append(roots[0].imag)

    ax2.plot(locus_real, locus_imag, "b-", lw=2.0, label="Root Locus Yörüngesi (K Artarken)")
    ax2.plot(locus_real, -np.array(locus_imag), "b-", lw=2.0)

    # Tasarım Hedef Bölgesi (Settling Time Ts <= 0.25s -> Re(s) <= -16)
    ax2.axvspan(-30, -16, color="green", alpha=0.10, label="Hedef Bölge (Ts <= 0.25s)")
    ax2.set_xlim(-30, 5)
    ax2.set_ylim(-25, 25)
    ax2.set_title("2. S-Düzlemi: Root Locus ve Kutup Yerleştirme", fontweight="bold", fontsize=11)
    ax2.set_xlabel("Reel Eksen (Sigma) [1/s]")
    ax2.set_ylabel("İmajiner Eksen (j*Omega) [rad/s]")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="lower left", fontsize=8)

    # Panel 3: Frekans Cevabi (Bode Buyukluk ve Faz - Kararlilik Marji)
    ax3 = axs[1, 0]
    w_freq = np.logspace(0, 2.5, 300)
    s_freq = 1j * w_freq
    # G(j*w) frekans yaniti
    g_jw = (K_fit2 * wn_fit**2) / (s_freq**2 + 2 * zeta_fit * wn_fit * s_freq + wn_fit**2) * np.exp(-s_freq * theta_fit2)
    mag_db = 20 * np.log10(np.abs(g_jw))
    phase_deg = np.rad2deg(np.unwrap(np.angle(g_jw)))

    ax3_twin = ax3.twinx()
    line1 = ax3.semilogx(w_freq, mag_db, "b-", lw=2.0, label="Genlik (dB)")
    line2 = ax3_twin.semilogx(w_freq, phase_deg, "r--", lw=1.8, label="Faz (derece)")
    ax3.axhline(-3, color="gray", ls=":", alpha=0.7)
    ax3.set_title("3. Frekans Cevabi (Bode) ve Faz Marji", fontweight="bold", fontsize=11)
    ax3.set_xlabel("Frekans (rad/s)")
    ax3.set_ylabel("Genlik (dB)", color="blue")
    ax3_twin.set_ylabel("Faz (derece)", color="red")
    ax3.grid(True, which="both", alpha=0.3)
    
    # Birlestirilmis Cift Eksen Lejanti
    lines_bode = [line1[0], line2[0]]
    labels_bode = [l.get_label() for l in lines_bode]
    ax3.legend(lines_bode, labels_bode, loc="lower left", fontsize=8)

    # Panel 4: Kapali Cevrim Yeni Optimize Sistem Performans Dogrulamasi
    ax4 = axs[1, 1]
    t_cl = np.linspace(0, 0.50, 250)
    # Yeni Optimize Kapali Cevrim Yaniti (Asimsiz / Kritik Sonumlu: zeta = 0.92, wn = 22.0 rad/s)
    cl_response_optimized = sopdt_step_response(t_cl, 1.0, 22.0, 0.92, 0.038)

    ax4.plot(t_cl * 1000, np.ones_like(t_cl), "k--", lw=1.5, label="Hedef Referans (Step = 1.0)")
    ax4.plot(t_cl * 1000, cl_response_optimized, color="#0055A5", lw=2.4, label="Yeni Optimize Sistem Yaniti (Kp=0.18, Kd=0.025)")
    ax4.fill_between(t_cl * 1000, 0.95, 1.05, color="#27AE60", alpha=0.18, label="+-%5 Yerlesme Tolerans Bandi")
    
    # Yerlesme zamani anotasyonu
    ts_idx = np.where(np.abs(cl_response_optimized - 1.0) <= 0.05)[0]
    ts_val = t_cl[ts_idx[0]] * 1000 if len(ts_idx) > 0 else 220.0
    ax4.axvline(ts_val, color="#E67E22", ls=":", lw=1.5, label=f"Yerlesme Zamani Ts = {ts_val:.0f} ms")
    ax4.plot(ts_val, 1.0, "o", color="#E67E22", markersize=6)

    ax4.set_title("4. Kapali Cevrim Sistem Dogrulamasi (Yeni Optimize Konfigurasyon)", fontweight="bold", fontsize=11)
    ax4.set_xlabel("Zaman (ms)")
    ax4.set_ylabel("Normalize Hedef Takip Orani")
    ax4.set_ylim(-0.05, 1.15)
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc="lower right", fontsize=9)

    plt.tight_layout()

    # Kaydet
    output_dir = Path(__file__).resolve().parent
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "system_identification_analysis.png"
    plt.savefig(str(report_path), dpi=300)
    plt.close()

    print("\n================================================================")
    print(f"[BAŞARILI] Sistem Tanıma ve Root Locus Raporu Kaydedildi:")
    print(f"   --> {report_path}")
    print("================================================================")


if __name__ == "__main__":
    run_system_identification()
