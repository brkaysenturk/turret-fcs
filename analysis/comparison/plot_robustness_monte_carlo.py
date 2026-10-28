#!/usr/bin/env python3
"""
================================================================================
MODUL: P CONTROLLER VS PPO RL MONTE-CARLO DAYANIKLILIK (ROBUSTNESS) GRAFIK MOTORU
================================================================================
Bu betik; degisen tasima gecikmesi (40 ms - 120 ms) ve degisen hedef manevra
siddeti altinda Tip-1 P Kontrolcusu ile PPO RL Kontrolcusunu 50 deterministik
tohum uzerinde Monte-Carlo simulasyonu ile karsilastirir.

Uretilen Grafik:
  analysis/figures/python/python_fig16_robustness_monte_carlo.png
================================================================================
"""

import math
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# Font ve stil ayarlari (Akademik format)
plt.rcParams['font.sans-serif'] = 'DejaVu Sans'
plt.rcParams['axes.edgecolor'] = '#333333'
plt.rcParams['axes.linewidth'] = 0.8

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from turret_fcs.config import CONFIG
from turret_fcs.p_controller import PController
from turret_fcs.rl_controller import RLController
from turret_fcs.rl.turret_env import TurretEnv

FIG_DIR = PROJECT_ROOT / "analysis" / "figures" / "python"
FIG_DIR.mkdir(parents=True, exist_ok=True)


def run_episode(env, controller_type, rl_ctrl=None, p_ctrl=None):
    """Tek bir bolumu (episode) isletir ve performans metriklerini doner."""
    obs, _ = env.reset()
    if controller_type == "P" and p_ctrl is not None:
        p_ctrl.on_target_lost()
    elif controller_type == "RL" and rl_ctrl is not None:
        rl_ctrl.reset()

    errors_deg = []
    actions_pan = []
    actions_tilt = []
    locked_steps = 0
    total_steps = 0

    for _ in range(env.max_steps):
        px, py = env.last_pixel_pos
        pan_err_deg = math.degrees(math.atan((px - env.center_x) / env.fx))
        tilt_err_deg = math.degrees(math.atan(-(py - env.center_y) / env.fy))
        total_err_deg = math.hypot(pan_err_deg, tilt_err_deg)
        errors_deg.append(total_err_deg)

        if (abs(pan_err_deg) <= CONFIG.control.deadband_pan_deg and
                abs(tilt_err_deg) <= CONFIG.control.deadband_tilt_deg):
            locked_steps += 1
        total_steps += 1

        if controller_type == "P":
            p_pan, p_tilt = p_ctrl.compute_angles(px, py, env.center_x, env.center_y, env.fx, env.fy)
            u_pan_deg, u_tilt_deg = p_ctrl.compute_p(p_pan, p_tilt, dt=env.dt)
            action = np.array([
                np.clip(u_pan_deg / max(1e-4, env.max_step_pan_deg), -1.0, 1.0),
                np.clip(u_tilt_deg / max(1e-4, env.max_step_tilt_deg), -1.0, 1.0)
            ], dtype=np.float32)
        else:
            if rl_ctrl is not None and rl_ctrl.is_loaded:
                d_pan, d_tilt = rl_ctrl.predict_angles(
                    px, py, env.center_x, env.center_y, env.screen_w, env.screen_h,
                    0.0, 0.0, CONFIG.ballistics.default_distance_m, pan_err_deg, tilt_err_deg, env.dt,
                )
                action = np.array([
                    np.clip(d_pan / max(1e-4, env.max_step_pan_deg), -1.0, 1.0),
                    np.clip(d_tilt / max(1e-4, env.max_step_tilt_deg), -1.0, 1.0)
                ], dtype=np.float32)
            else:
                action = np.array([
                    np.clip(pan_err_deg * 0.18 / env.max_step_pan_deg, -1.0, 1.0),
                    np.clip(tilt_err_deg * 0.12 / env.max_step_tilt_deg, -1.0, 1.0)
                ], dtype=np.float32)

        actions_pan.append(float(action[0]))
        actions_tilt.append(float(action[1]))
        _, _, done, truncated, _ = env.step(action)
        if done or truncated:
            break

    arr_err = np.array(errors_deg)
    rms_err = float(np.sqrt(np.mean(arr_err ** 2)))
    median_err = float(np.median(arr_err))
    lock_ratio = float((locked_steps / max(1, total_steps)) * 100.0)

    # Jerk / yon degisimi (flip rate)
    diffs_p = np.diff(np.sign(np.diff(actions_pan)))
    flip_rate = float(np.mean(diffs_p != 0)) if len(diffs_p) > 0 else 0.0

    return {
        "rms_err": rms_err,
        "median_err": median_err,
        "lock_ratio": lock_ratio,
        "flip_rate": flip_rate,
    }


def run_monte_carlo():
    """50 tohum uzerinde Monte Carlo dayaniklilik testlerini calistirir."""
    print("[1/2] Monte-Carlo ve Gecikme Dayaniklilik Simulasyonu Baslatiliyor...")

    rl_ctrl = RLController()
    p_ctrl = PController()

    # 1. Sabit Gecikmede 50 Tohumluk Dagitim (Seeds 100 to 149)
    seeds = list(range(100, 150))
    p_rms_list, rl_rms_list = [], []
    p_lock_list, rl_lock_list = [], []
    p_flip_list, rl_flip_list = [], []

    env = TurretEnv()
    for s in seeds:
        env.reset(seed=s)
        res_p = run_episode(env, "P", p_ctrl=p_ctrl)
        env.reset(seed=s)
        res_rl = run_episode(env, "RL", rl_ctrl=rl_ctrl)

        p_rms_list.append(res_p["rms_err"])
        rl_rms_list.append(res_rl["rms_err"])
        p_lock_list.append(res_p["lock_ratio"])
        rl_lock_list.append(res_rl["lock_ratio"])
        p_flip_list.append(res_p["flip_rate"])
        rl_flip_list.append(res_rl["flip_rate"])

    # 2. Degisen Tasima Gecikmesi Supurmesi (Latency Sweep: 33 ms to 120 ms)
    # Gecikme kuyrugu frame sayisi: 1, 2, 3, 4 kare (~33ms, ~67ms, ~90ms, ~120ms)
    latencies_ms = [33.3, 66.7, 90.0, 120.0]
    lat_p_rms, lat_rl_rms = [], []
    lat_p_lock, lat_rl_lock = [], []

    sweep_seeds = [101, 105, 110, 115, 120]
    from collections import deque
    for delay_frames in [1, 2, 3, 4]:
        sweep_p_rms, sweep_rl_rms = [], []
        sweep_p_lock, sweep_rl_lock = [], []
        for s in sweep_seeds:
            env_lat = TurretEnv(latency_steps=delay_frames, enable_randomization=False)
            env_lat.reset(seed=s)
            env_lat.latency_steps = delay_frames
            env_lat.obs_queue = deque(maxlen=delay_frames + 1)
            r_p = run_episode(env_lat, "P", p_ctrl=p_ctrl)

            env_lat.reset(seed=s)
            env_lat.latency_steps = delay_frames
            env_lat.obs_queue = deque(maxlen=delay_frames + 1)
            r_rl = run_episode(env_lat, "RL", rl_ctrl=rl_ctrl)

            sweep_p_rms.append(r_p["rms_err"])
            sweep_rl_rms.append(r_rl["rms_err"])
            sweep_p_lock.append(r_p["lock_ratio"])
            sweep_rl_lock.append(r_rl["lock_ratio"])

        lat_p_rms.append(float(np.mean(sweep_p_rms)))
        lat_rl_rms.append(float(np.mean(sweep_rl_rms)))
        lat_p_lock.append(float(np.mean(sweep_p_lock)))
        lat_rl_lock.append(float(np.mean(sweep_rl_lock)))

    return {
        "p_rms": p_rms_list, "rl_rms": rl_rms_list,
        "p_lock": p_lock_list, "rl_lock": rl_lock_list,
        "p_flip": p_flip_list, "rl_flip": rl_flip_list,
        "latencies_ms": latencies_ms,
        "lat_p_rms": lat_p_rms, "lat_rl_rms": lat_rl_rms,
        "lat_p_lock": lat_p_lock, "lat_rl_lock": lat_rl_lock,
    }


def generate_plots(data):
    """Sonuclari akademik 4 panelli sekil olarak kaydeder."""
    print("[2/2] Dayaniklilik Grafik Paneli Ciziliyor...")
    fig = plt.figure(figsize=(15, 10.5), dpi=300)
    gs = fig.add_gridspec(2, 2, hspace=0.36, wspace=0.25)

    c_p = "#1f77b4"      # Mavi (P Controller)
    c_rl = "#d62728"     # Kirmizi (RL Controller)

    # PANEL 1: 50 Tohum RMS Takip Hatasi Boxplot Dagitimi
    ax1 = fig.add_subplot(gs[0, 0])
    box_data = [data["p_rms"], data["rl_rms"]]
    bp = ax1.boxplot(
        box_data, patch_artist=True, tick_labels=["Tip-1 P Kontrolcu", "PPO RL Kontrolcu"],
        widths=0.45, medianprops=dict(color="black", lw=2.0),
    )
    bp['boxes'][0].set_facecolor(c_p)
    bp['boxes'][0].set_alpha(0.65)
    bp['boxes'][1].set_facecolor(c_rl)
    bp['boxes'][1].set_alpha(0.65)

    ax1.set_title("50 Monte-Carlo Tohumu Bileske RMS Hata Dagitimi", fontsize=11, fontweight="bold")
    ax1.set_ylabel("RMS Aci Hatasi (derece)", fontsize=10)
    ax1.grid(True, ls=":", alpha=0.6)

    # Sayisal aciklama (Kayan metinleri onleyen cerceveli rozetler)
    med_p = float(np.median(data["p_rms"]))
    med_rl = float(np.median(data["rl_rms"]))
    q3_p = float(np.percentile(data["p_rms"], 75))
    q3_rl = float(np.percentile(data["rl_rms"], 75))
    max_rms = max(max(data["p_rms"]), max(data["rl_rms"]))
    ax1.set_ylim(0.5, max_rms * 1.18)

    ax1.text(1, q3_p + 0.55, f"Medyan: {med_p:.2f}°", ha='center', va='bottom',
             bbox=dict(boxstyle='round,pad=0.35', fc='white', ec=c_p, lw=1.2, alpha=0.95),
             fontweight='bold', fontsize=9.5, color=c_p)
    ax1.text(2, q3_rl + 0.55, f"Medyan: {med_rl:.2f}°", ha='center', va='bottom',
             bbox=dict(boxstyle='round,pad=0.35', fc='white', ec=c_rl, lw=1.2, alpha=0.95),
             fontweight='bold', fontsize=9.5, color=c_rl)

    # PANEL 2: 50 Tohum Kilit Basari Orani (% Kilitte Kalma Suresi)
    ax2 = fig.add_subplot(gs[0, 1])
    box_lock = [data["p_lock"], data["rl_lock"]]
    bp2 = ax2.boxplot(
        box_lock, patch_artist=True, tick_labels=["Tip-1 P Kontrolcu", "PPO RL Kontrolcu"],
        widths=0.45, medianprops=dict(color="black", lw=2.0),
    )
    bp2['boxes'][0].set_facecolor(c_p)
    bp2['boxes'][0].set_alpha(0.65)
    bp2['boxes'][1].set_facecolor(c_rl)
    bp2['boxes'][1].set_alpha(0.65)

    ax2.set_title("Kilitlenme Basari Orani Dagitimi (% Zamansal Kilit)", fontsize=11, fontweight="bold")
    ax2.set_ylabel("Kilit Orani (%)", fontsize=10)
    ax2.grid(True, ls=":", alpha=0.6)

    l_med_p = float(np.median(data["p_lock"]))
    l_med_rl = float(np.median(data["rl_lock"]))
    q3_lock_p = float(np.percentile(data["p_lock"], 75))
    q3_lock_rl = float(np.percentile(data["rl_lock"], 75))
    max_lock = max(max(data["p_lock"]), max(data["rl_lock"]))
    ax2.set_ylim(-0.5, max(12.0, max_lock * 1.25))

    ax2.text(1, q3_lock_p + 1.2, f"Medyan: %{l_med_p:.1f}", ha='center', va='bottom',
             bbox=dict(boxstyle='round,pad=0.35', fc='white', ec=c_p, lw=1.2, alpha=0.95),
             fontweight='bold', fontsize=9.5, color=c_p)
    ax2.text(2, q3_lock_rl + 1.2, f"Medyan: %{l_med_rl:.1f}", ha='center', va='bottom',
             bbox=dict(boxstyle='round,pad=0.35', fc='white', ec=c_rl, lw=1.2, alpha=0.95),
             fontweight='bold', fontsize=9.5, color=c_rl)

    # PANEL 3: Tasima Gecikmesi (Latency) Dayaniklilik Egrisi
    ax3 = fig.add_subplot(gs[1, 0])
    lats = data["latencies_ms"]
    ax3.plot(lats, data["lat_p_rms"], marker='o', lw=2.2, color=c_p, label="P Kontrolcu RMS")
    ax3.plot(lats, data["lat_rl_rms"], marker='s', lw=2.2, color=c_rl, label="PPO RL Kontrolcu RMS")
    ax3.set_title("Tasima Gecikmesi Artisinda Takip Hatasi Artis Egilimi", fontsize=11, fontweight="bold")
    ax3.set_xlabel("Sistem Tasima Gecikmesi", fontsize=10)
    ax3.set_ylabel("Ortalama RMS Hata (derece)", fontsize=10)
    ax3.set_xticks(lats)
    ax3.set_xticklabels(["33 ms (1k)", "67 ms (2k)", "90 ms (3k)", "120 ms (4k)"])
    ax3.grid(True, ls=":", alpha=0.6)
    ax3.legend(loc="upper left", framealpha=0.95, facecolor="white")

    # PANEL 4: Gecikmeye Bagli Kilit Koruma Orani
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(lats, data["lat_p_lock"], marker='o', lw=2.2, color=c_p, label="P Kontrolcu Kilit %")
    ax4.plot(lats, data["lat_rl_lock"], marker='s', lw=2.2, color=c_rl, label="PPO RL Kontrolcu Kilit %")
    ax4.set_title("Tasima Gecikmesi Altinda Kilitlenme Kararliligi", fontsize=11, fontweight="bold")
    ax4.set_xlabel("Sistem Tasima Gecikmesi", fontsize=10)
    ax4.set_ylabel("Ortalama Kilit Orani (%)", fontsize=10)
    ax4.set_xticks(lats)
    ax4.set_xticklabels(["33 ms (1k)", "67 ms (2k)", "90 ms (3k)", "120 ms (4k)"])
    ax4.grid(True, ls=":", alpha=0.6)
    ax4.legend(loc="upper right", framealpha=0.95, facecolor="white")

    out_file = FIG_DIR / "python_fig16_robustness_monte_carlo.png"
    plt.savefig(out_file, bbox_inches="tight")
    plt.close()
    print(f"[+] Monte-Carlo dayaniklilik grafigi basariyla kaydedildi: {out_file}")


if __name__ == "__main__":
    data = run_monte_carlo()
    generate_plots(data)
