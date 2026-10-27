"""
================================================================================
MODUL: WEBOTS OTONOM ATIS KONTROL SISTEMI ANA DONGUSU (WEBOTS MAIN)
================================================================================
Bu modul, Turret FCS uretim mimarisini (FCSManager, P, RL, CV Kalman,
histerezisli LockManager ve TurretHUD) Webots simulasyon ortaminda kosturan
ince donanim adaptoru ve ana calistiricisidir.

CALISMA ZAMANI SOZLESMESI:
- Tesis Dogrulugu: Inkremental aci integrasyonu (theta += delta_theta), 46.9 deg/s limit (1.50 deg / 0.032 s).
- Kontrolcu: Uretim FCSManager; varsayilan P (Kp=0.22/0.11).
- RL: saf PPO, 11-64-64-2 NumPy politika; P harmani yok.
- Mod Yapisi: Ayrik 'P' veya 'RL'; WEBOTS_CONTROL_MODE ile baslangic modu secilir.
- Zaman Tabani: Saf simulasyon zamani (sim_t = hw.get_time()).
- Atis Emniyeti: 'Y' yalnizca kilitli, menzili dogrulanmis ve sekans bostayken atesler.
- Atis Protokolu: SPINUP/SLEW + SETTLE 0.15 s + 80 ms solenoid + RECOVER 0.35 s + EVAL 1.5 s.
- E-Stop: Bosluk tusu mandalli acil durdurma; motorlar merkezlenir, launcher kesilir.
================================================================================
"""

import sys
import math
import os
from pathlib import Path
import cv2

# Proje kokunu ve venv site-packages'i sys.path'e ekle (Webots dahili Python korumasi)
project_root = Path(__file__).resolve().parents[3]
turret_fcs_dir = project_root / "turret_fcs"
venv_site = project_root / "venv" / "Lib" / "site-packages"

for p in [str(project_root), str(turret_fcs_dir), str(venv_site)]:
    if p not in sys.path and Path(p).exists():
        sys.path.insert(0, p)

from ultralytics import YOLO

from turret_fcs.config import CONFIG, FCSState
from turret_fcs.fcs_manager import FCSManager
from turret_fcs.turret_hud import TurretHUD
from turret_fcs.engagement import EngagementManager
from turret_fcs.presenter import HUDPresenter
from turret_fcs.keymap import process_key_event

from webots_hardware_interface import WebotsHardwareInterface
from webots_flywheel_launcher import WebotsFlywheelLauncher
from webots_virtual_ballistics import VirtualBallisticsEvaluator
from webots_telemetry_logger import WebotsTelemetryLogger
from webots_tuning import apply_tuning, load_tuning


def main():

    # 1. Webots Donanim ve Atis Firlaticisini Baslat
    hw = WebotsHardwareInterface()
    launcher = WebotsFlywheelLauncher(hw.robot)
    hud = TurretHUD()
    presenter = HUDPresenter(hud)
    start_mode = os.environ.get("WEBOTS_CONTROL_MODE", "P").strip().upper()
    fcs = FCSManager(control_mode=start_mode if start_mode in ("P", "RL") else "P", patrol_enabled=False)
    engagement = EngagementManager()
    tuning = load_tuning()
    apply_tuning(fcs, tuning)
    ballistics = VirtualBallisticsEvaluator(bullet_speed=CONFIG.ballistics.projectile_speed_mps, supervisor=hw.robot)
    telemetry = WebotsTelemetryLogger(hw.robot) if CONFIG.telemetry_enabled else None

    # 2. YOLO Hedef Tespit Modelini Yukle (CONFIG SSOT yollariyla)
    model_path = project_root / CONFIG.paths.yolo_model
    if not model_path.exists():
        model_path = project_root / CONFIG.paths.fallback_yolo_model
    model = YOLO(str(model_path))

    WINDOW_NAME = "Turret FCS - Webots Digital Twin"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    # 3. Optik Kamera Parametreleri (Logitech C270 SSOT - CONFIG)
    fx = CONFIG.optics.compute_fx(hw.w_cam)
    fy = CONFIG.optics.compute_fy(hw.h_cam)
    center_x, center_y = hw.w_cam // 2, hw.h_cam // 2

    # 4. Durum Bayraklari
    is_estop = False
    prev_phase = None
    frame_id = 0

    hud.log_event("[SYS] WEBOTS TURRET FCS DIGITAL TWIN ONLINE", color=(0, 255, 128))
    hud.log_event("[CAM] C270 VIRTUAL OPTICS (1280x720, FOV: 48.8 deg)", color=(0, 255, 128))
    hud.log_event("[CTRL] P + RL READY (Ki=0, Kd=0)", color=(0, 215, 255))
    hud.log_event(
        f"[TUNE] {tuning['label']} | Kp={tuning['kp_pan']:.3f}/{tuning['kp_tilt']:.3f} "
        f"MAX={tuning['max_output_deg']:.2f}",
        color=(0, 215, 255),
    )

    try:
        while hw.step() != -1:
            frame_id += 1
            sim_t = hw.get_time()

            # 80 ms gecikmeli gozlem: goruntu + servo pozu + LiDAR ayni karede
            frame, curr_pan_deg, curr_tilt_deg, dist_cm, strength = hw.get_observation()
            if frame is None:
                continue

            fps = int(round(1.0 / max(0.001, hw.dt)))
            fcs.sync_turret_pose(curr_pan_deg, curr_tilt_deg)

            # YOLO Takip (ByteTrack ile)
            conf_val = fcs.selector.min_conf
            results = model.track(
                frame,
                imgsz=CONFIG.selector.yolo_imgsz,
                conf=conf_val,
                persist=True,
                tracker=CONFIG.selector.tracker_config,
                verbose=False,
            )
            boxes = results[0].boxes if len(results) > 0 else []

            # 2 Asamali Balistik Angajman Yonetimi (SSOT FSM)
            eng_state = engagement.update(sim_t, fcs.is_locked)

            # Volan ve Solenoid Kenar Tetik Gecisleri
            if prev_phase != eng_state.phase:
                if eng_state.phase == "SPINUP":
                    launcher.start_spinup()
                elif eng_state.phase == "FIRING":
                    # Uretim doktrini ile uyumlu isabet kapisi kontrolu
                    if not fcs.is_locked or fcs.hit_probability < CONFIG.lock.min_hit_probability_pct:
                        engagement.abort()
                        launcher.stop_flywheels()
                        hud.log_event(
                            f"[FIRE ABORTED] HIT PROB TOO LOW ({fcs.hit_probability:.1f}% < {CONFIG.lock.min_hit_probability_pct:.0f}%)",
                            color=(0, 0, 255),
                        )
                    else:
                        launcher.trigger()
                        ballistics.fire_shot(dist_m=fcs.last_valid_target_dist_m, sim_time=sim_t)
                elif eng_state.phase in ("EVAL", None):
                    launcher.stop_flywheels()
                prev_phase = eng_state.phase

            # Hakem ve Firlatici Simulasyon Guncellemeleri
            launcher.update(sim_time=sim_t)
            ballistics.update(sim_time=sim_t)

            # Hedef Secimi ve Algilama Titremesi (Gercek kameraya yakin simulasyon)
            best_box = fcs.select_best_target(boxes, center_x, center_y)
            if best_box is not None:
                jitter_px = float(os.environ.get("WEBOTS_BBOX_JITTER", "3.0"))
                if jitter_px > 0.0:
                    jx = jitter_px * (0.65 * math.sin(7.3 * sim_t) + 0.35 * math.sin(17.1 * sim_t))
                    jy = jitter_px * (0.60 * math.sin(9.7 * sim_t + 0.7) + 0.40 * math.sin(21.3 * sim_t))
                    b = list(best_box)
                    b[0] = max(0.0, b[0] + jx)
                    b[1] = max(0.0, b[1] + jy)
                    b[2] = max(b[0] + 2.0, b[2] + jx)
                    b[3] = max(b[1] + 2.0, b[3] + jy)
                    best_box = b

            # FCS Karar ve Kontrol Dongusu
            pan_cmd, tilt_cmd = fcs.update(
                best_box, center_x, center_y, fx, fy, hw.dt,
                is_firing=eng_state.is_physical_firing,
                dist_cm=dist_cm,
                slew_factor=eng_state.slew_factor,
            )

            # Flywheel idle: hedef kilitliyken bosta dondur, atista boost et
            if not eng_state.is_active and not is_estop:
                if fcs.is_locked:
                    launcher.start_idle()
                else:
                    launcher.stop_flywheels()

            # FCS Durum Bayragi
            fcs_state = FCSState.ESTOP if is_estop else eng_state.fcs_fire_cmd_state

            # E-STOP veya Normal Motor Surusu
            if is_estop:
                hw.set_neutral()
            else:
                hw.apply_delta_commands(pan_cmd, tilt_cmd)

            # Telemetri Kaydi (Schema v2)
            if telemetry:
                telemetry.log(
                    frame_id, sim_t, hw.dt, fps, fcs, hw, best_box,
                    fire_stage=eng_state.phase,
                    fcs_state=fcs_state,
                    ballistics=ballistics,
                    tuning=tuning,
                )

            # Atis Hakem Geri Bildirimi (Webots Sanal Balistik Sonucu)
            ballistic_hud_data = None
            if eng_state.is_active:
                res_txt, res_col = ballistics.get_hud_result(sim_t)
                if res_txt:
                    ballistic_hud_data = (res_txt, res_col)
                elif eng_state.phase in ("FIRING", "RECOVER", "EVAL"):
                    ballistic_hud_data = ("AWAITING IMPACT", (0, 215, 255))

            # Standart Askeri HUD Sunumu
            presenter.render(
                frame=frame,
                fcs=fcs,
                engagement_state=eng_state,
                target_box=best_box,
                center_x=center_x,
                center_y=center_y,
                fps=fps,
                dist_cm=dist_cm,
                strength=strength,
                is_estop=is_estop,
                hit_feedback=ballistic_hud_data,
            )

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            # Operator Klavye ve Etkilesim Haritasi
            prev_estop = is_estop
            should_exit, is_estop = process_key_event(
                key_code=key,
                fcs=fcs,
                engagement=engagement,
                is_estop=is_estop,
                now=sim_t,
                is_range_valid=(fcs.range_valid and fcs.range_stable),
                is_flywheel_idle=launcher.is_idle_ready(),
                log_event_cb=hud.log_event,
            )
            if should_exit:
                hud.log_event("[SYS] WEBOTS SESSION TERMINATED", color=(180, 180, 180))
                break
            if is_estop and not prev_estop:
                launcher.stop_flywheels()
                hw.set_neutral()

    finally:
        if telemetry:
            telemetry.save()
        launcher.stop_flywheels()
        hw.set_neutral()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

