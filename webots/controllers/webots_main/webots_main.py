import time
from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO

from webots_hardware_interface import WebotsHardwareInterface
from webots_flywheel_launcher import WebotsFlywheelLauncher
from webots_fcs_manager import WebotsFCSManager
from webots_turret_hud import TurretHUD
from webots_virtual_ballistics import VirtualBallisticsEvaluator
from webots_telemetry_logger import WebotsTelemetryLogger


def main():
    # 1. Webots Donanim ve Atis Firlaticisini Baslat
    hw = WebotsHardwareInterface()
    launcher = WebotsFlywheelLauncher(hw.robot)
    hud = TurretHUD()
    fcs = WebotsFCSManager()
    ballistics = VirtualBallisticsEvaluator(bullet_speed=10.0, supervisor=hw.robot)
    logger = WebotsTelemetryLogger(hw.robot)

    # 2. YOLO Hedef Tespit Modelini Yukle
    model_path = Path(__file__).resolve().parents[3] / "runs/detect/balloon_detector/weights/best.pt"
    model = YOLO(str(model_path))

    WINDOW_NAME = "Turret FCS - Webots Predictive"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    # 3. Optik Kamera Parametreleri (Logitech C270)
    H_FOV, V_FOV = 53.0, 31.5
    fx = (hw.w_cam / 2.0) / np.tan(np.radians(H_FOV / 2.0))
    fy = (hw.h_cam / 2.0) / np.tan(np.radians(V_FOV / 2.0))
    center_x, center_y = hw.w_cam // 2, hw.h_cam // 2

    # 4. Atis ve Zamanlama Yonetimi
    was_firing = False
    fire_start_time = 0.0
    GUN_FIRE_DURATION = 0.40
    ENGAGING_DISPLAY_DURATION = 1.00
    BALLISTIC_DISPLAY_DURATION = 2.00
    FIRE_DISPLAY_DURATION = ENGAGING_DISPLAY_DURATION + BALLISTIC_DISPLAY_DURATION
    prev_frame_time = 0.0

    print("[SYS] TURRET_FCS_WEBOTS // BASLATILDI & OPERASYONEL (MODULER MIMARI)")

    try:
        while hw.step() != -1:
            frame = hw.get_frame()
            if frame is None:
                continue

            # FPS ve Zamanlama
            now = time.time()
            fps = int(1 / (now - prev_frame_time)) if (now - prev_frame_time) > 0 else 30
            prev_frame_time = now
            sim_t = hw.get_time()

            # Sensor Okumalari
            dist_cm, strength = hw.get_distance_and_strength()
            curr_p, curr_t = hw.get_motor_positions()

            # YOLO Takip
            conf_val = getattr(fcs.selector, "min_conf", 0.30)
            results = model.track(frame, imgsz=640, conf=conf_val, persist=True, tracker="bytetrack.yaml", verbose=False)
            boxes = results[0].boxes if len(results) > 0 else []

            # Atis Durumu
            is_gun_firing = (now - fire_start_time) < GUN_FIRE_DURATION
            is_hud_firing = (now - fire_start_time) < FIRE_DISPLAY_DURATION

            # Alt Sistem Guncellemeleri (FCS, Hakem, Firlatici)
            ballistics.update(sim_time=sim_t)
            launcher.update(sim_time=sim_t)

            best_box = fcs.select_best_target(boxes, center_x, center_y)
            pan_cmd, tilt_cmd = fcs.update(
                best_box, center_x, center_y, fx, fy, hw.dt,
                is_firing=is_gun_firing, is_hud_firing=is_hud_firing, current_pan=curr_p, current_tilt=curr_t, dist_cm=dist_cm
            )
            hw.set_motor_commands(pan_cmd, tilt_cmd)

            # Atis Baslangici: Sanal Mermi + Fiziksel Solenoid Firlatma
            if is_gun_firing and not was_firing:
                target_d = (dist_cm / 100.0) if dist_cm > 0 else 2.0
                ballistics.fire_shot(pan_cmd, tilt_cmd, dist_m=target_d, sim_time=sim_t)
                launcher.trigger()
            was_firing = is_gun_firing

            # HUD Gorsellestirme
            target_detected = best_box is not None
            target_centered = False

            if target_detected:
                x1, y1, x2, y2 = best_box
                bx, by = (x1 + x2) // 2, (y1 + y2) // 2
                target_centered = hud.is_target_centered(center_x, center_y, x1, y1, x2, y2)
                status_color = (0, 0, 255) if fcs.is_locked else (0, 191, 255)
                status_text = "STATUS: TARGET LOCKED" if fcs.is_locked else "STATUS: TRACKING"
                angle_err_text = f"ANGULAR ERR:   [{fcs.actual_pan:+.1f} , {fcs.actual_tilt:+.1f}] deg"

                target_label = "" if is_hud_firing else hud.get_box_label(dist_cm, target_centered, fcs.is_locked)
                hud.draw_corner_box(frame, x1, y1, x2, y2, color=status_color, length=30, thickness=2, label=target_label)

                if not is_hud_firing:
                    if not fcs.is_locked:
                        hud.draw_tracking_line(frame, center_x, center_y, bx, by, status_color)
                    hud.draw_lead_indicator(frame, bx, by, int(fcs.pred_center[0]), int(fcs.pred_center[1]))
            else:
                status_text = "STATUS: PATROLLING"
                status_color = (0, 255, 0)
                angle_err_text = "ANGULAR ERR:   [--- , ---] deg"

            reticle_color = status_color
            hud.draw_grid_and_reticle(frame, center_x, center_y, reticle_color=reticle_color)

            ballistic_hud_data = None
            if is_hud_firing:
                elapsed_fire = now - fire_start_time
                if elapsed_fire >= ENGAGING_DISPLAY_DURATION:
                    # Ilk 1.0 sn "ENGAGING TARGET" gosterildikten sonraki 2.0 sn balistik sonuc gosterilir
                    b_text, b_color = ballistics.get_hud_result()
                    if b_text:
                        ballistic_hud_data = (b_text, b_color)

            hud.draw_telemetry(
                frame, fps, status_text, status_color,
                angle_err_text, status_color, dist_cm, strength,
                target_detected=target_detected, target_centered=target_centered,
                is_locked=fcs.is_locked, firing=is_hud_firing,
                center_x=center_x, center_y=center_y, ballistic_result=ballistic_hud_data
            )

            # Atis sirasinda kilit bari ve isabet olasiligi gizlenir
            if target_detected and not is_hud_firing:
                hud.draw_lock_bar(frame, center_x, center_y, fcs.lock_counter, fcs.LOCK_THRESHOLD_FRAMES, fcs.is_locked)
                hud.draw_lock_quality(frame, center_x, center_y, fcs.last_lock_quality)

            # Telemetri ve metrik kaydi
            logger.log(sim_t, fcs, hw, best_box, is_firing=is_gun_firing, ballistics=ballistics)

            # Kontrol Modu Banner (HUD)
            if fcs.control_mode == "HYBRID":
                rl_pct = int(getattr(fcs, "current_rl_weight", 0.0) * 100)
                mode_label = f"HYBRID: PID + RL (AI Boost: {rl_pct}%)"
                mode_color = (0, 255, 128)  # Parlak Yesil
            elif fcs.control_mode == "PID":
                mode_label = "CLASSICAL PID"
                mode_color = (0, 255, 255)  # Sari
            else:
                mode_label = "PURE PPO RL"
                mode_color = (255, 200, 0)  # Mavi/Turkuaz

            cv2.putText(
                frame,
                f"CONTROL: [{mode_label}] (Press 'M' to toggle)",
                (center_x - 240, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                mode_color,
                2,
                cv2.LINE_AA,
            )

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                break
            elif key == ord('m') or key == ord('M'):
                # 3 Asamali Dongu: HYBRID -> PID -> RL -> HYBRID
                mode_cycle = {"HYBRID": "PID", "PID": "RL", "RL": "HYBRID"}
                fcs.control_mode = mode_cycle.get(fcs.control_mode, "HYBRID")
                print(f"\n[MOD DEGISTI] Aktif Kontrol Modu: >>> {fcs.control_mode} <<<")
            elif key == 32:  # Space - Acil Durdurma
                fcs.reset()
                fire_start_time = 0.0
                ballistics.last_result_text = ""
                hw.set_motor_commands(0.0, 0.0)
                print("[SYS] EMERGENCY STOP: Taret durduruldu, motorlar merkezlendi.")
            elif key in [ord('y'), ord('Y')]:
                if (now - fire_start_time) > (GUN_FIRE_DURATION + 0.05):
                    fire_start_time = now
                    ballistics.last_result_text = ""
                    print("[FCS] 'Y' TIKLANDI: Atis emri verildi -- taret onleme ve atisa geciyor!")
            elif key in [ord('n'), ord('N')]:
                fcs.reset()
                fire_start_time = 0.0
                ballistics.last_result_text = ""

    finally:
        logger.save()
        cv2.destroyAllWindows()
        print("[SYS] TURRET_FCS_WEBOTS // KAPATILDI.")


if __name__ == "__main__":
    main()