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

    # 3. Optik Kamera Parametreleri (Logitech C270 dFOV 55° -> 16:9)
    H_FOV, V_FOV = 48.8, 28.6
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

    hud.log_event("[SYS] WEBOTS TURRET FCS INITIALIZED", color=(0, 255, 128))
    hud.log_event("[CAM] WEBOTS VIRTUAL CAM ONLINE (640x480)", color=(0, 255, 128))
    hud.log_event("[TLM] WEBOTS TELEMETRY ACTIVE", color=(0, 255, 128))

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
                if is_hud_firing:
                    status_text = "STATUS: ENGAGING TARGET"
                    status_color = (0, 0, 255)
                elif fcs.is_locked:
                    status_text = "STATUS: TARGET LOCKED"
                    status_color = (0, 0, 255)
                else:
                    status_text = "STATUS: TRACKING"
                    status_color = (0, 191, 255)

                angle_err_text = f"ANGULAR ERR:   [{fcs.actual_pan:+.1f} , {fcs.actual_tilt:+.1f}] deg"

                target_label = "" if is_hud_firing else hud.get_box_label(dist_cm, target_centered, fcs.is_locked)
                hud.draw_corner_box(frame, x1, y1, x2, y2, color=status_color, length=30, thickness=2, label=target_label)

                if not is_hud_firing:
                    if not fcs.is_locked:
                        hud.draw_tracking_line(frame, center_x, center_y, bx, by, status_color)
                    hud.draw_lead_indicator(frame, bx, by, int(fcs.pred_center[0]), int(fcs.pred_center[1]))
            else:
                status_text = "STATUS: PATROLLING" if fcs.patrol_enabled else "STATUS: STANDBY"
                status_color = (0, 255, 0) if fcs.patrol_enabled else (180, 180, 180)
                angle_err_text = "ANGULAR ERR:   [--, --] deg"

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

            # Kontrol Modu ve Devriye Durum Banneri (HUD)
            if fcs.control_mode == "HYBRID":
                rl_pct = int(getattr(fcs, "current_rl_weight", 0.0) * 100)
                mode_label = f"HYBRID CONTROLLER (RL:{rl_pct}%)" if rl_pct > 0 else "HYBRID CONTROLLER"
                mode_color = (0, 255, 128)
            elif fcs.control_mode in ["PID", "P"]:
                mode_label = "P CONTROLLER" if fcs.control_mode == "P" else "PID CONTROLLER"
                mode_color = (0, 255, 255)
            else:
                mode_label = "RL CONTROLLER"
                mode_color = (255, 200, 0)

            patrol_tag = "PATROL: AUTO" if fcs.patrol_enabled else "PATROL: OFF"
            banner_text = f"[{mode_label}]  |  [{patrol_tag}]"
            (b_w, _), _ = cv2.getTextSize(banner_text, cv2.FONT_HERSHEY_SIMPLEX, 0.56, 2)
            banner_x = max(10, center_x - (b_w // 2))

            # Yari saydam ust baslik paneli
            pad_x, pad_y = 12, 5
            bx1, by1 = max(0, banner_x - pad_x), max(0, 26 - 16 - pad_y)
            bx2, by2 = min(frame.shape[1], banner_x + b_w + pad_x), min(frame.shape[0], 26 + pad_y)
            roi_b = frame[by1:by2, bx1:bx2]
            black_b = np.zeros_like(roi_b)
            frame[by1:by2, bx1:bx2] = cv2.addWeighted(roi_b, 0.30, black_b, 0.70, 0)
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), (90, 100, 110), 1)

            cv2.putText(frame, banner_text, (banner_x, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.56, mode_color, 2, cv2.LINE_AA)

            # Alt operator kontrol lejandi
            footer_text = "[M] Mode  |  [P] Patrol  |  [SPACE] E-Stop  |  [Y] Fire  |  [ESC] Quit"
            (f_w, _), _ = cv2.getTextSize(footer_text, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
            cv2.putText(frame, footer_text, (center_x - f_w // 2, frame.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 170, 180), 1, cv2.LINE_AA)

            # Sol alt terminal olay gunlugu (Console Event Log)
            hud.draw_event_log(frame)

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                hud.log_event("[SYS] SESSION TERMINATED (EXIT)", color=(180, 180, 180))
                break
            elif key == ord('m') or key == ord('M'):
                # 3 Asamali Dongu: HYBRID -> PID -> RL -> HYBRID
                mode_cycle = {"HYBRID": "PID", "PID": "RL", "RL": "HYBRID"}
                fcs.control_mode = mode_cycle.get(fcs.control_mode, "HYBRID")
                mode_names = {"HYBRID": "HYBRID CONTROLLER", "PID": "PID CONTROLLER", "RL": "RL CONTROLLER"}
                hud.log_event(f"[MODE] {mode_names.get(fcs.control_mode, fcs.control_mode)}", color=(0, 215, 255))
            elif key == ord('p') or key == ord('P'):
                fcs.patrol_enabled = not fcs.patrol_enabled
                p_text = "CIRCULAR SCAN ENGAGED" if fcs.patrol_enabled else "STANDBY (DISABLED)"
                p_color = (0, 255, 128) if fcs.patrol_enabled else (180, 180, 180)
                hud.log_event(f"[PATROL] {p_text}", color=p_color)
            elif key == 32:  # Space - Acil Durdurma
                fcs.reset()
                fcs.patrol_enabled = False
                fire_start_time = 0.0
                ballistics.last_result_text = ""
                hw.set_motor_commands(0.0, 0.0)
                hud.log_event("[E-STOP] MOTORS HALTED & ZEROED", color=(0, 0, 255))
            elif key in [ord('y'), ord('Y')]:
                if (now - fire_start_time) > (GUN_FIRE_DURATION + 0.05):
                    fire_start_time = now
                    ballistics.last_result_text = ""
                    hud.log_event("[FIRE] SOLENOID TRIGGERED", color=(0, 0, 255))
            elif key in [ord('n'), ord('N')]:
                fcs.reset()
                fire_start_time = 0.0
                ballistics.last_result_text = ""

    finally:
        logger.save()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()