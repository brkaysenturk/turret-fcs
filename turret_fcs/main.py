"""
================================================================================
MODUL: ANA KONTROL DONGUSU VE SISTEM ORKESTRASYONU (MAIN / SYSTEM RUNNER)
================================================================================
Bu modul, otonom atis kontrol sisteminin tum alt bilesenlerini (Goruntu Isleme,
YOLOv8 Hedef Tespiti, FCS Durum Yonetimi, HUD Grafik Arayuzu, STM32 Seri Iletisimi
ve Telemetri Kaydedicisi) 30 FPS gercek zamanli eszamanli calisma dongusunde birlestirir.

TEMEL GOREVLER VE AKIS:
1. Donanim Baslatma: Logitech C270 kamera (DirectShow) ve STM32 (USB VID 0483) otomatik algilama.
2. Derin Ogrenme Cikarimi: Egitilmis YOLOv8 modeli ile balon tespiti (best.pt).
3. Kontrol Modlari (Klavyeden 'M' ile gecis):
   - "P": Hiz-uyarlamali yumusak olu bolgeli oransal kontrol (P CONTROLLER - Donanimda en kararli).
   - "HYBRID": Hizli hedeflerde gecikmeyi telafi eden AI takviyeli P + PPO RL harmanlama (HYBRID CONTROLLER).
   - "RL": Saf Reinforcement Learning (PPO) ile otonom yonlendirme (RL CONTROLLER).
4. Akilli Devriye (Klavyeden 'P' ile gecis):
   - Hedef kayboldugunda 1.5 sn bekleme, merkeze donus ve 2 eksenli dairesel/eliptik tarama.
5. E-STOP (Bosluk Tusu): Acil durumda aninda motor durdurma ve devriye kilitleme.
================================================================================
"""

import time
from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO

from camera_thread import CameraThread
from fcs_comm import FCSCommunicator
from fcs_manager import FCSManager
from turret_hud import TurretHUD
from telemetry import TelemetryLogger


def main():
    ENABLE_TELEMETRY = True
    model_path = Path(__file__).resolve().parents[1] / "runs/detect/balloon_detector/weights/best.pt"
    model = YOLO(str(model_path))

    cap = CameraThread()
    WINDOW_NAME = "Turret FCS"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    # Otomatik port ve kamera tespiti
    stm = FCSCommunicator()
    hud = TurretHUD()
    fcs = FCSManager(control_mode="P")
    telemetry = TelemetryLogger() if ENABLE_TELEMETRY else None

    # Logitech C270 Resmi Datasheet Gorus Acilari (dFOV 60° -> 16:9 Projeksiyon)
    H_FOV, V_FOV = 53.0, 31.5
    fx = (1280 / 2.0) / np.tan(np.radians(H_FOV / 2.0))
    fy = (720 / 2.0) / np.tan(np.radians(V_FOV / 2.0))

    prev_frame_time = time.time()
    fire_start_time = 0.0
    GUN_FIRE_DURATION = 0.12  # STM32 solenoid tetik sinyali (120ms < 230ms mekanik saykl, cift atisi kesin onler)
    ENGAGING_DISPLAY_DURATION = 1.00
    BALLISTIC_DISPLAY_DURATION = 2.00
    FIRE_DISPLAY_DURATION = ENGAGING_DISPLAY_DURATION + BALLISTIC_DISPLAY_DURATION

    prev_target_detected = False
    is_estop = False
    PAN_INVERT = True   # Yatay servo mekanik montaj acisina gore tersleme (True: -pan_cmd, False: +pan_cmd)
    TILT_INVERT = False  # Dikey servo mekanik montaj acisina gore tersleme (True: -tilt_cmd, False: +tilt_cmd)

    # 1. Sistem baslangic durumu
    hud.log_event("[SYS] TURRET FCS INITIALIZED & OPERATIONAL", color=(0, 255, 128))

    # 2. Kamera baglanti durumu
    if cap.isOpened():
        cam_tag = "LOGI C270" if "c270" in cap.cam_name.lower() else "CAMERA"
        hud.log_event(f"[CAM] {cam_tag} ONLINE (1280x720@30FPS)", color=(0, 255, 128))
    else:
        hud.log_event("[CAM] CAMERA OFFLINE / NOT FOUND", color=(0, 0, 255))

    # 3. STM32 baglanti durumu
    if stm.is_connected:
        hud.log_event(f"[STM32] {stm.port} CONNECTED (115200 BAUD)", color=(0, 255, 128))
    else:
        hud.log_event("[STM32] OFFLINE (SIMULATION / NO HW)", color=(0, 191, 255))

    # 4. TF-Luna LiDAR durumu (STM32 uzerinden)
    time.sleep(0.04)
    if stm.is_luna_online:
        hud.log_event("[LIDAR] TF-LUNA ONLINE", color=(0, 255, 128))
    else:
        hud.log_event("[LIDAR] TF-LUNA STANDBY / WAITING", color=(0, 191, 255))

    # 5. Telemetri kayit durumu
    if telemetry is not None:
        hud.log_event(f"[TLM] RECORDING ACTIVE ({telemetry.csv_path.name})", color=(0, 255, 128))
    else:
        hud.log_event("[TLM] RECORDING DISABLED", color=(180, 180, 180))

    prev_stm_connected = stm.is_connected
    prev_luna_online = stm.is_luna_online

    try:
        while cap.isOpened():
            # STM32 baglanti durumundaki anlik degisimleri izle
            if stm.is_connected != prev_stm_connected:
                prev_stm_connected = stm.is_connected
                if stm.is_connected:
                    hud.log_event(f"[STM32] {stm.port} CONNECTED (AUTO)", color=(0, 255, 128))
                else:
                    hud.log_event("[STM32] CONNECTION LOST (OFFLINE)", color=(0, 0, 255))

            # TF-Luna LiDAR canlilik durumundaki degisimleri izle
            curr_luna_online = stm.is_luna_online
            if curr_luna_online != prev_luna_online:
                prev_luna_online = curr_luna_online
                if curr_luna_online:
                    hud.log_event("[LIDAR] TF-LUNA ONLINE", color=(0, 255, 128))
                else:
                    hud.log_event("[LIDAR] TF-LUNA TIMEOUT / OFFLINE", color=(0, 0, 255))

            ret, frame = cap.read()
            if not ret or frame is None:
                hud.log_event("[CAM] FRAME TIMEOUT / CAMERA OFFLINE", color=(0, 0, 255))
                break

            now = time.time()
            dt = min(0.10, max(0.001, now - prev_frame_time))
            fps = int(1.0 / max(0.001, now - prev_frame_time))
            prev_frame_time = now

            fcs_state = 0
            active_pan_cmd = 0.0
            active_tilt_cmd = 0.0

            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2
            dist_cm, strength, _ = stm.get_luna_data()

            conf_val = fcs.selector.min_conf
            results = model.track(frame, imgsz=640, conf=conf_val, persist=True, tracker="bytetrack.yaml", verbose=False)
            boxes = results[0].boxes if len(results) > 0 else []

            is_gun_firing = (now - fire_start_time) < GUN_FIRE_DURATION
            is_hud_firing = (now - fire_start_time) < FIRE_DISPLAY_DURATION

            best_box = fcs.select_best_target(boxes, center_x, center_y)
            pan_cmd, tilt_cmd = fcs.update(
                best_box, center_x, center_y, fx, fy, dt, 
                is_firing=is_gun_firing, is_hud_firing=is_hud_firing, dist_cm=dist_cm
            )

            target_detected = best_box is not None
            target_centered = False

            if is_estop:
                fcs_state = 3
                active_pan_cmd = 0.0
                active_tilt_cmd = 0.0
                stm.send(0.0, 0.0, 3)
                status_text = "STATUS: EMERGENCY STOP (LATCHED)"
                status_color = (0, 0, 255)
                angle_err_text = "ANGULAR ERR:   [HALTED] deg"
                if target_detected:
                    x1, y1, x2, y2 = best_box
                    hud.draw_corner_box(frame, x1, y1, x2, y2, color=(100, 100, 100), length=25, thickness=1, label="TARGET (INHIBITED)")
            elif target_detected:
                prev_target_detected = True
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

                fcs_state = 2 if (fcs.is_locked and is_gun_firing) else (1 if fcs.is_locked else 0)
                active_pan_cmd = -pan_cmd if PAN_INVERT else pan_cmd
                active_tilt_cmd = -tilt_cmd if TILT_INVERT else tilt_cmd
                stm.send(active_pan_cmd, active_tilt_cmd, fcs_state)

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

                # Hedef yokken devriye etkinse komutlari tarete ilet (fcs.update zaten 142. satirda calisti)
                if fcs.patrol_enabled:
                    active_pan_cmd = -pan_cmd if PAN_INVERT else pan_cmd
                    active_tilt_cmd = -tilt_cmd if TILT_INVERT else tilt_cmd
                    stm.send(active_pan_cmd, active_tilt_cmd, 0)
                    prev_target_detected = False
                else:
                    active_pan_cmd = 0.0
                    active_tilt_cmd = 0.0
                    if prev_target_detected:
                        stm.send(0.0, 0.0, 0)
                        fcs.reset()
                        prev_target_detected = False

            if telemetry is not None:
                telemetry.log_frame(
                    dt=dt,
                    fps=fps,
                    target_detected=target_detected,
                    best_box=best_box,
                    fcs=fcs,
                    pan_cmd=active_pan_cmd if (target_detected or fcs.patrol_enabled) else 0.0,
                    tilt_cmd=active_tilt_cmd if (target_detected or fcs.patrol_enabled) else 0.0,
                    dist_cm=dist_cm,
                    strength=strength,
                    fcs_state=fcs_state if target_detected else 0,
                )

            hud.draw_grid_and_reticle(frame, center_x, center_y, reticle_color=status_color)

            ballistic_hud_data = None
            if is_hud_firing:
                elapsed_fire = now - fire_start_time
                if elapsed_fire >= ENGAGING_DISPLAY_DURATION:
                    ballistic_hud_data = ("BURST COMPLETED", (0, 255, 0))

            hud.draw_telemetry(
                frame,
                fps,
                status_text,
                status_color,
                angle_err_text,
                status_color,
                dist_cm,
                strength,
                target_detected=target_detected,
                target_centered=target_centered,
                is_locked=fcs.is_locked,
                firing=is_hud_firing,
                center_x=center_x,
                center_y=center_y,
                ballistic_result=ballistic_hud_data,
            )

            # Atis sirasinda kilit bari ve isabet olasiligi gizlenir
            if target_detected and not is_hud_firing:
                hud.draw_lock_bar(frame, center_x, center_y, fcs.lock_counter, fcs.LOCK_THRESHOLD_FRAMES, fcs.is_locked)
                hud.draw_lock_quality(frame, center_x, center_y, fcs.last_lock_quality)

            # Kontrol Modu ve Devriye Durum Banneri (HUD)
            if fcs.control_mode == "HYBRID":
                rl_pct = int(getattr(fcs, "current_rl_weight", 0.0) * 100)
                mode_label = f"HYBRID CONTROLLER (RL:{rl_pct}%)" if rl_pct > 0 else "HYBRID CONTROLLER"
                mode_color = (0, 255, 128)
            elif fcs.control_mode == "RL":
                mode_label = "RL CONTROLLER"
                mode_color = (255, 200, 0)
            else:
                mode_label = "P CONTROLLER"
                mode_color = (0, 255, 255)

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
            footer_text = "[M] Mode  |  [P] Patrol  |  [SPACE] E-Stop  |  [Y] Fire  |  [Q] Quit"
            (f_w, _), _ = cv2.getTextSize(footer_text, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
            cv2.putText(frame, footer_text, (center_x - f_w // 2, frame.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 170, 180), 1, cv2.LINE_AA)

            # Sol alt terminal olay gunlugu (Console Event Log)
            hud.draw_event_log(frame)

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == 27 or key == ord("q") or key == ord("Q"):
                hud.log_event("[SYS] SESSION TERMINATED (EXIT)", color=(180, 180, 180))
                break
            elif key == ord("m") or key == ord("M"):
                mode_cycle = {"P": "HYBRID", "HYBRID": "RL", "RL": "P"}
                fcs.control_mode = mode_cycle.get(fcs.control_mode, "P")
                mode_names = {
                    "P": "P CONTROLLER",
                    "HYBRID": "HYBRID CONTROLLER",
                    "RL": "RL CONTROLLER",
                }
                mode_str = mode_names.get(fcs.control_mode, fcs.control_mode)
                hud.log_event(f"[MODE] {mode_str}", color=(0, 215, 255))
            elif key == ord("p") or key == ord("P"):
                fcs.patrol_enabled = not fcs.patrol_enabled
                p_text = "CIRCULAR SCAN ENGAGED" if fcs.patrol_enabled else "STANDBY (DISABLED)"
                p_color = (0, 255, 128) if fcs.patrol_enabled else (180, 180, 180)
                hud.log_event(f"[PATROL] {p_text}", color=p_color)
            elif key == 32:  # E-STOP (Bosluk tusu)
                is_estop = not is_estop
                fcs.reset()
                fire_start_time = 0.0
                if is_estop:
                    fcs.patrol_enabled = False
                    stm.send(0.0, 0.0, 3)
                    hud.log_event("[E-STOP] MOTORS HALTED & ZEROED (LATCHED)", color=(0, 0, 255))
                else:
                    stm.send(0.0, 0.0, 0)
                    hud.log_event("[E-STOP] DISENGAGED - RESUMING FCS", color=(0, 255, 128))
            elif key in [ord("y"), ord("Y")] and fcs.is_locked:
                fire_start_time = time.time()
                hud.log_event("[FIRE] SOLENOID TRIGGERED", color=(0, 0, 255))
            elif key in [ord("n"), ord("N")]:
                fcs.reset()
                fire_start_time = 0.0

    finally:
        if telemetry is not None:
            telemetry.close()
        stm.close()
        cap.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()