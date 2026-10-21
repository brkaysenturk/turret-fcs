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
   - "P": Hiz-uyarlamali yumusak olu bolgeli oransal kontrol (P CONTROLLER - Hassas Kilit).
   - "RL": 11-Boyutlu Bryson durum uzayli ve eylem gecmisli PPO Reinforcement Learning (RL CONTROLLER - Cevik Takip).
4. Akilli Devriye (Klavyeden 'P' ile gecis):
   - Hedef kayboldugunda 1.5 sn bekleme, merkeze donus ve 2 eksenli dairesel/eliptik tarama.
5. E-STOP (Bosluk Tusu): Acil durumda aninda motor durdurma ve devriye kilitleme.
================================================================================
"""

import sys
import time
from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO

# Paket ve dogrudan betik calistirma uyumlulugu (sys.path oto-cozucu)
_pkg_root = Path(__file__).resolve().parent
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))

try:
    from .camera_thread import CameraThread
    from .fcs_comm import FCSCommunicator
    from .fcs_manager import FCSManager
    from .turret_hud import TurretHUD
    from .telemetry import TelemetryLogger
    from .config import CONFIG, FCSState
except (ImportError, ValueError):
    from camera_thread import CameraThread
    from fcs_comm import FCSCommunicator
    from fcs_manager import FCSManager
    from turret_hud import TurretHUD
    from telemetry import TelemetryLogger
    from config import CONFIG, FCSState


def main():
    ENABLE_TELEMETRY = True
    project_root = Path(__file__).resolve().parents[1]
    model_path = project_root / CONFIG.paths.yolo_model
    if not model_path.exists():
        model_path = project_root / CONFIG.paths.fallback_yolo_model
    model = YOLO(str(model_path))

    cap = CameraThread()
    WINDOW_NAME = "Turret FCS"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    # Otomatik port ve kamera tespiti
    stm = FCSCommunicator()
    hud = TurretHUD()
    fcs = FCSManager(control_mode="P")
    telemetry = TelemetryLogger() if ENABLE_TELEMETRY else None

    # Logitech C270 kamera optik parametreleri (SSOT)
    fx = CONFIG.optics.compute_fx(CONFIG.optics.default_width)
    fy = CONFIG.optics.compute_fy(CONFIG.optics.default_height)

    prev_frame_time = time.time()
    fire_start_time = 0.0
    # 2 Asamali Balistik Atis ve Spin-Up Senkronizasyon Parametreleri (SSOT):
    SPINUP_DURATION = CONFIG.ballistics.spinup_time_s
    GUN_FIRE_DURATION = CONFIG.ballistics.firing_pulse_s
    RECOVER_DURATION = CONFIG.ballistics.recovery_time_s
    BALLISTIC_DISPLAY_DURATION = 1.50
    TOTAL_FIRE_CYCLE = SPINUP_DURATION + GUN_FIRE_DURATION + RECOVER_DURATION + BALLISTIC_DISPLAY_DURATION

    prev_target_detected = False
    is_estop = False
    PAN_INVERT = CONFIG.mount.pan_invert
    TILT_INVERT = CONFIG.mount.tilt_invert

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

            ret, frame = cap.read(wait_for_new=True)
            if not ret or frame is None:
                hud.log_event("[CAM] FRAME TIMEOUT / CAMERA OFFLINE", color=(0, 0, 255))
                break

            now = time.time()
            dt = min(0.10, max(0.001, now - prev_frame_time))
            fps = int(1.0 / max(0.001, now - prev_frame_time))
            prev_frame_time = now

            fcs_state = FCSState.TRACKING
            active_pan_cmd = 0.0
            active_tilt_cmd = 0.0

            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2
            fx = CONFIG.optics.compute_fx(w)
            fy = CONFIG.optics.compute_fy(h)
            dist_cm, strength, temp_c = stm.get_luna_data()
            hw_pan, hw_tilt = stm.get_servo_angles()
            if hw_pan is not None and hw_tilt is not None:
                eff_hw_pan = -hw_pan if PAN_INVERT else hw_pan
                eff_hw_tilt = -hw_tilt if TILT_INVERT else hw_tilt
                fcs.sync_turret_pose(eff_hw_pan, eff_hw_tilt)

            conf_val = fcs.selector.min_conf
            results = model.track(frame, imgsz=640, conf=conf_val, persist=True, tracker="bytetrack.yaml", verbose=False)
            boxes = results[0].boxes if len(results) > 0 else []

            # 2 Asamali Balistik Firlatma Durum Makinesi (500 ms Spin-up & Slew Senkronizasyonu)
            t_fire = (now - fire_start_time) if fire_start_time > 0 else 999.0
            if fire_start_time > 0 and t_fire < TOTAL_FIRE_CYCLE:
                is_hud_firing = True
                if t_fire < SPINUP_DURATION:
                    fire_stage = "SPINUP"
                    spinup_progress = float(t_fire / SPINUP_DURATION)
                    # S-Egrisi ile yumusak suzulme (Cosine Ease-in-out)
                    slew_factor = float(0.5 * (1.0 - np.cos(np.pi * spinup_progress)))
                    fcs_fire_cmd_state = FCSState.FIRING  # STM32 volan motorlarini ac
                elif t_fire < (SPINUP_DURATION + GUN_FIRE_DURATION):
                    fire_stage = "FIRING"
                    spinup_progress = 1.0
                    slew_factor = 1.0
                    fcs_fire_cmd_state = FCSState.FIRING  # Solenoid tahrigi
                elif t_fire < (SPINUP_DURATION + GUN_FIRE_DURATION + RECOVER_DURATION):
                    fire_stage = "RECOVER"
                    spinup_progress = 1.0
                    rec_p = float((t_fire - SPINUP_DURATION - GUN_FIRE_DURATION) / RECOVER_DURATION)
                    slew_factor = float(0.5 * (1.0 + np.cos(np.pi * rec_p)))
                    fcs_fire_cmd_state = FCSState.LOCKED if fcs.is_locked else FCSState.TRACKING
                else:
                    fire_stage = "EVAL"
                    spinup_progress = 1.0
                    slew_factor = 0.0
                    fcs_fire_cmd_state = FCSState.LOCKED if fcs.is_locked else FCSState.TRACKING
            else:
                fire_stage = None
                spinup_progress = None
                slew_factor = 0.0
                is_hud_firing = False
                fcs_fire_cmd_state = FCSState.LOCKED if fcs.is_locked else FCSState.TRACKING
                if fire_start_time > 0:
                    fire_start_time = 0.0

            # Fiziksel atis ve slew hareketi yalnizca SPINUP, FIRING ve RECOVER asamalari boyunca surer.
            # EVAL asamasinda taret merkezdedir; kilit durumu donmaz ve gercek hedef durumunu yansitir.
            is_physical_firing = fire_stage in ("SPINUP", "FIRING", "RECOVER")

            best_box = fcs.select_best_target(boxes, center_x, center_y)
            pan_cmd, tilt_cmd = fcs.update(
                best_box, center_x, center_y, fx, fy, dt, 
                is_firing=is_physical_firing, 
                dist_cm=dist_cm,
                slew_factor=slew_factor
            )

            target_detected = best_box is not None
            target_centered = False

            if is_estop:
                fcs_state = FCSState.ESTOP
                active_pan_cmd = 0.0
                active_tilt_cmd = 0.0
                stm.send(0.0, 0.0, FCSState.ESTOP)
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

                if fire_stage == "SPINUP":
                    ms_left = max(0, int((1.0 - spinup_progress) * SPINUP_DURATION * 1000))
                    status_text = f"STATUS: SPIN-UP & SLEW ({ms_left:3d}ms)"
                    status_color = (0, 215, 255)
                elif fire_stage == "FIRING":
                    status_text = "STATUS: ENGAGING (SOLENOID RELEASE)"
                    status_color = (0, 0, 255)
                elif fire_stage == "RECOVER":
                    status_text = "STATUS: RECOVERING BORESIGHT"
                    status_color = (0, 255, 128)
                elif fire_stage == "EVAL":
                    status_text = "STATUS: ENGAGEMENT SUCCESSFUL"
                    status_color = (0, 255, 0)
                elif is_hud_firing:
                    status_text = "STATUS: ENGAGING TARGET"
                    status_color = (0, 0, 255)
                elif fcs.is_locked:
                    status_text = "STATUS: TARGET LOCKED"
                    status_color = (0, 0, 255)
                else:
                    status_text = "STATUS: TRACKING"
                    status_color = (0, 191, 255)

                fcs_state = fcs_fire_cmd_state if (target_detected and (fcs.is_locked or is_hud_firing)) else (FCSState.LOCKED if fcs.is_locked else FCSState.TRACKING)
                active_pan_cmd = -pan_cmd if PAN_INVERT else pan_cmd
                active_tilt_cmd = -tilt_cmd if TILT_INVERT else tilt_cmd
                stm.send(active_pan_cmd, active_tilt_cmd, fcs_state)

                angle_err_text = f"ANGULAR ERR:   [{fcs.actual_pan:+.1f} , {fcs.actual_tilt:+.1f}] deg"
                # Nisangah ve Hedef Kutusu: Takipte sari (0, 191, 255); Kilit ve Atis evresinde sabit kirmizi (0, 0, 255)
                # Bilgilendirme paneli evrelere gore renk degistirirken hedef kutusu ve nisangah kirmizi kalir
                box_color = (0, 0, 255) if (fcs.is_locked or is_hud_firing) else status_color
                target_label = "TARGET (LOCKED)" if is_hud_firing else hud.get_box_label(dist_cm, target_centered, fcs.is_locked, conf=fcs.target_conf)
                hud.draw_corner_box(frame, x1, y1, x2, y2, color=box_color, length=30, thickness=2, label=target_label)

                if not is_hud_firing:
                    if not fcs.is_locked:
                        hud.draw_tracking_line(frame, center_x, center_y, bx, by, status_color)
                    hud.draw_lead_indicator(frame, bx, by, int(fcs.pred_center[0]), int(fcs.pred_center[1]))
            else:
                if fcs.patrol_enabled:
                    status_text = f"PATROL: {fcs.patrol_status}"
                    status_color = (0, 255, 0)
                    angle_err_text = f"TURRET POSE:   [{fcs.turret_pan_deg:+.1f} , {fcs.turret_tilt_deg:+.1f}] deg"
                else:
                    status_text = "STATUS: STANDBY"
                    status_color = (180, 180, 180)
                    angle_err_text = "ANGULAR ERR:   [--, --] deg"

                # Hedef yokken devriye etkinse komutlari tarete ilet (fcs.update zaten 142. satirda calisti)
                if fcs.patrol_enabled:
                    active_pan_cmd = -pan_cmd if PAN_INVERT else pan_cmd
                    active_tilt_cmd = -tilt_cmd if TILT_INVERT else tilt_cmd
                    stm.send(active_pan_cmd, active_tilt_cmd, FCSState.TRACKING)
                    prev_target_detected = False
                else:
                    active_pan_cmd = 0.0
                    active_tilt_cmd = 0.0
                    if prev_target_detected:
                        stm.send(0.0, 0.0, FCSState.TRACKING)
                        if fcs.frames_since_lost >= fcs.max_lost_frames:
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
                    temp_c=temp_c,
                    fcs_state=int(fcs_state if target_detected else FCSState.TRACKING),
                    servo_pan_fb=hw_pan,
                    servo_tilt_fb=hw_tilt,
                    frame_id=cap.current_frame_id if cap else None,
                )

            reticle_color = (0, 0, 255) if (fcs.is_locked or is_hud_firing) else status_color
            hud.draw_grid_and_reticle(frame, center_x, center_y, reticle_color=reticle_color)

            ballistic_hud_data = None
            if is_hud_firing:
                elapsed_fire = now - fire_start_time
                if elapsed_fire >= (SPINUP_DURATION + GUN_FIRE_DURATION):
                    ballistic_hud_data = ("ENGAGEMENT SUCCESSFUL", (0, 255, 0))

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
                fire_stage=fire_stage,
                spinup_progress=spinup_progress,
            )

            # Atis sirasinda kilit bari ve isabet olasiligi gizlenir
            if target_detected and not is_hud_firing:
                hud.draw_lock_bar(frame, center_x, center_y, fcs.lock_counter, fcs.LOCK_THRESHOLD_FRAMES, fcs.is_locked)
                hud.draw_lock_quality(frame, center_x, center_y, fcs.last_lock_quality)

            # HUD Bilesenleri (Panel gorunurluk kurallari ve declutter TurretHUD icinde yonetilir)
            hud.draw_mode_banner(frame, fcs.control_mode, fcs.patrol_enabled, is_locked=fcs.is_locked, firing=is_hud_firing)
            hud.draw_footer(frame, is_locked=fcs.is_locked, firing=is_hud_firing)
            hud.draw_event_log(frame, is_locked=fcs.is_locked, firing=is_hud_firing)

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == 27 or key == ord("q") or key == ord("Q"):
                hud.log_event("[SYS] SESSION TERMINATED (EXIT)", color=(180, 180, 180))
                break
            elif key == ord("m") or key == ord("M"):
                mode_cycle = {"P": "RL", "RL": "P"}
                fcs.control_mode = mode_cycle.get(fcs.control_mode, "P")
                mode_names = {
                    "P": "P CONTROLLER (PRECISION LOCK)",
                    "RL": "RL CONTROLLER (AGILE PURSUIT)",
                }
                mode_str = mode_names.get(fcs.control_mode, fcs.control_mode)
                hud.log_event(f"[MODE] {mode_str}", color=(0, 215, 255))
            elif key == ord("p") or key == ord("P"):
                fcs.patrol_enabled = not fcs.patrol_enabled
                p_text = "FIGURE-8 SCAN ENGAGED" if fcs.patrol_enabled else "STANDBY (DISABLED)"
                p_color = (0, 255, 0) if fcs.patrol_enabled else (180, 180, 180)
                hud.log_event(f"[PATROL] {p_text}", color=p_color)
            elif key == 32:  # E-STOP (Bosluk tusu)
                is_estop = not is_estop
                fcs.reset()
                fire_start_time = 0.0
                if is_estop:
                    fcs.patrol_enabled = False
                    stm.send(0.0, 0.0, FCSState.ESTOP)
                    hud.log_event("[E-STOP] MOTORS HALTED & ZEROED (LATCHED)", color=(0, 0, 255))
                else:
                    stm.send(0.0, 0.0, FCSState.TRACKING)
                    hud.log_event("[E-STOP] DISENGAGED - RESUMING FCS", color=(0, 255, 128))
            elif key in [ord("y"), ord("Y")] and fcs.is_locked and fire_stage is None:
                fire_start_time = time.time()
                hud.log_event("[FIRE] 2-STAGE ENGAGEMENT: SPIN-UP & SLEW ENGAGED", color=(0, 215, 255))
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