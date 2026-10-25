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
   - "P": P (tek EMA, ivme sinirli, yumusak olu bolgeli Tip-1 P).
   - "RL": RL (11 boyutlu saf PPO, NumPy cikarim).
4. Akilli Devriye (Klavyeden 'P' ile gecis):
   - Hedef kayboldugunda 1.5 sn bekleme, merkeze donus ve 2 eksenli figure-8 tarama.
5. E-STOP (Bosluk Tusu): Acil durumda aninda motor durdurma ve devriye kilitleme.
================================================================================
"""

import time
from pathlib import Path
import cv2
from ultralytics import YOLO

from .camera_thread import CameraThread
from .fcs_comm import FCSCommunicator
from .fcs_manager import FCSManager
from .turret_hud import TurretHUD
from .telemetry import TelemetryLogger
from .config import CONFIG, FCSState
from .weights import ensure_weight_file
from .engagement import EngagementManager
from .presenter import HUDPresenter
from .keymap import process_key_event


def main():
    project_root = Path(__file__).resolve().parents[1]
    model_path = project_root / CONFIG.paths.yolo_model
    if not model_path.exists():
        try:
            model_path = ensure_weight_file(model_path, required=False)
        except Exception:
            pass
    if not model_path.exists():
        fallback_path = project_root / CONFIG.paths.fallback_yolo_model
        try:
            model_path = ensure_weight_file(fallback_path, required=True)
        except FileNotFoundError as err:
            raise FileNotFoundError(
                f"[FCS HATA] Model agirliklari temin edilemedi: {err}\n"
                f"Lutfen 'python -m turret_fcs.weights' komutunu calistirarak modelleri indirin."
            ) from err
    model = YOLO(str(model_path))

    cap = CameraThread()
    WINDOW_NAME = "Turret FCS"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    # Otomatik port ve kamera tespiti
    stm = FCSCommunicator()
    hud = TurretHUD()
    fcs = FCSManager(control_mode="P")
    telemetry = TelemetryLogger() if CONFIG.telemetry_enabled else None
    engagement = EngagementManager()
    presenter = HUDPresenter(hud)

    prev_frame_time = time.time()
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

            ret, frame, frame_id = cap.read(wait_for_new=True, return_id=True)
            if not ret or frame is None:
                hud.log_event("[CAM] FRAME TIMEOUT / CAMERA OFFLINE", color=(0, 0, 255))
                break

            now = time.time()
            dt = min(0.10, max(0.001, now - prev_frame_time))
            fps = int(1.0 / max(0.001, now - prev_frame_time))
            prev_frame_time = now
            loop_started = time.perf_counter()

            fcs_state = FCSState.TRACKING
            active_pan_cmd = 0.0
            active_tilt_cmd = 0.0

            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2
            fx = CONFIG.optics.compute_fx(w)
            fy = CONFIG.optics.compute_fy(h)
            dist_cm, strength = stm.get_luna_data()
            hw_pan, hw_tilt = stm.get_servo_angles()
            if hw_pan is not None and hw_tilt is not None:
                eff_hw_pan = -hw_pan if PAN_INVERT else hw_pan
                eff_hw_tilt = -hw_tilt if TILT_INVERT else hw_tilt
                fcs.sync_turret_pose(eff_hw_pan, eff_hw_tilt)

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

            # 1. Angajman durum makinesini guncelle
            eng_state = engagement.update(now, is_locked=fcs.is_locked)

            # 2. Hedef secimi ve FCS kontrol komut hesabi
            best_box = fcs.select_best_target(boxes, center_x, center_y)
            pan_cmd, tilt_cmd = fcs.update(
                best_box, center_x, center_y, fx, fy, dt,
                is_firing=eng_state.is_physical_firing,
                dist_cm=dist_cm,
                slew_factor=eng_state.slew_factor,
            )

            target_detected = best_box is not None

            # 3. Motor komutlarini belirle ve STM32'ye ilet
            if is_estop:
                fcs_state = FCSState.ESTOP
                active_pan_cmd = 0.0
                active_tilt_cmd = 0.0
                stm.send(0.0, 0.0, FCSState.ESTOP)
            elif target_detected:
                prev_target_detected = True
                fcs_state = eng_state.fcs_fire_cmd_state
                active_pan_cmd = -pan_cmd if PAN_INVERT else pan_cmd
                active_tilt_cmd = -tilt_cmd if TILT_INVERT else tilt_cmd
                stm.send(active_pan_cmd, active_tilt_cmd, fcs_state)
            elif fcs.patrol_enabled:
                fcs_state = FCSState.TRACKING
                active_pan_cmd = -pan_cmd if PAN_INVERT else pan_cmd
                active_tilt_cmd = -tilt_cmd if TILT_INVERT else tilt_cmd
                stm.send(active_pan_cmd, active_tilt_cmd, fcs_state)
                prev_target_detected = False
            else:
                fcs_state = FCSState.TRACKING
                active_pan_cmd = 0.0
                active_tilt_cmd = 0.0
                if prev_target_detected:
                    stm.send(0.0, 0.0, FCSState.TRACKING)
                    if fcs.frames_since_lost >= fcs.max_lost_frames:
                        prev_target_detected = False

            target_box_tuple = tuple(best_box[:4]) if best_box is not None else None

            # 4. Telemetri kaydi
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
                    fcs_state=int(fcs_state),
                    fire_stage=eng_state.phase,
                    servo_pan_fb=hw_pan,
                    servo_tilt_fb=hw_tilt,
                    frame_id=frame_id,
                    latency_ms=(time.perf_counter() - loop_started) * 1000.0,
                )

            # 5. Bas-Ustu Gosterge (HUD) render
            hit_feedback = "ENGAGEMENT COMPLETED" if eng_state.phase == "EVAL" else None
            presenter.render(
                frame=frame,
                fcs=fcs,
                engagement_state=eng_state,
                target_box=target_box_tuple,
                center_x=center_x,
                center_y=center_y,
                fps=fps,
                dist_cm=dist_cm,
                strength=strength,
                is_estop=is_estop,
                hit_feedback=hit_feedback,
            )

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            # 6. Operator klavye etkilesimi
            def emit_event(msg, color=(200, 210, 220)):
                hud.log_event(msg, color)
                if telemetry is not None:
                    telemetry.log_event("SYS", msg)

            should_exit, is_estop = process_key_event(
                key_code=key,
                fcs=fcs,
                engagement=engagement,
                is_estop=is_estop,
                now=now,
                is_range_valid=(fcs.range_valid and fcs.range_stable),
                is_flywheel_idle=(fcs.is_locked and not is_estop),
                log_event_cb=emit_event,
            )
            if should_exit:
                break

    finally:
        if telemetry is not None:
            telemetry.close()
        stm.close()
        cap.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

