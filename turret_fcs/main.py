import time
from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO

from camera_thread import CameraThread
from fcs_comm import FCSCommunicator
from fcs_manager import FCSManager
from turret_hud import TurretHUD
# from patrol_generator import PatrolGenerator  # [DEVREYE ALMA] Otonom Lissajous alan tarama modulu


def main():
    model_path = Path(__file__).resolve().parents[1] / "runs/detect/balon_detector_v3/weights/best.pt"
    model = YOLO(str(model_path))

    cap = CameraThread(src=0, width=1280, height=720)
    WINDOW_NAME = "Turret FCS"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    stm = FCSCommunicator(port="COM3")
    hud = TurretHUD()
    fcs = FCSManager()
    # patrol = PatrolGenerator(pan_amplitude=35.0, tilt_amplitude=12.0, smoothing_alpha=0.06)  # [DEVREYE ALMA] Tarama motoru

    H_FOV, V_FOV = 69.0, 42.0
    fx = (1280 / 2.0) / np.tan(np.radians(H_FOV / 2.0))
    fy = (720 / 2.0) / np.tan(np.radians(V_FOV / 2.0))

    prev_frame_time = time.time()
    fire_start_time = 0.0
    GUN_FIRE_DURATION = 0.40
    ENGAGING_DISPLAY_DURATION = 1.00
    BALLISTIC_DISPLAY_DURATION = 2.00
    FIRE_DISPLAY_DURATION = ENGAGING_DISPLAY_DURATION + BALLISTIC_DISPLAY_DURATION

    print("[SYS] TURRET_FCS // BASLATILDI & OPERASYONEL")

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            now = time.time()
            dt = max(0.001, now - prev_frame_time)
            fps = int(1.0 / dt)
            prev_frame_time = now

            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2
            dist_cm, strength, _ = stm.get_luna_data()

            conf_val = getattr(fcs.selector, "min_conf", 0.30)
            results = model.track(frame, imgsz=640, conf=conf_val, persist=True, tracker="bytetrack.yaml", verbose=False)
            boxes = results[0].boxes if len(results) > 0 else []

            is_gun_firing = (now - fire_start_time) < GUN_FIRE_DURATION
            is_hud_firing = (now - fire_start_time) < FIRE_DISPLAY_DURATION

            best_box = fcs.select_best_target(boxes, center_x, center_y)
            pan_cmd, tilt_cmd = fcs.update(best_box, center_x, center_y, fx, fy, dt, is_firing=is_gun_firing, is_hud_firing=is_hud_firing, dist_cm=dist_cm)

            target_detected = best_box is not None
            target_centered = False

            if target_detected:
                # patrol.reset()  # [DEVREYE ALMA] Hedef yakalandiginda arama desenini sifirla
                x1, y1, x2, y2 = best_box
                bx, by = (x1 + x2) // 2, (y1 + y2) // 2
                target_centered = hud.is_target_centered(center_x, center_y, x1, y1, x2, y2)

                status_text = "STATUS: TARGET LOCKED" if fcs.is_locked else "STATUS: TRACKING"
                status_color = (0, 0, 255) if fcs.is_locked else (0, 191, 255)

                fcs_state = 2 if (fcs.is_locked and is_gun_firing) else (1 if fcs.is_locked else 0)
                stm.send(pan_cmd, tilt_cmd, fcs_state)

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
                angle_err_text = "ANGULAR ERR:   [--, --] deg"

                # --- [DEVREYE ALMA] PATROL (OTONOM LISSAJOUS TARAMA) ---
                # Hedef yokken kadraji 3D Lissajous egrisi ile cevreye taratir:
                # pan_patrol, tilt_patrol = patrol.update(dt, current_pan=0.0, current_tilt=0.0)
                # stm.send(pan_patrol, tilt_patrol, 0)
                # --------------------------------------------------------
                stm.send(0.0, 0.0, 0)

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
                (center_x - 250, 30),
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
            elif key == ord("m") or key == ord("M"):
                mode_cycle = {"HYBRID": "PID", "PID": "RL", "RL": "HYBRID"}
                fcs.control_mode = mode_cycle.get(fcs.control_mode, "HYBRID")
                print(f"\n[MOD DEGISTI] Gercek Donanim Kontrol Modu: >>> {fcs.control_mode} <<<")
            elif key == 32:  # E-STOP
                fcs.reset()
                fire_start_time = 0.0
                stm.send(0.0, 0.0, 0)
                print("[SYS] EMERGENCY STOP: Taret durduruldu, motorlar sifirlandi.")
            elif key in [ord("y"), ord("Y")] and fcs.is_locked:
                fire_start_time = time.time()
            elif key in [ord("n"), ord("N")]:
                fcs.reset()
                fire_start_time = 0.0

    finally:
        stm.close()
        cap.stop()
        cv2.destroyAllWindows()
        print("[SYS] TURRET_FCS // DONANIM VE BAGLANTILAR KAPATILDI.")


if __name__ == "__main__":
    main()