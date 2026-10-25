"""
================================================================================
MODUL: WEBOTS TELEMETRI V2 KAYDEDICISI (WEBOTS TELEMETRY V2 - SSOT)
================================================================================
Bu modul, Webots FCS simulasyonu esnasinda uretilen durum, hata, eyleyici ve
Kalman kestirimlerini uretim Schema v2 formatinda tamponlu CSV olarak kaydeder.
Ayni zamanda zemin gercegi (Ground Truth) balon koordinatlarini da yan kolonlara
ekleyerek tam dogrulama (full verification) verisi saglar.
================================================================================
"""

import atexit
import csv
from datetime import datetime
from pathlib import Path
import numpy as np

from turret_fcs.config import CONFIG


class WebotsTelemetryLogger:
    """
    Webots FCS Simulasyonu Telemetri v2 Kaydedicisi.
    """

    SCHEMA_VERSION = "2.0-webots"

    HEADERS = [
        "schema_version", "frame_id", "sim_time_sec", "dt", "fps",
        "target_detected", "target_conf", "box_cx", "box_cy",
        "angular_err_pan", "angular_err_tilt", "cmd_pan_deg", "cmd_tilt_deg",
        "target_vel_x", "target_vel_y", "target_speed", "sigma_a",
        "dist_cm", "range_valid", "range_stable", "range_jump", "dist_geom_m", "range_hold_m",
        "signal_strength",
        "fcs_state", "control_mode", "fire_stage",
        "is_locked", "lock_counter", "lock_thr", "hit_prob",
        "turret_pan_deg", "turret_tilt_deg",
        "gt_x", "gt_y", "gt_z", "gt_dist_m",
        "gravity_comp_deg", "tilt_lag_deg", "dist_center_m",
        "impact_miss_cm", "ballistic_hit",
        "kp_pan", "kp_tilt", "max_output_deg", "tune_label"
    ]

    def __init__(self, supervisor=None, output_dir=None):
        self.supervisor = supervisor
        self.records = []
        self.is_saved = False

        if output_dir is None:
            # Webots loglari ayri klasorde: webots/logs/
            self.output_dir = Path(__file__).resolve().parents[2] / "logs"
        else:
            self.output_dir = Path(output_dir)

        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = self.output_dir / f"webots_telemetry_{self.session_id}.csv"

        # Balon dugumunu bagla
        self.balloon_node = None
        if self.supervisor is not None:
            try:
                self.balloon_node = self.supervisor.getFromDef("BALLOON")
            except Exception:
                self.balloon_node = None

        atexit.register(self.save)

    def log(self, frame_id, sim_t, dt, fps, fcs, hw, best_box, fire_stage=None, fcs_state=None, ballistics=None, tuning=None):
        """Her adimda Schema v2 telemetri satirini tampona ekler."""
        # 1. Zemin Gercegi (Ground Truth)
        gt_x, gt_y, gt_z = 0.0, 0.0, 0.0
        gt_dist_m = 0.0

        if self.balloon_node is not None:
            try:
                pos = self.balloon_node.getPosition()
                if pos:
                    gt_x, gt_y, gt_z = float(pos[0]), float(pos[1]), float(pos[2])
                    gt_dist_m = float(np.sqrt(gt_x**2 + gt_y**2 + gt_z**2))
            except Exception:
                pass

        # 2. Algoritma Durumlari
        has_target = best_box is not None
        conf = float(getattr(fcs, "target_conf", 0.0)) if has_target else 0.0
        cx = float((best_box[0] + best_box[2]) / 2.0) if has_target else 0.0
        cy = float((best_box[1] + best_box[3]) / 2.0) if has_target else 0.0

        # Kontrolcu ici filtre degil, gercek kamera acisal hatasi.
        p_err = float(getattr(fcs, "last_pan_err_deg", 0.0))
        t_err = float(getattr(fcs, "last_tilt_err_deg", 0.0))

        cmd_p = float(hw.cmd_pan_deg)
        cmd_t = float(hw.cmd_tilt_deg)

        vx = float(fcs.tracker.x[2, 0]) if hasattr(fcs.tracker, "x") else 0.0
        vy = float(fcs.tracker.x[3, 0]) if hasattr(fcs.tracker, "x") else 0.0
        v_speed = float(np.hypot(vx, vy))
        sigma_a = float(getattr(fcs.tracker, "sigma_a", CONFIG.kalman.sigma_a))

        dist_cm, strength = hw.get_distance_and_strength()
        curr_p, curr_t = hw.get_motor_positions_deg()

        range_valid = 1 if getattr(fcs, "range_valid", False) else 0
        range_stable = 1 if getattr(fcs, "range_stable", False) else 0
        range_jump = 1 if getattr(fcs, "range_jump", False) else 0
        dist_geom = getattr(fcs, "range_geom_m", None)
        dist_geom_val = round(float(dist_geom), 3) if dist_geom is not None else -1.0
        range_hold = float(getattr(fcs, "last_valid_target_dist_m", -1.0))

        # A/D tesihs alanlari: gravity, servo lag ve balistik sonuc
        gravity_comp = float(getattr(fcs, "gravity_comp_deg", 0.0))
        tilt_lag = float(cmd_t - curr_t)
        last_shot = getattr(ballistics, "last_shot", None) if ballistics is not None else None
        dist_center = float(last_shot.get("dist_center_m", -1.0)) if last_shot else -1.0
        miss_cm = getattr(ballistics, "last_miss_cm", None) if ballistics is not None else None
        hit = getattr(ballistics, "last_hit", None) if ballistics is not None else None
        miss_val = round(float(miss_cm), 1) if miss_cm is not None else -1.0
        hit_val = -1 if hit is None else (1 if hit else 0)
        tune = tuning if isinstance(tuning, dict) else {}

        state_code = int(fcs_state) if fcs_state is not None else (1 if fcs.is_locked else 0)

        row = [
            self.SCHEMA_VERSION,
            frame_id,
            round(sim_t, 4),
            round(dt, 4),
            fps,
            1 if has_target else 0,
            round(conf, 3),
            round(cx, 1),
            round(cy, 1),
            round(p_err, 3),
            round(t_err, 3),
            round(cmd_p, 3),
            round(cmd_t, 3),
            round(vx, 2),
            round(vy, 2),
            round(v_speed, 2),
            round(sigma_a, 2),
            dist_cm,
            range_valid,
            range_stable,
            range_jump,
            dist_geom_val,
            range_hold,
            strength,
            state_code,
            fcs.control_mode,
            str(fire_stage or "NONE"),
            1 if fcs.is_locked else 0,
            fcs.lock_counter,
            round(float(fcs.lock_mgr.compute_dynamic_threshold(fcs.target_velocity)), 2),
            round(float(fcs.hit_probability), 1),
            round(curr_p, 3),
            round(curr_t, 3),
            round(gt_x, 3),
            round(gt_y, 3),
            round(gt_z, 3),
            round(gt_dist_m, 3),
            round(gravity_comp, 2),
            round(tilt_lag, 3),
            round(dist_center, 3),
            miss_val,
            hit_val,
            round(float(tune.get("kp_pan", 0.0)), 4),
            round(float(tune.get("kp_tilt", 0.0)), 4),
            round(float(tune.get("max_output_deg", 0.0)), 3),
            str(tune.get("label", ""))
        ]
        self.records.append(row)

        if len(self.records) >= 100:
            self._flush()

    def _flush(self):
        if not self.records:
            return
        file_exists = self.csv_path.exists()
        with open(self.csv_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(self.HEADERS)
            writer.writerows(self.records)
        self.records = []

    def save(self):
        """Kapanista kalan kayitlari diske yazar."""
        if self.is_saved:
            return
        self._flush()
        self.is_saved = True
        print(f"[TELEMETRI] Webots telemetri dosyasi kaydedildi: {self.csv_path.name}")
