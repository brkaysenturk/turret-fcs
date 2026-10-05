import atexit
import csv
from pathlib import Path
import numpy as np


class WebotsTelemetryLogger:
    """
    Webots FCS Simulasyonu Telemetri ve Analiz Kaydedicisi.
    Simulasyondaki zemin gercegi (ground truth) ve algoritma kestirimlerini
    RAM'de toplar, cikista CSV ve MATLAB (.mat) formatinda kaydeder.
    """

    def __init__(self, supervisor=None, output_dir=None):
        self.supervisor = supervisor
        self.records = []
        self.is_saved = False

        if output_dir is None:
            self.output_dir = Path(__file__).resolve().parents[2] / "analysis"
        else:
            self.output_dir = Path(output_dir)

        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Balon dugumunu bagla (Ground truth pozisyon ve hiz icin)
        self.balloon_node = None
        if self.supervisor is not None:
            try:
                self.balloon_node = self.supervisor.getFromDef("BALLOON")
                if self.balloon_node is None:
                    # Alternatif: Children icinde ara
                    root_children = self.supervisor.getRoot().getField("children")
                    for i in range(root_children.getCount()):
                        node = root_children.getMFNode(i)
                        name_field = node.getField("name")
                        if name_field and name_field.getSFString() == "balloon":
                            self.balloon_node = node
                            break
            except Exception:
                self.balloon_node = None

        # Program aniden kapansa dahi veriyi kurtarmak icin atexit kaydi
        atexit.register(self.save)

    def log(self, sim_t, fcs, hw, best_box, is_firing=False, ballistics=None):
        """Her adimda tum FCS parametrelerini ve gercek verileri kaydeder."""
        # 1. Zemin Gercegi (Ground Truth) Okumalari
        gt_x, gt_y, gt_z = 0.0, 0.0, 0.0
        gt_vx, gt_vy, gt_vz = 0.0, 0.0, 0.0
        true_dist_m = 0.0

        if self.balloon_node is not None:
            try:
                pos = self.balloon_node.getPosition()
                if pos:
                    gt_x, gt_y, gt_z = float(pos[0]), float(pos[1]), float(pos[2])
                    true_dist_m = float(np.sqrt(gt_x**2 + gt_y**2 + gt_z**2))

                vel = self.balloon_node.getVelocity()
                if vel:
                    gt_vx, gt_vy, gt_vz = float(vel[0]), float(vel[1]), float(vel[2])
            except Exception:
                pass

        gt_speed = float(np.sqrt(gt_vx**2 + gt_vz**2))

        # 2. Donanim Okumalari
        curr_pan_rad, curr_tilt_rad = hw.get_motor_positions()
        curr_pan_deg = float(np.degrees(curr_pan_rad))
        curr_tilt_deg = float(np.degrees(curr_tilt_rad))
        dist_cm, strength = hw.get_distance_and_strength()

        # 3. FCS Kontrol ve Takip Metrikleri
        target_detected = 1 if best_box is not None else 0
        bbox_cx, bbox_cy = (0.0, 0.0)
        if best_box is not None:
            bbox_cx = float((best_box[0] + best_box[2]) / 2.0)
            bbox_cy = float((best_box[1] + best_box[3]) / 2.0)

        # Kalman Tahminleri
        kvx = float(fcs.target_velocity[0])
        kvy = float(fcs.target_velocity[1])
        kalman_speed = float(np.hypot(kvx, kvy))

        # Komuta ve Onleme Acilari
        cmd_pan_deg = float(np.degrees(fcs.pan_target))
        cmd_tilt_deg = float(np.degrees(fcs.tilt_target))
        lead_pan_deg = float(fcs.firing_lead_offset[0])
        lead_tilt_deg = float(fcs.firing_lead_offset[1])
        gravity_comp_deg = float(fcs.gravity_comp_deg)

        # Aci Takip Hatalari
        err_pan_deg = float(fcs.actual_pan)
        err_tilt_deg = float(fcs.actual_tilt)

        # Kilit Kalitesi ve Durumu
        lock_qual = float(fcs.lock_quality)
        is_locked = 1 if fcs.is_locked else 0
        firing_flag = 1 if is_firing else 0

        # Balistik Skor
        hits = 0
        total_shots = 0
        if ballistics is not None:
            hits = getattr(ballistics, "total_hits", 0)
            total_shots = getattr(ballistics, "total_shots", 0)

        row = {
            "time_sec": round(sim_t, 4),
            # Ground Truth (Gercek Fizik)
            "gt_x": round(gt_x, 4),
            "gt_y": round(gt_y, 4),
            "gt_z": round(gt_z, 4),
            "gt_vx": round(gt_vx, 4),
            "gt_vy": round(gt_vy, 4),
            "gt_vz": round(gt_vz, 4),
            "gt_speed": round(gt_speed, 4),
            "true_dist_cm": round(true_dist_m * 100.0, 1),
            # Algilama & Tespit
            "target_detected": target_detected,
            "bbox_cx": round(bbox_cx, 1),
            "bbox_cy": round(bbox_cy, 1),
            "lidar_dist_cm": dist_cm,
            # Kalman Durum Gozlemcisi
            "kalman_vx": round(kvx, 2),
            "kalman_vy": round(kvy, 2),
            "kalman_speed": round(kalman_speed, 2),
            # Balistik & Onleme
            "lead_pan_deg": round(lead_pan_deg, 3),
            "lead_tilt_deg": round(lead_tilt_deg, 3),
            "gravity_comp_deg": round(gravity_comp_deg, 3),
            # Taret Kontrol ve Motorlar
            "cmd_pan_deg": round(cmd_pan_deg, 3),
            "cmd_tilt_deg": round(cmd_tilt_deg, 3),
            "actual_pan_deg": round(curr_pan_deg, 3),
            "actual_tilt_deg": round(curr_tilt_deg, 3),
            "err_pan_deg": round(err_pan_deg, 3),
            "err_tilt_deg": round(err_tilt_deg, 3),
            # Kilit ve Atis
            "lock_quality": round(lock_qual, 1),
            "is_locked": is_locked,
            "is_firing": firing_flag,
            "hits": hits,
            "total_shots": total_shots,
        }
        self.records.append(row)

    def save(self, base_name="fcs_telemetry"):
        """Tamponlanan verileri CSV ve MATLAB .mat dosyasi olarak kaydeder."""
        if self.is_saved or len(self.records) == 0:
            return

        self.is_saved = True
        csv_path = self.output_dir / f"{base_name}.csv"
        mat_path = self.output_dir / f"{base_name}.mat"

        # 1. CSV Kaydi
        fieldnames = list(self.records[0].keys())
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)

        print(f"\n[OK] [TELEMETRI] {len(self.records)} kare basariyla kaydedildi:")
        print(f"     -> CSV : {csv_path}")
        print(f"     -> MATLAB icin 'plot_fcs_analysis.m' calistirabilirsiniz.")
