import numpy as np


class WebotsPIDController:
    """Kamera koordinatlarindan mutlak motor acilarini ve balistik kompanzasyonu hesaplar."""

    def __init__(self, kp=0.42, alpha=0.75, pan_limits=(-1.57, 1.57), tilt_limits=(-0.52, 0.78)):
        self.kp = kp
        self.alpha = alpha
        self.PAN_LIMITS = pan_limits
        self.TILT_LIMITS = tilt_limits

        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.gravity_comp_deg = 0.0
        self.first_detection = True

    def process_detection(self, best_box, center_x, center_y, fx, fy, current_pan, current_tilt):
        """Tespit edilen kutudan kamera sapmasi ve dunya koordinat piksellerini cikarir."""
        x1, y1, x2, y2 = best_box
        bx = (x1 + x2) // 2
        by = (y1 + y2) // 2

        target_err_x = bx - center_x
        target_err_y = by - center_y
        raw_actual_pan = np.degrees(np.arctan(target_err_x / max(fx, 1e-6)))
        raw_actual_tilt = np.degrees(np.arctan(-target_err_y / max(fy, 1e-6)))

        self.actual_pan = self.alpha * raw_actual_pan + (1.0 - self.alpha) * self.actual_pan
        self.actual_tilt = self.alpha * raw_actual_tilt + (1.0 - self.alpha) * self.actual_tilt

        if current_pan is not None and not np.isnan(current_pan) and current_tilt is not None and not np.isnan(current_tilt):
            target_world_pan_deg = np.degrees(current_pan) - raw_actual_pan
            target_world_tilt_deg = np.degrees(current_tilt) + raw_actual_tilt
        else:
            target_world_pan_deg = -raw_actual_pan
            target_world_tilt_deg = raw_actual_tilt

        world_px = center_x - np.tan(np.radians(target_world_pan_deg)) * fx
        world_py = center_y - np.tan(np.radians(target_world_tilt_deg)) * fy

        return bx, by, raw_actual_pan, raw_actual_tilt, world_px, world_py

    def aim(self, current_pan, current_tilt, raw_p, raw_t, target_d_m, firing_lead_offset, current_pan_cmd, current_tilt_cmd, is_firing=False):
        """Mutlak motor acilarini ve analitik yercekimi super-elevasyonunu hesaplar."""
        if current_pan is not None and not np.isnan(current_pan) and current_tilt is not None and not np.isnan(current_tilt):
            target_world_p = current_pan - np.radians(raw_p)
            target_world_t = current_tilt + np.radians(raw_t)

            if is_firing:
                pan_cmd = target_world_p - np.radians(firing_lead_offset[0])
                target_t_los = target_world_t + np.radians(firing_lead_offset[1])
                # Analitik balistik kompanzasyon: tan(theta_aim) = tan(theta_los) + (g * D) / (2 * v^2)
                tilt_cmd = float(np.arctan(np.tan(target_t_los) + (9.81 * target_d_m / 200.0)))
                self.gravity_comp_deg = float(np.degrees(tilt_cmd - target_t_los))
            else:
                self.gravity_comp_deg = float(np.degrees(np.arctan((9.81 * target_d_m) / 200.0)))
                if self.first_detection:
                    pan_cmd = target_world_p
                    tilt_cmd = target_world_t
                    self.first_detection = False
                else:
                    pan_cmd = 0.60 * target_world_p + 0.40 * current_pan_cmd
                    tilt_cmd = 0.60 * target_world_t + 0.40 * current_tilt_cmd
        else:
            self.gravity_comp_deg = float(np.degrees(np.arctan((9.81 * target_d_m) / 200.0)))
            active_p_err = self.actual_pan + (firing_lead_offset[0] if is_firing else 0.0)
            active_t_err = self.actual_tilt + ((firing_lead_offset[1] + self.gravity_comp_deg) if is_firing else 0.0)
            pan_cmd = current_pan_cmd - np.radians(self.kp * active_p_err)
            tilt_cmd = current_tilt_cmd + np.radians(self.kp * active_t_err)

        pan_cmd = max(self.PAN_LIMITS[0], min(pan_cmd, self.PAN_LIMITS[1]))
        tilt_cmd = max(self.TILT_LIMITS[0], min(tilt_cmd, self.TILT_LIMITS[1]))
        return pan_cmd, tilt_cmd

    def reset(self):
        """Kontrolcu gecmisini sifirlar."""
        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.gravity_comp_deg = 0.0
        self.first_detection = True

