import numpy as np


class WebotsPatrolGenerator:
    """
    1.5s bekleme, sakin merkeze donus ve 2 eksenli dairesel/eliptik taramali devriye ureteci (Webots).
    """

    def __init__(
        self,
        pan_limits=(-1.57, 1.57),
        tilt_limits=(-0.52, 0.78),
        wait_delay_sec=1.5,
        scan_pan_amp_deg=35.0,
        scan_tilt_amp_deg=8.0,
        scan_period_sec=5.0,
        return_speed_dps=20.0,
    ):
        self.PAN_LIMITS = pan_limits
        self.TILT_LIMITS = tilt_limits
        self.wait_delay_sec = float(wait_delay_sec)
        self.scan_pan_amp = float(np.radians(scan_pan_amp_deg))
        self.scan_tilt_amp = float(np.radians(scan_tilt_amp_deg))
        self.scan_freq = float(2.0 * np.pi / max(0.5, scan_period_sec))
        self.return_speed_rps = float(np.radians(return_speed_dps))

        self.state = "WAITING"   # "WAITING", "RETURNING", "SCANNING"
        self.lost_timer = 0.0
        self.scan_time = 0.0
        self.search_time = 0.0
        self.is_searching = False

    def on_target_detected(self):
        """Hedef tespit edildiginde devriye beklemede kalir, zamanlayicilar sifirlanir."""
        self.state = "WAITING"
        self.lost_timer = 0.0
        self.scan_time = 0.0
        self.search_time = 0.0
        self.is_searching = False

    def update(self, dt, current_pan, current_tilt):
        """
        Hedef yokken bir sonraki devriye adimini (pan_target, tilt_target radyan) dondurur.
        """
        dt = 0.016 if (dt is None or dt <= 0.0) else float(dt)
        self.is_searching = True
        self.search_time += dt
        curr_p = float(current_pan) if current_pan is not None else 0.0
        curr_t = float(current_tilt) if current_tilt is not None else 0.0

        # 1. ASAMA: HEDEF KAYBOLDUKTAN SONRA 1.5 SANIYE YERINDE SAKINCE BEKLE
        if self.state == "WAITING":
            self.lost_timer += dt
            if self.lost_timer < self.wait_delay_sec:
                return curr_p, curr_t
            else:
                self.state = "RETURNING"

        # 2. ASAMA: SAKINCE MERKEZE DONUS (0, 0 radyan)
        if self.state == "RETURNING":
            err_pan = -curr_p
            err_tilt = -curr_t
            dist_to_center = float(np.hypot(err_pan, err_tilt))

            # Merkeze 1.5 derece (0.026 rad) yaklastiysak 8-sekli taramaya gec
            if dist_to_center < np.radians(1.5):
                self.state = "SCANNING"
                self.scan_time = 0.0
                return 0.0, 0.0

            # Merkeze dogru sakin adim
            step = self.return_speed_rps * dt
            dir_pan = float(np.sign(err_pan))
            dir_tilt = float(np.sign(err_tilt))

            target_p = curr_p + dir_pan * min(abs(err_pan), step)
            target_t = curr_t + dir_tilt * min(abs(err_tilt), step)

            target_p = float(np.clip(target_p, self.PAN_LIMITS[0], self.PAN_LIMITS[1]))
            target_t = float(np.clip(target_t, self.TILT_LIMITS[0], self.TILT_LIMITS[1]))
            return target_p, target_t

        # 3. ASAMA: 8-SEKLI (SONSUZLUK) LISSAJOUS SEKTOR TARAMASI (Pan: ±35°, Tilt: ±8°)
        elif self.state == "SCANNING":
            self.scan_time += dt

            # Spiral genisleme (2 saniyede tam capa ulasma)
            ramp = min(1.0, self.scan_time / 2.0)

            pattern_pan = ramp * self.scan_pan_amp * float(np.sin(self.scan_freq * self.scan_time))
            pattern_tilt = ramp * self.scan_tilt_amp * float(np.sin(2.0 * self.scan_freq * self.scan_time))

            # Puruzsuz yumusak takip
            target_p = 0.92 * curr_p + 0.08 * pattern_pan
            target_t = 0.92 * curr_t + 0.08 * pattern_tilt

            target_p = float(np.clip(target_p, self.PAN_LIMITS[0], self.PAN_LIMITS[1]))
            target_t = float(np.clip(target_t, self.TILT_LIMITS[0], self.TILT_LIMITS[1]))
            return target_p, target_t

        return 0.0, 0.0

    def reset(self):
        """Arama durumunu ve saatini sifirlar."""
        self.state = "WAITING"
        self.lost_timer = 0.0
        self.scan_time = 0.0
        self.search_time = 0.0
        self.is_searching = False