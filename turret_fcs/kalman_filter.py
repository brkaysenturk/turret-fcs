"""
================================================================================
MODUL: HEDEF DURUM KESTIRIMI - SABIT HIZLI KALMAN FILTRESI (CV KALMAN)
================================================================================
Kamera piksel duzleminde hedefin konumunu (x, y) ve ego-kompanzasyonlu dunya
referansli hizini (vx, vy) kestiren dogrusal Sabit Hizli (CV) Kalman filtresi.

TASARIM KARARLARI:
1. Durum vektoru X = [x, y, vx, vy]^T, olcum Z = [x, y]^T.
2. Surekli zamanli beyaz ivme gurultusu (CWNA) spektral yogunluk ayriklastirmasi
   (dt^3/3, dt^2/2, dt) ile surec gurultusu kare hizindan (FPS) ve alt-adimlamadan
   bagimsizlastirilmistir.
3. Ego-hareket telafisi (ego_dx, ego_dy) ve servo/disli boslugu kaynakli yer
   degistirme belirsizligi (ego_var_x, ego_var_y) dogrudan kestirim adiminda
   uygulanir; boylece kamera sarsintisi sahte hedef ivmesi sanilmaz.
4. Cikista 30-60 px/s yumusak hiz olu bolgesi vardir.
5. Robust İnovasyon ve Tekil-Sicrama Kapisi (Outlier + NIS Gate):
   Hem piksel sicrama esigini (100 px) hem de kovaryans agirlikli normalize
   inovasyon karesi (NIS > chi2) esigini asan sahte YOLO sicramalari reddedilir.
   Ayni yeni bolgede ust uste 3 kare tutarli olcum gorulurse aday gecmisi
   yeniden oynatilarak (replay) yeni hedefe gecilir.
================================================================================
"""

import numpy as np

from .config import CONFIG


class KalmanFilter:
    """2B konum ve hiz icin sabit hizli (CV) Kalman filtresi.

    Durum: X = [x, y, vx, vy]^T  (piksel, piksel/s, ego-kompanzasyonlu)
    Olcum: Z = [x, y]^T          (piksel)
    """

    def __init__(
        self,
        dt=0.033,
        r_var=CONFIG.kalman.r_var,
        sigma_a=CONFIG.kalman.sigma_a,
        initial_velocity_var=CONFIG.kalman.initial_velocity_var,
        deadband_low=CONFIG.kalman.velocity_deadband_low_px_s,
        deadband_high=CONFIG.kalman.velocity_deadband_high_px_s,
        outlier_jump_px=CONFIG.kalman.outlier_jump_px,
        reacquire_frames=CONFIG.kalman.reacquire_frames,
        innovation_gate_chi2=CONFIG.kalman.innovation_gate_chi2,
        reacquire_window_s=CONFIG.kalman.reacquire_window_s,
    ):
        self.dt = float(dt)
        self.r_var = float(r_var)
        self.sigma_a = float(sigma_a)
        self.initial_velocity_var = float(initial_velocity_var)
        self.velocity_deadband_low_px_s = float(deadband_low)
        self.velocity_deadband_high_px_s = float(deadband_high)
        self.outlier_jump_px = float(outlier_jump_px)
        self.reacquire_frames = int(reacquire_frames)
        self.innovation_gate_chi2 = float(innovation_gate_chi2)
        self.reacquire_window_s = float(reacquire_window_s)

        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ], dtype=float)
        self.R = np.eye(2, dtype=float) * self.r_var

        self.reset()

    def reset(self):
        """Filtreyi ilk kare durumuna dondurur."""
        self.x = np.zeros((4, 1), dtype=float)
        self.P = self._initial_covariance()
        self.initialized = False
        self.last_residual = 0.0
        self.last_nis = 0.0
        self.measurement_status = "reset"
        self.time_since_measurement_s = 0.0
        self._candidate_age_s = 0.0
        self._outlier_frames = 0
        self._outlier_history = []
        self.raw_vx = 0.0
        self.raw_vy = 0.0
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0

    def _initial_covariance(self):
        return np.diag([
            self.r_var,
            self.r_var,
            self.initial_velocity_var,
            self.initial_velocity_var,
        ])

    def _start_track(self, mx, my):
        """Yeni takip baslatir: konum olcumden, hiz sifirdan."""
        self.x = np.array([[float(mx)], [float(my)], [0.0], [0.0]], dtype=float)
        self.P = self._initial_covariance()
        self.initialized = True
        self.last_residual = 0.0
        self.last_nis = 0.0
        self.measurement_status = "initialized"
        self.time_since_measurement_s = 0.0
        self._candidate_age_s = 0.0
        self._outlier_frames = 0
        self._outlier_history = []
        self.raw_vx = 0.0
        self.raw_vy = 0.0
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0
        return float(mx), float(my), 0.0, 0.0

    def _process_noise(self, dt):
        """Tam CWNA ayriklastirmasi; spektral yogunluk q = sigma_a^2 * dt_ref (px^2/s^3).

        Nominal 30 FPS'deki hiz varyansini korurken biriken surec gurultusunu
        kare hizindan (FPS) ve alt-adimlamadan bagimsiz hale getirir.
        """
        dt2 = dt * dt
        dt3 = dt2 * dt
        q = self.sigma_a * self.sigma_a * CONFIG.kalman.noise_reference_dt_s
        return np.array([
            [dt3 / 3.0, 0.0, dt2 / 2.0, 0.0],
            [0.0, dt3 / 3.0, 0.0, dt2 / 2.0],
            [dt2 / 2.0, 0.0, dt, 0.0],
            [0.0, dt2 / 2.0, 0.0, dt],
        ], dtype=float) * q

    def _apply_velocity_deadband(self):
        """Ham durum hizina 30-60 px/s yumusak olu bolge uygular."""
        self.raw_vx = float(self.x[2, 0])
        self.raw_vy = float(self.x[3, 0])
        speed = float(np.hypot(self.raw_vx, self.raw_vy))
        band = max(1e-3, self.velocity_deadband_high_px_s - self.velocity_deadband_low_px_s)
        scale = float(np.clip(
            (speed - self.velocity_deadband_low_px_s) / band, 0.0, 1.0
        ))
        self.smooth_vx = self.raw_vx * scale
        self.smooth_vy = self.raw_vy * scale
        return float(self.x[0, 0]), float(self.x[1, 0]), self.smooth_vx, self.smooth_vy

    def _normalize_dt(self, dt):
        if dt is None:
            return self.dt
        dt = float(dt)
        return dt if np.isfinite(dt) and dt > 0.0 else self.dt

    def _clear_candidates(self):
        self._outlier_history = []
        self._outlier_frames = 0
        self._candidate_age_s = 0.0

    def _predict_state(self, dt, ego_dx, ego_dy, ego_var_x=0.0, ego_var_y=0.0):
        """Durum ve kovaryans icin alt-adimli ileri kestirim (predict) adimini hesaplar."""
        ego_var_x = max(0.0, float(ego_var_x)) if np.isfinite(ego_var_x) else 0.0
        ego_var_y = max(0.0, float(ego_var_y)) if np.isfinite(ego_var_y) else 0.0
        steps = max(1, int(np.ceil(dt / CONFIG.kalman.max_prediction_step_s)))
        step_dt = dt / steps
        F = np.array([
            [1.0, 0.0, step_dt, 0.0],
            [0.0, 1.0, 0.0, step_dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=float)
        x_pred, P_pred = self.x.copy(), self.P.copy()
        Q = self._process_noise(step_dt)
        for _ in range(steps):
            x_pred = F @ x_pred
            P_pred = F @ P_pred @ F.T + Q
        x_pred[0, 0] += float(ego_dx)
        x_pred[1, 0] += float(ego_dy)
        P_pred[0, 0] += ego_var_x
        P_pred[1, 1] += ego_var_y
        return x_pred, P_pred

    def _correct_state(self, x_pred, P_pred, mx, my):
        """Verilen olcumle Joseph formunda durum ve simetrik kovaryans duzeltmesi yapar."""
        z = np.array([[mx], [my]], dtype=float)
        nu = z - self.H @ x_pred
        S = self.H @ P_pred @ self.H.T + self.R
        K = np.linalg.solve(S, self.H @ P_pred).T  # K: 4x2
        self.x = x_pred + K @ nu
        i_kh = np.eye(4, dtype=float) - K @ self.H
        self.P = i_kh @ P_pred @ i_kh.T + K @ self.R @ K.T
        self.P = (self.P + self.P.T) * 0.5

    def predict_only(self, dt=None, ego_dx=0.0, ego_dy=0.0, ego_var_x=0.0, ego_var_y=0.0):
        """Olcum olmayan kisa kayip karelerde durumu ve ego-hareketi ileri sarar (coasting)."""
        self._clear_candidates()
        self.measurement_status = "coasting"
        if not self.initialized:
            return 0.0, 0.0, 0.0, 0.0
        dt = self._normalize_dt(dt)
        self.time_since_measurement_s += dt
        self.x, self.P = self._predict_state(dt, ego_dx, ego_dy, ego_var_x, ego_var_y)
        return self._apply_velocity_deadband()

    def update(self, measurement, dt=None, ego_dx=0.0, ego_dy=0.0, ego_var_x=0.0, ego_var_y=0.0):
        """Bir olcumu filtreye isler.

        measurement: (mx, my) piksel merkezi
        dt: onceki olcumden bu yana gecen sure (s)
        ego_dx, ego_dy: taretin kendi donusunden kaynaklanan piksel kaymasi
        ego_var_x, ego_var_y: taretin donus/backlash belirsizliginden kaynaklanan piksel varyansi

        Returns: (px, py, smooth_vx, smooth_vy)
        """
        dt = self._normalize_dt(dt)
        mx, my = float(measurement[0]), float(measurement[1])
        if not np.isfinite(mx) or not np.isfinite(my):
            output = self.predict_only(dt, ego_dx, ego_dy, ego_var_x, ego_var_y)
            self.measurement_status = "invalid"
            return output

        if not self.initialized:
            return self._start_track(mx, my)

        # 1. Kestirim: hedef hizi + bilinen ego kaymasi ve varyansi
        x_pred, P_pred = self._predict_state(dt, ego_dx, ego_dy, ego_var_x, ego_var_y)
        self.time_since_measurement_s += dt

        # 2. Inovasyon ve NIS (Normalized Innovation Squared)
        z = np.array([[mx], [my]], dtype=float)
        nu = z - self.H @ x_pred
        residual = float(np.hypot(nu[0, 0], nu[1, 0]))
        self.last_residual = residual
        S = self.H @ P_pred @ self.H.T + self.R
        self.last_nis = float((nu.T @ np.linalg.solve(S, nu)).item())

        # 3. Robust inovasyon ve tekil-sicrama kapisi (100 px + Ki-Kare NIS kapisi)
        if residual > self.outlier_jump_px and self.last_nis > self.innovation_gate_chi2:
            self._candidate_age_s += dt
            if self._candidate_age_s > self.reacquire_window_s:
                self._clear_candidates()
            if self._outlier_history:
                prev_mx, prev_my, *_ = self._outlier_history[-1]
                if float(np.hypot(mx - prev_mx - ego_dx, my - prev_my - ego_dy)) > self.outlier_jump_px:
                    self._clear_candidates()

            self._outlier_history.append((mx, my, dt, float(ego_dx), float(ego_dy), float(ego_var_x), float(ego_var_y)))
            self._outlier_frames = len(self._outlier_history)

            if self._outlier_frames >= self.reacquire_frames:
                history = list(self._outlier_history)
                first_mx, first_my, *_ = history[0]
                self._start_track(first_mx, first_my)
                for cand_mx, cand_my, cand_dt, cand_ego_dx, cand_ego_dy, var_x, var_y in history[1:]:
                    cand_x_pred, cand_P_pred = self._predict_state(cand_dt, cand_ego_dx, cand_ego_dy, var_x, var_y)
                    self._correct_state(cand_x_pred, cand_P_pred, cand_mx, cand_my)
                self.last_residual = 0.0
                self.measurement_status = "reacquired"
                return self._apply_velocity_deadband()

            self.x = x_pred
            self.P = P_pred
            self.measurement_status = "rejected"
            return self._apply_velocity_deadband()

        self._clear_candidates()
        self.time_since_measurement_s = 0.0
        self.measurement_status = "accepted"

        # 4. Kalman kazanci, durum ve Joseph formu kovaryans guncellemesi
        self._correct_state(x_pred, P_pred, mx, my)

        # 5. Cikti: ham hiz + yumusak olu bolge
        return self._apply_velocity_deadband()

    def get_velocity(self):
        """Yumusak olu bolgeden gecmis hedef hizini (vx, vy) px/s olarak doner."""
        return float(self.smooth_vx), float(self.smooth_vy)
