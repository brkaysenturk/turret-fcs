"""
================================================================================
MODUL: HEDEF DURUM KESTIRIMI - SABIT HIZLI KALMAN FILTRESI (CV KALMAN)
================================================================================
Kamera piksel duzleminde hedefin konumunu (x, y) ve dunya referansli hizini
(vx, vy) kestiren sade, tek katmanli bir dogrusal Kalman filtresi.

TASARIM KARARLARI (neden bu kadar sade?):
1. Durum vektoru X = [x, y, vx, vy]^T, olcum Z = [x, y]^T.
   Sabit hizli (Constant Velocity, CV) model yeterlidir; hedef balonun
   manevralari surec gurultusu (sigma_a) ile temsil edilir. IMM/CA gibi
   karmasik modellere gerek yoktur.
2. Ego-hareket telafisi (ego_dx, ego_dy) dogrudan kestirim adiminda uygulanir.
   Boylece hiz durumu dunya referanslidir; taret donerken hedefin gercek
   hizi ile kamera kaynakli gorunur kayma birbirine karismaz.
3. Tek bir surec gurultusu (sigma_a) ve tek bir olcum gurultusu (r_var)
   kullanilir. Joseph formu ile sayisal kararli kovaryans guncellemesi saglanir.
   Adaptif Q, hiz EMA'si, ivme sinirlamasi, donus dedektoru ve olcum egimi
   gibi gecikme ureten katmanlar kaldirilmistir.
4. Cikista yalnizca 30-60 px/s yumusak bir hiz olu bolgesi vardir. Bu bir filtre
   katmani degil, duran hedefin sifir civarindaki gurultusunu lead noktasindan
   uzak tutan statik bir esiklemedir.
5. Robust Tekil-Sicrama Kapisi: Tek karelik 150 px ustu sahte YOLO kutu
   sicramalarinda olcum reddedilip yalnizca kestirim (predict-only) surdurulur;
   hiz, lead ve kovaryans P hic kirlenmez. Ayni yeni bolgede 3 kare tutarli
   olcum gorulurse gecmis aday olcumler yeniden oynatilarak (replay) yeni
   hedefe sifir gecikmeyle gecilir.
================================================================================
"""

import numpy as np

from .config import CONFIG


class KalmanFilter:
    """2B konum ve hiz icin sabit hizli (CV) Kalman filtresi.

    Durum: X = [x, y, vx, vy]^T  (piksel, piksel/s, dunya referansli)
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
    ):
        self.dt = float(dt)
        self.r_var = float(r_var)
        self.sigma_a = float(sigma_a)
        self.initial_velocity_var = float(initial_velocity_var)
        self.velocity_deadband_low_px_s = float(deadband_low)
        self.velocity_deadband_high_px_s = float(deadband_high)
        self.outlier_jump_px = float(outlier_jump_px)
        self.reacquire_frames = int(reacquire_frames)

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
        self._outlier_frames = 0
        self._outlier_history = []
        self.raw_vx = 0.0
        self.raw_vy = 0.0
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0
        return float(mx), float(my), 0.0, 0.0

    def _process_noise(self, dt):
        """CWNA surec gurultusu: Q = sigma_a^2 * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]."""
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        q = self.sigma_a * self.sigma_a
        return np.array([
            [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
            [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
            [dt3 / 2.0, 0.0, dt2, 0.0],
            [0.0, dt3 / 2.0, 0.0, dt2],
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

    def _predict_state(self, dt, ego_dx, ego_dy):
        """Durum ve kovaryans icin ileri kestirim (predict) adimini hesaplar."""
        F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=float)
        x_pred = F @ self.x
        x_pred[0, 0] += float(ego_dx)
        x_pred[1, 0] += float(ego_dy)
        P_pred = F @ self.P @ F.T + self._process_noise(dt)
        return x_pred, P_pred

    def _correct_state(self, x_pred, P_pred, mx, my):
        """Verilen olcumle Joseph formunda durum ve kovaryans duzeltmesi yapar."""
        z = np.array([[mx], [my]], dtype=float)
        nu = z - self.H @ x_pred
        S = self.H @ P_pred @ self.H.T + self.R
        K = np.linalg.solve(S, self.H @ P_pred).T  # K: 4x2
        self.x = x_pred + K @ nu
        i_kh = np.eye(4, dtype=float) - K @ self.H
        self.P = i_kh @ P_pred @ i_kh.T + K @ self.R @ K.T

    def predict_only(self, dt=0.033, ego_dx=0.0, ego_dy=0.0):
        """Olcum olmayan kisa kayip karelerde durumu ve ego-hareketi ileri sarar (coasting)."""
        if not self.initialized:
            return 0.0, 0.0, 0.0, 0.0
        dt = max(0.010, min(float(dt), 0.100))
        self.x, self.P = self._predict_state(dt, ego_dx, ego_dy)
        return self._apply_velocity_deadband()

    def update(self, measurement, dt=0.033, ego_dx=0.0, ego_dy=0.0):
        """Bir olcumu filtreye isler.

        measurement: (mx, my) piksel merkezi
        dt: onceki olcumden bu yana gecen sure (s)
        ego_dx, ego_dy: taretin kendi donusunden kaynaklanan piksel kaymasi

        Returns: (px, py, smooth_vx, smooth_vy)
        """
        dt = max(0.010, min(float(dt), 0.100))
        mx, my = float(measurement[0]), float(measurement[1])

        if not self.initialized:
            return self._start_track(mx, my)

        # 1. Kestirim: hedef hizi + bilinen ego kaymasi
        x_pred, P_pred = self._predict_state(dt, ego_dx, ego_dy)

        # 2. Inovasyon
        z = np.array([[mx], [my]], dtype=float)
        nu = z - self.H @ x_pred
        residual = float(np.hypot(nu[0, 0], nu[1, 0]))
        self.last_residual = residual

        # 3. Robust tekil-sicrama kapisi (150 px)
        # Tek karelik sahte YOLO kutu sicramalarinda olcum tamamen reddedilip
        # saf kestirim (predict-only) korunur; hiz, lead ve kovaryans P kirlenmez.
        # Ayni yeni bolgede ust uste 3 tutarli olcum gorulurse aday gecmisi
        # yeniden oynatilarak (replay) yeni hedefe gecilir.
        if residual > self.outlier_jump_px:
            if self._outlier_history:
                prev_mx, prev_my, _, _, _ = self._outlier_history[-1]
                if float(np.hypot(mx - prev_mx, my - prev_my)) > self.outlier_jump_px:
                    self._outlier_history = []
                    self._outlier_frames = 0

            self._outlier_history.append((mx, my, dt, float(ego_dx), float(ego_dy)))
            self._outlier_frames = len(self._outlier_history)

            if self._outlier_frames >= self.reacquire_frames:
                history = list(self._outlier_history)
                first_mx, first_my, _, _, _ = history[0]
                self._start_track(first_mx, first_my)
                for cand_mx, cand_my, cand_dt, cand_ego_dx, cand_ego_dy in history[1:]:
                    cand_x_pred, cand_P_pred = self._predict_state(cand_dt, cand_ego_dx, cand_ego_dy)
                    self._correct_state(cand_x_pred, cand_P_pred, cand_mx, cand_my)
                self.last_residual = 0.0
                return self._apply_velocity_deadband()

            self.x = x_pred
            self.P = P_pred
            return self._apply_velocity_deadband()

        self._outlier_frames = 0
        self._outlier_history = []

        # 4. Kalman kazanci, durum ve Joseph formu kovaryans guncellemesi
        self._correct_state(x_pred, P_pred, mx, my)

        # 5. Cikti: ham hiz + yumusak olu bolge (ek filtre katmani yok)
        return self._apply_velocity_deadband()

    def get_velocity(self):
        """Yumusak olu bolgeden gecmis hedef hizini (vx, vy) px/s olarak doner."""
        return float(self.smooth_vx), float(self.smooth_vy)
