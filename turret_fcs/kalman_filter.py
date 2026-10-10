"""
================================================================================
MODUL: ADAPTIF DURUM KESTIRIMI VE KALMAN FILTRESI (KALMAN FILTER)
================================================================================
Bu modul, kamera piksel duzleminde hedefin konumunu (x, y) ve hiz vektorunu (vx, vy)
kestiren, olcum gurultusunu suzen ve balistik atis onleme (Lead Time) kestirimi yapan
Surekli Beyaz Gurultulu Ivme (CWNA) tabanli genisletilmis Kalman Filtresidir.

DURUM VE OLCUM UZAYI:
- Durum Vektoru: X = [x, y, vx, vy]^T
- Olcum Vektoru: Z = [x_meas, y_meas]^T
- Durum Gecis Matrisi F: x(k+1) = x(k) + vx(k)*dt

GELISMIS OZELLIKLER:
1. Inovasyon Tabanli Adaptif Surec Gurultusu (Adaptive Q Matrisi):
   Hedef sabitken kucuk varyans (sigma_a_min=5.0 px/s^2) ile mukemmel gurultu suzme;
   hedef ani yon degistirdiginde (residual > 32 px) sigma_a_max=320.0 px/s^2 seviyesine
   yukselerek gecikmesiz hizli manevra takibi.
2. Olcum Gurultusu Kovaryansi R_var = 25.0 px^2:
   YOLOv8 sinirlayici kutu titresimlerini (jitter) motora iletmeden duzlestirir.
3. Cift Kademeli Hiz Yumusatma (Exponential Velocity Smoothing):
   Hiz turevindeki anlik pikleri (spike) suzerek atis onleme noktasinin (Lead Reticle)
   ekranda titremesini engeller.
================================================================================
"""

import numpy as np


class KalmanFilter:
    """
    2D hedef takip icin Surekli Beyaz Gurultulu Ivme (CWNA) ve 
    Inovasyon Tabanli Adaptif Surec Gurultusu (Adaptive Q) mimarili Kalman Filtresi.
    
    Durum vektoru: X = [x, y, vx, vy]^T
    Olcum vektoru: Z = [x, y]^T
    """

    DEFAULT_LEAD_TIME = 0.18  # saniye

    def __init__(
        self,
        dt=0.033,
        r_var=25.0,          # Olcum gurultusu varyansi (pixel^2) - YOLO kutu titremesini yumusatmak icin artirildi
        sigma_a_min=5.0,     # Kararli/yavas hedef ivme gurultusu (px/s^2)
        sigma_a_max=320.0,   # Hizli manevra/yon degisimi ivme gurultusu (px/s^2)
        residual_low=8.0,    # Alt inovasyon esigi (pixel)
        residual_high=32.0,  # Ust inovasyon esigi (pixel)
        jump_threshold=90.0  # Hedef atlamasi/yanlis tespit esigi (pixel)
    ):
        self.dt = float(dt)
        self.r_var = float(r_var)
        self.sigma_a_min = float(sigma_a_min)
        self.sigma_a_max = float(sigma_a_max)
        self.residual_low = float(residual_low)
        self.residual_high = float(residual_high)
        self.jump_threshold = float(jump_threshold)

        # Durum ve Kovaryans
        self.x = np.zeros((4, 1), dtype=float)
        self.P = np.diag([10.0, 10.0, 100.0, 100.0])
        self.initialized = False

        # Olcum Matrisi H: [2 x 4]
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ], dtype=float)

        # Sabit Olcum Gurultusu Matrisi R: [2 x 2]
        self.R = np.eye(2, dtype=float) * self.r_var

        self.last_meas_x = 0.0
        self.last_meas_y = 0.0
        self.last_residual = 0.0
        self.current_sigma_a = self.sigma_a_min
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0

    def update(self, measurement, dt=0.033):
        """
        Durum kestirimi ve Joseph formunda kovaryans guncellemesi.
        Returns: px, py, smooth_vx, smooth_vy
        """
        dt = max(0.010, min(float(dt), 0.1))
        mx, my = float(measurement[0]), float(measurement[1])
        meas = np.array([[mx], [my]], dtype=float)

        # Ilk kare ilklendirmesi
        if not self.initialized:
            self.x = np.array([[mx], [my], [0.0], [0.0]], dtype=float)
            self.P = np.diag([10.0, 10.0, 100.0, 100.0])
            self.last_meas_x = mx
            self.last_meas_y = my
            self.initialized = True
            return mx, my, 0.0, 0.0

        # Anormal hedef atlamasi (farkli nesne veya sahte tespit)
        meas_jump = float(np.hypot(mx - self.last_meas_x, my - self.last_meas_y))
        if meas_jump > self.jump_threshold:
            self.x = np.array([[mx], [my], [0.0], [0.0]], dtype=float)
            self.P = np.diag([10.0, 10.0, 80.0, 80.0])
            self.last_meas_x = mx
            self.last_meas_y = my
            return mx, my, 0.0, 0.0

        # 1. Durum Gecis Matrisi F (Sabit Hizli Model)
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=float)

        x_pred = F @ self.x

        # 2. Inovasyon (Residual)
        y = meas - self.H @ x_pred
        residual = float(np.hypot(y[0, 0], y[1, 0]))
        self.last_residual = residual

        # Dinamik Adaptasyon:
        alpha = float(np.clip((residual - self.residual_low) / (self.residual_high - self.residual_low), 0.0, 1.0))
        sigma_a = self.sigma_a_min + alpha * (self.sigma_a_max - self.sigma_a_min)
        self.current_sigma_a = sigma_a
        sigma_a2 = sigma_a * sigma_a

        # CWNA Surec Gurultusu Matrisi Q
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt

        Q = np.array([
            [dt4 / 4.0, 0.0,       dt3 / 2.0, 0.0      ],
            [0.0,       dt4 / 4.0, 0.0,       dt3 / 2.0],
            [dt3 / 2.0, 0.0,       dt2,       0.0      ],
            [0.0,       dt3 / 2.0, 0.0,       dt2      ],
        ], dtype=float) * sigma_a2

        # 3. Kovaryans Kestirimi & Kalman Kazanci
        P_pred = F @ self.P @ F.T + Q
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)

        # Durum Guncellemesi
        self.x = x_pred + K @ y

        # Numerik Kararlilik: Joseph Form Kovaryans Guncellemesi
        I_KH = np.eye(4, dtype=float) - K @ self.H
        self.P = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T

        # Hiz sinirlamasi
        MAX_VEL = 700.0  # px/s
        raw_vx = float(np.clip(self.x[2, 0], -MAX_VEL, MAX_VEL))
        raw_vy = float(np.clip(self.x[3, 0], -MAX_VEL, MAX_VEL))

        # 4. Duragan Hedef Hiz Olu Bolgesi (Velocity Deadband)
        speed = float(np.hypot(raw_vx, raw_vy))
        if speed < 30.0:
            deadband_decay = max(0.0, (speed - 8.0) / 22.0)
            raw_vx *= deadband_decay
            raw_vy *= deadband_decay
            self.x[2, 0] = raw_vx
            self.x[3, 0] = raw_vy

        # Hizi ustel yumusat (Exponential Moving Average)
        self.smooth_vx = 0.70 * self.smooth_vx + 0.30 * raw_vx
        self.smooth_vy = 0.70 * self.smooth_vy + 0.30 * raw_vy

        self.last_meas_x = mx
        self.last_meas_y = my

        px = float(self.x[0, 0])
        py = float(self.x[1, 0])
        return px, py, float(self.smooth_vx), float(self.smooth_vy)

    def predict_next_position(self, lead_time=DEFAULT_LEAD_TIME):
        """
        Hedefin lead_time saniye sonrasindaki konumunu hesaplar.
        Dusuk hizlarda mikrotitremeleri onlemek icin yumusak sonumleme uygular.
        """
        px, py = float(self.x[0, 0]), float(self.x[1, 0])
        vx, vy = float(self.smooth_vx), float(self.smooth_vy)
        speed = float(np.hypot(vx, vy))

        attenuation = float(np.clip((speed - 25.0) / 35.0, 0.0, 1.0))
        effective_lead = float(lead_time) * attenuation

        pred_x = px + (vx * effective_lead)
        pred_y = py + (vy * effective_lead)

        return float(pred_x), float(pred_y), float(vx * attenuation), float(vy * attenuation)

    def reset(self):
        """Filtre durumunu sifirlar."""
        self.initialized = False
        self.x = np.zeros((4, 1), dtype=float)
        self.P = np.diag([10.0, 10.0, 100.0, 100.0])
        self.last_residual = 0.0
        self.current_sigma_a = self.sigma_a_min
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0
