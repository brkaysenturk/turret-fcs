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
        r_var=4.0,           # Olcum gurultusu varyansi (pixel^2)
        sigma_a_min=20.0,    # Kararli/yavas hedef ivme gurultusu (px/s^2)
        sigma_a_max=450.0,   # Hizli manevra/yon degisimi ivme gurultusu (px/s^2)
        residual_low=4.0,    # Alt inovasyon esigi (pixel)
        residual_high=22.0,  # Ust inovasyon esigi (pixel)
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

    def update(self, measurement, dt=0.033):
        """
        Adim 1: Durum Kestirimi (Predict)
        Adim 2: Inovasyon (Residual) hesaplayarak sigma_a ve CWNA Q matrisini dinamik uyarlama
        Adim 3: Kovaryans kestirimi ve Joseph formunda Kalman guncellemesi (Update)
        
        Returns:
            px, py, vx, vy: Kestirilen konum ve hiz degerleri
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

        # On kestirim (Durum kestirimi Q'dan bagimsizdir: x_pred = F * x)
        x_pred = F @ self.x

        # 2. Inovasyon (Residual): Tahmin ile olcum arasindaki piksel farki
        y = meas - self.H @ x_pred
        residual = float(np.hypot(y[0, 0], y[1, 0]))
        self.last_residual = residual

        # Dinamik Adaptasyon:
        # Residual kucukse (hedef duzenli gidiyor/duruyor) -> sigma_a dusuk (servolarda sifir titreme)
        # Residual buyukse (hedef manevra yapti/ani dondu) -> sigma_a yuksek (aninda yapis, sifir lag)
        alpha = float(np.clip((residual - self.residual_low) / (self.residual_high - self.residual_low), 0.0, 1.0))
        sigma_a = self.sigma_a_min + alpha * (self.sigma_a_max - self.sigma_a_min)
        self.current_sigma_a = sigma_a
        sigma_a2 = sigma_a * sigma_a

        # CWNA (Continuous White Noise Acceleration) Surec Gurultusu Matrisi Q
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt

        Q = np.array([
            [dt4 / 4.0, 0.0,       dt3 / 2.0, 0.0      ],
            [0.0,       dt4 / 4.0, 0.0,       dt3 / 2.0],
            [dt3 / 2.0, 0.0,       dt2,       0.0      ],
            [0.0,       dt3 / 2.0, 0.0,       dt2      ],
        ], dtype=float) * sigma_a2

        # 3. Kovaryans Kestirimi
        P_pred = F @ self.P @ F.T + Q

        # 4. Kalman Kazanci K
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)

        # Durum Guncellemesi
        self.x = x_pred + K @ y

        # Numerik Kararlilik: Joseph Form Kovaryans Guncellemesi
        # P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
        I_KH = np.eye(4, dtype=float) - K @ self.H
        self.P = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T

        # Hiz sinirlamasi (asiri piksel ziplamalarini onleme)
        MAX_VEL = 900.0  # px/s
        self.x[2, 0] = float(np.clip(self.x[2, 0], -MAX_VEL, MAX_VEL))
        self.x[3, 0] = float(np.clip(self.x[3, 0], -MAX_VEL, MAX_VEL))

        self.last_meas_x = mx
        self.last_meas_y = my

        px, py, vx, vy = self.x.ravel()
        return float(px), float(py), float(vx), float(vy)

    def predict_next_position(self, lead_time=DEFAULT_LEAD_TIME):
        """
        Hedefin lead_time saniye sonrasindaki konumunu hesaplar.
        Dusuk hizlarda mikrotitremeleri onlemek icin yumusak sonumleme (attenuation) uygular.
        """
        px, py, vx, vy = self.x.ravel()
        speed = float(np.hypot(vx, vy))

        # 3.0 px/s altinda sifir onleme, 10.0 px/s ustunde tam onleme
        attenuation = float(np.clip((speed - 3.0) / 7.0, 0.0, 1.0))
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
