import numpy as np


class WebotsKalmanFilter:
    """
    2D hedef takip icin Surekli Beyaz Gurultulu Ivme (CWNA) ve 
    Inovasyon Tabanli Adaptif Surec Gurultusu (Adaptive Q) mimarili Webots Kalman Filtresi.
    
    Durum vektoru: X = [x, y, vx, vy]^T
    Olcum vektoru: Z = [x, y]^T
    """

    DEFAULT_LEAD_TIME = 0.18  # saniye

    def __init__(
        self,
        dt=0.016,
        r_var=4.0,           # Olcum gurultusu varyansi (pixel^2)
        sigma_a_min=20.0,    # Kararli hedef ivme gurultusu (px/s^2)
        sigma_a_max=450.0,   # Hizli manevra ivme gurultusu (px/s^2)
        residual_low=3.0,    # Alt inovasyon esigi (pixel)
        residual_high=20.0,  # Ust inovasyon esigi (pixel)
        jump_threshold=60.0  # Hedef atlamasi esigi (pixel)
    ):
        self.dt = float(dt)
        self.r_var = float(r_var)
        self.sigma_a_min = float(sigma_a_min)
        self.sigma_a_max = float(sigma_a_max)
        self.residual_low = float(residual_low)
        self.residual_high = float(residual_high)
        self.jump_threshold = float(jump_threshold)

        self.x = np.zeros((4, 1), dtype=float)
        self.P = np.diag([10.0, 10.0, 100.0, 100.0])
        self.initialized = False

        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ], dtype=float)

        self.R = np.eye(2, dtype=float) * self.r_var

        self.last_meas_x = 0.0
        self.last_meas_y = 0.0
        self.last_residual = 0.0
        self.current_sigma_a = self.sigma_a_min

    def update(self, measurement, dt=0.016):
        """Once predict, inovasyon bazli adaptif Q hesabi, sonra Joseph-form update yapar."""
        dt = max(0.005, min(float(dt), 0.1))
        mx, my = float(measurement[0]), float(measurement[1])
        meas = np.array([[mx], [my]], dtype=float)

        if not self.initialized:
            self.x = np.array([[mx], [my], [0.0], [0.0]], dtype=float)
            self.P = np.diag([10.0, 10.0, 100.0, 100.0])
            self.last_meas_x = mx
            self.last_meas_y = my
            self.initialized = True
            return mx, my, 0.0, 0.0

        meas_jump = float(np.hypot(mx - self.last_meas_x, my - self.last_meas_y))
        if meas_jump > self.jump_threshold:
            self.x = np.array([[mx], [my], [0.0], [0.0]], dtype=float)
            self.P = np.diag([10.0, 10.0, 60.0, 60.0])
            self.last_meas_x = mx
            self.last_meas_y = my
            return mx, my, 0.0, 0.0

        # 1. State Prediction
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=float)

        x_pred = F @ self.x

        # 2. Inovasyon (Residual) ve Dinamik Q
        y = meas - self.H @ x_pred
        residual = float(np.hypot(y[0, 0], y[1, 0]))
        self.last_residual = residual

        alpha = float(np.clip((residual - self.residual_low) / (self.residual_high - self.residual_low), 0.0, 1.0))
        sigma_a = self.sigma_a_min + alpha * (self.sigma_a_max - self.sigma_a_min)
        self.current_sigma_a = sigma_a
        sigma_a2 = sigma_a * sigma_a

        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt

        Q = np.array([
            [dt4 / 4.0, 0.0,       dt3 / 2.0, 0.0      ],
            [0.0,       dt4 / 4.0, 0.0,       dt3 / 2.0],
            [dt3 / 2.0, 0.0,       dt2,       0.0      ],
            [0.0,       dt3 / 2.0, 0.0,       dt2      ],
        ], dtype=float) * sigma_a2

        # 3. Covariance Prediction
        P_pred = F @ self.P @ F.T + Q

        # 4. Update
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)

        self.x = x_pred + K @ y

        # Joseph Form Covariance Update
        I_KH = np.eye(4, dtype=float) - K @ self.H
        self.P = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T

        MAX_VEL = 600.0
        self.x[2, 0] = float(np.clip(self.x[2, 0], -MAX_VEL, MAX_VEL))
        self.x[3, 0] = float(np.clip(self.x[3, 0], -MAX_VEL, MAX_VEL))

        self.last_meas_x = mx
        self.last_meas_y = my

        px, py, vx, vy = self.x.ravel()
        return float(px), float(py), float(vx), float(vy)

    def predict_next_position(self, lead_time=DEFAULT_LEAD_TIME):
        """Hedefin hareket yonunde lead_time sonrasindaki konumunu tahmin eder."""
        px, py, vx, vy = self.x.ravel()
        speed = float(np.hypot(vx, vy))

        attenuation = float(np.clip((speed - 3.0) / 7.0, 0.0, 1.0))
        effective_lead = float(lead_time) * attenuation

        pred_x = px + (vx * effective_lead)
        pred_y = py + (vy * effective_lead)

        return float(pred_x), float(pred_y), float(vx * attenuation), float(vy * attenuation)

    def reset(self):
        self.initialized = False
        self.x = np.zeros((4, 1), dtype=float)
        self.P = np.diag([10.0, 10.0, 100.0, 100.0])
        self.last_residual = 0.0
        self.current_sigma_a = self.sigma_a_min
