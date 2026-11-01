"""Sensorsuz servo yer degistirme, disli boslugu (backlash) ve ego-hareket belirsizlik modeli."""

import numpy as np

from .config import CONFIG


class ServoMotionEstimator:
    """Pan/tilt eksenlerinde 1. derece servo gecikmesi, yon donus boslugu (slack) ve olu bolge modeli."""

    def __init__(self):
        self.reset()

    def reset(self, pan=0.0, tilt=0.0):
        self.target = np.array([float(pan), float(tilt)], dtype=float)
        self.drive = self.target.copy()
        self.position = self.target.copy()
        self.reported = self.target.copy()
        self.direction = np.zeros(2, dtype=float)
        self.slack = np.zeros(2, dtype=float)

    def command(self, pan_step, tilt_step):
        """Uygulanan motor komut adimini hedef aciya ekler."""
        self.target += (float(pan_step), float(tilt_step))
        self.target = np.clip(
            self.target,
            [CONFIG.mount.pan_limit_min, CONFIG.mount.tilt_limit_min],
            [CONFIG.mount.pan_limit_max, CONFIG.mount.tilt_limit_max],
        )

    def advance(self, dt, fx, fy, tau=None):
        """dt suresi boyunca servo hareketini, disli boslugunu ve piksel varyansini ilerletir."""
        dt = max(1e-4, float(dt))
        tau = CONFIG.control.ego_tau_s if tau is None else max(1e-6, float(tau))
        delta = (self.target - self.drive) * (-np.expm1(-dt / tau))
        self.drive += delta

        for axis in range(2):
            if abs(delta[axis]) < 1e-12:
                continue
            direction = float(np.sign(delta[axis]))
            if self.direction[axis] != 0.0 and direction != self.direction[axis]:
                self.slack[axis] = CONFIG.actuator.backlash_deg
            self.direction[axis] = direction
            takeup = min(abs(delta[axis]), self.slack[axis])
            self.slack[axis] -= takeup
            self.position[axis] += direction * (abs(delta[axis]) - takeup)

        step = self.position - self.reported
        step = np.where(abs(step) >= CONFIG.actuator.servo_deadband_deg, step, 0.0)
        self.reported += step

        focal = np.array([float(fx), float(fy)], dtype=float)
        shift = focal * np.tan(np.radians(step)) * (-1.0, 1.0)

        # Hareket ve disli boslugu alimi sirasinda artan yer degistirme belirsizligi
        # konum kovaryansi olarak Kalman'a enjekte edilir; boylece sahte ivme sanilmaz.
        active = abs(delta) > 1e-8
        std_deg = CONFIG.control.ego_motion_uncertainty_ratio * abs(delta)
        std_deg += active * CONFIG.actuator.servo_deadband_deg
        std_deg += active * CONFIG.control.ego_backlash_uncertainty_ratio * self.slack
        variance = (focal * np.radians(std_deg)) ** 2
        return step, shift, variance

