import numpy as np


class WebotsPatrolGenerator:
    """Hedef kayboldugunda genis acili 3D Lissajous devriye/tarama paternini uretir."""

    def __init__(self, pan_limits=(-1.57, 1.57), tilt_limits=(-0.52, 0.78)):
        self.PAN_LIMITS = pan_limits
        self.TILT_LIMITS = tilt_limits
        self.search_time = 0.0
        self.is_searching = False

    def update(self, dt, current_pan, current_tilt):
        """Devriye paternini hesaplar ve sarsintisiz motor acilari dondurur."""
        self.is_searching = True
        self.search_time += dt

        # Taktiksel Genisletilmis 3D Lissajous Arama (Pan +/- 68 deg, Tilt -5..+25 deg)
        pattern_pan = 1.18 * np.sin(0.75 * self.search_time)
        pattern_tilt = np.radians(10.0) + np.radians(15.0) * np.sin(1.20 * self.search_time)

        # Puruzsuz yumusak gecis (Sarsintisiz suzulus)
        pan_target = 0.95 * current_pan + 0.05 * pattern_pan
        tilt_target = 0.95 * current_tilt + 0.05 * pattern_tilt

        pan_target = max(self.PAN_LIMITS[0], min(pan_target, self.PAN_LIMITS[1]))
        tilt_target = max(self.TILT_LIMITS[0], min(tilt_target, self.TILT_LIMITS[1]))

        return pan_target, tilt_target

    def reset(self):
        """Arama durumunu ve saatini sifirlar."""
        self.search_time = 0.0
        self.is_searching = False