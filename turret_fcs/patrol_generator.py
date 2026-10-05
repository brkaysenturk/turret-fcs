import numpy as np


class PatrolGenerator:
    """
    Hedef kayboldugunda (veya arama moduna gecildiginde) taretin kadrajini
    cevreye taratarak hedefi yeniden yakalamasini saglayan parametrik Lissajous
    tarama yorungesi ureteci.

    NOT: Ilk sabit taret testlerinde devre disi birakilmistir. Ilerleyen asamalarda
    fcs_manager veya main.py icerisinden cagrilabilir.
    """

    def __init__(
        self,
        pan_amplitude=35.0,     # Pan tarama genligi (+/- 35 derece)
        tilt_amplitude=12.0,    # Tilt tarama genligi (+/- 12 derece)
        tilt_center=10.0,       # Tilt ufuk acisi (+10 derece yukari egim)
        pan_freq=0.75,          # Pan tarama frekansi (rad/s)
        tilt_freq=1.20,         # Tilt tarama frekansi (rad/s)
        pan_limits=(-45.0, 45.0),
        tilt_limits=(-15.0, 30.0),
        smoothing_alpha=0.06    # Motor disli sagligi icin yumusak suzulus katsayisi
    ):
        self.pan_amplitude = float(pan_amplitude)
        self.tilt_amplitude = float(tilt_amplitude)
        self.tilt_center = float(tilt_center)
        self.pan_freq = float(pan_freq)
        self.tilt_freq = float(tilt_freq)
        self.pan_limits = pan_limits
        self.tilt_limits = tilt_limits
        self.smoothing_alpha = float(smoothing_alpha)

        self.search_time = 0.0
        self.is_searching = False
        self.current_pan_cmd = 0.0
        self.current_tilt_cmd = 0.0

    def start(self):
        """Arama modunu baslatir."""
        self.is_searching = True

    def stop(self):
        """Arama modunu durdurur."""
        self.is_searching = False

    def update(self, dt, current_pan=0.0, current_tilt=0.0):
        """
        Zamana bagli Lissajous tarama paternini hesaplar ve yumusatilmis
        motor hedef acilarini (derece) dondurur.
        """
        self.is_searching = True
        dt_safe = max(0.001, min(float(dt), 0.1))
        self.search_time += dt_safe

        # Harmonik Lissajous Arama Deseni (Kor nokta birakmayan 3D tarama)
        raw_pan = self.pan_amplitude * np.sin(self.pan_freq * self.search_time)
        raw_tilt = self.tilt_center + self.tilt_amplitude * np.sin(self.tilt_freq * self.search_time)

        # Fiziksel Limitlere Kirpma
        target_pan = float(np.clip(raw_pan, self.pan_limits[0], self.pan_limits[1]))
        target_tilt = float(np.clip(raw_tilt, self.tilt_limits[0], self.tilt_limits[1]))

        # Sarsintisiz Yumusak Surus (Exponential Smoothing)
        self.current_pan_cmd = (
            (1.0 - self.smoothing_alpha) * current_pan + self.smoothing_alpha * target_pan
        )
        self.current_tilt_cmd = (
            (1.0 - self.smoothing_alpha) * current_tilt + self.smoothing_alpha * target_tilt
        )

        return self.current_pan_cmd, self.current_tilt_cmd

    def reset(self):
        """Arama durumunu ve zamanlayiciyi sifirlar."""
        self.search_time = 0.0
        self.is_searching = False
        self.current_pan_cmd = 0.0
        self.current_tilt_cmd = 0.0


if __name__ == "__main__":
    import time

    print("[*] PatrolGenerator Test Baslatiliyor...")
    patroller = PatrolGenerator()
    patroller.start()

    pan, tilt = 0.0, 0.0
    dt = 0.033  # 30 FPS simulasyonu

    for step in range(30):
        pan, tilt = patroller.update(dt, pan, tilt)
        print(f"Adim {step+1:02d} | Pan: {pan:+.2f}° | Tilt: {tilt:+.2f}°")
        time.sleep(0.02)

    print("[OK] PatrolGenerator testi basariyla tamamlandi.")
