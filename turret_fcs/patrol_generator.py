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
        pan_amplitude=30.0,     # Pan tarama genligi (+/- 30 derece)
        tilt_amplitude=8.0,     # Tilt tarama genligi (+/- 8 derece)
        tilt_center=0.0,        # Ufuk hizzasi merkezli (derece)
        pan_period=3.5,         # 1 tam yatay tarama suresi (3.5 saniye)
        tilt_period=2.8,        # 1 tam dikey tarama suresi (2.8 saniye)
        pan_limits=(-40.0, 40.0),
        tilt_limits=(-15.0, 20.0),
    ):
        self.pan_amplitude = float(pan_amplitude)
        self.tilt_amplitude = float(tilt_amplitude)
        self.tilt_center = float(tilt_center)
        self.pan_freq = float(2.0 * np.pi / pan_period)    # ~1.795 rad/s
        self.tilt_freq = float(2.0 * np.pi / tilt_period)  # ~2.244 rad/s
        self.pan_limits = pan_limits
        self.tilt_limits = tilt_limits

        self.search_time = 0.0
        self.is_searching = False
        self.last_target_pan = 0.0
        self.last_target_tilt = 0.0

    def start(self):
        """Arama modunu baslatir."""
        self.is_searching = True

    def stop(self):
        """Arama modunu durdurur."""
        self.is_searching = False

    def update(self, dt):
        """
        3.5 saniyelik Lissajous tarama desenini hesaplar.
        STM32'nin bekledigi acisal adim (delta) komutlarini dondurur.
        """
        self.is_searching = True
        dt_safe = max(0.001, min(float(dt), 0.1))
        self.search_time += dt_safe

        # Harmonik Lissajous Arama Deseni (3.5 saniyede tam tur)
        raw_pan = self.pan_amplitude * np.sin(self.pan_freq * self.search_time)
        raw_tilt = self.tilt_center + self.tilt_amplitude * np.sin(self.tilt_freq * self.search_time)

        # Fiziksel Guvenlik Limitlerine Kirpma
        target_pan = float(np.clip(raw_pan, self.pan_limits[0], self.pan_limits[1]))
        target_tilt = float(np.clip(raw_tilt, self.tilt_limits[0], self.tilt_limits[1]))

        # STM32 kapali cevrimine incremental (delta) aci komutu
        d_pan = float(target_pan - self.last_target_pan)
        d_tilt = float(target_tilt - self.last_target_tilt)

        self.last_target_pan = target_pan
        self.last_target_tilt = target_tilt

        # Hareketi yumusak tutmak icin tek karelik emniyet limiti
        d_pan = float(np.clip(d_pan, -2.5, 2.5))
        d_tilt = float(np.clip(d_tilt, -2.5, 2.5))

        return d_pan, d_tilt

    def reset(self):
        """Arama durumunu ve zamanlayiciyi sifirlar."""
        self.search_time = 0.0
        self.is_searching = False
        self.last_target_pan = 0.0
        self.last_target_tilt = 0.0


if __name__ == "__main__":
    import time

    print("[*] PatrolGenerator Test Baslatiliyor...")
    patroller = PatrolGenerator()
    patroller.start()

    pan, tilt = 0.0, 0.0
    dt = 0.033  # 30 FPS simulasyonu

    for step in range(30):
        d_p, d_t = patroller.update(dt)
        pan += d_p
        tilt += d_t
        print(f"Adim {step+1:02d} | Delta Pan: {d_p:+.2f}° (Toplam: {pan:+.2f}°) | Delta Tilt: {d_t:+.2f}° (Toplam: {tilt:+.2f}°)")
        time.sleep(0.02)

    print("[OK] PatrolGenerator testi basariyla tamamlandi.")
