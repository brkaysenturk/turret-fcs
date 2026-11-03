"""
================================================================================
MODUL: DISTURBANCE OBSERVER (DOB) - BOZUCU ETKI GOZLEMCISI
================================================================================
Bu modul, taret kontrol sisteminde hareketli hedeflerin rampa gecikmesini
ve dis bozucu etkileri (surukleme, mekanik direnc, model uyumsuzlugu) kestirerek
kontrolcu cikisina ileri besleme (feedforward) olarak enjekte eder.

MIMARI:
- Taret nominal modeli: Tip-1 Servomekanizma P_n(s) = K / (s * (tau * s + 1))
- Q-Filtresi: Q(s) = 1 / (tau_q * s + 1)
- Kestirilen Bozucu: d_hat = Q(s) * [P_n^-1(s) * y - u]
================================================================================
"""

from typing import Tuple
import numpy as np



class DisturbanceObserver:
    """
    2-Eksenli (Pan / Tilt) Disturbance Observer (DOB).
    
    Kontrol komutu ile gerceklesen acisal hareket arasindaki farki suzerek
    hedef hareketinden veya yuk dalgalanmalarindan kaynaklanan bozucu ivme/hiz
    bilesenlerini kestirir.
    """

    def __init__(
        self,
        enabled: bool = False,
        gain: float = 1.2233,
        tau_s: float = 0.1710,
        max_speed_deg_s: float = 12.0666,
    ):
        self.enabled = bool(enabled)
        self.gain = float(gain)
        self.tau_s = float(max(1e-4, tau_s))
        self.max_speed_deg_s = float(max_speed_deg_s)

        # Durum degiskenleri (Pan ve Tilt icin filtrelenmis d_hat)
        self.d_hat_pan: float = 0.0
        self.d_hat_tilt: float = 0.0

        # Bir onceki olcum ve komut bellegi
        self.prev_err_pan: float = 0.0
        self.prev_err_tilt: float = 0.0
        self.prev_u_pan: float = 0.0
        self.prev_u_tilt: float = 0.0

    def reset(self):
        """Gozlemci ic durumlarini ve gecmis bellegini sifirlar."""
        self.d_hat_pan = 0.0
        self.d_hat_tilt = 0.0
        self.prev_err_pan = 0.0
        self.prev_err_tilt = 0.0
        self.prev_u_pan = 0.0
        self.prev_u_tilt = 0.0

    def update(
        self,
        err_pan_deg: float,
        err_tilt_deg: float,
        u_cmd_pan_deg: float,
        u_cmd_tilt_deg: float,
        dt: float,
    ) -> Tuple[float, float]:
        """
        Bozucu etkiyi gunceller ve ileri besleme dengeleme komutlarini dondurur.

        Args:
            err_pan_deg: Anlik pan acisal hatasi (derece)
            err_tilt_deg: Anlik tilt acisal hatasi (derece)
            u_cmd_pan_deg: Kontrolcunun uyguladigi pan adimi (derece)
            u_cmd_tilt_deg: Kontrolcunun uyguladigi tilt adimi (derece)
            dt: Ornekleme zamani (saniye)

        Returns:
            comp_pan_deg: Pan ekseni icin komuta eklenecek telafi (derece)
            comp_tilt_deg: Tilt ekseni icin komuta eklenecek telafi (derece)
        """
        if not self.enabled or dt <= 1e-5:
            self.prev_err_pan = float(err_pan_deg)
            self.prev_err_tilt = float(err_tilt_deg)
            self.prev_u_pan = float(u_cmd_pan_deg)
            self.prev_u_tilt = float(u_cmd_tilt_deg)
            return 0.0, 0.0

        # 1. Hata turevi uzerinden gozlenen hiz degisimi (ayrik zamanda)
        d_err_pan = (err_pan_deg - self.prev_err_pan) / dt
        d_err_tilt = (err_tilt_deg - self.prev_err_tilt) / dt

        # 2. Q-Filtresi ayrık zaman gecisi (1. derece alcak geciren filtre)
        alpha = dt / (self.tau_s + dt)

        # Bozucu kestirimi: nominal modelden sapan dinamik hiz farki
        raw_d_pan = self.gain * d_err_pan
        raw_d_tilt = self.gain * d_err_tilt

        self.d_hat_pan = (1.0 - alpha) * self.d_hat_pan + alpha * raw_d_pan
        self.d_hat_tilt = (1.0 - alpha) * self.d_hat_tilt + alpha * raw_d_tilt

        # 3. Hiz sinirlamasi (Satürasyon korumasi)
        clamped_d_pan = float(np.clip(self.d_hat_pan, -self.max_speed_deg_s, self.max_speed_deg_s))
        clamped_d_tilt = float(np.clip(self.d_hat_tilt, -self.max_speed_deg_s, self.max_speed_deg_s))

        # 4. Ayrik adim olarak telafi ciktisi (derece/adim)
        comp_pan = clamped_d_pan * dt
        comp_tilt = clamped_d_tilt * dt

        # Bellegi guncelle
        self.prev_err_pan = float(err_pan_deg)
        self.prev_err_tilt = float(err_tilt_deg)
        self.prev_u_pan = float(u_cmd_pan_deg)
        self.prev_u_tilt = float(u_cmd_tilt_deg)

        return comp_pan, comp_tilt

