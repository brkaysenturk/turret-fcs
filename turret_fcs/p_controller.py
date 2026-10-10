"""
================================================================================
YILDIZ TEKNIK UNIVERSITESI - KONTROL VE OTOMASYON MUHENDISLIGI BOLUMU
BITIRME PROJESI: OTONOM PAN-TILT TARET VE ATIS KONTROL SISTEMI (FCS)
--------------------------------------------------------------------------------
MODUL: HIZ-UYARLAMALI YUMUSAK OLU BOLGELI P KONTROLCU (P CONTROLLER)
================================================================================
Bu modul, piksellerden acisal hata donusumunu ve taret motorlarinin gercek zamanli
servomekanizma kontrol komutlarini uretir.

KONTROL TEORISI VE MATEMATIKSEL MIMARI:
1. Hiz-Uyarlamali Yumusak Olu Bolge (Velocity-Adaptive Soft Deadband):
   - Sabit Hedef (v < 25 px/s): Pan Kp=0.10, Deadband=0.90° | Tilt Kp=0.07, Deadband=0.95°
     YOLO piksel gurultusu motorlara iletilmez, taret sessiz kilitte kalir.
   - Hareketli Hedef (v >= 25 px/s): Pan Kp=0.16, Deadband=0.20° | Tilt Kp=0.12, Deadband=0.25°
     Klasik Tip-0 rampa gecikmesi (Lag error) dinamik kazanc takviyesiyle sifirlanir.
2. Neden Kd = 0 ve Ki = 0?
   - Kd: 30 FPS goruntu kuantalama gurultusunu turev patlamasina donusturerek mekanik
     rezonans ve akustik vizilti urettigi icin devreden cikarilmistir.
   - Ki: STM32 acisal artis biriktirme denklemi theta(k) = theta(k-1) + delta_theta
     zaten dogal bir integrator gorevi gordugunden (Tip-1 sistem), harici Ki cift
     integrator faz cokmesine yol acacagi icin kullanilmamistir.
3. Eyleyici Doyum ve Egim Limiti (Slew Rate Limiter):
   - Mekanik torku ve disli kutusunu korumak icin kare basi maks komut: 2.75°/kare (82.5°/s).
================================================================================
"""

import numpy as np


class PController:
    """
    Kamera goruntusunden hedefin acisal sapmasini, balistik yercekimi dususunu (super-elevation),
    ve hedefin hareket hizina gore dinamik uyarlanan yumusak olu bolgeli oransal 
    (Velocity-Adaptive Soft Deadband P) motor komutlarini hesaplar.
    
    Kontrol Mimarisi Mantigi:
      1. Ayrik Eksen Dinamigi: Pan (yatay) ve Tilt (dikey) eksenleri bagimsiz hesaplanir.
         Tilt ekseni kamera ve sensor kutlesini yercekimi kuvvetine karsi bir kaldirac gibi 
         tasidigi icin, dikey sekme rezonansini (bobbing) onlemek adina daha yumusak kazanc 
         (kp=0.07) ve daha genis olu bolge (0.95 deg) kullanilir.
      2. Hiza Duyarli Dinamik Olu Bolge (Velocity-Adaptive Deadband):
         - Sabit Hedef (|v| < 25 px/s): Genis olu bolge (0.90 / 0.95 deg) aktif olur; 
           YOLO'nun dogal piksel titresimi servolara iletilmez, motorlar sessiz ve tam kilitte kalir.
         - Hareketli Hedef (|v| >= 25 px/s): Olu bolge otomatik olarak 0.20 / 0.25 deg seviyesine daraltilir
           ve oransal kazanc dinamik olarak artirilarak hedefin arkasinda kalma (lag hatasi) engellenir.
      3. Yumusak Gecisli Olu Bolge (Soft Deadband): Esik asildiginda motor komutu aniden yuklenmez,
         hata ile olu bolge arasindaki fark oraniyla kademeli baslar; bu da mekanik vuruntulari onler.
    """

    DEFAULT_DISTANCE_M = 2.0
    PROJECTILE_SPEED_MPS = 10.0  # Mermi namlu cikis hizi (m/s)
    GRAVITY_ACCEL = 9.81         # Yercekimi ivmesi (m/s^2)
    MAX_OUTPUT_DEG = 2.75        # Her dongude gonderilebilecek maksimum adim acisi (Derece)

    def __init__(
        self,
        kp_pan=0.10,
        kp_tilt=0.07,
        alpha=0.75,
        max_output=MAX_OUTPUT_DEG,
        static_deadband_pan=0.90,
        static_deadband_tilt=0.95,
        moving_deadband_pan=0.20,
        moving_deadband_tilt=0.25,
    ):
        self.kp_pan_base = kp_pan
        self.kp_tilt_base = kp_tilt
        self.alpha = alpha
        self.max_output = max_output

        # Sabit ve hareketli durum olu bolge parametreleri
        self.static_deadband_pan = static_deadband_pan
        self.static_deadband_tilt = static_deadband_tilt
        self.moving_deadband_pan = moving_deadband_pan
        self.moving_deadband_tilt = moving_deadband_tilt

        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0

    def compute_angles(self, bx, by, center_x, center_y, fx, fy):
        """
        Kamera optik merkezinden tespit edilen hedefin yatay ve dikey gorus acisini (derece) hesaplar.
        Dusuk dereceli alçak geciren filtre (EMA) ile ani piksel ziplamalarini suzer.
        """
        target_err_x = bx - center_x
        target_err_y = by - center_y
        raw_pan = np.degrees(np.arctan(target_err_x / max(fx, 1e-6)))
        raw_tilt = np.degrees(np.arctan(-target_err_y / max(fy, 1e-6)))

        self.actual_pan = self.alpha * raw_pan + (1.0 - self.alpha) * self.actual_pan
        self.actual_tilt = self.alpha * raw_tilt + (1.0 - self.alpha) * self.actual_tilt
        return self.actual_pan, self.actual_tilt

    def compute_gravity_comp(self, dist_cm):
        """
        Balistik yercekimi dususu telafisi (Super-Elevation):
        Merminin hedefe varis suresi boyunca yercekiminden oturu asagi dusus acisini ekler.
        Formul: theta = arctan((g * D) / (2 * v^2))
        """
        target_d_m = (dist_cm / 100.0) if (dist_cm is not None and dist_cm > 20) else self.DEFAULT_DISTANCE_M
        v_sq_2 = 2.0 * (self.PROJECTILE_SPEED_MPS ** 2)
        return float(np.degrees(np.arctan((self.GRAVITY_ACCEL * target_d_m) / v_sq_2)))

    def compute_lead_angles(self, d_lead_x, d_lead_y, fx, fy):
        """
        Merminin ucus suresi boyunca hedefin kat edecegi mesafeye gore atis onleme acisini hesaplar.
        """
        lead_pan = np.degrees(np.arctan(d_lead_x / max(fx, 1e-6)))
        lead_tilt = np.degrees(np.arctan(-d_lead_y / max(fy, 1e-6)))
        return float(lead_pan), float(lead_tilt)

    def compute_p(self, active_pan, active_tilt, dt, target_speed=0.0):
        """
        Hedefin hareket hizina gore dinamik olu bolge ve ayrik eksenli oransal (P) kontrol komutunu hesaplar.
        
        Parametreler:
            active_pan: Yatay eksendeki toplam acisal hata (derece)
            active_tilt: Dikey eksendeki toplam acisal hata (derece)
            dt: Gecen dongu suresi (saniye)
            target_speed: Kalman filtresinden gelen puruzsuz hedef hizi (piksel/saniye)
        """
        self.filtered_pan = self.alpha * active_pan + (1.0 - self.alpha) * self.filtered_pan
        self.filtered_tilt = self.alpha * active_tilt + (1.0 - self.alpha) * self.filtered_tilt

        # 1. Hedefin hareket durumuna gore adaptasyon orani (0.0: Sabit Durum, 1.0: Hizli Hareket)
        spd = float(target_speed)
        motion_ratio = float(np.clip((spd - 25.0) / 45.0, 0.0, 1.0))

        # Hiza bagli dinamik olu bolge genisligi (Hedef hizlandikca olu bolge daralir)
        deadband_pan = (1.0 - motion_ratio) * self.static_deadband_pan + motion_ratio * self.moving_deadband_pan
        deadband_tilt = (1.0 - motion_ratio) * self.static_deadband_tilt + motion_ratio * self.moving_deadband_tilt

        # Hiza bagli dinamik oransal kazanc (Hedef hizlandikca arkada kalmamak icin kazanc artar)
        kp_pan = (1.0 - motion_ratio) * self.kp_pan_base + motion_ratio * 0.16
        kp_tilt = (1.0 - motion_ratio) * self.kp_tilt_base + motion_ratio * 0.12

        # 2. Pan Ekseni Komutu (Yatay Yumusak Olu Bolge)
        abs_pan = abs(self.filtered_pan)
        if abs_pan <= deadband_pan:
            u_pan = 0.0
        else:
            eff_pan = np.sign(self.filtered_pan) * (abs_pan - deadband_pan)
            u_pan = kp_pan * eff_pan

        # 3. Tilt Ekseni Komutu (Dikey Yumusak Olu Bolge)
        abs_tilt = abs(self.filtered_tilt)
        if abs_tilt <= deadband_tilt:
            u_tilt = 0.0
        else:
            eff_tilt = np.sign(self.filtered_tilt) * (abs_tilt - deadband_tilt)
            u_tilt = kp_tilt * eff_tilt

        # 4. Mekanik Aci Sinirlandirmasi (Servolari ani asiri yuklenmelerden korur)
        u_pan = float(np.clip(u_pan, -self.max_output, self.max_output))
        u_tilt = float(np.clip(u_tilt, -self.max_output, self.max_output))

        return u_pan, u_tilt

    def reset(self):
        """Kontrolcunun ic filtre ve durum degerlerini sifirlar."""
        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0
