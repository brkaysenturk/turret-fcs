"""
================================================================================
YILDIZ TEKNIK UNIVERSITESI - KONTROL VE OTOMASYON MUHENDISLIGI BOLUMU
BITIRME PROJESI: OTONOM PAN-TILT TARET VE ATIS KONTROL SISTEMI (FCS)
--------------------------------------------------------------------------------
MODUL: IKI ASAMALI AKILLI DEVIYE VE HEDEF ARAMA URETECI (PATROL GENERATOR)
================================================================================
Bu modul, otonom taretin hedef arama doktrinini iki farkli senaryoya gore yonetir:

1. ILK ACILIS / GENEL ARAMA (COLD START & GLOBAL SECTOR PATROL):
   - Sistem ilk calistirildiginda veya hedef cok uzun sure boyunca bulunamadiginda.
   - Taret merkez (0°, 0°) referansli genis bir sahada (+/- 28° Pan, +/- 7° Tilt)
     yumusak harmonik Lissajous taramasi yapar.

2. HEDEF KAYBEDILDIGINDE YEREL YENIDEN YAKALAMA (LOCAL RE-ACQUISITION):
   - Takip edilmekte olan bir hedef aniden kadrajdan ciktiginda (hizli kacis veya
     engelin arkasina gecis).
   - Asama 2A (0.0s - 1.2s) [GRACE PERIOD]: Taret son bilinen acida sessizce bekler.
     Gereksiz ani motor savrulmalarini onler.
   - Asama 2B (1.2s - 5.0s) [LOCAL SEARCH]: Hedefin en son goruldugu acinin
     hemen cevresinde dar/hizli zikzak arama yapar (+/- 12° Pan, +/- 4° Tilt).
   - Asama 2C (> 5.0s) [GLOBAL TIMEOUT]: Yerel aramada bulunamazsa taret yumusakca
     merkeze donerek tekrar genel sektor taramasina (Global Patrol) gecer.

3. HEDEF BULUNDUGUNDA:
   - Devriye aninda durdurulur ve kontrol tamamen takip algoritmalarina gecer.
================================================================================
"""

import numpy as np


class PatrolGenerator:
    def __init__(
        self,
        # Genel Sektor Arama Parametreleri
        global_pan_amp=28.0,      # Genis yatay tarama genligi (+/- 28 derece)
        global_tilt_amp=7.0,      # Genis dikey tarama genligi (+/- 7 derece)
        global_pan_period=4.2,    # 1 tam yatay periyot (4.2 saniye - yumusak)
        global_tilt_period=3.1,   # 1 tam dikey periyot (3.1 saniye - Lissajous)
        # Yerel Yeniden Yakalama Parametreleri
        local_pan_amp=12.0,       # Hedefin son konumunda dar yatay tarama (+/- 12 derece)
        local_tilt_amp=4.0,       # Hedefin son konumunda dar dikey tarama (+/- 4 derece)
        local_pan_period=2.2,     # Hizli yerel zikzak (2.2 saniye)
        local_tilt_period=1.6,    # Hizli yerel dikey salinim (1.6 saniye)
        grace_period_sec=1.2,     # Kaybolduktan sonra son konumda bekleme suresi (saniye)
        local_timeout_sec=5.0,    # Yerel arama zaman asimi (5 saniye sonra genele donus)
        # Mekanik Guvenlik Limitleri
        pan_limits=(-42.0, 42.0),
        tilt_limits=(-14.0, 18.0),
        max_slew_per_frame=1.25,  # Devriye sirasinda kare basi maks acisal adim (deg/frame)
    ):
        self.global_pan_amp = float(global_pan_amp)
        self.global_tilt_amp = float(global_tilt_amp)
        self.global_pan_freq = float(2.0 * np.pi / global_pan_period)
        self.global_tilt_freq = float(2.0 * np.pi / global_tilt_period)

        self.local_pan_amp = float(local_pan_amp)
        self.local_tilt_amp = float(local_tilt_amp)
        self.local_pan_freq = float(2.0 * np.pi / local_pan_period)
        self.local_tilt_freq = float(2.0 * np.pi / local_tilt_period)

        self.grace_period_sec = float(grace_period_sec)
        self.local_timeout_sec = float(local_timeout_sec)

        self.pan_limits = pan_limits
        self.tilt_limits = tilt_limits
        self.max_slew = float(max_slew_per_frame)

        # Durum Degiskenleri
        self.has_seen_target = False
        self.last_target_pan = 0.0
        self.last_target_tilt = 0.0
        self.lost_time = 0.0
        self.global_time = 0.0
        self.is_active = False
        self.current_cmd_pan = 0.0
        self.current_cmd_tilt = 0.0

    def on_target_detected(self, actual_pan, actual_tilt):
        """
        Hedef kadraja girdiginde cagirilir. Devriyeyi durdurur ve son konumu kaydeder.
        """
        self.has_seen_target = True
        self.last_target_pan = float(actual_pan)
        self.last_target_tilt = float(actual_tilt)
        self.lost_time = 0.0
        self.is_active = False
        self.current_cmd_pan = self.last_target_pan
        self.current_cmd_tilt = self.last_target_tilt

    def update(self, dt, actual_pan=None, actual_tilt=None):
        """
        Hedef gorunmediginde cagirilir. Duruma uygun arama acisi komutlarini dondurur.
        
        Donus Degeri:
            (d_pan, d_tilt, status_str)
        """
        self.is_active = True
        dt_safe = max(0.005, min(float(dt), 0.1))

        # Referans acilar
        ref_pan = actual_pan if actual_pan is not None else self.current_cmd_pan
        ref_tilt = actual_tilt if actual_tilt is not None else self.current_cmd_tilt

        # ---------------------------------------------------------------------
        # SENARYO 1: SISTEM ILK ACILISI (HENUZ HIC HEDEF GORULMEDI)
        # ---------------------------------------------------------------------
        if not self.has_seen_target:
            self.global_time += dt_safe
            target_pan = self.global_pan_amp * np.sin(self.global_pan_freq * self.global_time)
            target_tilt = self.global_tilt_amp * np.sin(self.global_tilt_freq * self.global_time)
            status_str = "GLOBAL PATROL (±28°)"

        # ---------------------------------------------------------------------
        # SENARYO 2: TAKIP EDILEN HEDEF ANIDEN KAYBOLDU
        # ---------------------------------------------------------------------
        else:
            self.lost_time += dt_safe

            # 2A. Grace Period (0.0s - 1.2s): Son acida sabit bekle (sessiz motor)
            if self.lost_time <= self.grace_period_sec:
                target_pan = self.last_target_pan
                target_tilt = self.last_target_tilt
                status_str = f"STANDBY GRACE ({self.lost_time:.1f}s)"

            # 2B. Yerel Yeniden Yakalama (1.2s - 5.0s): Son konum etrafinda dar tarama
            elif self.lost_time <= self.local_timeout_sec:
                t_local = self.lost_time - self.grace_period_sec
                d_p_local = self.local_pan_amp * np.sin(self.local_pan_freq * t_local)
                d_t_local = self.local_tilt_amp * np.sin(self.local_tilt_freq * t_local)
                target_pan = self.last_target_pan + d_p_local
                target_tilt = self.last_target_tilt + d_t_local
                status_str = f"LOCAL RE-ACQ (±12°)"

            # 2C. Zaman Asimi (> 5.0s): Hedef sahada yok, genis genel taramaya gec
            else:
                self.global_time += dt_safe
                target_pan = self.global_pan_amp * np.sin(self.global_pan_freq * self.global_time)
                target_tilt = self.global_tilt_amp * np.sin(self.global_tilt_freq * self.global_time)
                status_str = "GLOBAL TIMEOUT (±28°)"

        # Mekanik limitlere kirpma
        target_pan = float(np.clip(target_pan, self.pan_limits[0], self.pan_limits[1]))
        target_tilt = float(np.clip(target_tilt, self.tilt_limits[0], self.tilt_limits[1]))

        # STM32 incremental adim komutu (slew limiting ile yumusak hareket)
        err_pan = target_pan - ref_pan
        err_tilt = target_tilt - ref_tilt

        # Tek karelik guvenli artis
        d_pan = float(np.clip(err_pan, -self.max_slew, self.max_slew))
        d_tilt = float(np.clip(err_tilt, -self.max_slew, self.max_slew))

        self.current_cmd_pan = ref_pan + d_pan
        self.current_cmd_tilt = ref_tilt + d_tilt

        return d_pan, d_tilt, status_str

    def reset(self):
        """Devriye durumunu tamamen sifirlar."""
        self.has_seen_target = False
        self.last_target_pan = 0.0
        self.last_target_tilt = 0.0
        self.lost_time = 0.0
        self.global_time = 0.0
        self.is_active = False
        self.current_cmd_pan = 0.0
        self.current_cmd_tilt = 0.0


if __name__ == "__main__":
    import time

    print("==================================================================")
    print("   PATROL GENERATOR // IKI ASAMALI AKILLI DEVIYE SIMULASYONU     ")
    print("==================================================================")

    patrol = PatrolGenerator()
    dt = 0.033  # 30 FPS

    print("\n--- 1. Asama: Sistem Ilk Acilisi (Cold Start / Global Sektor) ---")
    pos_p, pos_t = 0.0, 0.0
    for i in range(15):
        dp, dt_cmd, status = patrol.update(dt, pos_p, pos_t)
        pos_p += dp
        pos_t += dt_cmd
        if i % 3 == 0:
            print(f"Step {i:02d} | Pan: {pos_p:+5.1f}° | Tilt: {pos_t:+5.1f}° | Durum: {status}")

    print("\n--- 2. Asama: Hedef Goruldu (+25.0° Pan, -5.0° Tilt) ---")
    patrol.on_target_detected(actual_pan=25.0, actual_tilt=-5.0)
    pos_p, pos_t = 25.0, -5.0
    print(f"Hedefe Kilitlenildi: Pan = {pos_p:.1f}°, Tilt = {pos_t:.1f}°")

    print("\n--- 3. Asama: Hedef Kayboldu (Grace Period -> Local -> Global) ---")
    # 0.5s sonra (Grace Period icinde)
    for step in range(1, 160):
        dp, dt_cmd, status = patrol.update(dt, pos_p, pos_t)
        pos_p += dp
        pos_t += dt_cmd
        t_now = step * dt
        if step in [10, 35, 60, 100, 150]:
            print(f"Zaman: {t_now:4.1f}s | Pan: {pos_p:+5.1f}° | Tilt: {pos_t:+5.1f}° | Durum: {status}")

    print("\n[OK] Iki Asamali Akilli Devriye Testi Basariyla Tamamlandi.")
