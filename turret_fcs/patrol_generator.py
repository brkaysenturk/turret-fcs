"""
================================================================================
MODUL: SAKIN MERKEZE DONUS VE DAIRESEL SEKTOR DEVIYE URETECI (PATROL GENERATOR)
================================================================================
Bu modul, hedef gorunmediginde taretin sakin ve kararli bir sekilde alan taramasi
yapmasini saglar:

1. HEDEF KAYBOLDUGUNDA (1.5 SANIYE BEKLEME):
   - Hedef kadrajdan ciktiginda veya anlik kesintilerde taret hemen yerinden oynamaz.
   - Son goruldugu konumda 1.5 saniye sakince bekler. Hedef tekrar belirirse
     hic motor hareketi yapmadan aninda takibe devam eder.

2. MERKEZE DONUS (0°, 0°):
   - 1.5 saniye icinde hedef donmezse, taret sakin ve orta duzey bir hizla (20°/s)
     merkez (0°, 0°) noktasina doner.

3. DAIRESEL / ELIPTIK SEKTOR TARAMASI (Pan: +/- 28°, Tilt: +/- 8°):
   - Merkeze ulasinca hem yatay hem dikey eksenleri eszamanli (sin/cos) calistirarak
     gokyuzunde genis bir konik cember/elips taramasi yapar.
   - Merkezden cembere gecerken yumusak bir spiral ile acilir (sifir vuruntu).
   - Koseli hareket olmadigi icin servolar surekli akici, titresimsiz ve sakin doner.

4. HEDEF GORULDUGUNDE:
   - Devriye aninda devreden cikar, kontrol P takip kontrolcusune gecer.
================================================================================
"""

import numpy as np


class PatrolGenerator:
    """
    1.5s bekleme, sakin merkeze donus ve 2 eksenli dairesel taramali devriye ureteci.
    """

    def __init__(
        self,
        wait_delay_sec=1.5,      # Hedef kaybolduktan sonra bekleme suresi (1.5 saniye)
        scan_pan_amp=35.0,       # Yatay tarama genligi (+/- 35 derece -> FOV ile ~125 derece hava sahasi)
        scan_tilt_amp=8.0,       # Dikey tarama genligi (+/- 8 derece -> FOV ile ~48 derece dikey koridor)
        scan_period_sec=5.0,     # 1 tam 8-sekli (sonsuzluk) turu suresi (5.0 saniye -> dinamik ve seri tarama)
        return_speed_dps=20.0,   # Merkeze donus acisal hizi (20 derece/saniye)
        max_slew_per_frame=1.5,  # Kare basi maksimum motor adimi (derece - kirkilmayi onler)
    ):
        self.wait_delay_sec = float(wait_delay_sec)
        self.scan_pan_amp = float(scan_pan_amp)
        self.scan_tilt_amp = float(scan_tilt_amp)
        self.scan_freq = float(2.0 * np.pi / max(0.5, scan_period_sec))
        self.return_speed_dps = float(return_speed_dps)
        self.max_slew = float(max_slew_per_frame)

        self.state = "WAITING"   # "WAITING", "RETURNING", "SCANNING"
        self.lost_timer = 0.0
        self.scan_time = 0.0
        self.current_pan = 0.0
        self.current_tilt = 0.0

    def on_target_detected(self, actual_pan=0.0, actual_tilt=0.0):
        """Hedef tespit edildiginde devriye beklemede kalir, zamanlayici sifirlanir."""
        self.state = "WAITING"
        self.lost_timer = 0.0
        self.scan_time = 0.0
        self.current_pan = 0.0
        self.current_tilt = 0.0

    def update(self, dt, current_pan=None, current_tilt=None):
        """
        Hedef yokken bir sonraki devriye adimini (delta_pan, delta_tilt) ve durum metnini dondurur.
        """
        dt = 0.033 if (dt is None or dt <= 0.0) else float(dt)
        curr_p = self.current_pan if current_pan is None else float(current_pan)
        curr_t = self.current_tilt if current_tilt is None else float(current_tilt)

        # 1. ASAMA: HEDEF KAYBOLDUKTAN SONRA 1.5 SANIYE YERINDE SAKINCE BEKLE
        if self.state == "WAITING":
            self.lost_timer += dt
            if self.lost_timer < self.wait_delay_sec:
                remaining = self.wait_delay_sec - self.lost_timer
                return 0.0, 0.0, f"BEKLEMEDE ({remaining:.1f}s)"
            else:
                self.state = "RETURNING"

        # 2. ASAMA: SAKINCE MERKEZE DONUS (0, 0)
        if self.state == "RETURNING":
            err_pan = -curr_p
            err_tilt = -curr_t
            dist_to_center = float(np.hypot(err_pan, err_tilt))

            # Merkeze yeterince yaklastiysak 8-sekli taramaya basla
            if dist_to_center < 1.0:
                self.state = "SCANNING"
                self.scan_time = 0.0
                self.current_pan = 0.0
                self.current_tilt = 0.0
                return 0.0, 0.0, "MERKEZE ULASILDI -> 8-SEKLI TARAMA BASLIYOR"

            # Merkeze dogru sakin adim
            step = min(self.return_speed_dps * dt, self.max_slew)
            dir_pan = float(np.sign(err_pan))
            dir_tilt = float(np.sign(err_tilt))

            delta_p = float(np.clip(dir_pan * min(abs(err_pan), step), -self.max_slew, self.max_slew))
            delta_t = float(np.clip(dir_tilt * min(abs(err_tilt), step), -self.max_slew, self.max_slew))

            self.current_pan += delta_p
            self.current_tilt += delta_t
            return delta_p, delta_t, f"MERKEZE DONUYOR ({dist_to_center:.1f}°)"

        # 3. ASAMA: 8-SEKLI (SONSUZLUK) LISSAJOUS TARAMASI (Pan: ±35°, Tilt: ±8°)
        elif self.state == "SCANNING":
            self.scan_time += dt

            # Merkezden 8 sekline gecerken 2 saniyelik yumusak spiral genisleme (sifir vuruntu)
            ramp = min(1.0, self.scan_time / 2.0)

            # 1:2 frekans orani ile yatay sonsuzluk (figure-8) yorungesi
            target_pan = ramp * self.scan_pan_amp * float(np.sin(self.scan_freq * self.scan_time))
            target_tilt = ramp * self.scan_tilt_amp * float(np.sin(2.0 * self.scan_freq * self.scan_time))

            err_pan = target_pan - curr_p
            err_tilt = target_tilt - curr_t

            delta_p = float(np.clip(err_pan, -self.max_slew, self.max_slew))
            delta_t = float(np.clip(err_tilt, -self.max_slew, self.max_slew))

            self.current_pan += delta_p
            self.current_tilt += delta_t
            return delta_p, delta_t, f"8-SEKLI TARAMA (±{int(self.scan_pan_amp)}° x ±{int(self.scan_tilt_amp)}°)"

        return 0.0, 0.0, "STANDBY"

    def reset(self):
        """Devriye durumunu sifirlar."""
        self.state = "WAITING"
        self.lost_timer = 0.0
        self.scan_time = 0.0
        self.current_pan = 0.0
        self.current_tilt = 0.0
