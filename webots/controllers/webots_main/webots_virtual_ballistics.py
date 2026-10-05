"""
=============================================================================
TURRET FCS - WEBOTS SANAL BALISTIK VE CANLI 3D ISABET DEGERLENDIRICI
=============================================================================
Bu modul mevcut takip ve motor kodlarini KESINLIKLE degistirmez veya karistirmaz.
Webots icerisinde 'Y' ile ates edildiginde 10 m/s hizindaki sanal merminin
balonla carpisma anini 3 boyutlu olarak (X ve Z) hesaplayan hakem sinifidir.
=============================================================================
"""

import time
import math
from pathlib import Path


class VirtualBallisticsEvaluator:
    """
    Sanal Mermi Ucus ve Canli 3D Carpisma Hakemi.
    'Y' ile ateslendiginde merminin balistik ucusunu takip eder,
    hedefe vardigi anda gercek balon koordinatiyla kiyaslayip HUD'a 3D isabet basar.
    """

    def __init__(self, bullet_speed=10.0, target_radius_m=0.15, supervisor=None):
        self.bullet_speed = float(bullet_speed)
        self.target_radius_m = float(target_radius_m)
        self.supervisor = supervisor

        # Balon dugumu baglantisi (Hakem olarak balonu yonetebilmek icin)
        self.balloon_node = None
        if self.supervisor is not None:
            try:
                self.balloon_node = self.supervisor.getFromDef("BALLOON")
            except Exception:
                self.balloon_node = None

        # Aktif sanal mermiler listesi
        self.active_shots = []

        # Son atis sonucu (HUD gosterimi icin)
        self.last_result_text = ""
        self.last_result_color = (0, 255, 0)
        self.last_result_time = 0.0
        self.DISPLAY_DURATION = 3.0  # HUD'da sonucun kalma suresi (saniye)

        # Istatistikler
        self.total_shots = 0
        self.total_hits = 0

    def get_hud_result(self):
        """HUD gosterimi icin son isabet sonucunu doner."""
        if (time.time() - self.last_result_time) < self.DISPLAY_DURATION and self.last_result_text:
            return self.last_result_text, self.last_result_color
        return None, None

    def fire_shot(self, pan_rad, tilt_rad, dist_m=2.0, sim_time=0.0):
        """Atis tetiklendiginde sanal mermiyi firlatir."""
        dist = max(0.5, float(dist_m)) if dist_m else 2.0
        t_flight = dist / self.bullet_speed  # 2.0m / 10m/s = 0.20s
        t_delay = 0.08  # Donanim ve motor kirma gecikmesi

        # Fiziksel Yercekimi Dusumu (g = 9.81 m/s^2): delta_z = 0.5 * g * t^2
        gravity_drop_m = 0.5 * 9.81 * (t_flight ** 2)

        # Namlunun baktigi aci ve yercekimi dusumuyle nihai 3D varis noktasi
        # Taret pivot yuksekligi Z = 0.828 m
        impact_x = -math.tan(pan_rad) * dist
        impact_z = 0.828 + math.tan(tilt_rad) * dist - gravity_drop_m

        shot = {
            "fire_sim_time": sim_time,
            "impact_sim_time": sim_time + t_flight + t_delay,
            "flight_time_s": t_flight + t_delay,
            "impact_x": impact_x,
            "impact_z": impact_z,
            "dist_m": dist,
            "gravity_drop_cm": gravity_drop_m * 100.0,
        }
        self.active_shots.append(shot)
        self.total_shots += 1

        superelev_deg = math.degrees(math.atan(gravity_drop_m / dist))
        print(f"[BALISTIK] SANAL MERMI ATILDI // V=10 m/s, Ucus: {t_flight * 1000:.0f} ms, Yercekimi: -{gravity_drop_m * 100.0:.1f} cm, Kompanzasyon: +{superelev_deg:.1f} deg (Menzil: {dist:.2f} m)")

    def _get_balloon_true_position(self, t_target):
        """
        balloon_mover.py dosyasindaki guncel hareket parametrelerini dinamik okur.
        Hedef sabitse (AMPLITUDE=0) veya hareketliyse gercek 3D koordinati doner.
        """
        amp_x, speed_x = 0.70, 1.3
        amp_z, speed_z = 0.20, 1.8
        bm_path = Path(__file__).resolve().parents[1] / "balloon_mover" / "balloon_mover.py"
        if bm_path.exists():
            content = bm_path.read_text(encoding="utf-8")
            for line in content.splitlines():
                line = line.strip()
                if line.startswith("AMPLITUDE_X"):
                    amp_x = float(line.split("=")[1].split("#")[0].strip())
                elif line.startswith("SPEED_X"):
                    speed_x = float(line.split("=")[1].split("#")[0].strip())
                elif line.startswith("AMPLITUDE_Z"):
                    amp_z = float(line.split("=")[1].split("#")[0].strip())
                elif line.startswith("SPEED_Z"):
                    speed_z = float(line.split("=")[1].split("#")[0].strip())

        true_x = amp_x * math.sin(speed_x * t_target)
        true_z = 1.055 + amp_z * math.sin(speed_z * t_target)
        return true_x, true_z

    def update(self, sim_time):
        """
        Her simulasyon adiminda ucus suresini kontrol eder.
        Mermi hedefe vardiginda balonun o anki 3D konumuyla carpismayi hesaplar.
        """
        remaining_shots = []
        for shot in self.active_shots:
            if sim_time >= shot["impact_sim_time"]:
                # Mermi balonun mesafesine ulasti!
                balloon_pos = None
                if self.balloon_node is not None:
                    try:
                        balloon_pos = self.balloon_node.getPosition()
                    except Exception:
                        balloon_pos = None

                if balloon_pos and balloon_pos[2] > -5.0:
                    balloon_true_x = float(balloon_pos[0]) - 0.02
                    balloon_true_z = float(balloon_pos[2]) + 0.155
                else:
                    t_target = shot["impact_sim_time"]
                    balloon_true_x, balloon_true_z = self._get_balloon_true_position(t_target)

                # Tam 3 Boyutlu Carpisma Mesafesi (3D Euclidean Distance)
                dx = shot["impact_x"] - balloon_true_x
                dz = shot["impact_z"] - balloon_true_z
                miss_distance_m = math.hypot(dx, dz)
                miss_distance_cm = miss_distance_m * 100.0

                is_hit = miss_distance_m <= self.target_radius_m
                if is_hit:
                    self.total_hits += 1
                    self.last_result_text = f"BALLISTIC HIT! ({miss_distance_cm:.1f} cm)"
                    self.last_result_color = (0, 255, 0)  # Parlak Yesil
                    print(f"[BALISTIK] >>> ISABET (HIT)! <<< 3D Sapma: {miss_distance_cm:.1f} cm [dx: {abs(dx)*100:.1f} cm, dz: {abs(dz)*100:.1f} cm] (Hedef VURULDU!)")
                    # Sadece ve sadece gercek isabette balona patlama emri gonder
                    if self.balloon_node is not None:
                        try:
                            custom_field = self.balloon_node.getField("customData")
                            if custom_field:
                                custom_field.setSFString("POP")
                        except Exception:
                            pass
                else:
                    self.last_result_text = f"BALLISTIC MISS! ({miss_distance_cm:.1f} cm)"
                    self.last_result_color = (0, 0, 255)  # Kirmizi
                    print(f"[BALISTIK] >>> ISKA (MISS)! <<< 3D Sapma: {miss_distance_cm:.1f} cm [dx: {abs(dx)*100:.1f} cm, dz: {abs(dz)*100:.1f} cm] (Hedef Iskalandi - Balon Patlamadi)")

                self.last_result_time = time.time()
            else:
                remaining_shots.append(shot)

        self.active_shots = remaining_shots