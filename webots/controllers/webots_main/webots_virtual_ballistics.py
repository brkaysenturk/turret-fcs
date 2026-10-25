"""
================================================================================
MODUL: WEBOTS SANAL BALISTIK VE 3D ISABET HAKEMI (VIRTUAL BALLISTICS - v2.0)
================================================================================
Bu modul; Webots icerisinde atis gerceklestiginde namlu rotasyon matrisinden
(KAFA_BLOGU orientation) dogrudan 3B namlu dogrultu vektorunu alarak yercekimi
dususu ve mermi ucus suresiyle hedef balon uzerindeki 3B carpisma noktasini (X, Y, Z)
analitik ve fiziksel olarak hesaplayan hakem sinifidir.

v2.0 UYUMLULUKLARI:
- Koordinat Dogrulugu: KAFA_BLOGU 3B oryantasyon vektoru [fx, fy, fz] uzerinden
  hesaplanan dunya carpisma koordinati; eksen isareti veya tanjant sapmasi icermez.
- Zaman Tabani: Saf simulasyon zamani (sim_t), sifir wall-clock kaymasi.
- Gosterim Suresi: 1.50 saniye (BALLISTIC_DISPLAY_DURATION ile tam senkron).
- Hedef Geometrisi: 3D balon merkezi ve 0.15 m yaricap toleransi.
================================================================================
"""

import math

from turret_fcs.config import CONFIG


class VirtualBallisticsEvaluator:
    """
    Sanal Mermi Ucus ve Canli 3D Carpisma Hakemi.
    Atis tetiklendiginde balistik ucusu takip eder, hedefe vardigi anda
    gercek balon 3D koordinatiyla kiyaslayip isabeti tesciller.
    """

    def __init__(self, bullet_speed=None, target_radius_m=None, supervisor=None):
        self.bullet_speed = float(bullet_speed) if bullet_speed else CONFIG.ballistics.projectile_speed_mps
        self.target_radius_m = float(target_radius_m) if target_radius_m else CONFIG.lock.target_radius_m
        self.supervisor = supervisor

        # Balon ve Kafa blogu dugum baglantilari
        self.balloon_node = None
        self.kafa_node = None
        if self.supervisor is not None:
            try:
                self.balloon_node = self.supervisor.getFromDef("BALLOON")
                self.kafa_node = self.supervisor.getFromDef("KAFA_BLOGU")
            except Exception:
                pass

        self.active_shots = []

        # Son atis sonucu
        self.last_result_text = ""
        self.last_result_color = (0, 255, 0)
        self.last_result_sim_time = -999.0
        self.DISPLAY_DURATION = CONFIG.ballistics.eval_display_time_s

        self.total_shots = 0
        self.total_hits = 0

        # Son atis telemetrisi (A/D): merkez mesafesi ve isabet/iska metrikleri
        self.last_hit = None          # None: beklemede, True: isabet, False: iska
        self.last_miss_cm = None
        self.last_shot = None

    def get_hud_result(self, sim_time):
        """HUD gosterimi icin son isabet sonucunu doner (sim_t zaman tabanli)."""
        if (sim_time - self.last_result_sim_time) < self.DISPLAY_DURATION and self.last_result_text:
            return self.last_result_text, self.last_result_color
        return None, None

    def fire_shot(self, dist_m=2.0, sim_time=0.0):
        """
        Atis aninda KAFA_BLOGU 3B oryantasyonundan namlu ileri vektorunu alir.
        LiDAR on yuzey mesafesini olcer; hedef merkezine gitmek icin yaricap eklenir.
        """
        dist_front = max(0.5, float(dist_m)) if dist_m else CONFIG.ballistics.default_distance_m
        # LiDAR ön yüzeyi ölçer; balistik merkez için +hedef yarıçapı
        dist = dist_front + self.target_radius_m
        t_flight = dist / self.bullet_speed
        t_delay = CONFIG.ballistics.system_latency_s  # 80 ms donanim gecikmesi

        g = CONFIG.ballistics.gravity_accel
        gravity_drop_m = 0.5 * g * (t_flight ** 2)

        # Namlu merkez konumu ve 3B donus matrisi
        head_pos = self.kafa_node.getPosition() if self.kafa_node else [0.0, 0.0, 0.828]
        head_rot = self.kafa_node.getOrientation() if self.kafa_node else [1, 0, 0, 0, 1, 0, 0, 0, 1]

        # Webots kafa blogunda yerel Y ekseni namlunun ileri dogrultusudur
        fx_dir = head_rot[1]
        fy_dir = head_rot[4]
        fz_dir = head_rot[7]

        # 3B dunya carpisma koordinati (merkez mesafesiyle)
        impact_x = head_pos[0] + fx_dir * dist
        impact_y = head_pos[1] + fy_dir * dist
        impact_z = head_pos[2] + fz_dir * dist - gravity_drop_m

        shot = {
            "fire_sim_time": sim_time,
            "impact_sim_time": sim_time + t_flight + t_delay,
            "flight_time_s": t_flight + t_delay,
            "impact_x": impact_x,
            "impact_y": impact_y,
            "impact_z": impact_z,
            "dist_front_m": dist_front,
            "dist_center_m": dist,
            "gravity_drop_cm": gravity_drop_m * 100.0,
        }
        self.active_shots.append(shot)
        self.total_shots += 1
        self.last_shot = shot
        self.last_hit = None
        self.last_miss_cm = None

    def update(self, sim_time):
        """Ucus suresi dolan mermileri balon konumuyla karsilastirir ve hakem kararini uretir."""
        remaining_shots = []

        for shot in self.active_shots:
            if sim_time >= shot["impact_sim_time"]:
                # Carpisma aninda gercek balon konumunu al
                b_pos = self._get_balloon_position()
                if b_pos:
                    # 3D mesafe farki
                    dx = shot["impact_x"] - b_pos[0]
                    dy = shot["impact_y"] - b_pos[1]
                    dz = shot["impact_z"] - b_pos[2]
                    err_3d = math.sqrt(dx*dx + dy*dy + dz*dz)

                    is_hit = err_3d <= self.target_radius_m
                    self.last_hit = bool(is_hit)
                    self.last_miss_cm = float(err_3d * 100.0)
                    if is_hit:
                        self.total_hits += 1
                        self.last_result_text = f"HIT! (ERR: {err_3d*100.0:.1f}cm)"
                        self.last_result_color = (0, 255, 0)
                        self.last_result_sim_time = sim_time
                        self._pop_balloon()
                    else:
                        self.last_result_text = f"MISSED (ERR: {err_3d*100.0:.1f}cm)"
                        self.last_result_color = (0, 0, 255)
                        self.last_result_sim_time = sim_time
                else:
                    self.last_result_text = "ENGAGEMENT COMPLETE"
                    self.last_result_color = (0, 215, 255)
                    self.last_result_sim_time = sim_time
            else:
                remaining_shots.append(shot)

        self.active_shots = remaining_shots

    def _get_balloon_position(self):
        """Balonun dunya koordinatlarindaki anlik 3D merkez noktasini dondurur."""
        if self.balloon_node:
            try:
                pos = self.balloon_node.getPosition()
                if pos:
                    # Balon geometrik ofsetleri (boundingObject translation)
                    return [pos[0] - 0.02, pos[1] - 0.255, pos[2] + 0.155]
            except Exception:
                pass
        return None

    def _pop_balloon(self):
        """Balon dugumune POP sinyali gonderir; balloon_mover bunu patlamaya cevirir."""
        if not self.balloon_node:
            print("[BALISTIK] UYARI: BALLOON dugumu bulunamadi; patlama sinyali gonderilemedi.")
            return
        sent = False
        try:
            self.balloon_node.setCustomData("POP")
            sent = True
        except Exception:
            # Bazi Webots surumlerinde Node.setCustomData yoktur; alan uzerinden yaz.
            try:
                field = self.balloon_node.getField("customData")
                if field:
                    field.setSFString("POP")
                    sent = True
            except Exception:
                pass
        print("[BALISTIK] POP sinyali:", "GONDERILDI" if sent else "BASARISIZ")
