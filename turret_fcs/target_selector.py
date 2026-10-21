"""
================================================================================
MODUL: HEDEF SECICI VE ONCELIKLENDIRME MOTORU (TARGET SELECTOR)
================================================================================
Bu modul, kameradan gelen coklu nesne tespitleri (YOLOv8 Bounding Boxes) arasindan
en oncelikli ve vurulabilir birincil hedefi (Primary Threat) secer.

ELEME VE PUANLAMA KRITERLERI:
1. Guvenilirlik Esigi (Confidence Threshold): conf >= 0.28 altindaki sahte tespitler elenir.
2. Geometrik Alan ve Boyut Filtresi: Cok kucuk (area < 400 px^2) gurultuler ve asiri buyuk
   yanlis tespitler (area > 400.000 px^2) elenir.
3. En Boy Orani (Aspect Ratio): 0.35 - 2.50 arasindaki gecerli sekiller kabul edilir.
4. Nisangah Mesafesi (Crosshair Distance): Nisangah merkezine (reticle center) en yakin
   olan hedef en yuksek oncelikle secilir.
================================================================================
"""


class TargetSelector:
    """
    YOLO tespitlerinden en uygun hedef sinir kutusunu guvene, alana, aspect orana ve 
    nisangaha olan mesafeye gore secen sinif.
    """

    def __init__(self, min_conf=0.28, min_area=400, max_area=400000, min_dim=18, max_lost_frames=5):
        self.search_min_conf = float(min_conf)  # Ilk tespit arama esigi (Ev: 0.28 | Okul: 0.45)
        self.track_min_conf = 0.16              # Kilitliyken hareket bulanikligi (motion-blur) tolerans esigi
        self.min_area = min_area
        self.max_area = max_area
        self.min_dim = min_dim
        self.max_lost_frames = max_lost_frames
        self.locked_track_id = None
        self.last_target_center = None          # (cx, cy) Mekansal sadakat (spatial persistence)
        self.lost_counter = 0
        self.last_target_conf = 0.0

    @property
    def is_tracking(self):
        """Aktif bir hedef takibi suruyor mu?"""
        return (self.locked_track_id is not None or self.last_target_center is not None) and (self.lost_counter == 0)

    @property
    def min_conf(self):
        """Kilitli hedef takipteyken hareket bulanikligi aninda kopmamasi icin dinamik esik (Histerezis)."""
        if self.is_tracking:
            return self.track_min_conf
        return self.search_min_conf

    def select_best_target(self, boxes, center_x, center_y):
        if len(boxes) == 0:
            self.lost_counter += 1
            if self.lost_counter > self.max_lost_frames:
                self.reset()
            self.last_target_conf = 0.0
            return None

        candidates = []
        effective_conf_thresh = self.min_conf
        for box in boxes:
            try:
                conf = float(box.conf[0].cpu().numpy()) if hasattr(box.conf[0], "cpu") else float(box.conf[0])
            except Exception:
                conf = float(box.conf) if hasattr(box, "conf") else 0.0

            if conf < effective_conf_thresh:
                continue

            try:
                raw_coords = box.xyxy[0].cpu().numpy() if hasattr(box.xyxy[0], "cpu") else box.xyxy[0]
                x1, y1, x2, y2 = map(int, raw_coords)
            except Exception:
                continue

            bw = x2 - x1
            bh = y2 - y1
            area = bw * bh

            if area < self.min_area or area > self.max_area or bw < self.min_dim or bh < self.min_dim:
                continue

            aspect_ratio = bw / float(bh)
            if not (0.35 < aspect_ratio < 2.50):
                continue

            track_id = None
            if hasattr(box, "id") and box.id is not None:
                try:
                    track_id = int(box.id[0].cpu().numpy()) if hasattr(box.id[0], "cpu") else int(box.id[0])
                except Exception:
                    track_id = None

            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            dist_sq_center = (cx - center_x) ** 2 + (cy - center_y) ** 2

            dist_sq_last = None
            if self.last_target_center is not None:
                dist_sq_last = (cx - self.last_target_center[0]) ** 2 + (cy - self.last_target_center[1]) ** 2

            candidates.append({
                "box": (x1, y1, x2, y2),
                "center": (cx, cy),
                "track_id": track_id,
                "dist_sq_center": dist_sq_center,
                "dist_sq_last": dist_sq_last,
                "conf": conf
            })

        if not candidates:
            self.lost_counter += 1
            if self.lost_counter > self.max_lost_frames:
                self.reset()
            self.last_target_conf = 0.0
            return None

        # 1. ByteTrack kimligi (track_id) hala kadrajdaysa ona sadik kal
        if self.locked_track_id is not None:
            for c in candidates:
                if c["track_id"] is not None and c["track_id"] == self.locked_track_id:
                    self.lost_counter = 0
                    self.last_target_center = c["center"]
                    self.last_target_conf = float(c["conf"])
                    return c["box"]

        # 2. ByteTrack ID yoksa bile bir onceki karedeki konuma en yakin adayi sec (Mekansal Sadakat)
        if self.last_target_center is not None and self.lost_counter <= self.max_lost_frames:
            # Onceki konuma 180 piksel icindeki en yakin adayi al
            spatial_candidates = [c for c in candidates if c["dist_sq_last"] is not None and c["dist_sq_last"] <= (180 ** 2)]
            if spatial_candidates:
                best = min(spatial_candidates, key=lambda c: c["dist_sq_last"])
                self.locked_track_id = best["track_id"]
                self.last_target_center = best["center"]
                self.lost_counter = 0
                self.last_target_conf = float(best["conf"])
                return best["box"]

        # 3. Kilitli veya takip edilen hedef yoksa: Ekran merkezine en yakin olan birincil tehdidi sec
        best = min(candidates, key=lambda c: c["dist_sq_center"])
        self.locked_track_id = best["track_id"]
        self.last_target_center = best["center"]
        self.lost_counter = 0
        self.last_target_conf = float(best["conf"])
        return best["box"]

    def reset(self):
        """Hedef secici kilit hafizasini sifirlar."""
        self.locked_track_id = None
        self.last_target_center = None
        self.lost_counter = 0
        self.last_target_conf = 0.0