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


from .config import CONFIG


class TargetSelector:
    """
    YOLO tespitlerinden en uygun hedef sinir kutusunu guvene, alana, aspect orana ve
    nisangaha olan mesafeye gore secen sinif.
    """

    def __init__(
        self,
        min_conf=CONFIG.selector.min_conf,
        min_area=CONFIG.selector.min_area,
        max_area=CONFIG.selector.max_area,
        min_dim=CONFIG.selector.min_dim,
        max_lost_frames=CONFIG.selector.max_lost_frames,
        track_min_conf=CONFIG.selector.track_min_conf,
        spatial_gate_px=CONFIG.selector.spatial_gate_px,
        min_aspect_ratio=CONFIG.selector.min_aspect_ratio,
        max_aspect_ratio=CONFIG.selector.max_aspect_ratio,
    ):
        self.search_min_conf = float(min_conf)
        self.track_min_conf = float(track_min_conf)
        self.min_area = min_area
        self.max_area = max_area
        self.min_dim = min_dim
        self.max_lost_frames = max_lost_frames
        self.spatial_gate_px = float(spatial_gate_px)
        self.min_aspect_ratio = float(min_aspect_ratio)
        self.max_aspect_ratio = float(max_aspect_ratio)
        self.locked_track_id = None
        self.last_target_center = None
        self.lost_counter = 0
        self.last_target_conf = 0.0

    @property
    def is_tracking(self):
        """Hedef, tolerans penceresi icinde hala takip ediliyor mu?"""
        has_target = self.locked_track_id is not None or self.last_target_center is not None
        return has_target and self.lost_counter <= self.max_lost_frames

    @property
    def min_conf(self):
        """Kilitli takipte motion-blur toleransi; arama modunda daha siki esik."""
        return self.track_min_conf if self.is_tracking else self.search_min_conf

    @staticmethod
    def _box_conf(box):
        try:
            value = box.conf[0]
            return float(value.cpu().numpy()) if hasattr(value, "cpu") else float(value)
        except Exception:
            return float(getattr(box, "conf", 0.0))

    @staticmethod
    def _box_coords(box):
        try:
            value = box.xyxy[0]
            coords = value.cpu().numpy() if hasattr(value, "cpu") else value
            return tuple(int(v) for v in coords)
        except Exception:
            return None

    @staticmethod
    def _box_id(box):
        try:
            if getattr(box, "id", None) is None:
                return None
            value = box.id[0]
            return int(value.cpu().numpy()) if hasattr(value, "cpu") else int(value)
        except Exception:
            return None

    def _register_loss(self):
        self.lost_counter += 1
        if self.lost_counter > self.max_lost_frames:
            self.reset()
        self.last_target_conf = 0.0

    def select_best_target(self, boxes, center_x, center_y):
        if len(boxes) == 0:
            self._register_loss()
            return None

        candidates = []
        effective_conf_thresh = self.min_conf
        for box in boxes:
            conf = self._box_conf(box)
            if conf < effective_conf_thresh:
                continue

            coords = self._box_coords(box)
            if coords is None:
                continue
            x1, y1, x2, y2 = coords
            bw, bh = x2 - x1, y2 - y1
            area = bw * bh
            if area < self.min_area or area > self.max_area or bw < self.min_dim or bh < self.min_dim:
                continue
            if not (self.min_aspect_ratio < bw / float(bh) < self.max_aspect_ratio):
                continue

            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            dist_sq_last = None
            if self.last_target_center is not None:
                dist_sq_last = (cx - self.last_target_center[0]) ** 2 + (cy - self.last_target_center[1]) ** 2
            candidates.append({
                "box": (x1, y1, x2, y2),
                "center": (cx, cy),
                "track_id": self._box_id(box),
                "dist_sq_last": dist_sq_last,
                "dist_sq_center": (cx - center_x) ** 2 + (cy - center_y) ** 2,
                "conf": conf,
            })

        if not candidates:
            self._register_loss()
            return None

        # 1. ByteTrack kimligi: yalnizca son konuma makul uzakliktaysa kabul et.
        if self.locked_track_id is not None:
            for c in candidates:
                if c["track_id"] == self.locked_track_id and self._within_spatial_gate(c):
                    return self._accept(c)

        # 2. ID yoksa veya ID atlamasi supheliyse onceki konuma en yakin aday.
        if self.last_target_center is not None and self.lost_counter <= self.max_lost_frames:
            spatial_candidates = [c for c in candidates if self._within_spatial_gate(c)]
            if spatial_candidates:
                return self._accept(min(spatial_candidates, key=lambda c: c["dist_sq_last"]))

        # 3. Yeni hedef: ekran merkezine en yakin aday.
        return self._accept(min(candidates, key=lambda c: c["dist_sq_center"]))

    def _within_spatial_gate(self, candidate):
        if candidate["dist_sq_last"] is None:
            return False
        return candidate["dist_sq_last"] <= self.spatial_gate_px ** 2

    def _accept(self, candidate):
        self.locked_track_id = candidate["track_id"]
        self.last_target_center = candidate["center"]
        self.lost_counter = 0
        self.last_target_conf = float(candidate["conf"])
        return candidate["box"]

    def reset(self):
        """Hedef secici kilit hafizasini sifirlar."""
        self.locked_track_id = None
        self.last_target_center = None
        self.lost_counter = 0
        self.last_target_conf = 0.0