"""
================================================================================
MODUL: HEDEF SECICI VE ONCELIKLENDIRME MOTORU (TARGET SELECTOR)
================================================================================
Bu modul, kameradan gelen coklu nesne tespitleri (YOLOv8 Bounding Boxes) arasindan
en oncelikli ve vurulabilir birincil hedefi (Primary Threat) secer.

ELEME VE PUANLAMA KRITERLERI:
1. Guvenilirlik Esigi (Confidence Threshold): Arama ve takip modunda dinamik esik.
2. Geometrik Alan, Boyut ve En-Boy Orani Filtresi.
3. Ego-Hareket Telafili Uzamsal Kapi ve IoU Tabanli ByteTrack Kimlik Devri (Handoff).
4. Nisangah Mesafesi ve Guven Agirligi (dist_sq / conf).
================================================================================
"""

import math

from .config import CONFIG


class TargetSelector:
    """
    YOLO tespitlerinden en uygun hedef sinir kutusunu guvene, alana, aspect orana,
    ego-kompanzasyonlu uzamsal kapiya ve nisangaha olan mesafeye gore secen sinif.
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
        acquisition_frames=CONFIG.selector.acquisition_frames,
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
        self.acquisition_frames = max(1, int(acquisition_frames))
        self.reset()

    @property
    def is_tracking(self):
        """Hedef, tolerans penceresi icinde hala takip ediliyor mu?"""
        has_target = self.locked_track_id is not None or self.last_target_center is not None
        return has_target and self.lost_counter <= self.max_lost_frames

    @property
    def min_conf(self):
        """Kilitli takipte motion-blur toleransi; arama modunda daha siki esik."""
        return self.track_min_conf if self.is_tracking else self.search_min_conf

    @property
    def association_center(self):
        if self.last_target_center is not None:
            return self.last_target_center
        return self._pending["center"] if self._pending is not None else None

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

    @staticmethod
    def _box_class(box):
        value = getattr(box, "cls", None)
        if value is None:
            return None
        try:
            v = value[0]
            return int(v.cpu().numpy()) if hasattr(v, "cpu") else int(v)
        except Exception:
            return None

    @staticmethod
    def _translate_box(box, shift):
        return tuple(v + shift[i % 2] for i, v in enumerate(box))

    def _handoff_matches(self, candidate, reference):
        """ByteTrack ID degisiminde yeni kutunun eski hedefle geometrik/IoU uyumunu denetler."""
        a, b = candidate["box"], reference["box"]
        aw, ah, bw, bh = a[2] - a[0], a[3] - a[1], b[2] - b[0], b[3] - b[1]
        if min(aw, ah, bw, bh) <= 0:
            return False
        if candidate["class_id"] is not None and reference["class_id"] is not None:
            if candidate["class_id"] != reference["class_id"]:
                return False
        S = CONFIG.selector
        if max(aw / bw, bw / aw, ah / bh, bh / ah) > S.handoff_max_size_ratio:
            return False
        if any(abs(x - y) > S.handoff_max_center_fraction * dim
               for x, y, dim in zip(candidate["center"], reference["center"], (bw, bh))):
            return False
        inter_w = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
        inter_h = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
        intersection = inter_w * inter_h
        iou = intersection / max(1e-6, aw * ah + bw * bh - intersection)
        return iou >= S.handoff_min_iou

    def _register_loss(self, dt=1.0 / 30.0):
        self.lost_counter += 1
        self.lost_time_s += dt
        if self.lost_counter > self.max_lost_frames:
            self.reset()
        self.last_target_conf = 0.0

    def select_best_target(self, boxes, center_x, center_y, dt=None, ego_shift=(0.0, 0.0), allowed_class_ids=None):
        dt = 1.0 / 30.0 if (dt is None or not math.isfinite(float(dt)) or float(dt) <= 0.0) else float(dt)

        if self.last_target_center is not None and (ego_shift[0] != 0.0 or ego_shift[1] != 0.0):
            self.last_target_center = (
                self.last_target_center[0] + float(ego_shift[0]),
                self.last_target_center[1] + float(ego_shift[1]),
            )
            if self.last_target_box is not None:
                self.last_target_box = self._translate_box(self.last_target_box, ego_shift)

        if self._pending is not None and (ego_shift[0] != 0.0 or ego_shift[1] != 0.0):
            self._pending["center"] = (
                self._pending["center"][0] + float(ego_shift[0]),
                self._pending["center"][1] + float(ego_shift[1]),
            )
            self._pending["box"] = self._translate_box(self._pending["box"], ego_shift)

        if len(boxes) == 0:
            self._register_loss(dt)
            return None

        sx = max(1.0, center_x * 2.0) / CONFIG.optics.default_width
        sy = max(1.0, center_y * 2.0) / CONFIG.optics.default_height
        gate = self.spatial_gate_px * min(4.0, max(1.0, (self.lost_time_s + dt) * 30.0))

        candidates = []
        effective_conf_thresh = self.min_conf
        for box in boxes:
            conf = self._box_conf(box)
            if not math.isfinite(conf) or conf < effective_conf_thresh:
                continue

            cls_id = self._box_class(box)
            if allowed_class_ids is not None and cls_id not in allowed_class_ids:
                continue

            coords = self._box_coords(box)
            if coords is None or len(coords) != 4:
                continue
            x1, y1, x2, y2 = coords
            bw, bh = (x2 - x1) / sx, (y2 - y1) / sy
            area = bw * bh
            if area < self.min_area or area > self.max_area or bw < self.min_dim or bh < self.min_dim:
                continue
            if not (self.min_aspect_ratio < bw / float(bh) < self.max_aspect_ratio):
                continue

            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            dist_sq_last = None
            if self.last_target_center is not None:
                dist_sq_last = (
                    ((cx - self.last_target_center[0]) / sx) ** 2
                    + ((cy - self.last_target_center[1]) / sy) ** 2
                )
            candidates.append({
                "box": (x1, y1, x2, y2),
                "center": (cx, cy),
                "track_id": self._box_id(box),
                "class_id": cls_id,
                "dist_sq_last": dist_sq_last,
                "dist_sq_center": (((cx - center_x) / sx) ** 2 + ((cy - center_y) / sy) ** 2),
                "conf": conf,
            })

        if not candidates:
            self._register_loss(dt)
            return None

        # 1. ByteTrack kimligi: yalnizca son konuma makul uzakliktaysa kabul et.
        if self.locked_track_id is not None:
            continuing = [
                c for c in candidates
                if c["track_id"] == self.locked_track_id and self._within_spatial_gate(c, gate)
            ]
            if continuing:
                self._pending = None
                self._pending_count = 0
                return self._accept(min(continuing, key=lambda c: c["dist_sq_last"] / max(c["conf"], 0.1)))

        # 2. ID degisimi (Handoff) veya ID yoksa uzamsal kapilama:
        # Once IoU ve kutu geometrisi eslesen adaylari tercih et, yoksa uzamsal kapidaki en yakini al.
        if self.last_target_center is not None and self.lost_counter <= self.max_lost_frames:
            spatial_candidates = [c for c in candidates if self._within_spatial_gate(c, gate)]
            if spatial_candidates:
                if self.last_target_box is not None:
                    reference = {
                        "box": self.last_target_box,
                        "center": self.last_target_center,
                        "class_id": self.locked_class_id,
                    }
                    handoff_candidates = [c for c in spatial_candidates if self._handoff_matches(c, reference)]
                    if handoff_candidates:
                        self._pending = None
                        self._pending_count = 0
                        return self._accept(min(handoff_candidates, key=lambda c: c["dist_sq_last"] / max(c["conf"], 0.1)))
                self._pending = None
                self._pending_count = 0
                return self._accept(min(spatial_candidates, key=lambda c: c["dist_sq_last"] / max(c["conf"], 0.1)))

        # 3. Yeni hedef: ekran merkezine en yakin ve arama guven esigini gecen aday.
        searching = [c for c in candidates if c["conf"] >= self.search_min_conf]
        if not searching:
            self._register_loss(dt)
            return None

        if self.acquisition_frames <= 1:
            return self._accept(min(searching, key=lambda c: c["dist_sq_center"] / max(c["conf"], 0.1)))

        if self._pending is not None:
            consistent = [
                c for c in searching
                if c["track_id"] == self._pending["track_id"]
                and c["class_id"] == self._pending["class_id"]
                and (((c["center"][0] - self._pending["center"][0]) / sx) ** 2
                     + ((c["center"][1] - self._pending["center"][1]) / sy) ** 2) <= gate ** 2
            ]
        else:
            consistent = []

        if consistent:
            candidate = min(
                consistent,
                key=lambda c: ((c["center"][0] - self._pending["center"][0]) ** 2
                               + (c["center"][1] - self._pending["center"][1]) ** 2),
            )
            self._pending_count += 1
        else:
            candidate = min(searching, key=lambda c: c["dist_sq_center"] / max(c["conf"], 0.1))
            self._pending_count = 1

        self._pending = candidate
        if self._pending_count >= self.acquisition_frames:
            self._pending = None
            self._pending_count = 0
            return self._accept(candidate)

        self.last_target_conf = 0.0
        return None

    def _within_spatial_gate(self, candidate, gate=None):
        if candidate["dist_sq_last"] is None:
            return False
        limit = self.spatial_gate_px if gate is None else float(gate)
        return candidate["dist_sq_last"] <= limit ** 2

    def _accept(self, candidate):
        self.locked_track_id = candidate["track_id"]
        self.locked_class_id = candidate["class_id"]
        self.last_target_center = candidate["center"]
        self.last_target_box = candidate["box"]
        self.lost_counter = 0
        self.lost_time_s = 0.0
        self.last_target_conf = float(candidate["conf"])
        return candidate["box"]

    def reset(self):
        """Hedef secici kilit hafizasini sifirlar."""
        self.locked_track_id = None
        self.locked_class_id = None
        self.last_target_center = None
        self.last_target_box = None
        self.lost_counter = 0
        self.lost_time_s = 0.0
        self.last_target_conf = 0.0
        self._pending = None
        self._pending_count = 0


def resolve_target_classes(names, target_name=CONFIG.selector.target_class_name):
    """Tek sinifli ozel modelde sinifi doner; cok sinifli modelde hedef sinif adini dogrular."""
    items = list(names.items()) if isinstance(names, dict) else list(enumerate(names))
    if len(items) == 1:
        return [int(items[0][0])]
    selected = [int(i) for i, name in items if str(name).casefold() == target_name.casefold()]
    if not selected:
        raise ValueError(
            f"Target class {target_name!r} is absent from model classes. "
            "Set CONFIG.selector.target_class_name or use the custom model."
        )
    return selected
