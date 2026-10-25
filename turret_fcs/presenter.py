"""
================================================================================
MODUL: BAS-USTU GOSTERGE SUNUCUSU (HUD PRESENTER)
================================================================================
Bu modul, Turret FCS ve Angajman durumunu OpenCV gorsel katmanina (`TurretHUD`)
aktaran yuksek seviyeli sunum motorudur (Presenter). `main.py` ve `webots_main.py`
arasindaki tum gorsel render siralamasini ve renk sozlesmesini tek yerde toplar.
================================================================================
"""

from typing import Optional, Tuple, Union
import numpy as np


class HUDPresenter:
    """FCS, sensor ve angajman verilerini MIL-STD HUD arayuzune cizen sunucu."""

    def __init__(self, hud):
        self.hud = hud

    def render(
        self,
        frame: np.ndarray,
        fcs,
        engagement_state,
        target_box: Optional[Tuple[int, int, int, int]],
        center_x: int,
        center_y: int,
        fps: float,
        dist_cm: int,
        strength: int,
        is_estop: bool,
        hit_feedback: Optional[Union[str, Tuple[str, Tuple[int, int, int]]]] = None,
    ):
        """HUD katmanlarini askeri hiyerarsiye gore kare uzerine cizer."""
        target_detected = target_box is not None
        hud_firing = engagement_state.is_active

        # 1. Hedef Belirleme ve Nisangah Renk Secimi
        if is_estop:
            reticle_color = (100, 100, 100)
            status_text = "STATUS: EMERGENCY STOP (LATCHED)"
            status_color = (0, 0, 255)
            angle_err_text = "ANGULAR ERR:   [HALTED] deg"
            if target_detected:
                x1, y1, x2, y2 = target_box
                self.hud.draw_corner_box(
                    frame, x1, y1, x2, y2,
                    color=(100, 100, 100), length=25, thickness=1,
                    label="TARGET (INHIBITED)"
                )
        elif target_detected:
            x1, y1, x2, y2 = target_box
            status_text = engagement_state.status_text
            status_color = engagement_state.status_color
            angle_err_text = f"ANGULAR ERR:   [{fcs.actual_pan:+.1f} , {fcs.actual_tilt:+.1f}] deg"

            is_red = fcs.is_locked or hud_firing
            box_color = (0, 0, 255) if is_red else (0, 191, 255)
            reticle_color = box_color

            self.hud.draw_corner_box(
                frame, x1, y1, x2, y2,
                color=box_color, length=25, thickness=2,
                label=self.hud.get_box_label()
            )

            # 2. Takip Cizgisi (Kilit oncesi optik merkezden hedefe yonlendirme hatti)
            bx, by = (x1 + x2) // 2, (y1 + y2) // 2
            if not hud_firing and not fcs.is_locked:
                self.hud.draw_tracking_line(frame, center_x, center_y, bx, by, status_color)

            # 3. Balistik Onleme Ibresi ve Oku (Lead Indicator Vector)
            lead_ox, lead_oy = fcs.firing_lead_offset
            if abs(lead_ox) > 1.0 or abs(lead_oy) > 1.0:
                self.hud.draw_lead_indicator(
                    frame, bx, by, int(fcs.pred_center[0]), int(fcs.pred_center[1])
                )

        else:
            # Hedef yok: Arama / Devriye durumlari
            reticle_color = (0, 255, 0)
            if getattr(fcs, "patrol_enabled", False):
                status_text = f"PATROL: {getattr(fcs, 'patrol_status', 'STANDBY')}"
                status_color = (0, 255, 0)
                t_pan = getattr(fcs, "turret_pan_deg", 0.0)
                t_tilt = getattr(fcs, "turret_tilt_deg", 0.0)
                angle_err_text = f"TURRET POSE:   [{t_pan:+.1f} , {t_tilt:+.1f}] deg"
            else:
                status_text = "STATUS: SEARCHING"
                status_color = (0, 255, 0)
                angle_err_text = "ANGULAR ERR:   [--.- , --.-] deg"

        # 4. Merkez Taktik Nisangahi
        self.hud.draw_grid_and_reticle(frame, center_x, center_y, reticle_color=reticle_color)

        # 5. Telemetri ve Muharebe Durum Bilgileri
        target_centered = False
        if target_detected:
            x1, y1, x2, y2 = target_box
            target_centered = self.hud.is_target_centered(center_x, center_y, x1, y1, x2, y2)

        if isinstance(hit_feedback, tuple):
            ballistic_hud_data = hit_feedback
        elif hit_feedback:
            ballistic_hud_data = (str(hit_feedback), (0, 255, 0))
        else:
            ballistic_hud_data = None

        self.hud.draw_telemetry(
            frame,
            fps,
            status_text,
            status_color,
            angle_err_text,
            status_color,
            dist_cm,
            strength,
            target_detected=target_detected,
            target_centered=target_centered,
            is_locked=fcs.is_locked,
            firing=hud_firing,
            center_x=center_x,
            center_y=center_y,
            ballistic_result=ballistic_hud_data,
            fire_stage=engagement_state.phase,
            spinup_progress=engagement_state.spinup_progress,
            spinup_duration_s=getattr(engagement_state, "spinup_duration", 0.25),
        )

        # 6. Atis sirasinda kilit bari ve isabet olasiligi gizlenir
        if target_detected and not hud_firing:
            max_lock_frames = getattr(fcs, "LOCK_THRESHOLD_FRAMES", 10)
            self.hud.draw_lock_bar(frame, center_x, center_y, fcs.lock_counter, max_lock_frames, fcs.is_locked)
            self.hud.draw_lock_quality(frame, center_x, center_y, getattr(fcs, "lock_quality", 0.0))

        # 7. Taktik Banner, Footer ve Olay Gunlugu
        self.hud.draw_mode_banner(
            frame, fcs.control_mode, getattr(fcs, "patrol_enabled", False),
            is_locked=fcs.is_locked, firing=hud_firing
        )
        self.hud.draw_footer(frame, is_locked=fcs.is_locked, firing=hud_firing)
        self.hud.draw_event_log(frame, is_locked=fcs.is_locked, firing=hud_firing)
