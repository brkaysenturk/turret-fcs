"""
================================================================================
MODUL: BAS-USTU GOSTERGESI VE GRAFIK ARAYUZ (HEADS-UP DISPLAY / TURRET HUD)
================================================================================
Bu modul, askeri standartlarda taktik atis kontrol arayuzunu (MIL-STD HUD)
kamera goruntusunun uzerine dinamik katman olarak bindiren gorsel render motorudur.

GOSTERGE VE TELEMETRI BILESENLERI:
1. Taktik Nisangah (Combat Reticle): Duruma gore renk degistiren (Arama: Sari,
   Kilit: Yesil, Atesleme: Kirmizi) merkez hedefleme vizoru.
2. Dinamik Kilit Bari (Lock Stability Progress Bar): Hedef kilit penceresinde
   kaldikca 0 - 100% dolan gorsel atis hazirlik cubugu.
3. Analitik Isabet Olasiligi Gostergesi (Hit Probability %): Hedef acisal hatasina
   ve mesafesine gore hesaplanan anlik vurus kalitesi.
4. Balistik Onleme Ibre ve Oku (Lead Indicator Vector): Hedefin gelecekteki
   konumunu gosteren tahmin vizoru ve hedef yon oku.
5. Canli Telemetri Paneli: FPS, TF-Luna LiDAR mesafesi, sinyal gucu, acisal sapma,
   kontrol modu (P CONTROLLER / RL CONTROLLER) ve akilli devriye durumu.
================================================================================
"""

import cv2
import numpy as np

try:
    from turret_fcs.config import CONFIG
except (ImportError, ValueError):
    from config import CONFIG


class TurretHUD:
    """
    Hedef kutusu etiketi, merkez nisangah, kilit dolum cizgisi, 
    iyonlama (lead) oku, isabet olasiligi, merkez atis paneli ve atis sonrasi
    geri bildirim metinlerini cizer.
    """
    def __init__(self):
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.event_logs = []

    def log_event(self, msg, color=(200, 210, 220)):
        """Taktiksel HUD olay gunlugune yeni bir olay ekler (saat damgasi olmadan)."""
        clean_msg = msg.strip().replace("\n", " ")
        self.event_logs.append((clean_msg, color))
        if len(self.event_logs) > 6:
            self.event_logs.pop(0)

    @staticmethod
    def is_target_centered(center_x, center_y, x1, y1, x2, y2):
        """Optik merkez / vizor hedef kutusunun icinde mi kontrolu."""
        return (x1 <= center_x <= x2) and (y1 <= center_y <= y2)

    def get_box_label(self, dist_cm=None, target_centered=False, is_locked=False, conf=None):
        """Hedef cercevesinin taktiksel etiket metnini dondurur."""
        label_parts = ["TARGET"]
        if conf is not None and conf > 0.0:
            pct = int(conf * 100) if conf <= 1.0 else int(conf)
            label_parts.append(f"%{pct}")
        if is_locked:
            label_parts.append("[LOCKED]")
        elif target_centered:
            label_parts.append("[CENTERED]")
        if dist_cm is not None and dist_cm > 20 and dist_cm <= 400:
            label_parts.append(f"{int(dist_cm)}cm")
        return " ".join(label_parts)

    def draw_corner_box(self, img, x1, y1, x2, y2, color, length=18, thickness=2, label=None):
        """Hedef cercevesi."""
        cv2.line(img, (x1, y1), (x1 + length, y1), color, thickness)
        cv2.line(img, (x1, y1), (x1, y1 + length), color, thickness)
        cv2.line(img, (x2, y1), (x2 - length, y1), color, thickness)
        cv2.line(img, (x2, y1), (x2, y1 + length), color, thickness)
        cv2.line(img, (x1, y2), (x1 + length, y2), color, thickness)
        cv2.line(img, (x1, y2), (x1, y2 - length), color, thickness)
        cv2.line(img, (x2, y2), (x2 - length, y2), color, thickness)
        cv2.line(img, (x2, y2), (x2, y2 - length), color, thickness)

        if label:
            cv2.putText(img, label, (x1, y1 - 10), self.font, 0.55, color, 2, cv2.LINE_AA)

    def draw_grid_and_reticle(self, frame, center_x, center_y, reticle_color=(0, 255, 0)):
        """Merkez nisangah."""
        cv2.circle(frame, (center_x, center_y), 34, reticle_color, 2, cv2.LINE_AA)

        # Nisangah cizgileri
        cv2.line(frame, (center_x - 46, center_y), (center_x - 34, center_y), reticle_color, 2, cv2.LINE_AA)
        cv2.line(frame, (center_x + 34, center_y), (center_x + 46, center_y), reticle_color, 2, cv2.LINE_AA)
        cv2.line(frame, (center_x, center_y - 46), (center_x, center_y - 34), reticle_color, 2, cv2.LINE_AA)
        cv2.line(frame, (center_x, center_y + 34), (center_x, center_y + 46), reticle_color, 2, cv2.LINE_AA)
        
        cv2.circle(frame, (center_x, center_y), 3, reticle_color, -1)

    def draw_lock_bar(self, frame, center_x, center_y, lock_counter, max_frames, is_locked):
        """Kilitlenme dolum bari."""
        bar_w, bar_h = 140, 12
        bar_x = center_x - (bar_w // 2)
        bar_y = center_y + 50

        # Dis Cerceve
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (180, 180, 180), 2)
        
        # Doluluk orani hesabi (ekrandan tasmasini onlemek icin 0.0 - 1.0 arasina sinirla)
        fill_ratio = min(1.0, max(0.0, lock_counter / float(max(1, max_frames))))
        current_fill_w = int((bar_w - 4) * fill_ratio)

        # Dolum ilerleme cubugu
        if current_fill_w > 0:
            bar_fill_color = (0, 0, 255) if is_locked else (0, 191, 255)
            cv2.rectangle(
                frame, 
                (bar_x + 2, bar_y + 2), 
                (bar_x + 2 + current_fill_w, bar_y + bar_h - 2), 
                bar_fill_color, 
                -1
            )

    def draw_tracking_line(self, frame, center_x, center_y, bx, by, color):
        """Merkez nisangah ile hedef merkezi arasindaki yon cizgisi."""
        cv2.line(frame, (center_x, center_y), (bx, by), color, 1, cv2.LINE_AA)
        cv2.circle(frame, (bx, by), 3, color, -1)

    def draw_lead_indicator(self, frame, bx, by, pred_px, pred_py):
        """Hedef merkezinden onleme (lead) noktasina dogru olan vektor ve cizelge."""
        cv2.line(frame, (bx, by), (pred_px, pred_py), (255, 100, 0), 1, cv2.LINE_AA)
        cv2.circle(frame, (pred_px, pred_py), 5, (255, 0, 0), -1)
        cv2.putText(frame, "LEAD", (pred_px + 8, pred_py - 4), self.font, 0.40, (255, 0, 0), 1, cv2.LINE_AA)

    def draw_lock_quality(self, frame, center_x, center_y, quality_val):
        """Isabet olasiligi gostergesi."""
        if quality_val <= 0.0:
            return
        p_hit_pct = int(quality_val * 100) if quality_val <= 1.0 else int(quality_val)
        if p_hit_pct >= 70:
            q_color = (0, 0, 255)      # Kirmizi (Yuksek isabet / Kilit)
        elif p_hit_pct >= 40:
            q_color = (0, 191, 255)    # Amber (Orta isabet / Takip)
        else:
            q_color = (180, 180, 180)  # Gri (Dusuk isabet)

        text = f"HIT PROBABILITY: %{p_hit_pct}"
        font_scale = 0.55
        (t_w, _), _ = cv2.getTextSize(text, self.font, font_scale, 1)
        text_x = center_x - (t_w // 2)
        text_y = center_y + 86

        cv2.putText(
            frame,
            text,
            (text_x, text_y),
            self.font,
            font_scale,
            q_color,
            1,
            cv2.LINE_AA
        )

    def draw_combat_panel(self, frame, center_x, center_y, dist_cm, firing=False, ballistic_result=None, fire_stage=None, spinup_progress=None):
        """Kilit aninda acilan merkez atis paneli."""
        h, w, _ = frame.shape

        green_color = (0, 255, 0)
        amber_color = (0, 191, 255)
        red_color = (0, 0, 255)
        white_color = (255, 255, 255)

        # Duruma gore gosterilecek metinler
        if firing:
            if fire_stage == "SPINUP" and spinup_progress is not None:
                panel_color = (0, 215, 255)
                spinup_total_ms = int(CONFIG.ballistics.spinup_time_s * 1000)
                ms_rem = max(0, int((1.0 - spinup_progress) * spinup_total_ms))
                lines = [
                    ("FIRE AUTHORIZED", 0.65, amber_color, 2),
                    (f"SPINUP & SLEW: {ms_rem:3d} ms", 0.65, (0, 215, 255), 2)
                ]
                line_y_offsets = [28, 54]
                panel_h = 78
            elif fire_stage == "FIRING":
                panel_color = red_color
                lines = [
                    ("FIRE AUTHORIZED", 0.65, red_color, 2),
                    ("SOLENOID RELEASE [SHOT AWAY]", 0.62, red_color, 2)
                ]
                line_y_offsets = [30, 58]
                panel_h = 72
            elif fire_stage == "RECOVER":
                panel_color = (0, 255, 128)
                lines = [
                    ("FIRE COMPLETE", 0.65, (0, 255, 128), 2),
                    ("RECOVERING BORESIGHT", 0.62, (0, 255, 128), 2)
                ]
                line_y_offsets = [30, 58]
                panel_h = 72
            elif ballistic_result and ballistic_result[0]:
                res_text, res_color = ballistic_result
                panel_color = res_color
                lines = [
                    ("ENGAGEMENT COMPLETE", 0.65, green_color, 2),
                    (res_text, 0.65, res_color, 2)
                ]
                line_y_offsets = [30, 58]
                panel_h = 72
            else:
                panel_color = red_color
                lines = [
                    ("FIRE AUTHORIZED", 0.65, red_color, 2),
                    ("ENGAGING TARGET", 0.65, red_color, 2)
                ]
                line_y_offsets = [30, 58]
                panel_h = 72
        else:
            panel_color = amber_color
            range_str = f"{dist_cm} cm" if dist_cm > 0 else "CALCULATING"
            telemetry_str = f"RANGE: {range_str}"

            lines = [
                ("TARGET LOCKED", 0.65, amber_color, 2),
                (telemetry_str, 0.55, white_color, 2),
                ("AUTHORIZE FIRE? [Y/N]", 0.55, amber_color, 2)
            ]
            line_y_offsets = [28, 56, 82]
            panel_h = 94

        # Metne gore kutu genisligi
        max_text_w = 0
        for text, scale, _, thickness in lines:
            (t_w, _), _ = cv2.getTextSize(text, self.font, scale, thickness)
            if t_w > max_text_w:
                max_text_w = t_w

        padding_x = 24
        panel_w = max_text_w + (2 * padding_x)

        # Kutunun konumu
        px1 = center_x - (panel_w // 2)
        py1 = center_y + 115
        px2 = px1 + panel_w
        py2 = py1 + panel_h

        px1, py1 = max(0, px1), max(0, py1)
        px2, py2 = min(w, px2), min(h, py2)

        # Yari saydam arka plan
        roi = frame[py1:py2, px1:px2]
        black_layer = np.zeros_like(roi)
        frame[py1:py2, px1:px2] = cv2.addWeighted(roi, 0.40, black_layer, 0.60, 0)

        cv2.rectangle(frame, (px1, py1), (px2, py2), panel_color, 2)

        # Spinup sirasinda minik ilerleme cubugu
        if firing and fire_stage == "SPINUP" and spinup_progress is not None:
            bar_w = int((panel_w - 24) * float(np.clip(spinup_progress, 0.0, 1.0)))
            cv2.rectangle(frame, (px1 + 12, py2 - 9), (px1 + 12 + bar_w, py2 - 4), (0, 215, 255), -1)

        # Metinleri ortala ve yaz
        for (text, scale, color, thickness), y_offset in zip(lines, line_y_offsets):
            (t_w, _), _ = cv2.getTextSize(text, self.font, scale, thickness)
            x_pos = center_x - (t_w // 2)
            cv2.putText(frame, text, (x_pos, py1 + y_offset), self.font, scale, color, thickness, cv2.LINE_AA)

    def draw_telemetry(self, frame, fps, status_text, status_color, 
                       angle_err_text, angle_err_color, dist_cm, strength,
                       target_detected=False, target_centered=False,
                       is_locked=False, firing=False, center_x=None, center_y=None,
                       ballistic_result=None, fire_stage=None, spinup_progress=None):
        """Sol ust telemetri paneli."""
        h, w, _ = frame.shape
        if center_x is None or center_y is None:
            center_x, center_y = w // 2, h // 2

        # Kilit varsa veya atis devredeyse merkez muharebe panelini ac
        if is_locked or firing:
            self.draw_combat_panel(
                frame, center_x, center_y, dist_cm, 
                firing=firing, ballistic_result=ballistic_result,
                fire_stage=fire_stage, spinup_progress=spinup_progress
            )
        else:
            # Menzil yazisi
            if not target_detected:
                dist_str = "RANGE: --"
            elif not target_centered:
                dist_str = "RANGE: ACQUIRING"
            else:
                if dist_cm > 0:
                    dist_str = f"RANGE: {dist_cm:3d} cm" if dist_cm <= 300 else "OUT OF RANGE"
                else:
                    dist_str = "RANGE: ERROR"

            # Sinyal yazisi
            if strength > 1000:
                sig_str = "SIGNAL: STRONG"
            elif strength > 100:
                sig_str = "SIGNAL: MODERATE"
            elif dist_cm > 0 or strength > 0:
                sig_str = "SIGNAL: WEAK"
            else:
                sig_str = "SIGNAL: --"

            status_line = status_text
            telemetry_line = f"{dist_str}   |   {sig_str}"
            text_color = (255, 255, 255) if target_detected else (190, 190, 190)

            title_scale = 0.55
            sub_scale = 0.50
            thick = 2

            # Metne gore panel genisligi
            (w_status, _), _ = cv2.getTextSize(status_line, self.font, title_scale, thick)
            (w_telemetry, _), _ = cv2.getTextSize(telemetry_line, self.font, sub_scale, 1)
            (w_angle, _), _ = cv2.getTextSize(angle_err_text, self.font, sub_scale, 1)

            max_text_w = max(w_status, w_telemetry, w_angle)

            px1, py1 = 15, 46
            px2 = px1 + max_text_w + 24
            py2 = py1 + 86

            px2 = min(px2, w)
            py2 = min(py2, h)

            # Yari saydam arka plan
            roi = frame[py1:py2, px1:px2]
            black_layer = np.zeros_like(roi)
            frame[py1:py2, px1:px2] = cv2.addWeighted(roi, 0.35, black_layer, 0.65, 0)

            # Hedef varken sari, devriyede ayni devriye yesili (0, 255, 0), bostayken gri cerceve
            is_patrol = status_line.startswith("PATROL")
            if is_patrol:
                box_border_color = (0, 255, 0)
                text_color = (0, 255, 0)
                angle_color = (0, 255, 0)
                telemetry_line = "AIRSPACE SCAN: WIDE FIGURE-8 (8.0s)"
            elif target_detected:
                box_border_color = (0, 191, 255)
                angle_color = angle_err_color if angle_err_color is not None else (200, 210, 220)
            else:
                box_border_color = (90, 90, 90)
                angle_color = (200, 210, 220)

            cv2.rectangle(frame, (px1, py1), (px2, py2), box_border_color, 2)

            cv2.putText(frame, status_line, (26, py1 + 24), self.font, title_scale, status_color, thick, cv2.LINE_AA)
            cv2.putText(frame, telemetry_line, (26, py1 + 50), self.font, sub_scale, text_color, 1, cv2.LINE_AA)
            cv2.putText(frame, angle_err_text, (26, py1 + 72), self.font, sub_scale, angle_color, 1, cv2.LINE_AA)

        # Sag ust FPS parametresi (Takip ve devriyede aktif; kilit ve atis aninda gizlenir)
        if not (is_locked or firing):
            fps_text = f"FPS: {fps}"
            (fps_w, _), _ = cv2.getTextSize(fps_text, self.font, 0.50, 2)
            fps_color = (0, 255, 0) if fps >= 25 else (0, 165, 255)
            cv2.putText(frame, fps_text, (w - fps_w - 18, 26), self.font, 0.50, fps_color, 2, cv2.LINE_AA)

    def draw_mode_banner(self, frame, control_mode, patrol_enabled=False, is_locked=False, firing=False):
        """
        Ust orta kontrol modu ve devriye durumu baslik banneri.
        Atis izni (is_locked=True) ve atis protokolunde (firing=True) otomatik olarak gizlenir.
        """
        if is_locked or firing:
            return

        h, w, _ = frame.shape
        center_x = w // 2

        if control_mode == "RL":
            mode_label = "RL CONTROLLER (AGILE PURSUIT)"
            mode_color = (0, 255, 128)
        else:
            mode_label = "P CONTROLLER (PRECISION LOCK)"
            mode_color = (0, 255, 255)

        if patrol_enabled:
            banner_text = f"[{mode_label}]  |  [PATROL: FIGURE-8]"
            banner_color = (0, 255, 0)
            banner_border = (0, 255, 0)
        else:
            banner_text = f"[{mode_label}]  |  [PATROL: OFF]"
            banner_color = mode_color
            banner_border = (90, 100, 110)

        (b_w, _), _ = cv2.getTextSize(banner_text, self.font, 0.56, 2)
        banner_x = max(10, center_x - (b_w // 2))

        # Yari saydam ust baslik paneli
        pad_x, pad_y = 12, 5
        bx1, by1 = max(0, banner_x - pad_x), max(0, 26 - 16 - pad_y)
        bx2, by2 = min(w, banner_x + b_w + pad_x), min(h, 26 + pad_y)
        roi_b = frame[by1:by2, bx1:bx2]
        black_b = np.zeros_like(roi_b)
        frame[by1:by2, bx1:bx2] = cv2.addWeighted(roi_b, 0.30, black_b, 0.70, 0)
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), banner_border, 1)

        cv2.putText(frame, banner_text, (banner_x, 26), self.font, 0.56, banner_color, 2, cv2.LINE_AA)

    def draw_footer(self, frame, is_locked=False, firing=False):
        """
        Alt operator kontrol lejandi.
        Atis protokolunde gizlenir; atis izni aninda kilit onay tuslarini gosterir.
        """
        if firing:
            return

        h, w, _ = frame.shape
        center_x = w // 2

        if is_locked:
            footer_text = "[Y] Authorize Fire  |  [N] Abort Lock  |  [SPACE] E-Stop"
        else:
            footer_text = "[M] Mode  |  [P] Patrol  |  [SPACE] E-Stop  |  [Y] Fire  |  [ESC] Quit"

        (f_w, _), _ = cv2.getTextSize(footer_text, self.font, 0.42, 1)
        cv2.putText(
            frame,
            footer_text,
            (center_x - f_w // 2, h - 14),
            self.font,
            0.42,
            (160, 170, 180),
            1,
            cv2.LINE_AA,
        )

    def draw_event_log(self, frame, event_logs=None, max_lines=4, is_locked=False, firing=False):
        """
        Sol alt kosede terminaldeki sistem ve operator mesajlarini gosteren
        yari saydam taktiksel askeri olay gunlugu (Console Event Log).
        Atis izni (is_locked=True) ve atis protokolunde (firing=True) otomatik olarak gizlenir.
        """
        if is_locked or firing:
            return

        logs_to_draw = event_logs if event_logs is not None else self.event_logs
        if not logs_to_draw:
            return

        display_logs = logs_to_draw[-max_lines:]
        h, w, _ = frame.shape
        line_height = 18
        header_height = 20
        box_h = header_height + (len(display_logs) * line_height) + 8

        px1 = 15
        py2 = h - 30
        py1 = py2 - box_h

        # Genislik hesabi
        max_txt_w = 220
        for item in display_logs:
            txt = str(item[0]) if isinstance(item, (list, tuple)) else str(item)
            (tw, _), _ = cv2.getTextSize(txt, self.font, 0.38, 1)
            if tw > max_txt_w:
                max_txt_w = tw

        px2 = min(w - 20, px1 + max_txt_w + 24)

        # Yari saydam arka plan
        roi = frame[py1:py2, px1:px2]
        black_layer = np.zeros_like(roi)
        frame[py1:py2, px1:px2] = cv2.addWeighted(roi, 0.25, black_layer, 0.75, 0)

        # Askeri cerceve
        cv2.rectangle(frame, (px1, py1), (px2, py2), (75, 85, 95), 1)

        # Baslik
        header_text = "EVENT LOG"
        cv2.putText(frame, header_text, (px1 + 10, py1 + 14), self.font, 0.36, (0, 200, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (px1 + 8, py1 + 18), (px2 - 8, py1 + 18), (55, 65, 75), 1)

        # Log satirlari
        curr_y = py1 + 18 + line_height
        for item in display_logs:
            if isinstance(item, (list, tuple)):
                full_text = str(item[0])
                color = item[1] if len(item) > 1 else (200, 210, 220)
            else:
                color = (200, 210, 220)
                full_text = str(item)

            cv2.putText(frame, full_text, (px1 + 10, curr_y - 3), self.font, 0.38, color, 1, cv2.LINE_AA)
            curr_y += line_height