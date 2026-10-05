import cv2
import numpy as np


class TurretHUD:
    """
    Hedef kutusu etiketi, merkez nisangah, kilit dolum cizgisi, 
    iyonlama (lead) oku, isabet olasiligi, merkez atis paneli ve atis sonrasi
    geri bildirim metinlerini cizer.
    """
    def __init__(self):
        self.font = cv2.FONT_HERSHEY_SIMPLEX

    @staticmethod
    def is_target_centered(center_x, center_y, x1, y1, x2, y2):
        """Optik merkez / vizor hedef kutusunun icinde mi kontrolu."""
        return (x1 <= center_x <= x2) and (y1 <= center_y <= y2)

    def get_box_label(self, dist_cm, target_centered, is_locked):
        """Hedef kutusu etiketi."""
        if is_locked or not target_centered or dist_cm <= 0:
            return "TARGET"
        mesafe_m = dist_cm / 100.0
        return f"TARGET | {mesafe_m:.2f}m"

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
            q_color = (0, 255, 0)
        elif p_hit_pct >= 40:
            q_color = (0, 255, 255)
        else:
            q_color = (0, 165, 255)

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

    def draw_combat_panel(self, frame, center_x, center_y, dist_cm, firing=False, ballistic_result=None):
        """Kilit aninda acilan merkez atis paneli."""
        h, w, _ = frame.shape

        amber_color = (0, 191, 255)
        red_color = (0, 0, 255)
        white_color = (255, 255, 255)

        # Duruma gore gosterilecek metinler
        if firing:
            if ballistic_result and ballistic_result[0]:
                res_text, res_color = ballistic_result
                panel_color = res_color
                lines = [
                    ("FIRE AUTHORIZED", 0.65, amber_color, 2),
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

        # Metinleri ortala ve yaz
        for (text, scale, color, thickness), y_offset in zip(lines, line_y_offsets):
            (t_w, _), _ = cv2.getTextSize(text, self.font, scale, thickness)
            x_pos = center_x - (t_w // 2)
            cv2.putText(frame, text, (x_pos, py1 + y_offset), self.font, scale, color, thickness, cv2.LINE_AA)

    def draw_telemetry(self, frame, fps, status_text, status_color, 
                       angle_err_text, angle_err_color, dist_cm, strength,
                       target_detected=False, target_centered=False,
                       is_locked=False, firing=False, center_x=None, center_y=None,
                       ballistic_result=None):
        """Sol ust telemetri paneli."""
        h, w, _ = frame.shape
        if center_x is None or center_y is None:
            center_x, center_y = w // 2, h // 2

        # Kilit varsa veya atis devredeyse merkez muharebe panelini ac
        if is_locked or firing:
            self.draw_combat_panel(frame, center_x, center_y, dist_cm, firing=firing, ballistic_result=ballistic_result)
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

            telemetry_line = f"{dist_str}   |   {sig_str}"
            text_color = (255, 255, 255) if target_detected else (190, 190, 190)

            title_scale = 0.65
            sub_scale = 0.55
            thick = 2

            # Metne gore panel genisligi
            (w_status, _), _ = cv2.getTextSize(status_text, self.font, title_scale, thick)
            (w_telemetry, _), _ = cv2.getTextSize(telemetry_line, self.font, sub_scale, thick)
            (w_angle, _), _ = cv2.getTextSize(angle_err_text, self.font, sub_scale, thick)

            max_text_w = max(w_status, w_telemetry, w_angle)

            px1, py1 = 15, 12
            px2 = px1 + max_text_w + 24
            py2 = py1 + 92

            px2 = min(px2, w)
            py2 = min(py2, h)

            # Yari saydam arka plan
            roi = frame[py1:py2, px1:px2]
            black_layer = np.zeros_like(roi)
            frame[py1:py2, px1:px2] = cv2.addWeighted(roi, 0.40, black_layer, 0.60, 0)

            # Hedef varken sari, yokken gri cerceve
            box_border_color = (0, 191, 255) if target_detected else (90, 90, 90)
            cv2.rectangle(frame, (px1, py1), (px2, py2), box_border_color, 2)

            cv2.putText(frame, status_text, (27, 36), self.font, title_scale, status_color, thick, cv2.LINE_AA)
            cv2.putText(frame, telemetry_line, (27, 64), self.font, sub_scale, text_color, thick, cv2.LINE_AA)
            cv2.putText(frame, angle_err_text, (27, 88), self.font, sub_scale, text_color, thick, cv2.LINE_AA)

        # Sag ust FPS
        if not (is_locked or firing):
            fps_text = f"FPS: {fps}"
            (fps_w, _), _ = cv2.getTextSize(fps_text, self.font, 0.60, 2)
            fps_color = (0, 255, 0) if fps >= 25 else (0, 165, 255)
            cv2.putText(frame, fps_text, (w - fps_w - 20, 36), self.font, 0.60, fps_color, 2, cv2.LINE_AA)