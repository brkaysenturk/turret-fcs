"""
================================================================================
MODUL: BASKET CHALLENGE SISTEM ORKESTRATORU (BASKET MANAGER)
================================================================================
turret_fcs projesindeki FCSManager mimarisini birebir ornek alan bu yonetici sinif,
Hedef Tespiti (YOLOv8), LiDAR Menzil Kapilama, Kilit Kararliligi, Balistik AI Cikarimi
ve Atis Angajman Durum Makinesini tek bir merkezde koordine eder.
================================================================================
"""

from dataclasses import dataclass
from typing import Optional, Tuple, Callable
import numpy as np

from turret_basket.config import CONFIG
from turret_basket.protocol import STATE_TRACKING, STATE_LOCKED, STATE_ESTOP
from turret_basket.yolo.cup_detector import CupDetector, CupDetection
from turret_basket.rl_tracker import RLTracker
from turret_basket.rl_ballistics import RLBallistics
from turret_basket.engagement import BasketEngagement, EngagementStep
from turret_basket.basket_comm import FCSCommunicator


@dataclass
class BasketStatus:
    """HUD ve istemciler icin anlik sistem durumu."""
    cup_detected: bool
    bbox: Optional[Tuple[int, int, int, int]]
    confidence: float
    distance_m: float
    err_pan_deg: float
    err_tilt_deg: float
    is_locked: bool
    firing: bool
    is_estop: bool
    lock_ratio: float
    speed_mps: float
    tilt_offset_deg: float


class BasketManager:
    """Kupa basket gorevinin tum karar ve kontrol dongusunu yoneten orkestrator."""

    def __init__(
        self,
        comm: Optional[FCSCommunicator] = None,
        detector: Optional[CupDetector] = None,
        tracker: Optional[RLTracker] = None,
        ballistics: Optional[RLBallistics] = None,
        engagement: Optional[BasketEngagement] = None,
    ):
        self.comm = comm or FCSCommunicator(
            port=CONFIG.hardware.manual_port,
            baudrate=CONFIG.hardware.baudrate
        )
        self.detector = detector or CupDetector()
        self.tracker = tracker or RLTracker()
        self.ballistics = ballistics or RLBallistics()
        self.engagement = engagement or BasketEngagement()

        # Sistem durum degiskenleri
        self.is_estop = False
        self.lock_counter = 0
        self.MAX_LOCK_FRAMES = CONFIG.cup.lock_threshold_frames
        self.is_locked = False
        self.lock_ratio = 0.0

        self.last_valid_dist_m = CONFIG.cup.nominal_distance_m
        self.dist_m = CONFIG.cup.nominal_distance_m
        self.target_speed_mps = 4.4
        self.target_tilt_offset = 21.8

    def trigger_throw(
        self,
        now: float,
        log_cb: Optional[Callable[[str, Tuple[int, int, int]], None]] = None,
    ) -> bool:
        """Kilitli durumdaysa atis sekansini baslatir."""
        if self.is_estop:
            if log_cb:
                log_cb("[WARN] CANNOT FIRE - E-STOP ACTIVE", (0, 60, 255))
            return False

        if not self.is_locked:
            if log_cb:
                log_cb("[WARN] CANNOT FIRE - CUP NOT LOCKED", (0, 60, 255))
            return False

        current_tilt = self.comm.tilt_angle if self.comm else 0.0
        super_elev = self.ballistics.compute_super_elevation(self.target_tilt_offset)

        ok, reason = self.engagement.trigger(
            now=now,
            tilt_at_lock=current_tilt,
            fire_speed=self.target_speed_mps,
            fire_tilt=self.target_tilt_offset,
            super_elevation=super_elev,
        )
        if ok and log_cb:
            log_cb(f"[AUTH] {reason}", (0, 191, 255))
        return ok

    def abort_throw(
        self,
        log_cb: Optional[Callable[[str, Tuple[int, int, int]], None]] = None,
    ):
        """Atisi derhal iptal eder."""
        if self.engagement.is_active:
            self.engagement.abort()
            if log_cb:
                log_cb("[ABORT] THROW CANCELLED", (0, 60, 255))

    def toggle_estop(
        self,
        log_cb: Optional[Callable[[str, Tuple[int, int, int]], None]] = None,
    ) -> bool:
        """E-Stop mandalini degistirir ve guvenlik komutunu iletir."""
        self.is_estop = not self.is_estop
        if self.is_estop:
            self.abort_throw()
            if self.comm:
                self.comm.send_command(0.0, 0.0, state=STATE_ESTOP)
            if log_cb:
                log_cb("[E-STOP] MOTORS HALTED & ZEROED (LATCHED)", (0, 60, 255))
        else:
            if self.comm:
                self.comm.send_command(0.0, 0.0, state=STATE_TRACKING)
            if log_cb:
                log_cb("[E-STOP] DISENGAGED - RESUMING FCS", (0, 255, 128))
        return self.is_estop

    def reset(
        self,
        log_cb: Optional[Callable[[str, Tuple[int, int, int]], None]] = None,
    ):
        """Takipci ve kilit sayacini sifirlar."""
        self.tracker.reset()
        self.lock_counter = 0
        self.is_locked = False
        self.lock_ratio = 0.0
        self.abort_throw()
        if log_cb:
            log_cb("[TRACKER] SYSTEM FILTERS FLUSHED", (255, 215, 0))

    def step(
        self,
        frame: np.ndarray,
        dt: float,
        now: float,
        on_event_cb: Optional[Callable[[str, Tuple[int, int, int]], None]] = None,
        on_shot_cb: Optional[Callable[[bool], None]] = None,
    ) -> BasketStatus:
        """Tek bir cevrim adimini calistirir."""
        h, w = frame.shape[:2]
        cx_screen, cy_screen = w // 2, h // 2

        # 1. Kupa Tespiti (YOLOv8)
        det: CupDetection = self.detector.detect(frame)

        # 2. Telemetri Oku ve Lidar Menzil Kapilama
        if self.comm:
            raw_dist_m = (self.comm.distance / 100.0) if self.comm.distance > 0 else 0.0
            if CONFIG.cup.lidar_range_min_m <= raw_dist_m <= CONFIG.cup.lidar_range_max_m:
                self.dist_m = raw_dist_m
                self.last_valid_dist_m = self.dist_m
            else:
                self.dist_m = self.last_valid_dist_m
            current_tilt = self.comm.tilt_angle
        else:
            current_tilt = 0.0

        # 3. Kilit Kararliligi Kontrolu
        if det.detected:
            is_close = (
                abs(det.err_pan_deg) < CONFIG.cup.lock_tolerance_pan_deg
                and abs(det.err_tilt_deg) < CONFIG.cup.lock_tolerance_tilt_deg
            )
            if is_close:
                self.lock_counter = min(self.MAX_LOCK_FRAMES, self.lock_counter + 1)
            else:
                self.lock_counter = max(0, self.lock_counter - 1)
        else:
            self.lock_counter = max(0, self.lock_counter - 2)

        self.is_locked = (self.lock_counter >= self.MAX_LOCK_FRAMES)
        self.lock_ratio = float(self.lock_counter) / float(self.MAX_LOCK_FRAMES)

        # 4. Model 2 (RLBallistics) Balistik Cikarim
        if not self.engagement.is_active:
            launch_params = self.ballistics.compute_launch_parameters(self.dist_m)
            self.target_tilt_offset = launch_params.tilt_offset_deg
            self.target_speed_mps = launch_params.speed_mps

        # 5. Hareket ve Atis Yonetimi
        if self.is_estop:
            if self.comm:
                self.comm.send_command(0.0, 0.0, state=STATE_ESTOP)

        elif self.engagement.is_active:
            step: EngagementStep = self.engagement.update(now=now, current_tilt=current_tilt)

            if step.just_fired:
                if on_event_cb:
                    on_event_cb(
                        f"[THROW] DART FIRED ({step.active_fire_speed:.1f} m/s @ +{step.applied_super_elevation:.1f} deg)",
                        (0, 255, 128)
                    )
                if on_shot_cb:
                    on_shot_cb(True)

            if step.is_finished:
                self.tracker.reset()
                if on_event_cb:
                    on_event_cb("[SYS] RECOVERED - READY FOR NEXT THROW", (255, 215, 0))

            if self.comm:
                self.comm.send_command(0.0, step.step_tilt, state=step.cmd_state)

        else:
            # Normal Takip Modu
            if det.detected:
                cx, cy = det.center_px
                d_pan, d_tilt, _ = self.tracker.compute(
                    cx=float(cx),
                    cy=float(cy),
                    center_x=float(cx_screen),
                    center_y=float(cy_screen),
                    screen_w=float(w),
                    screen_h=float(h),
                    distance_m=float(self.dist_m),
                    dt=float(dt)
                )
                cmd_state = STATE_LOCKED if self.is_locked else STATE_TRACKING
                eff_pan = -d_pan if CONFIG.hardware.pan_invert else d_pan
                eff_tilt = -d_tilt if CONFIG.hardware.tilt_invert else d_tilt
                if self.comm:
                    self.comm.send_command(eff_pan, eff_tilt, state=cmd_state)
            else:
                self.tracker.on_target_lost()
                if self.comm:
                    self.comm.send_command(0.0, 0.0, state=STATE_TRACKING)

        disp_speed = self.engagement.active_fire_speed if self.engagement.is_active else self.target_speed_mps
        disp_tilt = self.engagement.active_fire_tilt if self.engagement.is_active else self.target_tilt_offset

        return BasketStatus(
            cup_detected=det.detected,
            bbox=det.bbox if det.detected else None,
            confidence=det.confidence if det.detected else 0.0,
            distance_m=self.dist_m,
            err_pan_deg=det.err_pan_deg if det.detected else 0.0,
            err_tilt_deg=det.err_tilt_deg if det.detected else 0.0,
            is_locked=self.is_locked,
            firing=self.engagement.is_active,
            is_estop=self.is_estop,
            lock_ratio=self.lock_ratio,
            speed_mps=disp_speed,
            tilt_offset_deg=disp_tilt,
        )

