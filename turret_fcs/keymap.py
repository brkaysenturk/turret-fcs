"""
================================================================================
MODUL: OPERATOR KLAVYE GIRDISI VE EYLEM YONETICISI (KEYMAP)
================================================================================
Bu modul, OpenCV ve Webots ortamlarinda operatorden gelen tus girdilerini
ayristirarak FCS, atis kontrol ve guvenlik mekanizmalarini tetikler.
================================================================================
"""

from typing import Tuple, Callable, Optional


def process_key_event(
    key_code: int,
    fcs,
    engagement,
    is_estop: bool,
    now: float,
    is_range_valid: bool,
    is_flywheel_idle: bool,
    log_event_cb: Optional[Callable[[str, Tuple[int, int, int]], None]] = None,
) -> Tuple[bool, bool]:
    """
    Operator tus kodunu isler.
    
    Returns:
        (should_exit: bool, new_is_estop: bool)
    """
    if key_code == -1:
        return False, is_estop

    def log(msg, color=(200, 210, 220)):
        if log_event_cb is not None:
            log_event_cb(msg, color)

    # 1. Cikis (Q veya ESC)
    if key_code in (ord("q"), ord("Q"), 27):
        log("[SYS] SESSION TERMINATED (EXIT)", color=(180, 180, 180))
        return True, is_estop

    # 2. Kontrol Modu Gecisi (M)
    if key_code in (ord("m"), ord("M")):
        if fcs.control_mode == "P":
            fcs.control_mode = "RL"
            if getattr(fcs, "rl_controller", None) is None:
                fcs._init_rl()
            mode_str = "RL CONTROLLER (AGILE PURSUIT)"
        else:
            fcs.control_mode = "P"
            mode_str = "P CONTROLLER (PRECISION LOCK)"
        log(f"[MODE] {mode_str}", color=(0, 215, 255))

    # 3. Akilli Devriye Modu (P)
    elif key_code in (ord("p"), ord("P")):
        fcs.patrol_enabled = not fcs.patrol_enabled
        if fcs.patrol_enabled:
            p_text = "ENABLED (FIGURE-8 SCAN)"
            p_color = (0, 255, 128)
        else:
            p_text = "DISABLED (MANUAL SEARCH)"
            p_color = (180, 180, 180)
            if hasattr(fcs, "patrol") and fcs.patrol is not None:
                fcs.patrol.reset()
            fcs.patrol_status = "STANDBY"
        log(f"[PATROL] {p_text}", color=p_color)

    # 4. Mandalli Acil Durdurma (Space)
    elif key_code == 32:
        new_estop = not is_estop
        if new_estop:
            fcs.patrol_enabled = False
            if hasattr(fcs, "patrol") and fcs.patrol is not None:
                fcs.patrol.reset()
            fcs.patrol_status = "STANDBY"
            fcs.reset()
            engagement.abort()
            log("[E-STOP] MOTORS HALTED & ZEROED (LATCHED)", color=(0, 0, 255))
        else:
            fcs.reset()
            log("[E-STOP] DISENGAGED - RESUMING FCS", color=(0, 255, 128))
        return False, new_estop

    # 5. Angajman ve Atis Tetikleme (Y)
    elif key_code in (ord("y"), ord("Y")):
        if not is_estop:
            ok, reason = engagement.trigger(
                now=now,
                is_locked=fcs.is_locked,
                is_range_valid=is_range_valid,
                is_flywheel_idle=is_flywheel_idle,
                hit_probability=getattr(fcs, "hit_probability", None),
            )
            if ok:
                log(f"[FIRE] {reason}", color=(0, 215, 255))
            else:
                log(f"[FIRE INHIBIT] {reason}", color=(0, 0, 255))

    # 6. Kilit Birakma / Manuel Devre Disi (N)
    elif key_code in (ord("n"), ord("N")):
        fcs.reset()
        engagement.abort()
        log("[LOCK] TARGET DISENGAGED BY OPERATOR", color=(180, 180, 180))

    return False, is_estop
