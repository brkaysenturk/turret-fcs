"""
================================================================================
MODUL: BASKET OPERATOR KLAVYE GIRDISI VE EYLEM YONETICISI (KEYMAP)
================================================================================
Bu modul, Basket Challenge arayuzunde operatorden gelen tus girdilerini
ayristirarak BasketManager uzerinden atis, emniyet (E-Stop) ve takipci
mekanizmalarini tetikler.
================================================================================
"""

import time
from turret_basket.basket_manager import BasketManager
from turret_basket.basket_hud import BasketHUD


def process_key_event(
    key_code: int,
    manager: BasketManager,
    hud: BasketHUD,
) -> bool:
    """
    Operator klavye girdisini isler.

    Args:
        key_code: OpenCV cv2.waitKey kod degeri
        manager: BasketManager orkestrator nesnesi
        hud: BasketHUD nesnesi

    Returns:
        should_exit: bool (True ise sistem kapatilir)
    """
    if key_code == -1 or key_code == 255:
        return False

    # 1. Cikis (Q veya ESC)
    if key_code in (ord("q"), ord("Q"), 27):
        hud.log_event("[SYS] SESSION TERMINATED (EXIT)", (180, 180, 180))
        return True

    # 2. Atis Baslat / Authorize Throw (Y)
    if key_code in (ord("y"), ord("Y")):
        manager.trigger_throw(now=time.time(), log_cb=hud.log_event)
        return False

    # 3. E-Stop Acil Durum Freni (SPACE veya E) - Mandalli
    if key_code in (32, ord("e"), ord("E")):
        manager.toggle_estop(log_cb=hud.log_event)
        return False

    # 4. Atisi Iptal Et (N)
    if key_code in (ord("n"), ord("N")):
        manager.abort_throw(log_cb=hud.log_event)
        return False

    # 5. Manuel Takipci ve Filtre Sifirlama (R)
    if key_code in (ord("r"), ord("R")):
        manager.reset(log_cb=hud.log_event)
        return False

    return False
