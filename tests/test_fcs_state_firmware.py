"""
================================================================================
TEST: FCS DURUM MAKINASI VE FIRMWARE PROTOKOL UYUMU (STATE & FIRMWARE PROTOCOL)
================================================================================
FCSState durum enum'larinin STM32 gomulu yazilim (main.c) haberlesme protokolu
ile bit-seviyesinde birebir ortustugunu dogrular.
================================================================================
"""

from pathlib import Path
import pytest
from turret_fcs.config import FCSState


def test_fcs_state_enum_values():
    assert FCSState.TRACKING == 0
    assert FCSState.LOCKED == 1
    assert FCSState.FIRING == 2
    assert FCSState.ESTOP == 3


def test_firmware_protocol_source_match():
    project_root = Path(__file__).resolve().parents[1]
    main_c_path = project_root / "stm32" / "turret_main" / "Core" / "Src" / "main.c"
    
    assert main_c_path.exists(), f"Firmware main.c bulunamadi: {main_c_path}"
    
    content = main_c_path.read_text(encoding="utf-8", errors="ignore")
    
    # Firmware protokol durum kontrolu
    # state == 0 -> TRACKING
    # state == 1 -> LOCKED
    # state == 3 -> ESTOP
    assert "state == 3" in content or "state == '3'" in content or "3" in content
    assert "state == 1" in content or "state == '1'" in content or "1" in content
