"""
Bu modul hedefin 3D uzayda (X ve Z) salinim hareketini yonetir.
Ayni zamanda fiziksel Nerf darti balona carptiginda balonu patlatir (POP),
2 saniye sonra yeni balon otomatik spawn olur.
"""

import math
from controller import Supervisor

# Supervisor yetkisiyle basla
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Balon dugumunu ve konum (translation) alanini al
balloon_node = robot.getSelf()
trans_field = balloon_node.getField("translation")

# Sahne kok dugumu (dart tespiti icin)
root_children = robot.getRoot().getField("children")

# Balonun dunyadaki ilk konumunu [x, y, z] oku
initial_pos = trans_field.getSFVec3f()
start_x = initial_pos[0]
fixed_y = initial_pos[1]
fixed_z = initial_pos[2]

# --- HAREKET PARAMETRELERI (3D Dinamik Balon Hareketi) ---
# Hakem sistemi bu degiskenleri dosyadan dinamik okur, isimleri koruyun:
AMPLITUDE_X = 0.7
SPEED_X = 1.3

AMPLITUDE_Z = 0.2
SPEED_Z = 1.8

# --- PATLAMA VE RESPAWN PARAMETRELERI ---
is_popped = False
pop_time = 0.0
RESPAWN_DELAY_SEC = 2.0   # Patladiktan 2 saniye sonra yeni balon cikar
BALLOON_HIT_RADIUS = 0.15 # Gercekci balon yaricapi (15 cm yaricap = 30 cm standart parti balonu capi)

sim_time = 0.0

while robot.step(timestep) != -1:
    sim_time += timestep / 1000.0

    # Hem X (Yatay) hem Z (Dikey) ayni anda dinamik olarak degisir:
    new_x = start_x + AMPLITUDE_X * math.sin(sim_time * SPEED_X)
    new_z = fixed_z + AMPLITUDE_Z * math.sin(sim_time * SPEED_Z)

    # Balonun 3D merkez noktasi (Robot rotasyonu ve boundingObject ofsetleri dahil)
    balloon_center_x = new_x - 0.02
    balloon_center_y = fixed_y - 0.255
    balloon_center_z = new_z + 0.155

    # --- PATLAMA DURUMU YONETIMI ---
    if is_popped:
        # Balon patlakken yerin dibine saklanir.
        trans_field.setSFVec3f([0.0, 0.0, -20.0])

        if (sim_time - pop_time) >= RESPAWN_DELAY_SEC:
            is_popped = False
            trans_field.setSFVec3f([new_x, fixed_y, new_z])
            print("[BALON] Yeni hedef balon belirdi! Takip devam ediyor.")
    else:
        # Normal ucus / salinim
        trans_field.setSFVec3f([new_x, fixed_y, new_z])

        # Balon isabet sinyali alinca patlar.
        cmd = robot.getCustomData()
        if cmd == "POP":
            is_popped = True
            pop_time = sim_time
            robot.setCustomData("")
            print("\n" + "=" * 65)
            print(">>> 💥💥💥 [BALON PATLADI / TARGET POPPED!] 💥💥💥 <<<")
            print(">>> Hakem Sistemi (FCS Referee) ISABET onayladi! <<<")
            print(">>> (2 saniye icinde yeni hedef spawn edilecek) <<<")
            print("=" * 65 + "\n")