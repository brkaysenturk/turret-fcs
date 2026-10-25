"""
=============================================================================
FLYWHEEL CAGE BLASTER SIMULATOR (FF-180 6V 23,500 RPM)
=============================================================================
Classic Nerf Elite Dart dual counter-rotating FF-180 motor
flywheel launcher test controller with solenoid pusher & aim controls.

Controls:
  - [SPACE] or [F] : Fire Nerf Dart (solenoid push + physical projectile)
  - [A] / [LEFT]   : Rotate Blaster LEFT (Aim Left Can)
  - [D] / [RIGHT]  : Rotate Blaster RIGHT (Aim Right Can)
  - [S]            : Toggle Slow-Motion Mode (10 m/s Normal <-> 3 m/s Slow-Mo)
  - [R]            : Motors On/Off (23,500 RPM Rev Up/Down)
  - [C]            : Reset Target Cans on Bench
=============================================================================
"""

import math
import sys
from pathlib import Path

from controller import Supervisor

# Proje kokunu sys.path'e ekle (Webots dahili Python korumasi)
_project_root = Path(__file__).resolve().parents[3]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from turret_fcs.config import CONFIG


# --- CONSTANTS ---
TARGET_RPM = CONFIG.flywheel.target_speed_rad_s * 60.0 / (2.0 * math.pi)
TARGET_RAD_S = CONFIG.flywheel.target_speed_rad_s  # 23,500 RPM (CONFIG.flywheel)
DART_MASS_KG = 0.012      # 12.0 gram for stable ODE physics impact
FIRE_COOLDOWN = CONFIG.ballistics.recovery_time_s + CONFIG.ballistics.firing_pulse_s  # atis dongusu araligi
KEY_DEBOUNCE_SEC = 0.30   # seconds debounce for toggle keys (R, C, S)
YAW_STEP_RAD = 0.025      # radians per step when rotating left/right (~1.4 deg)
MAX_YAW_RAD = 0.55        # max horizontal rotation (~31 deg left/right)

# Solenoid timing and stroke
SOLENOID_PUSH_POS = 0.050     # 50 mm forward push into the flywheel gap
SOLENOID_RETRACT_POS = 0.0    # 0 mm resting position
SOLENOID_HOLD_DURATION = 0.06 # seconds held at maximum forward extension

# Canonical Bench Positions for the 3 target cans
CAN_BENCH = {
    "CAN_CENTER": [0.0, 2.0, 0.801],
    "CAN_LEFT":   [-0.16, 2.0, 0.801],
    "CAN_RIGHT":  [0.16, 2.0, 0.801],
}
CAN_DEFAULT_ROT = [0.0, 0.0, 1.0, 0.0]


def get_dart_rotation_vrml(yaw):
    """Compute axis-angle rotation for dart aligned with blaster yaw."""
    c_y = math.cos(yaw / 2.0)
    s_y = math.sin(yaw / 2.0)
    c_x = math.cos(math.pi / 4.0)
    s_x = math.sin(math.pi / 4.0)
    w = c_y * c_x
    x = c_y * s_x
    y = s_y * s_x
    z = s_y * c_x
    angle = 2.0 * math.acos(max(-1.0, min(1.0, w)))
    s = math.sin(angle / 2.0)
    if abs(s) < 1e-6:
        return "1 0 0 1.5708"
    return f"{x/s:.5f} {y/s:.5f} {z/s:.5f} {angle:.5f}"


def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    # --- KEYBOARD ---
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)

    # --- ROBOT ROTATION FIELD (AIMING) ---
    robot_node = robot.getSelf()
    robot_rot_field = robot_node.getField("rotation")
    blaster_yaw = 0.0  # radians (0 = center, + = left, - = right)

    # --- SPEED MODE ---
    slow_mo_active = False

    # --- FF-180 FLYWHEEL MOTORS ---
    left_motor = robot.getDevice("left_motor")
    right_motor = robot.getDevice("right_motor")
    if left_motor and right_motor:
        left_motor.setPosition(float("inf"))
        right_motor.setPosition(float("inf"))
        left_motor.setVelocity(TARGET_RAD_S)
        right_motor.setVelocity(-TARGET_RAD_S)
    motors_active = True

    # --- SOLENOID PUSHER ---
    solenoid = robot.getDevice("solenoid")
    solenoid_sensor = robot.getDevice("solenoid_sensor")
    if solenoid_sensor:
        solenoid_sensor.enable(timestep)
    if solenoid:
        solenoid.setPosition(SOLENOID_RETRACT_POS)

    # --- SCENE NODES (TARGET CANS) ---
    root_children = robot.getRoot().getField("children")

    target_cans = {}
    for name in ["CAN_CENTER", "CAN_LEFT", "CAN_RIGHT"]:
        node = robot.getFromDef(name)
        if node:
            target_cans[name] = {
                "node": node,
                "trans_field": node.getField("translation"),
                "rot_field": node.getField("rotation"),
                "knocked": False,
            }

    # --- STATE & TIMERS ---
    total_darts_fired = 0
    active_dart_nodes = []
    last_fire_time = 0.0
    last_r_time = 0.0
    last_c_time = 0.0
    last_s_time = 0.0
    last_reset_time = -1.0
    prev_key = -1

    # Solenoid state machine: "idle" -> "pushing" -> "holding" -> "retracting" -> "idle"
    solenoid_state = "idle"
    solenoid_hold_start = 0.0
    pending_dart_spawn = False

    print("\n" + "=" * 70)
    print(" [FLYWHEEL] FF-180 VOLANLI FIRLATICI TEST ORTAMI HAZIR!")
    print(f" Motorlar: 2x FF-180 @ 6V // Hedef Devir: {TARGET_RPM:,.0f} RPM ({TARGET_RAD_S:.1f} rad/s)")
    print(" Mermi Hizi: 10.0 m/s (FCS Balistik Normal) -- [S] ile Agir Cekim (3.0 m/s)")
    print("-" * 70)
    print(" KONTROLLER:")
    print("   [SPACE] / [F]   : DART ATESLE (Solenoid Itme + Fiziksel Mermi)")
    print("   [A] / [SOL OK]  : NAMLUSU SOLA CEVIR (Sol Kutuyu Hedefle)")
    print("   [D] / [SAG OK]  : NAMLUSU SAGA CEVIR (Sag Kutuyu Hedefle)")
    print("   [S]             : AGIR CEKIM MODU (10 m/s <-> 3 m/s Slow-Mo)")
    print("   [R]             : MOTORLARI AC / KAPA (Rev Up / Down)")
    print("   [C]             : HEDEF KUTULARI YENIDEN DIZ (Reset Targets)")
    print("=" * 70 + "\n")

    def reset_targets():
        nonlocal last_reset_time
        last_reset_time = robot.getTime()
        for name, data in target_cans.items():
            bench_pos = CAN_BENCH[name]
            data["trans_field"].setSFVec3f(bench_pos)
            data["rot_field"].setSFRotation(CAN_DEFAULT_ROT)
            try:
                data["node"].setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                data["node"].resetPhysics()
            except Exception:
                pass
            data["knocked"] = False

        # Clear all existing darts from the scene upon reset
        while active_dart_nodes:
            item = active_dart_nodes.pop(0)
            try:
                item["node"].remove()
            except Exception:
                pass

        print("[HEDEF] 3 Kutu masaya dizildi, yerdeki eski dartlar temizlendi!")

    def spawn_physical_dart():
        nonlocal total_darts_fired
        total_darts_fired += 1

        speed = 3.0 if slow_mo_active else 10.0
        dist = 1.88
        flight_t = dist / speed
        vz = 0.5 * 9.81 * flight_t

        # Calculate directional velocities based on blaster yaw
        vx = -speed * math.sin(blaster_yaw)
        vy = speed * math.cos(blaster_yaw)

        # Spawn offset along barrel
        barrel_offset = 0.12
        spawn_x = -barrel_offset * math.sin(blaster_yaw)
        spawn_y = barrel_offset * math.cos(blaster_yaw)
        spawn_z = 0.80

        rot_vrml = get_dart_rotation_vrml(blaster_yaw)

        dart_vrml = f"""
        DEF DART_{total_darts_fired} Solid {{
          translation {spawn_x:.4f} {spawn_y:.4f} {spawn_z:.4f}
          rotation {rot_vrml}
          children [
            Shape {{
              appearance PBRAppearance {{
                baseColor 0.12 0.38 0.95
                roughness 0.85
              }}
              geometry Cylinder {{
                radius 0.0064
                height 0.058
              }}
            }}
            Pose {{
              translation 0 0 -0.029
              children [
                Shape {{
                  appearance PBRAppearance {{
                    baseColor 1.0 0.38 0.0
                    roughness 0.4
                  }}
                  geometry Sphere {{
                    radius 0.0064
                  }}
                }}
              ]
            }}
          ]
          contactMaterial "dart_foam"
          boundingObject Capsule {{
            radius 0.0064
            height 0.058
          }}
          physics Physics {{
            density -1
            mass {DART_MASS_KG}
            centerOfMass [ 0 0 0 ]
            inertiaMatrix [
              0.000008 0.000008 0.000008
              0 0 0
            ]
          }}
        }}
        """

        root_children.importMFNodeFromString(-1, dart_vrml)
        dart_node = root_children.getMFNode(-1)
        t_field = dart_node.getField("translation")

        dart_node.setVelocity([vx, vy, vz, 0.0, 0.0, 0.0])

        yaw_deg = math.degrees(blaster_yaw)
        mode_str = "SLOW-MO" if slow_mo_active else "NORMAL"
        print(f"[ATES] DART #{total_darts_fired} FIRLADI! [{mode_str}] // "
              f"Aci: {yaw_deg:+.1f} deg | V={speed:.1f} m/s | Vx={vx:+.2f} Vy={vy:.2f} Vz={vz:.2f}")

        active_dart_nodes.append({
            "node": dart_node,
            "trans_field": t_field,
            "spawn_t": robot.getTime(),
            "hit": False,
        })

    def trigger_fire():
        nonlocal solenoid_state, pending_dart_spawn
        if solenoid:
            solenoid_state = "pushing"
            pending_dart_spawn = True
            solenoid.setPosition(SOLENOID_PUSH_POS)
            print("[SOLENOID] Plunger ileri uzaniyor -- dart volanlara itiliyor...")
        else:
            spawn_physical_dart()

    # Initial reset to guarantee clean state
    reset_targets()

    # --- MAIN SIMULATION LOOP ---
    while robot.step(timestep) != -1:
        sim_time = robot.getTime()
        key = keyboard.getKey()

        # Edge-detect
        is_new_press = (key != -1 and key != prev_key)
        prev_key = key

        # --- ROTATION CONTROLS (AIM LEFT / RIGHT) ---
        # A (65, 97) or Left Arrow (314) -> Turn Left (+yaw)
        if key in [65, 97, ord('a'), ord('A'), 314]:
            blaster_yaw = min(MAX_YAW_RAD, blaster_yaw + YAW_STEP_RAD)
            robot_rot_field.setSFRotation([0.0, 0.0, 1.0, blaster_yaw])

        # D (68, 100) or Right Arrow (316) -> Turn Right (-yaw)
        elif key in [68, 100, ord('d'), ord('D'), 316]:
            blaster_yaw = max(-MAX_YAW_RAD, blaster_yaw - YAW_STEP_RAD)
            robot_rot_field.setSFRotation([0.0, 0.0, 1.0, blaster_yaw])

        # --- ACTION CONTROLS ---
        # Fire (SPACE=32, F=70, 'f', 'F')
        if key in [32, 70, ord('f'), ord('F')]:
            if is_new_press or ((sim_time - last_fire_time) >= FIRE_COOLDOWN):
                if (sim_time - last_fire_time) >= FIRE_COOLDOWN:
                    if motors_active:
                        if solenoid_state == "idle":
                            trigger_fire()
                            last_fire_time = sim_time
                    else:
                        print("[UYARI] Volan motorlari calismiyor! Once [R] ile motorlari calistir.")

        # Slow-Motion Toggle (S=83, 's', 'S')
        elif key in [83, ord('s'), ord('S')]:
            if is_new_press and ((sim_time - last_s_time) >= KEY_DEBOUNCE_SEC):
                last_s_time = sim_time
                slow_mo_active = not slow_mo_active
                if slow_mo_active:
                    print("\n>>> [MOD] AGIR CEKIM (SLOW-MO) AKTIF: Mermi hizi 3.0 m/s <<<")
                else:
                    print("\n>>> [MOD] NORMAL FCS MODU AKTIF: Mermi hizi 10.0 m/s <<<")

        # Motor toggle (R=82, 'r', 'R')
        elif key in [82, ord('r'), ord('R')]:
            if is_new_press and ((sim_time - last_r_time) >= KEY_DEBOUNCE_SEC):
                last_r_time = sim_time
                motors_active = not motors_active
                if left_motor and right_motor:
                    if motors_active:
                        left_motor.setVelocity(TARGET_RAD_S)
                        right_motor.setVelocity(-TARGET_RAD_S)
                        print(f"[MOTOR] FF-180 DEVREDE // {TARGET_RPM:,.0f} RPM ({TARGET_RAD_S:.1f} rad/s)")
                    else:
                        left_motor.setVelocity(0.0)
                        right_motor.setVelocity(0.0)
                        print("[MOTOR] Motorlar durduruldu (0 RPM).")

        # Reset targets (C=67, 'c', 'C')
        elif key in [67, ord('c'), ord('C')]:
            if is_new_press and ((sim_time - last_c_time) >= KEY_DEBOUNCE_SEC):
                last_c_time = sim_time
                reset_targets()

        # --- SOLENOID STATE MACHINE ---
        if solenoid and solenoid_sensor:
            if solenoid_state == "pushing":
                sol_pos = solenoid_sensor.getValue()
                if sol_pos >= (SOLENOID_PUSH_POS * 0.80):
                    solenoid_state = "holding"
                    solenoid_hold_start = sim_time
                    if pending_dart_spawn:
                        spawn_physical_dart()
                        pending_dart_spawn = False

            elif solenoid_state == "holding":
                if (sim_time - solenoid_hold_start) >= SOLENOID_HOLD_DURATION:
                    solenoid.setPosition(SOLENOID_RETRACT_POS)
                    solenoid_state = "retracting"

            elif solenoid_state == "retracting":
                sol_pos = solenoid_sensor.getValue()
                if sol_pos <= 0.005:
                    solenoid_state = "idle"
        else:
            if pending_dart_spawn:
                spawn_physical_dart()
                pending_dart_spawn = False
            solenoid_state = "idle"

        # --- IMPACT & COLLISION DETECTION MONITOR ---
        for dart_info in active_dart_nodes:
            if dart_info["hit"]:
                continue
            try:
                dart_pos = dart_info["trans_field"].getSFVec3f()
            except Exception:
                continue

            for can_name, can_data in target_cans.items():
                can_pos = can_data["trans_field"].getSFVec3f()
                dx = dart_pos[0] - can_pos[0]
                dy = dart_pos[1] - can_pos[1]
                dz = dart_pos[2] - can_pos[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                if dist < 0.08:
                    dart_info["hit"] = True
                    print(f"\n>>> 🎯 [ISABET / HIT!] Dart {can_name} kutusuna carpti! (Mesafe: {dist*100:.1f} cm) 🎯 <<<")
                    break

        # --- KNOCKDOWN DETECTION MONITOR ---
        # Grace period: ignore during the first 0.4s after reset
        if (sim_time - last_reset_time) > 0.4:
            for can_name, can_data in target_cans.items():
                if not can_data["knocked"]:
                    can_pos = can_data["trans_field"].getSFVec3f()
                    # If can fell below bench top (0.73m) or moved > 10cm along Y
                    if can_pos[2] < 0.70 or abs(can_pos[1] - 2.0) > 0.10:
                        can_data["knocked"] = True
                        print(f">>> 💥 [DEVRILDI / KNOCKED DOWN!] {can_name} devrildi ve uctu! 💥 <<<\n")

        # --- CLEANUP: remove darts older than 6 seconds ---
        while active_dart_nodes and (sim_time - active_dart_nodes[0]["spawn_t"]) > 6.0:
            old_item = active_dart_nodes.pop(0)
            try:
                old_item["node"].remove()
            except Exception:
                pass


if __name__ == "__main__":
    main()
