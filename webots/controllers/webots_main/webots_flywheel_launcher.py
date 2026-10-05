import math


def rot_matrix_to_axis_angle(R):
    """3x3 donus matrisini Webots eksen-aci [x, y, z, angle] formatina cevirir."""
    trace = R[0][0] + R[1][1] + R[2][2]
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    theta = math.acos(cos_theta)
    if abs(theta) < 1e-4:
        return "0 0 1 0"
    sin_theta = math.sin(theta)
    x = (R[2][1] - R[1][2]) / (2.0 * sin_theta)
    y = (R[0][2] - R[2][0]) / (2.0 * sin_theta)
    z = (R[1][0] - R[0][1]) / (2.0 * sin_theta)
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-6:
        return "0 0 1 0"
    return f"{x/norm:.4f} {y/norm:.4f} {z/norm:.4f} {theta:.4f}"


class WebotsFlywheelLauncher:
    """Cift FF-180 volan motoru, solenoid itici ve fiziksel Nerf dart uretim yoneticisi."""

    def __init__(self, robot):
        self.robot = robot
        self.TARGET_RAD_S = 2460.9  # 23,500 RPM
        self.MUZZLE_VELOCITY = 10.0 # m/s

        # Volan motorlari
        self.flywheel_left = robot.getDevice('flywheel_left')
        self.flywheel_right = robot.getDevice('flywheel_right')
        if self.flywheel_left and self.flywheel_right:
            self.flywheel_left.setPosition(float('inf'))
            self.flywheel_right.setPosition(float('inf'))
            self.flywheel_left.setVelocity(self.TARGET_RAD_S)
            self.flywheel_right.setVelocity(-self.TARGET_RAD_S)

        # Solenoid itici ve konum sensoru
        self.solenoid = robot.getDevice('solenoid')
        self.solenoid_sensor = robot.getDevice('solenoid_sensor')
        timestep = int(robot.getBasicTimeStep())
        if self.solenoid_sensor:
            self.solenoid_sensor.enable(timestep)
        if self.solenoid:
            self.solenoid.setPosition(0.0)

        self.root_children = robot.getRoot().getField("children")
        self.kafa_node = robot.getFromDef("KAFA_BLOGU")

        self.total_darts_fired = 0
        self.active_dart_nodes = []
        self.solenoid_state = "idle"
        self.solenoid_hold_start = 0.0
        self.pending_dart_spawn = False

    def trigger(self):
        """Atis tetiklendiginde solenoidi ileri iter."""
        if self.solenoid:
            self.solenoid_state = "pushing"
            self.pending_dart_spawn = True
            self.solenoid.setPosition(0.05)
        else:
            self._spawn_dart()

    def update(self, sim_time):
        """Solenoid durum makinesini ve eski dartlarin temizligini yonetir."""
        if self.solenoid and self.solenoid_sensor:
            if self.solenoid_state == "pushing":
                if self.solenoid_sensor.getValue() >= 0.040:
                    self.solenoid_state = "holding"
                    self.solenoid_hold_start = sim_time
                    if self.pending_dart_spawn:
                        self._spawn_dart()
                        self.pending_dart_spawn = False
            elif self.solenoid_state == "holding":
                if (sim_time - self.solenoid_hold_start) >= 0.06:
                    self.solenoid.setPosition(0.0)
                    self.solenoid_state = "retracting"
            elif self.solenoid_state == "retracting":
                if self.solenoid_sensor.getValue() <= 0.005:
                    self.solenoid_state = "idle"

        # 2.5 saniyeden eski dartlari sahneden kaldir.
        while self.active_dart_nodes and (sim_time - self.active_dart_nodes[0][1]) > 2.5:
            old_node, _ = self.active_dart_nodes.pop(0)
            try:
                old_node.remove()
            except Exception:
                pass

    def _spawn_dart(self):
        """Kafa blogunun baktigi yonde fiziksel 10 m/s hizli 12g Nerf darti uretir."""
        self.total_darts_fired += 1

        head_pos = self.kafa_node.getPosition() if self.kafa_node else [0.0, 0.0, 0.828]
        head_rot = self.kafa_node.getOrientation() if self.kafa_node else [1, 0, 0, 0, 1, 0, 0, 0, 1]

        fx_dir = head_rot[1]
        fy_dir = head_rot[4]
        fz_dir = head_rot[7]

        ux_dir = head_rot[2]
        uy_dir = head_rot[5]
        uz_dir = head_rot[8]

        # Namlu ucunun agzi: Kafa merkezinden local Y = +0.09 m, Z = 0.0 m
        spawn_x = head_pos[0] + 0.09 * fx_dir
        spawn_y = head_pos[1] + 0.09 * fy_dir
        spawn_z = head_pos[2] + 0.09 * fz_dir

        vx = self.MUZZLE_VELOCITY * fx_dir
        vy = self.MUZZLE_VELOCITY * fy_dir
        vz = self.MUZZLE_VELOCITY * fz_dir

        c1x = uy_dir * (-fz_dir) - uz_dir * (-fy_dir)
        c1y = uz_dir * (-fx_dir) - ux_dir * (-fz_dir)
        c1z = ux_dir * (-fy_dir) - uy_dir * (-fx_dir)

        R_dart = [
            [c1x, ux_dir, -fx_dir],
            [c1y, uy_dir, -fy_dir],
            [c1z, uz_dir, -fz_dir]
        ]
        rot_vrml = rot_matrix_to_axis_angle(R_dart)

        dart_vrml = f"""
        DEF DART_{self.total_darts_fired} Solid {{
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
            mass 0.012
            centerOfMass [ 0 0 0 ]
            inertiaMatrix [
              0.000008 0.000008 0.000008
              0 0 0
            ]
          }}
        }}
        """

        self.root_children.importMFNodeFromString(-1, dart_vrml)
        dart_node = self.root_children.getMFNode(-1)
        dart_node.setVelocity([vx, vy, vz, 0.0, 0.0, 0.0])
        self.active_dart_nodes.append((dart_node, self.robot.getTime()))

        print(f"\n[ATES] FIZIKSEL NERF DARTI #{self.total_darts_fired} FIRLADI! // "
              f"V=10.0 m/s | Vx={vx:+.2f} Vy={vy:+.2f} Vz={vz:+.2f} | Volan: 23,500 RPM")