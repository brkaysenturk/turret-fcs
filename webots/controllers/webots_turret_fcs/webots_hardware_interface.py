"""
================================================================================
MODUL: WEBOTS DONANIM VE SIMULASYON ARAYUZU (WEBOTS HARDWARE INTERFACE - v2.0)
================================================================================
Bu modul; Webots robot supervisor, motorlar, enkoderler, C270 kamera ve TF-Luna
sensorlerini yoneten yuksek sadakatli (High-Fidelity) donanim soyutlama katmanidir (HAL).

v2.0 YUKSEK SADAKATLI (SIM-TO-REAL) FIZIKSEL MODELLER:
- 64 ms (2 Kare) Tasiyici Gecikmesi: 2 karelik kuyruk (deque) ile C270 sensor pozu + USB + YOLO gecikmesi.
- 45 ms Servo Dinamigi: MG996R motor zaman sabiti (tau = 0.045 s) 1. derece gecikme modeli.
- 1.20 Derece Disli Boslugu (Backlash): Yon degisimlerinde mekanik histerezis olu bolgesi.
- 0.10 Derece STM32 UART Kuantalamasi: Dijital paket cozunurluk benzetimi.
- Tesis Modeli: Anti-windup inkremental aci integrasyonu (cmd = curr + delta).
- Hiz Limiti: 1.5 derece/kare (30 FPS'te 45 derece/s, CONFIG.control.max_output_deg).
- Mekanik Hareket Alani: Pan [-75.0, 75.0] derece, Tilt [-25.0, 25.0] derece.
- Zaman Tabani: Saf simulasyon zamani (robot.getTime()), sifir wall-clock bagimliligi.
================================================================================
"""

import collections
import math
import os
import cv2
import numpy as np
from controller import Supervisor

from turret_fcs.config import CONFIG


class WebotsHardwareInterface:
    """
    Webots robot donanimlarini (motorlar, sensorler, kamera, LiDAR) yoneten
    ve uretim v2.0 yuksek sadakatli fiziksel parametreleriyle calisan donanim arayuzu.
    """

    def __init__(self):
        self.robot = Supervisor()
        basic_step = int(self.robot.getBasicTimeStep())
        # 30 FPS kamera dongusu ile uyumlu 32 ms timestep (8 x 4 ms basicTimeStep)
        if basic_step > 0:
            steps_32ms = max(1, round(32.0 / basic_step))
            self.timestep = steps_32ms * basic_step
        else:
            self.timestep = 32

        self.dt = self.timestep / 1000.0

        # Taret gimbal motorlari: 1.5 derece/kare (30 FPS'te 45 derece/s)
        self.pan_motor = self.robot.getDevice('pan_motor')
        self.tilt_motor = self.robot.getDevice('tilt_motor')
        self.max_step_deg = CONFIG.control.max_output_deg
        self.max_velocity = math.radians(self.max_step_deg) / max(1e-6, self.dt)
        if self.pan_motor:
            self.pan_motor.setVelocity(self.max_velocity)
        if self.tilt_motor:
            self.tilt_motor.setVelocity(self.max_velocity)

        # Aci sensorleri (Enkoderler)
        self.pan_sensor = self.robot.getDevice('pan_sensor')
        self.tilt_sensor = self.robot.getDevice('tilt_sensor')
        if self.pan_sensor:
            self.pan_sensor.enable(self.timestep)
        if self.tilt_sensor:
            self.tilt_sensor.enable(self.timestep)

        # Logitech C270 Kamera (1280x720, 48.8 deg FOV)
        self.camera = self.robot.getDevice('c270')
        if self.camera:
            self.camera.enable(self.timestep)
            self.w_cam = self.camera.getWidth()
            self.h_cam = self.camera.getHeight()
        else:
            self.w_cam = CONFIG.optics.default_width
            self.h_cam = CONFIG.optics.default_height

        # Kamera/algilama tasiyici gecikmesi: Tam 3 karelik donanim gecikmesi (3 * timestep = 96 ms ~ 90 ms).
        # SSOT: CONFIG.ballistics.system_latency_s (90.0 ms @ 30 FPS, Webots'ta 3 * dt = 96 ms).
        default_delay = 3 * self.dt
        self.observation_delay_s = float(
            os.environ.get("WEBOTS_OBS_DELAY_S", str(default_delay))
        )
        self.observation_delay_s = max(0.03, min(0.12, self.observation_delay_s))
        self.observation_queue = collections.deque(maxlen=8)

        # TF-Luna LiDAR
        self.lidar = self.robot.getDevice('tf_luna')
        if self.lidar:
            self.lidar.enable(self.timestep)

        # Inkremental komut konumu biriktiricisi
        self.cmd_pan_deg = 0.0
        self.cmd_tilt_deg = 0.0

        # Yuksek Sadakatli Servo Tesis Parametreleri
        self.servo_tau = CONFIG.mount.servo_tau  # CONFIG SSOT motor zaman sabiti
        self.servo_lag_pan_deg = 0.0
        self.servo_lag_tilt_deg = 0.0
        self.backlash_deg = CONFIG.actuator.backlash_deg  # CONFIG SSOT mekanik disli boslugu
        self.backlash_pan_out = 0.0
        self.backlash_tilt_out = 0.0

        # Donanim limitleri (CONFIG SSOT)
        self.pan_min = CONFIG.mount.pan_limit_min
        self.pan_max = CONFIG.mount.pan_limit_max
        self.tilt_min = CONFIG.mount.tilt_limit_min
        self.tilt_max = CONFIG.mount.tilt_limit_max
        self.pan_invert = CONFIG.mount.pan_invert
        self.tilt_invert = CONFIG.mount.tilt_invert

    def step(self):
        """Simulasyonu bir adim (32 ms) ilerletir."""
        return self.robot.step(self.timestep)

    def get_time(self):
        """Webots simulasyon zamanini (saniye) dondurur (Saf sim_t)."""
        return self.robot.getTime()

    def _read_frame(self):
        """Kameradan ham BGR karesini okur (gecikme uygulamadan)."""
        if not self.camera:
            return None
        img_raw = self.camera.getImage()
        if img_raw is None:
            return None
        frame = np.frombuffer(img_raw, np.uint8).reshape((self.h_cam, self.w_cam, 4))
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    def get_observation(self, delay_s=None):
        """
        Kamera karesini, servo pozunu ve LiDAR mesafesini AYNI zaman damgasiyla
        kuyruga alir; en az delay_s yasinda olan en yeni kaydi dondurur.
        Boylece goruntu ile pose/mesafe ayni anda gecikmis olur (80 ms).
        """
        if delay_s is None:
            delay_s = self.observation_delay_s

        sim_t = self.get_time()
        frame = self._read_frame()
        if frame is None:
            return None, 0.0, 0.0, 0, 0

        pan_deg, tilt_deg = self.get_motor_positions_deg()
        dist_cm, strength = self.get_distance_and_strength()
        self.observation_queue.append((sim_t, frame, pan_deg, tilt_deg, dist_cm, strength))

        # En az delay_s yasinda olan en yeni kaydi sec; henuz yoksa en eskiyi kullan.
        # Sayisal yuvarlama payi icin 0.005 s tolerans eklenir (tam 2 kareyi / 64 ms yakalar).
        chosen = self.observation_queue[0]
        tolerance = 0.005
        for sample in reversed(self.observation_queue):
            if (sim_t - sample[0]) >= (delay_s - tolerance):
                chosen = sample
                break

        return chosen[1], chosen[2], chosen[3], chosen[4], chosen[5]

    def get_distance_and_strength(self):
        """LiDAR'dan santimetre cinsinden mesafe ve sinyal gucunu okur."""
        if not self.lidar:
            return 0, 0
        dist_val = self.lidar.getValue()
        dist_cm = (
            int(dist_val * 100.0)
            if (CONFIG.lidar.min_distance_m <= dist_val <= CONFIG.lidar.max_distance_m)
            else 0
        )
        strength = 1500 if dist_cm > 0 else 0
        return dist_cm, strength

    def get_motor_positions_deg(self):
        """Guncel pan ve tilt acilarini derece olarak dondurur (FCS koordinat sistemi)."""
        raw_p_rad = self.pan_sensor.getValue() if self.pan_sensor else 0.0
        raw_t_rad = self.tilt_sensor.getValue() if self.tilt_sensor else 0.0

        p_deg = math.degrees(raw_p_rad)
        t_deg = math.degrees(raw_t_rad)

        eff_p = -p_deg if self.pan_invert else p_deg
        eff_t = -t_deg if self.tilt_invert else t_deg
        return float(eff_p), float(eff_t)

    def apply_delta_commands(self, delta_pan_deg, delta_tilt_deg):
        """
        STM32 inkremental komut biriktiricisini birebir taklit eder:
        cmd = cmd + delta; ardindan 45 ms servo lag, 1.2 derece backlash ve
        0.1 derece kuantalama ile fiziksel servo acisini Webots motoruna yazar.
        """
        # 1. Uretimdeki gibi komut biriktirici (firmware current_pan += d_pan)
        self.cmd_pan_deg += float(delta_pan_deg)
        self.cmd_tilt_deg += float(delta_tilt_deg)

        # 2. Mekanik emniyet limitleri
        self.cmd_pan_deg = float(np.clip(self.cmd_pan_deg, self.pan_min, self.pan_max))
        self.cmd_tilt_deg = float(np.clip(self.cmd_tilt_deg, self.tilt_min, self.tilt_max))

        # 3. 0.1 derece STM32 UART kuantalamasi (SSOT)
        q_step = float(getattr(CONFIG.actuator, "servo_deadband_deg", 0.10))
        if q_step > 0.0:
            q_pan = round(self.cmd_pan_deg / q_step) * q_step
            q_tilt = round(self.cmd_tilt_deg / q_step) * q_step
        else:
            q_pan, q_tilt = self.cmd_pan_deg, self.cmd_tilt_deg

        # 4. 30 ms servo dinamiği (1. derece gecikme)
        alpha_servo = min(1.0, max(0.01, self.dt / max(0.001, self.servo_tau)))
        self.servo_lag_pan_deg += alpha_servo * (q_pan - self.servo_lag_pan_deg)
        self.servo_lag_tilt_deg += alpha_servo * (q_tilt - self.servo_lag_tilt_deg)

        # 5. 1.2 derece mekanik disli boslugu (backlash)
        act_pan_deg = self._apply_backlash(self.servo_lag_pan_deg, self.backlash_pan_out, self.backlash_deg)
        self.backlash_pan_out = act_pan_deg

        act_tilt_deg = self._apply_backlash(self.servo_lag_tilt_deg, self.backlash_tilt_out, self.backlash_deg)
        self.backlash_tilt_out = act_tilt_deg

        # 6. Webots koordinat sistemine donustur ve motora ilet
        act_w_pan = -act_pan_deg if self.pan_invert else act_pan_deg
        act_w_tilt = -act_tilt_deg if self.tilt_invert else act_tilt_deg

        if self.pan_motor:
            self.pan_motor.setPosition(math.radians(act_w_pan))
        if self.tilt_motor:
            self.tilt_motor.setPosition(math.radians(act_w_tilt))

    def _apply_backlash(self, target_angle, last_output, backlash_deg=CONFIG.actuator.backlash_deg):
        """
        Dogru mekanik bosluk modeli: cikis, hedefe backlash kadar yakinlasinca
        bosluk icinde kalir; hedef bosluktan fazla uzaklasirsa aradaki fark
        korunarak takip eder. Boylece yon degisiminde siçrama olmaz.
        """
        delta = float(target_angle) - float(last_output)
        if delta > backlash_deg:
            return float(target_angle) - backlash_deg
        if delta < -backlash_deg:
            return float(target_angle) + backlash_deg
        return float(last_output)

    def set_neutral(self):
        """Acil durumda veya kapatmada motorlari merkez (0, 0) konumuna ceker."""
        self.cmd_pan_deg = 0.0
        self.cmd_tilt_deg = 0.0
        self.servo_lag_pan_deg = 0.0
        self.servo_lag_tilt_deg = 0.0
        self.backlash_pan_out = 0.0
        self.backlash_tilt_out = 0.0
        if self.pan_motor:
            self.pan_motor.setPosition(0.0)
        if self.tilt_motor:
            self.tilt_motor.setPosition(0.0)