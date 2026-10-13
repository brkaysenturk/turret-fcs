import cv2
import numpy as np
from controller import Supervisor


class WebotsHardwareInterface:
    """
    Webots robot donanimlarini (motorlar, sensorler, kamera, LiDAR) yoneten arayuz.
    """

    def __init__(self):
        self.robot = Supervisor()
        basic_step = int(self.robot.getBasicTimeStep())
        self.timestep = max(16, (16 // basic_step) * basic_step) if basic_step > 0 else 16
        self.dt = self.timestep / 1000.0

        # Taret gimbal motorlari
        self.pan_motor = self.robot.getMotor('pan_motor')
        self.tilt_motor = self.robot.getMotor('tilt_motor')
        self.pan_motor.setVelocity(3.8)
        self.tilt_motor.setVelocity(3.8)

        # Aci sensorleri (Enkoderler)
        self.pan_sensor = self.robot.getPositionSensor('pan_sensor')
        self.tilt_sensor = self.robot.getPositionSensor('tilt_sensor')
        self.pan_sensor.enable(self.timestep)
        self.tilt_sensor.enable(self.timestep)

        # Logitech C270 Kamera
        self.camera = self.robot.getCamera('c270')
        self.camera.enable(self.timestep)
        self.w_cam = self.camera.getWidth()
        self.h_cam = self.camera.getHeight()

        # TF-Luna LiDAR
        self.lidar = self.robot.getDistanceSensor('tf_luna')
        self.lidar.enable(self.timestep)

    def step(self):
        """Simulasyonu bir adim (16 ms) ilerletir."""
        return self.robot.step(self.timestep)

    def get_frame(self):
        """Kameradan BGR formatinda goruntu karesi alir."""
        img_raw = self.camera.getImage()
        if img_raw is None:
            return None
        frame = np.frombuffer(img_raw, np.uint8).reshape((self.h_cam, self.w_cam, 4))
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    def get_distance_and_strength(self):
        """LiDAR'dan santimetre cinsinden mesafe ve sinyal gucunu okur."""
        dist_val = self.lidar.getValue()
        dist_cm = int(dist_val * 100.0) if (0.20 <= dist_val <= 8.0) else 0
        strength = 1500 if dist_cm > 0 else 0
        return dist_cm, strength

    def get_motor_positions(self):
        """Guncel pan ve tilt acilarini radyan olarak dondurur."""
        return self.pan_sensor.getValue(), self.tilt_sensor.getValue()

    def set_motor_commands(self, pan_rad, tilt_rad):
        """Motorlara hedef acilari radyan olarak iletir."""
        self.pan_motor.setPosition(pan_rad)
        self.tilt_motor.setPosition(tilt_rad)

    def get_time(self):
        """Webots simulasyon zamanini (saniye) dondurur."""
        return self.robot.getTime()