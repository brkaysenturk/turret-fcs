"""
================================================================================
MODUL: SIFIR-GECIKMELI KAMERA IS PARCACIGI (CAMERA THREAD / DIRECTSHOW)
================================================================================
Bu modul, kameradan goruntu okuma islemini ana kontrol dongusunden izole eden
ve ayri bir is parcaciginda (daemon thread) calistiran yuksek basarimli goruntu motorudur.

KRITIK GOREVLER VE COZULEN PROBLEMLER:
1. Donanim Algilama: PyGrabber & DirectShow Graph API ile bagli USB video aygitlarini
   tarayarak "Logitech C270" kamerasini tak-calistir seklinde otomatik secer.
2. Tampon Gecikmesini (Buffer Lag) Sifirlama:
   Standart cv2.VideoCapture(), arka planda 3-5 karelik bir FIFO tamponu tutar.
   Bu durum gercek dunyada 100-150 ms'lik bayat goruntu (stale frame) gecikmesine yol acar.
   Bu sinif, en son gelen tek kareyi (latest frame) anlik saklar ve tamponu surekli
   bosaltarak goruntu gecikmesini sifira indirir.
3. Donanimsal Cozunurluk: 640x480 @ 30 FPS optimize parametrelerle calisir.
================================================================================
"""

import threading
import time
import cv2


class CameraThread:
    """
    OpenCV VideoCapture nesnesini arka planda bagimsiz bir thread ile sarmalayan sinif.
    Kamera tamponundaki gecikmeyi sifirlar ve DirectShow ile Logitech C270'i otomatik secer.
    """

    @staticmethod
    def detect_camera(preferred_name="Logi C270"):
        """Sistemdeki kameralari listeler ve tercihen Logitech C270'i otomatik secer."""
        try:
            from pygrabber.dshow_graph import FilterGraph
            devices = FilterGraph().get_input_devices()
            if devices:
                for idx, dev in enumerate(devices):
                    if preferred_name.lower() in dev.lower():
                        return idx, dev
                return 0, devices[0]
        except Exception:
            pass
        return 0, "Default Camera"

    def __init__(self, src=None, width=1280, height=720, fps=30):
        if src is None:
            self.src, self.cam_name = self.detect_camera()
        else:
            self.src = src
            self.cam_name = f"Index {src}"

        self.cap = cv2.VideoCapture(self.src, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.grabbed, self.frame = self.cap.read()
        self.running = False
        self.lock = threading.Lock()
        self.thread = None

        if self.grabbed:
            self.start()

    def start(self):
        """Kare yakalama is parcacigini baslatir."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._update, daemon=True)
            self.thread.start()

    def _update(self):
        """Arka planda surekli en guncel kareyi okur."""
        while self.running and self.cap.isOpened():
            grabbed, frame = self.cap.read()
            if not grabbed:
                time.sleep(0.005)
                continue

            with self.lock:
                self.grabbed = grabbed
                self.frame = frame

    def read(self):
        """Ana donguye sifir gecikmeli en son kareyi kopyalar."""
        with self.lock:
            if not self.grabbed or self.frame is None:
                return False, None
            return self.grabbed, self.frame.copy()

    def isOpened(self):
        return self.running and self.cap.isOpened() and self.grabbed

    def stop(self):
        self.release()

    def release(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        if self.cap.isOpened():
            self.cap.release()

    def __getattr__(self, name):
        return getattr(self.cap, name)
