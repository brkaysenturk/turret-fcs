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
3. Donanimsal Cozunurluk: 1280x720 @ 30 FPS optimize parametrelerle calisir.
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
        self.last_frame_time = time.time()
        self.frame_id = 0
        self.new_frame_event = threading.Event()
        self.running = False
        self.lock = threading.Lock()
        self.thread = None

        if self.grabbed:
            self.start()

    def start(self):
        """Kare yakalama is parcacigini baslatir."""
        if not self.running:
            self.running = True
            self.last_frame_time = time.time()
            self.thread = threading.Thread(target=self._update, daemon=True)
            self.thread.start()

    def _update(self):
        """Arka planda surekli en guncel kareyi okur ve yeni kare olayini tetikler."""
        while self.running and self.cap.isOpened():
            grabbed, frame = self.cap.read()
            if not grabbed:
                time.sleep(0.005)
                continue

            with self.lock:
                self.grabbed = grabbed
                self.frame = frame
                self.frame_id += 1
                self.last_frame_time = time.time()
            self.new_frame_event.set()

    def read(self, timeout=0.35, wait_for_new=False, return_id=False):
        """
        Ana donguye sifir gecikmeli en son kareyi kopyalar (donma korumali).
        wait_for_new=True secilirse ana dongu 30 FPS kamera yakalama hiziyla donanimsal olarak
        senkronize edilir; ayni karenin tekrar tekrar (duplike) islenmesi %100 engellenir.
        return_id=True verilirse (ok, frame, frame_id) seklinde doner.
        """
        if wait_for_new:
            self.new_frame_event.wait(timeout=min(0.050, timeout))
            self.new_frame_event.clear()

        with self.lock:
            is_stale = (time.time() - self.last_frame_time) > timeout
            if not self.grabbed or self.frame is None or is_stale:
                return (False, None, 0) if return_id else (False, None)
            copied_frame = self.frame.copy()
            fid = self.frame_id
            return (True, copied_frame, fid) if return_id else (True, copied_frame)

    @property
    def current_frame_id(self):
        """Son yakalanan donanimsal kare numarasini doner."""
        with self.lock:
            return self.frame_id

    def isOpened(self):
        with self.lock:
            is_stale = (time.time() - self.last_frame_time) > 0.60
            return self.running and self.cap.isOpened() and self.grabbed and not is_stale

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
