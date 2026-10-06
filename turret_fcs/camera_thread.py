import cv2
import threading
import time


class CameraThread:
    """
    OpenCV VideoCapture nesnesini arka planda bagimsiz bir thread ile sarmalayan sinif.
    Kamera tamponundaki gecikmeyi sifirlar ve VideoCapture API'si ile tam uyumludur.
    """

    def __init__(self, src=0, width=1280, height=720, fps=30):
        self.src = src
        self.cap = cv2.VideoCapture(self.src, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
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
            print(f"[OK] [Camera] CameraThread baslatildi (Src: {self.src}).")

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
        """Kamera akisinin ve thread'in aktifligini dogrular."""
        return self.running and self.cap.isOpened() and self.grabbed

    def stop(self):
        """Kamerayi ve thread'i guvenli sonlandirir."""
        self.release()

    def release(self):
        """Thread'i ve donanim portunu guvenli kapatir."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        if self.cap.isOpened():
            self.cap.release()
        print("[OK] [Camera] CameraThread serbest birakildi.")

    def __getattr__(self, name):
        """OpenCV metotlarini dogrudan orijinal cv2.VideoCapture nesnesine yonlendirir."""
        return getattr(self.cap, name)
