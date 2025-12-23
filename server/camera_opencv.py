import time
import threading
import cv2
from picamera2 import Picamera2

from base_camera import BaseCamera


class Camera(BaseCamera):
    """
    Captura con Picamera2 y delega overlays/lógica CV a CVProcessor.
    - Singleton sin deadlocks (__new__ + __init__ idempotente)
    - CVProcessor se crea 'lazy' en frames() para evitar ciclos de import
    """

    _instance = None
    _lock = threading.Lock()

    # Defaults defensivos para evitar condiciones de carrera si frames() entra muy pronto
    _cv = None
    _cv_overlays = True

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        # Idempotente: no re-inicializar si ya está lista
        if getattr(self, "_initialized", False):
            return
        self._initialized = True

        self._jpeg_quality = 40
        self.modeSelect = "none"     # nombre del modo activo (compat)
        self._fps_last_t = time.time()
        self._fps_counter = 0
        self._fps_value = 0.0

        # === Picamera2 ===
        self._p2 = Picamera2()
        cfg = self._p2.create_preview_configuration(
            main={"size": (640, 480)},
            controls={"FrameRate": 60.0},
            buffer_count=4,
        )
        self._p2.configure(cfg)
        self._p2.set_controls({"FrameRate": 60.0})
        self._p2.start()
        """
        try:
            self._p2.set_controls({
                "AeEnable": True,      # Desactivar Exposición Automática
                "AnalogueGain": 0,    # Aumentar sensibilidad (prueba 4.0 a 10.0)
                "ExposureTime": 0   # Forzar más tiempo de exposición (en microsegundos)
            })
        except Exception as e:
            print(f"[Camera] No se pudieron ajustar controles de exposición: {e}")
        """
        # === CVProcessor (lazy) ===
        # Se construye en frames() para evitar ciclos de import y arrancar limpio
        self._cv = None
        self._cv_overlays = True

        # Arranca el hilo productor de BaseCamera
        super(Camera, self).__init__()

    # ---------- FACTORÍA ----------
    @classmethod
    def get_instance(cls):
        return cls()


    @property
    def cv_thread(self):
        """
        Compat con functions.py:
        - Expone el CVProcessor
        - Lo inicializa si aún no existe (orden de arranque seguro)
        """
        if self._cv is None:
            from cv_processor import CVProcessor  # import diferido
            self._cv = CVProcessor()
            self._cv.draw_overlays = self._cv_overlays
            self._cv.mode = self.modeSelect
        return self._cv

    # ---------- Compat con tu servidor ----------
    def start_background_feed(self):
        """En algunas versiones el servidor llama a esto; no hace falta nada especial."""
        return True

    # ---------- API nueva de modo lineBlack ----------
    def enable_line_black(self, enable: bool = True):
        self.modeSelect = "trackLine" if enable else "none"
        if self._cv is not None:
            self._cv.mode = self.modeSelect

    def set_overlays(self, enabled: bool = True):
        self._cv_overlays = enabled
        if self._cv is not None:
            self._cv.draw_overlays = enabled

    # ---------- SHIM de compatibilidad (antiguo modeselect) ----------
    def modeselect(self, mode: str):
        """
        Compat:
        - 'lineBlack' / 'trackLine' / 'automatic' -> enable_line_black(True)
        - 'none' / 'pause' / 'stop'               -> enable_line_black(False)
        - 'scanQR' / 'findColor' / 'watchDog'     -> delega en app.flask_app si existe
        """
        print(f"[Camera] modeselect -> {mode}")
        self.modeSelect = mode
        
        if self.modeSelect == 'trackLine':
        
            self.cv_thread.mode = 'trackLine'
            self.cv_thread.img_to_process = None
        
        elif self.modeSelect == 'none':
            self.cv_thread.pause()

    @classmethod
    def modeselect_class(cls, mode: str):
        """Compat por si alguna parte del código llama Camera.modeselect('...') de forma estática."""
        cls.get_instance().modeselect(mode)

    # ============================================================
    # Generador de frames (consumido por el hilo BaseCamera._thread)
    # ============================================================
    @staticmethod
    def frames():
        cam = Camera.get_instance()

        # Blindaje por si algún hilo entra muy pronto
        if not hasattr(cam, "_cv"):
            cam._cv = None
        if not hasattr(cam, "_cv_overlays"):
            cam._cv_overlays = True

        # Cargar CVProcessor en diferido (primera vez que entramos aquí)
        if cam._cv is None:
            from cv_processor import CVProcessor  # import diferido para evitar ciclos
            cam._cv = CVProcessor()
            cam._cv.draw_overlays = cam._cv_overlays
            cam._cv.mode = cam.modeSelect

        while True:
            # 1) Captura (RGB) y conversión a BGR
            rgb = cam._p2.capture_array()
            img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # 2) Delegar overlays/procesado al CVProcessor (no bloquea la cámara)
            try:
                cam._cv.mode = cam.modeSelect
                img = cam._cv.draw_elements_on_frame(img)
            except Exception:
                # Nunca romper el stream por errores de visión
                pass

            # 3) FPS overlay (barato)
            cam._fps_counter += 1
            now = time.time()
            if (now - cam._fps_last_t) >= 1.0:
                cam._fps_value = cam._fps_counter / (now - cam._fps_last_t)
                cam._fps_counter = 0
                cam._fps_last_t = now

            # Leer estado publicado por functions.py → cv_processor.set_vehicle_status(...)
            try:
                vs = cam._cv.get_vehicle_status() if (cam._cv is not None) else {}
                v_speed = int(vs.get('speed', 0))
                v_steer = int(vs.get('steer', 90))
            except Exception:
                v_speed, v_steer = 0, 90

            # Clamp del ángulo a tu rango físico (60..130)
            v_steer = max(60, min(130, v_steer))

            # Pintar HUD: FPS | Q | v | θ
            try:
                hud = f"FPS {cam._fps_value:.1f}  Q{cam._jpeg_quality}  v {v_speed:>3}  \u03B8 {v_steer:>3}"
                cv2.putText(
                    img,
                    hud,
                    (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (30, 230, 30),
                    2,
                    cv2.LINE_AA,
                )
            except Exception:
                pass


            # 4) JPEG + yield
            ok, jpeg = cv2.imencode(
                ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), cam._jpeg_quality]
            )
            if not ok:
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
            )

    # ---------- Cierre limpio ----------
    def close(self):
        try:
            self._p2.stop()
        except Exception:
            pass
