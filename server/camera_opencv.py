# camera_opencv.py
# Descripción: Módulo de abstracción de hardware para la cámara del robot.
# Implementa el patrón Singleton y utiliza libcamera (a través de Picamera2) para la captura eficiente de video.
# Gestiona el ciclo completo de adquisición de imágenes, preprocesamiento y distribución en formato MJPEG.

import time
import threading
import cv2
from picamera2 import Picamera2

from base_camera import BaseCamera


class Camera(BaseCamera):
    """
    Controlador principal de la cámara.
    
    Esta clase hereda de BaseCamera y añade la lógica específica para interactuar con el hardware (Picamera2).
    Características principales:
    - Patrón Singleton: Garantiza una única instancia de control para la cámara física.
    - Integración con OpenCV: Facilita la manipulación de imágenes y superposición de gráficos.
    - Procesamiento Asíncrono: Delega algoritmos de visión artificial a `cv_processor`.
    """

    _instance = None
    _lock = threading.Lock()

    # Inicialización de atributos estáticos para seguridad entre hilos
    _cv = None
    _cv_overlays = True

    def __new__(cls, *args, **kwargs):
        """
        Implementación del patrón Singleton.
        Asegura que solo se cree una instancia de la clase Camera, protegiendo el acceso concurrente con un Lock.
        """
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        """
        Constructor de la clase.
        Inicializa la cámara física, configura la resolución y framerate, y prepara las estructuras de control.
        """
        # Prevención de re-inicialización en patrón Singleton
        if getattr(self, "_initialized", False):
            return
        self._initialized = True

        self._jpeg_quality = 40      # Calidad de compresión JPEG (0-100) para balancear ancho de banda/calidad.
        self.modeSelect = "none"     # Modo de operación actual (ej. 'trackLine', 'none').
        self._fps_last_t = time.time()
        self._fps_counter = 0
        self._fps_value = 0.0

        # --- Configuración de Hardware (Picamera2) ---
        # Se establece una resolución de trabajo de 640x480 a 60 FPS para fluidez en tiempo real.
        self._p2 = Picamera2()
        cfg = self._p2.create_preview_configuration(
            main={"size": (640, 480)},
            controls={"FrameRate": 60.0},
            buffer_count=4,
        )
        self._p2.configure(cfg)
        self._p2.set_controls({"FrameRate": 60.0})
        self._p2.start()
        
        # Referencia al procesador de visión (Lazy Loading)
        self._cv = None
        self._cv_overlays = True

        super(Camera, self).__init__()

    # ---------------------------------------------------------
    # Gestión de Instancia y Accesores
    # ---------------------------------------------------------

    @classmethod
    def get_instance(cls):
        """Devuelve la instancia única de la clase Camera."""
        return cls()

    @property
    def cv_thread(self):
        """
        Propiedad para acceso al procesador de visión (CVProcessor).
        Utiliza instanciación diferida (Lazy Initialization) para evitar errores de importación circular y asegurar que se cree solo cuando sea necesario.
        """
        if self._cv is None:
            from cv_processor import CVProcessor
            self._cv = CVProcessor()
            self._cv.draw_overlays = self._cv_overlays
            self._cv.mode = self.modeSelect
        return self._cv

    def modeselect(self, mode: str):
        """
        Cambia el modo de operación de la visión artificial.
        
        Parámetros:
        - mode (str): Identificador del modo (ej. 'trackLine', 'none').
        
        Esta función configura el procesador de visión según el modo solicitado,
        activando o pausando el procesamiento según corresponda.
        """
        print(f"[Camera] Cambio de modo -> {mode}")
        self.modeSelect = mode
        
        if self.modeSelect == 'trackLine':
            self.cv_thread.mode = 'trackLine'
            self.cv_thread.img_to_process = None
        
        elif self.modeSelect == 'none':
            self.cv_thread.pause()

    @classmethod
    def modeselect_class(cls, mode: str):
        """Método de clase conveniente para cambiar el modo desde contextos estáticos."""
        cls.get_instance().modeselect(mode)

    # ---------------------------------------------------------
    # Generador de Frames (Bucle Principal)
    # ---------------------------------------------------------
    
    @staticmethod
    def frames():
        """
        Generador principal del flujo de video (Pipeline).
        
        Este método estático se ejecuta en el hilo de fondo de BaseCamera.
        Etapas del Pipeline:
        1. Captura: Obtiene el array de píxeles RGB desde el hardware (libcamera).
        2. Conversión: Transforma el espacio de color a BGR (estándar OpenCV).
        3. Procesamiento: Delega en CVProcessor la detección de objetos o líneas.
        4. Telemetría: Calcula FPS e inserta datos en pantalla (HUD).
        5. Codificación: Comprime la imagen resultante a formato JPEG.
        """
        cam = Camera.get_instance()

        # Aseguramiento de atributos en entorno concurrente (inicialización defensiva)
        if not hasattr(cam, "_cv"):
            cam._cv = None
        if not hasattr(cam, "_cv_overlays"):
            cam._cv_overlays = True

        # Carga inicial del procesador de visión si no existe
        if cam._cv is None:
            from cv_processor import CVProcessor
            cam._cv = CVProcessor()
            cam._cv.draw_overlays = cam._cv_overlays
            cam._cv.mode = cam.modeSelect

        while True:
            # 1. Captura de imagen (Hardware - Picamera2)
            rgb = cam._p2.capture_array()
            img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # 2. Procesamiento de Visión (Delegado)
            try:
                cam._cv.mode = cam.modeSelect
                img = cam._cv.draw_elements_on_frame(img)
            except Exception:
                pass

            # 3. Cálculo de métricas de rendimiento (FPS)
            cam._fps_counter += 1
            now = time.time()
            if (now - cam._fps_last_t) >= 1.0:
                cam._fps_value = cam._fps_counter / (now - cam._fps_last_t)
                cam._fps_counter = 0
                cam._fps_last_t = now

            # 4. Superposición de Telemetría (HUD)
            try:
                vs = cam._cv.get_vehicle_status() if (cam._cv is not None) else {}
                v_speed = int(vs.get('speed', 0))
                v_steer = int(vs.get('steer', 88.5))
            except Exception:
                v_speed, v_steer = 0, 88.5

            v_steer = max(60, min(130, v_steer)) # Limitación visual del ángulo

            try:
                hud = f"FPS {cam._fps_value:.1f}  Vel {v_speed:>3}  Giro {v_steer:>3}"
                cv2.putText(
                    img, hud, (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 230, 30), 2, cv2.LINE_AA,
                )
            except Exception:
                pass

            # 5. Codificación y Envío
            # Conversión de la matriz de imagen a bytes JPEG comprimidos.
            ok, jpeg = cv2.imencode(
                ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), cam._jpeg_quality]
            )
            if not ok:
                continue

            # Generación del frame en formato MJPEG multipart.
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
            )

    def close(self):
        """Detiene la cámara y libera los recursos de hardware."""
        try:
            self._p2.stop()
        except Exception:
            pass