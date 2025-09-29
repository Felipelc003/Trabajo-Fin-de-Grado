# camera_opencv.py (VERSIÓN CON colorFindSet CORREGIDO Y PRECISO)

import os
import cv2
from base_camera import BaseCamera
import RPIservo
import numpy as np
import move
import datetime
import threading
import imutils
from pyzbar import pyzbar # <--- AÑADIDO

# Definimos los servos aquí para que el script sea más claro
SERVO_TILT = 0
SERVO_PAN = 1

class CVProcessor(threading.Thread):
    def __init__(self):
        super(CVProcessor, self).__init__()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.mode = 'none'
        self.is_processing = False
        self.img_to_process = None
        self.drawing_elements = {}
        
        # Valores iniciales (se sobrescribirán con el primer "Color Set")
        self.color_lower = np.array([24, 100, 100])
        self.color_upper = np.array([44, 255, 255])

        self.pan_angle = 90.0
        self.tilt_angle = 90.0
        self.avg_background = None

        self.last_qr_result = None # <--- AÑADIDO: Para guardar el resultado del QR
        self.qr_scanning = False
        self.qr_pan = 0
        self.qr_step = 5         # grados por paso del barrido
        self.qr_reported = False

        self._flag = threading.Event()
        self._flag.clear()

    def scan_qr(self, frame):
        """
        Barrido horizontal 0→180 buscando QR.
        Si lo detecta, imprime el contenido y sale del modo.
        Si termina el barrido sin encontrar, lo informa y sale del modo.
        """
        # Inicialización del barrido la primera vez
        if not self.qr_scanning:
            self.qr_scanning = True
            self.qr_pan = 0
            self.qr_step = 5
            self.qr_reported = False
            self.last_qr_result = None
            self.drawing_elements = {}
            print("[scanQR] Inicializando barrido 0→180")
            try:
                RPIservo.move(SERVO_TILT, 90)
                RPIservo.move(SERVO_PAN, 0)
            except Exception:
                pass

        # Limpia overlays previos
        self.drawing_elements = {}

        # 1) Intentar decodificar un QR en este frame
        barcodes = pyzbar.decode(frame)
        if barcodes:
            barcode = max(barcodes, key=lambda b: b.rect[2] * b.rect[3])
            data = barcode.data.decode("utf-8")
            self.last_qr_result = data

            (x, y, w, h) = barcode.rect
            self.drawing_elements['qr_rect'] = (x, y, x + w, y + h)
            self.drawing_elements['qr_text'] = data

            if not self.qr_reported:
                print(f"[scanQR] ✅ QR detectado: {data}")
                self.qr_reported = True

            # Termina el modo tras detectar
            self.qr_scanning = False
            self.mode = 'none'
            try:
                from camera_opencv import Camera
                Camera.get_instance().modeSelect = 'none'
            except Exception:
                pass
            return
           
           

        # 2) No hay QR -> avanzar barrido PAN
        if self.qr_pan <= 180:
            try:
                RPIservo.move(SERVO_PAN, int(self.qr_pan))
            except Exception:
                pass
            print(f"[scanQR] PAN -> {self.qr_pan}°")
            self.qr_pan += self.qr_step
        else:
            if not self.qr_reported:
                print("[scanQR] ❌ No se ha encontrado ningún QR")
            # Fin del modo sin detectar
            self.qr_scanning = False
            self.mode = 'none'
            try:
                from camera_opencv import Camera
                Camera.get_instance().modeSelect = 'none'
            except Exception:
                pass

            RPIservo.move(SERVO_PAN,90)
            print("Deteniendo")
            self.pause()


    def set_mode(self, new_mode, image):
        if new_mode != self.mode:
            self.last_qr_result = None
        self.mode = new_mode
        self.img_to_process = image
        self.resume()

    def find_color(self, frame):
        KP = 0.1
        MIN_AREA = 500
        (h, w) = frame.shape[:2]
        center_x, center_y = w // 2, h // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        self.drawing_elements = {}
        target_found = False

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_AREA:
                target_found = True
                (x, y, w, h) = cv2.boundingRect(c)
                obj_center_x = x + w // 2
                obj_center_y = y + h // 2
                
                self.drawing_elements['rect'] = (x, y, x + w, y + h)
                self.drawing_elements['text'] = 'Target Locked'

                error_x = center_x - obj_center_x
                error_y = center_y - obj_center_y

                self.pan_angle += error_x * KP
                self.tilt_angle += error_y * KP

                self.pan_angle = max(0, min(180, self.pan_angle))
                self.tilt_angle = max(0, min(180, self.tilt_angle))
                
                RPIservo.move(SERVO_PAN, int(self.pan_angle))
                RPIservo.move(SERVO_TILT, int(self.tilt_angle))
        
        if not target_found:
            self.drawing_elements['text'] = 'Target Detecting'

    # --- El resto de la clase CVProcessor no cambia ---
    def watch_dog(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        if self.avg_background is None: self.avg_background = gray.copy().astype("float"); return
        cv2.accumulateWeighted(gray, self.avg_background, 0.5)
        frame_delta = cv2.absdiff(gray, cv2.convertScaleAbs(self.avg_background))
        thresh = cv2.threshold(frame_delta, 5, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        self.drawing_elements.pop('motion_rect', None)
        for c in cnts:
            if cv2.contourArea(c) < 5000: continue
            (x, y, w, h) = cv2.boundingRect(c)
            self.drawing_elements['motion_rect'] = (x, y, x + w, y + h)

    def draw_elements_on_frame(self, frame):
        if self.mode == 'findColor':
            if 'text' in self.drawing_elements: cv2.putText(frame, self.drawing_elements['text'], (40, 60), self.font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            if 'rect' in self.drawing_elements: x1, y1, x2, y2 = self.drawing_elements['rect']; cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        elif self.mode == 'watchDog':
            if 'motion_rect' in self.drawing_elements: x1, y1, x2, y2 = self.drawing_elements['motion_rect']; cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        elif self.mode == 'scanQR':
            if 'qr_rect' in self.drawing_elements:
                x1, y1, x2, y2 = self.drawing_elements['qr_rect']
                text = self.drawing_elements.get('qr_text', '')
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, text, (x1, y1 - 10), self.font, 0.7, (0, 255, 0), 2)
        return frame

    def run(self):
        while True:
            self._flag.wait()
            if self.mode == 'none': self.is_processing = False; self._flag.clear(); continue
            self.is_processing = True
            if self.img_to_process is not None:
                if self.mode == 'findColor': self.find_color(self.img_to_process)
                elif self.mode == 'watchDog': self.watch_dog(self.img_to_process)
                # --- AÑADIDO: Llamada a la función de escaneo de QR ---
                elif self.mode == 'scanQR': self.scan_qr(self.img_to_process)
            self.is_processing = False; self._flag.clear()

    def pause(self):
        self.pan_angle = 90; self.tilt_angle = 90; self.mode = 'none'; self.drawing_elements = {}; self._flag.clear()
    def resume(self):
        self._flag.set()

class Camera(BaseCamera):
    _instance = None
    _picam2 = None           # <-- singleton Picamera2 compartido
    _picam2_lock = threading.Lock()

    def __init__(self):
        if Camera._instance is not None: raise RuntimeError("Camera is a singleton, use get_instance()")
        super(Camera, self).__init__()
        self.modeSelect = 'none'
        self.cv_thread = CVProcessor()
        self.cv_thread.start()
        Camera._instance = self

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            print("Creando la instancia del objeto Camera por primera vez...")
            cls._instance = Camera()
        return cls._instance

    def modeselect(self, mode: str):
        """
        API pública para cambiar de modo sin reabrir la cámara.
        Ej: Camera.get_instance().modeselect('scanQR')
        """
        print(f"[Camera] modeselect -> {mode}")
        self.modeSelect = mode

    def start_background_feed(self):
        """
        Inicia un consumidor en segundo plano que itera frames()
        y así el hilo CV recibe imágenes incluso sin clientes HTTP.
        No abre otra cámara: usa la misma tubería.
        """
        if getattr(self, "_bg_thread", None):
            return  # ya está corriendo

        import threading, time

        def _pump():
            try:
                for _ in self.frames():
                    # No necesitamos los bytes JPEG aquí; sólo hacer avanzar el pipeline.
                    time.sleep(0.01)
            except Exception as e:
                print(f"[Camera] background feed stopped: {e}")

        t = threading.Thread(target=_pump, daemon=True)
        t.start()
        self._bg_thread = t
        print("[Camera] background feed started")


    # --- FUNCIÓN colorFindSet COMPLETAMENTE REESCRITA ---
    def colorFindSet(self, invarH, invarS, invarV):
        """
        Crea un rango de color HSV inteligente y preciso a partir
        del punto de color enviado desde la GUI.
        """
        # Tono (Hue): Usamos un rango de ±10, que es bueno para la mayoría de los colores.
        H_LOWER = max(invarH - 10, 0)
        H_UPPER = min(invarH + 10, 179)

        # Saturación (Saturation): Ignoramos el valor de la GUI.
        # Forzamos un mínimo de 100 para evitar grises y blancos.
        S_LOWER = 100
        S_UPPER = 255

        # Valor (Value): Ignoramos el valor de la GUI.
        # Forzamos un mínimo de 80 para evitar sombras y negros.
        V_LOWER = 80
        V_UPPER = 255
        
        # Asignamos el nuevo rango al hilo de procesamiento
        self.cv_thread.color_lower = np.array([H_LOWER, S_LOWER, V_LOWER])
        self.cv_thread.color_upper = np.array([H_UPPER, S_UPPER, V_UPPER])
        
        print("--- Rango de Color Inteligente Configurado ---")
        print(f"LOWER: {self.cv_thread.color_lower}")
        print(f"UPPER: {self.cv_thread.color_upper}")

    @staticmethod
    def frames():
        try:
            with Camera._picam2_lock:
                if Camera._picam2 is None:
                    from picamera2 import Picamera2
                    print("Inicializando hardware de Picamera2 (una sola vez)...")
                    picam2 = Picamera2()
                    config = picam2.create_preview_configuration(main={"size": (640, 480)})
                    picam2.configure(config)
                    picam2.start()
                    Camera._picam2 = picam2
                    print("Picamera2 inicializada correctamente (singleton).")
                else:
                    picam2 = Camera._picam2
        except Exception as e:
            print(f"Error al iniciar Picamera2: {e}")
            error_img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(error_img, "Camera Error", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            while True:
                 yield cv2.imencode('.jpg', error_img)[1].tobytes()

        cam_instance = Camera.get_instance()
        while True:
           img_rgb = picam2.capture_array()
           img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
           if cam_instance.modeSelect != 'none' and not cam_instance.cv_thread.is_processing:
               cam_instance.cv_thread.set_mode(cam_instance.modeSelect, img)
           img = cam_instance.cv_thread.draw_elements_on_frame(img)
           yield cv2.imencode('.jpg', img)[1].tobytes()
