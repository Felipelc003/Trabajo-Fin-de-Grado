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

        self._flag = threading.Event()
        self._flag.clear()

    def scan_qr(self, frame):
        self.drawing_elements = {}
        barcodes = pyzbar.decode(frame)
        if barcodes:
            # Nos centramos en el QR más grande/cercano
            barcode = max(barcodes, key=lambda b: b.rect[2] * b.rect[3])
            barcodeData = barcode.data.decode("utf-8")
            
            # Guardamos el resultado para que el otro hilo pueda leerlo
            self.last_qr_result = barcodeData
            
            # Preparamos los elementos para dibujar en pantalla
            (x, y, w, h) = barcode.rect
            self.drawing_elements['qr_rect'] = (x, y, x + w, y + h)
            self.drawing_elements['qr_text'] = barcodeData

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
        return frame
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
            from picamera2 import Picamera2
            print("Inicializando hardware de Picamera2...")
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(main={"size": (640, 480)})
            picam2.configure(config)
            picam2.start()
            print("Picamera2 inicializada correctamente.")
        except Exception as e:
            print(f"Error al iniciar Picamera2: {e}")
            error_img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(error_img, "Camera Error", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            while True: yield cv2.imencode('.jpg', error_img)[1].tobytes()
        
        cam_instance = Camera.get_instance()
        while True:
            img_rgb = picam2.capture_array()
            img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            if cam_instance.modeSelect != 'none' and not cam_instance.cv_thread.is_processing:
                cam_instance.cv_thread.set_mode(cam_instance.modeSelect, img)
            img = cam_instance.cv_thread.draw_elements_on_frame(img)
            yield cv2.imencode('.jpg', img)[1].tobytes()
