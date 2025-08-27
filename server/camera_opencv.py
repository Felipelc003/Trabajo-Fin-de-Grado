# camera_opencv.py (VERSIÓN CON CORRECCIÓN DE COLOR FORZADA Y DEFINITIVA)
import os
import cv2
from base_camera import BaseCamera
import RPIservo
import numpy as np
import move
import datetime
import threading
import imutils

# --- Clase para todo el procesamiento de Visión por Computadora ---
class CVProcessor(threading.Thread):
    def __init__(self):
        super(CVProcessor, self).__init__()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        
        self.mode = 'none'
        self.is_processing = False
        self.img_to_process = None
        self.drawing_elements = {}

        self.color_upper = np.array([44, 255, 255])
        self.color_lower = np.array([24, 100, 100])
        self.avg_background = None
        
        self._flag = threading.Event()
        self._flag.clear()

    def set_mode(self, new_mode, image):
        self.mode = new_mode
        self.img_to_process = image
        self.resume()

    def find_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        self.drawing_elements = {}
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((box_x, box_y), radius) = cv2.minEnclosingCircle(c)
            if radius > 10:
                self.drawing_elements['rect'] = (int(box_x - radius), int(box_y - radius), int(box_x + radius), int(box_y + radius))
                self.drawing_elements['text'] = 'Target Detected'
        else:
            self.drawing_elements['text'] = 'Target Detecting'

    def watch_dog(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        if self.avg_background is None:
            self.avg_background = gray.copy().astype("float")
            return

        cv2.accumulateWeighted(gray, self.avg_background, 0.5)
        frame_delta = cv2.absdiff(gray, cv2.convertScaleAbs(self.avg_background))
        thresh = cv2.threshold(frame_delta, 5, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        self.drawing_elements.pop('motion_rect', None)
        for c in cnts:
            if cv2.contourArea(c) < 5000:
                continue
            (x, y, w, h) = cv2.boundingRect(c)
            self.drawing_elements['motion_rect'] = (x, y, x + w, y + h)

    def draw_elements_on_frame(self, frame):
        if self.mode == 'findColor':
            if 'text' in self.drawing_elements:
                cv2.putText(frame, self.drawing_elements['text'], (40, 60), self.font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            if 'rect' in self.drawing_elements:
                x1, y1, x2, y2 = self.drawing_elements['rect']
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        elif self.mode == 'watchDog':
            if 'motion_rect' in self.drawing_elements:
                x1, y1, x2, y2 = self.drawing_elements['motion_rect']
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        return frame

    def run(self):
        while True:
            self._flag.wait()
            if self.mode == 'none':
                self.is_processing = False
                self._flag.clear()
                continue
            
            self.is_processing = True
            if self.img_to_process is not None:
                if self.mode == 'findColor':
                    self.find_color(self.img_to_process)
                elif self.mode == 'watchDog':
                    self.watch_dog(self.img_to_process)
            
            self.is_processing = False
            self._flag.clear()

    def pause(self):
        self.mode = 'none'
        self.drawing_elements = {}
        self._flag.clear()

    def resume(self):
        self._flag.set()

class Camera(BaseCamera):
    _instance = None

    def __init__(self):
        if Camera._instance is not None:
            raise RuntimeError("Camera is a singleton, use get_instance()")
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

    def colorFindSet(self, invarH, invarS, invarV):
        HUE_1 = min(invarH + 15, 179)
        HUE_2 = max(invarH - 15, 0)
        SAT_1 = min(invarS + 150, 255)
        SAT_2 = max(invarS - 150, 0)
        VAL_1 = min(invarV + 150, 255)
        VAL_2 = max(invarV - 150, 0)
        
        self.cv_thread.color_upper = np.array([HUE_1, SAT_1, VAL_1])
        self.cv_thread.color_lower = np.array([HUE_2, SAT_2, VAL_2])
        print(f"Nuevo rango de color configurado (LOWER): {self.cv_thread.color_lower}")
        print(f"Nuevo rango de color configurado (UPPER): {self.cv_thread.color_upper}")

    @staticmethod
    def frames():
        try:
            from picamera2 import Picamera2
            print("Inicializando hardware de Picamera2...")
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(main={"size": (640, 480)}) # Dejamos la config simple
            picam2.configure(config)
            picam2.start()
            print("Picamera2 inicializada correctamente.")
        except Exception as e:
            print(f"Error al iniciar Picamera2: {e}")
            error_img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(error_img, "Camera Error", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            while True:
                yield cv2.imencode('.jpg', error_img)[1].tobytes()
        
        cam_instance = Camera.get_instance()

        while True:
            # 1. Capturamos el fotograma (llega en formato RGB)
            img_rgb = picam2.capture_array()
            # 2. Forzamos la conversión de RGB a BGR, el formato que OpenCV usa
            img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            
            # El resto del código sigue igual
            if cam_instance.modeSelect != 'none' and not cam_instance.cv_thread.is_processing:
                cam_instance.cv_thread.set_mode(cam_instance.modeSelect, img)
            
            img = cam_instance.cv_thread.draw_elements_on_frame(img)
            yield cv2.imencode('.jpg', img)[1].tobytes()
