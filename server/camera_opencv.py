# camera_opencv.py (VERSIÓN CORREGIDA)
import os
import cv2
from base_camera import BaseCamera
import numpy as np
import move
import switch
import datetime
import Kalman_filter
import PID
import time
import threading
import imutils
import robotLight
import RPIservo
from picamera2 import Picamera2

pid = PID.PID()
pid.SetKp(0.5)
pid.SetKd(0)
pid.SetKi(0)

SERVO_TILT = 0
SERVO_PAN = 1
SERVO_STEERING = 2

class CVThread(threading.Thread):
    font = cv2.FONT_HERSHEY_SIMPLEX
    kalman_filter_X = Kalman_filter.Kalman_filter(0.01, 0.1)
    kalman_filter_Y = Kalman_filter.Kalman_filter(0.01, 0.1)
    P_direction = -1
    T_direction = -1
    P_anglePos = 90.0
    T_anglePos = 90.0
    Y_lock = 0
    X_lock = 0
    tor = 27
    move.setup()
    switch.switchSetup()

    def __init__(self, *args, **kwargs):
        super(CVThread, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()
        self.CVMode = 'none'

    @staticmethod
    def servoMove(ID, Dir, errorInput):
        errorGenOut = 0
        if ID == SERVO_PAN:
            errorGenOut = CVThread.kalman_filter_X.kalman(errorInput)
            CVThread.P_anglePos += 0.1 * (errorGenOut * Dir)
            if CVThread.P_anglePos > 180: CVThread.P_anglePos = 180
            if CVThread.P_anglePos < 0: CVThread.P_anglePos = 0
            RPIservo.move(ID, CVThread.P_anglePos)
            CVThread.X_lock = 1 if abs(errorInput) < CVThread.tor else 0
        elif ID == SERVO_TILT:
            errorGenOut = CVThread.kalman_filter_Y.kalman(errorInput)
            CVThread.T_anglePos += 0.1 * (errorGenOut * Dir)
            if CVThread.T_anglePos > 180: CVThread.T_anglePos = 180
            if CVThread.T_anglePos < 0: CVThread.T_anglePos = 0
            RPIservo.move(ID, CVThread.T_anglePos)
            CVThread.Y_lock = 1 if abs(errorInput) < CVThread.tor else 0

class Camera(BaseCamera):
    picam2 = None  # Variable de clase para contener el objeto de la cámara
    modeSelect = 'none'

    def __init__(self):
        # La inicialización del hardware de la cámara ahora ocurre aquí
        if Camera.picam2 is None:
            print("Inicializando hardware de Picamera2...")
            try:
                Camera.picam2 = Picamera2()
                config = Camera.picam2.create_preview_configuration(main={"size": (640, 480)})
                Camera.picam2.configure(config)
                Camera.picam2.start()
                print("Picamera2 inicializada correctamente.")
                time.sleep(1.0)
            except Exception as e:
                print(f"ERROR CRÍTICO: No se pudo inicializar Picamera2. {e}")
                Camera.picam2 = None
        super(Camera, self).__init__()

    @staticmethod
    def frames():
        if Camera.picam2 is None:
            print("La cámara no está disponible, no se pueden generar frames.")
            # Genera un frame de error para no romper el streaming
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(img, 'Camera Error', (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            ret, buffer = cv2.imencode('.jpg', img)
            while True:
                yield buffer.tobytes()

        cvt = CVThread()
        cvt.start()

        while True:
            img = Camera.picam2.capture_array()
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
            # Aquí iría la lógica de OpenCV si se necesita
            
            ret, buffer = cv2.imencode('.jpg', img)
            if ret:
                yield buffer.tobytes()
