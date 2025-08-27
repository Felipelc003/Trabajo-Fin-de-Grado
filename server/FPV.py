#!/usr/bin/env python3
# File name   : FPV.py
# Adaptado para Raspberry Pi OS Bookworm + Picamera2
# Original: Adeept PiCar-B
# Autor adaptación: Felipe + ChatGPT

import time
import cv2
import zmq
import base64
from picamera2 import Picamera2
from libcamera import Transform
from collections import deque
import servo
import PID
import LED
import datetime
import GUImove as move
import ultra
import numpy as np

pid = PID.PID()
pid.SetKp(0.5)
pid.SetKd(0)
pid.SetKi(0)

# Estados de control
FindColorMode = 0
WatchDogMode = 0
FindLineMode = 0
UltraData = 3
CVrun = 1

# Parámetros
speed_set = 90
back_R = 0.4
forward_R = 0.6
linePos_1 = 440
linePos_2 = 380
lineColorSet = 255
frameRender = 1
findLineError = 20
colorUpper = np.array([44, 255, 255])
colorLower = np.array([24, 100, 100])

def findLineCtrl(posInput, setCenter):
    """Control de seguimiento de línea"""
    if posInput:
        if posInput > (setCenter + findLineError):
            move.motorStop()
            error = (posInput - 320) / 5
            outv = int(round(pid.GenOut(error), 0))
            servo.lookright(outv)
            servo.turnRight()
        elif posInput < (setCenter - findLineError):
            move.motorStop()
            error = (320 - posInput) / 5
            outv = int(round(pid.GenOut(error), 0))
            servo.lookleft(outv)
            servo.turnLeft()
        else:
            if CVrun:
                move.move(speed_set, 'forward')
    else:
        if CVrun:
            move.move(speed_set, 'backward')

def cvFindLine(frame):
    """Procesamiento para detección de línea"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    binary = cv2.erode(binary, None, iterations=6)

    colorPos_1 = binary[linePos_1]
    colorPos_2 = binary[linePos_2]
    try:
        lineIndex_Pos1 = np.where(colorPos_1 == lineColorSet)[0]
        lineIndex_Pos2 = np.where(colorPos_2 == lineColorSet)[0]

        if len(lineIndex_Pos1) > 0 and len(lineIndex_Pos2) > 0:
            center_Pos1 = int((lineIndex_Pos1[-1] + lineIndex_Pos1[0]) / 2)
            center_Pos2 = int((lineIndex_Pos2[-1] + lineIndex_Pos2[0]) / 2)
            center = int((center_Pos1 + center_Pos2) / 2)
        else:
            center = None
    except:
        center = None

    findLineCtrl(center, 320)

class FPV: 
    def __init__(self):
        self.picam2 = None
        self.running = False
        self.IP = "127.0.0.1"

    def SetIP(self, invar):
        self.IP = invar

    def colorFindSet(self, h, s, v):
        global colorUpper, colorLower
        HUE_1 = min(h + 11, 255)
        HUE_2 = max(h - 11, 0)
        SAT_1 = min(s + 170, 255)
        SAT_2 = max(s - 20, 0)
        VAL_1 = min(v + 170, 255)
        VAL_2 = max(v - 20, 0)
        colorUpper = np.array([HUE_1, SAT_1, VAL_1])
        colorLower = np.array([HUE_2, SAT_2, VAL_2])
        print(f"[FPV] Color detectado - Upper: {colorUpper}, Lower: {colorLower}")

    def FindColor(self, mode):
        global FindColorMode
        FindColorMode = mode
        if not mode:
            servo.ahead()

    def WatchDog(self, mode):
        global WatchDogMode
        WatchDogMode = mode

    def UltraData(self, data):
        global UltraData
        UltraData = data

    def capture_thread(self, IPinver):
        """Hilo principal de captura y procesamiento"""
        global FindColorMode, WatchDogMode, FindLineMode
        self.running = True

        # Inicializar cámara
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(
            main={"size": (640, 480)}, transform=Transform(vflip=1)
        ))
        self.picam2.start()
        time.sleep(2)

        # Conexión ZMQ
        context = zmq.Context()
        footage_socket = context.socket(zmq.PUB)
        footage_socket.connect(f"tcp://{IPinver}:5555")

        print(f"[FPV] Streaming iniciado a {IPinver}")

        while self.running:
            frame = self.picam2.capture_array()

            # --- Modo seguimiento de color ---
            if FindColorMode:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, colorLower, colorUpper)
                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts = cnts[0] if len(cnts) == 2 else cnts[1]
                if cnts:
                    c = max(cnts, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    if radius > 10:
                        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        error = (x - 320) / 5
                        servo.turnRight() if error > 0 else servo.turnLeft()

            # --- Modo perro guardián (detección de movimiento) ---
            if WatchDogMode:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray = cv2.GaussianBlur(gray, (21, 21), 0)
                if not hasattr(self, "avg_frame"):
                    self.avg_frame = gray.copy().astype("float")
                else:
                    cv2.accumulateWeighted(gray, self.avg_frame, 0.5)
                    frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(self.avg_frame))
                    thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
                    if cv2.countNonZero(thresh) > 5000:
                        print("[WatchDog] Movimiento detectado")
                        LED.ledfunc = 'police'

            # --- Modo seguir línea ---
            if FindLineMode:
                cvFindLine(frame)

            # --- Renderizar guías ---
            if frameRender:
                cv2.line(frame, (300, 240), (340, 240), (128, 255, 128), 1)
                cv2.line(frame, (320, 220), (320, 260), (128, 255, 128), 1)

            # Enviar frame
            encoded, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer)
            footage_socket.send(jpg_as_text)

            time.sleep(0.03)  # ~30 FPS

        self.picam2.stop()

    def stop(self):
        self.running = False
        time.sleep(0.5)
        if self.picam2:
            self.picam2.close()
        print("[FPV] Cámara detenida")

if __name__ == '__main__':
    fpv = FPV()
    try:
        fpv.capture_thread('192.168.0.110')
    except KeyboardInterrupt:
        fpv.stop()
