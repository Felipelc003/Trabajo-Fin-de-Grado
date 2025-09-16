#!/usr/bin/python3
# File name   : functions.py
# Description : Control Functions using modern libraries
# Author      : William (Adaptado por Felipe y Gemini)
# Date        : 2025/08/28

import time
import RPi.GPIO as GPIO
import threading
import os
import json
import ultra
import Kalman_filter
import move
import RPIservo
import asyncio

# --- Definiciones de Servos para Claridad ---
SERVO_TILT = 0
SERVO_PAN = 1
SERVO_STEERING = 2

# --- Inicialización de Módulos ---
move.setup()
kalman_filter_X = Kalman_filter.Kalman_filter(0.01, 0.1)
line_pin_right, line_pin_middle, line_pin_left = 20, 16, 19

MPU_connection = 1
try:
    from mpu6500 import mpu6500
    sensor = mpu6500(0x68)
    print('mpu6500 conectado.')
except:
    MPU_connection = 0
    print('mpu6500 desconectado o librería no instalada. El modo "Steady" no funcionará.')

def setup_line_pins():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(line_pin_right, GPIO.IN)
    GPIO.setup(line_pin_middle, GPIO.IN)
    GPIO.setup(line_pin_left, GPIO.IN)

class Functions(threading.Thread):
    def __init__(self, *args, **kwargs):
        self.functionMode = 'none'
        self.camera = None # <--- AÑADIDO: Para guardar la referencia a la cámara
        self.last_turn_direction = 'none'
        self.steadyGoal = 0
        self.websocket = None
        self.event_loop = None
        setup_line_pins()

        # Estados para el modo automático
        self.auto_state = "AVANZAR"
        self.turn_direction = "derecha"
        self.dist_frente = 0
        
        self.last_turn_direction = 'none'

        super(Functions, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    # --- El resto de funciones (radarScan, automatic, etc.) no necesitan cambios ---
    def send_radar_data(self, distance, angle):
        if self.websocket and self.event_loop:
            try:
                response = {'title': 'scanResult', 'data': [[distance, angle]]}
                asyncio.run_coroutine_threadsafe(
                    self.websocket.send(json.dumps(response)),
                    self.event_loop
                )
            except Exception as e:
                print(f"Error al enviar datos del radar: {e}")

    def radarScan(self):
        print("Iniciando escaneo de radar...")
        full_scan_result = []
        RPIservo.move(SERVO_TILT, 90)
        time.sleep(0.5)
        for angle in range(0, 181, 10):
            RPIservo.move(SERVO_PAN, angle)
            time.sleep(0.1)
            dist = ultra.checkdist()
            if dist is not None and 0.02 < dist < 4.0:
               display_angle = 180 - angle
               full_scan_result.append([dist, display_angle])
        time.sleep(0.5)
        RPIservo.move(SERVO_PAN, 90)
        print("Escaneo de radar finalizado.")
        return full_scan_result

    def pause(self):
        self.functionMode = 'none'
        self.websocket = None
        self.event_loop = None
        move.motorStop()
        self.__flag.clear()
        print("Pausando todas las funciones activas.")

    def resume(self):
        self.__flag.set()

    def automatic(self, websocket, loop):
        print("🤖 Activando modo 'Automático' con radar en vivo.")
        self.websocket = websocket
        self.event_loop = loop
        self.functionMode = 'Automatic'
        self.auto_state = "AVANZAR"
        self.resume()

    def trackLine(self):
        self.functionMode = 'trackLine'
        self.last_turn_direction = 'none' 
        self.resume()

    def automaticProcessing(self):
        # ... (código sin cambios)
        pass 

    # --- FUNCIÓN DE SEGUIMIENTO DE LÍNEA CON MANIOBRA DE BÚSQUEDA ---
    def trackLineProcessing(self):
        """
        Seguimiento de línea simple y suave:
        - Ajuste proporcional del servo de dirección según los sensores.
        - Sin cruces ni QR.
        - Si se pierde la línea, busca en la última dirección conocida.
        """
        VELOCIDAD = 60
        KP = 10   # Ganancia proporcional para el servo (ajusta sensibilidad)

        s_left = GPIO.input(line_pin_left)
        s_mid = GPIO.input(line_pin_middle)
        s_right = GPIO.input(line_pin_right)
        print(f"Sensores (I-M-D): {s_left}-{s_mid}-{s_right}")

        # Valor base del servo (recto)
        target_angle = 95  

        if s_mid == 1 and s_left == 0 and s_right == 0:
            # Línea centrada
            target_angle = 95
            self.last_turn_direction = 'center'

        elif s_left == 1 and s_mid == 0:
            # Línea a la izquierda → corregir proporcionalmente
            target_angle = 95 + KP
            self.last_turn_direction = 'left'

        elif s_right == 1 and s_mid == 0:
            # Línea a la derecha → corregir proporcionalmente
            target_angle = 95 - KP
            self.last_turn_direction = 'right'

        elif s_left == 1 and s_mid == 1 and s_right == 0:
            # Entre centro e izquierda
            target_angle = 95 + KP // 2
            self.last_turn_direction = 'left'

        elif s_right == 1 and s_mid == 1 and s_left == 0:
            # Entre centro y derecha
            target_angle = 95 - KP // 2
            self.last_turn_direction = 'right'

        else:
            # Línea perdida (000 o estado extraño)
            print("⚠️ Línea perdida, buscando...")
            if self.last_turn_direction == 'left':
                target_angle = 130   # gira buscando izquierda
            elif self.last_turn_direction == 'right':
                target_angle = 66  # gira buscando derecha
            else:
                target_angle = 95   # si no hay historial, mantener recto

        # Limitar ángulo entre 45° y 135°
        target_angle = max(66, min(130, target_angle))
        RPIservo.move(SERVO_STEERING, target_angle)

        # Avanza siempre hacia delante
        move.motor(1, 0, VELOCIDAD)
        time.sleep(0.05)

    def functionGoing(self):
        if self.functionMode == 'none':
            pass
        elif self.functionMode == 'Automatic':
            self.automaticProcessing()
        elif self.functionMode == 'trackLine':
            self.trackLineProcessing()
    
    def run(self):
        while True:
            self.__flag.wait()
            if self.functionMode != 'none':
                self.functionGoing()
