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
        VELOCIDAD = 60
        VELOCIDAD_BUSQUEDA = 50 # Una velocidad más lenta para la marcha atrás

        s_left = GPIO.input(line_pin_left)
        s_mid = GPIO.input(line_pin_middle)
        s_right = GPIO.input(line_pin_right)
        
        print(f"Estado de los sensores (I-M-D): ({s_left}-{s_mid}-{s_right})")

        # Caso 0: Línea no encontrada al arrancar (0 0 0)

        if s_left == 0 and s_mid == 0 and s_right == 0:
            print("Acción: Recto")
            RPIservo.move(SERVO_STEERING, 90)
            move.motor(1, 0, VELOCIDAD)
            self.last_turn_direction = 'none'

        # Caso 1: Línea perfectamente centrada (0 1 0)
        elif s_left == 0 and s_mid == 1 and s_right == 0:
            print("Acción: Recto")
            RPIservo.move(SERVO_STEERING, 90)
            move.motor(1, 0, VELOCIDAD)
            self.last_turn_direction = 'none'

        # Caso 2: La línea está a la izquierda del coche (1 X X) -> Girar a la izquierda
        elif s_left == 1:
            print("Acción: Corrigiendo a la izquierda")
            RPIservo.move(SERVO_STEERING, 45)
            move.motor(1, 0, VELOCIDAD)
            self.last_turn_direction = 'left'

        # Caso 3: La línea está a la derecha del coche (X X 1) -> Girar a la derecha
        elif s_right == 1:
            print("Acción: Corrigiendo a la derecha")
            RPIservo.move(SERVO_STEERING, 135)
            move.motor(1, 0, VELOCIDAD)
            self.last_turn_direction = 'right'

        # --- NUEVA LÓGICA MEJORADA PARA LÍNEA PERDIDA (0 0 0) ---
        elif s_left == 0 and s_mid == 0 and s_right == 0:
            print("Acción: ¡Línea perdida! Iniciando maniobra de búsqueda...")
            
            # 1. Damos un pequeño paso marcha atrás para reposicionarnos.
            move.motor(1, 0, VELOCIDAD_BUSQUEDA) # Marcha atrás lento
            time.sleep(0.25)
            move.motorStop()

            # 2. Giramos las ruedas en la dirección OPUESTA al último giro.
            if self.last_turn_direction == 'left':
                print("   Último giro fue a la izquierda, buscando a la DERECHA.")
                RPIservo.move(SERVO_STEERING, 135) # Gira a la DERECHA
            elif self.last_turn_direction == 'right':
                print("   Último giro fue a la derecha, buscando a la IZQUIERDA.")
                RPIservo.move(SERVO_STEERING, 45)  # Gira a la IZQUIERDA
            else:
                 # Si se perdió en una recta, mantenemos las ruedas centradas.
                 print("   Se perdió en una recta, las ruedas se mantienen centradas.")
                 RPIservo.move(SERVO_STEERING, 90)
        
        # Caso final: Cruce (1 1 1) o estado inesperado -> Parar
        else:
            print("Acción: Cruce o estado desconocido. Parando.")
            move.motorStop()

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
