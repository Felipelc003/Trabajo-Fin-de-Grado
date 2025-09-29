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
from camera_opencv import Camera
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

        # Estados para QR
        self.qr_latched = False
        self.qr_last_try = 0.0
        self.qr_cooldown_s = 12

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

    def radarScan(self, step=5, dwell_ms=80, samples=3, stream=True):
        """
        Barrido ultrasónico tipo RADAR con servo PAN:
        - step: resolución angular en grados (5–10° recomendado).
        - dwell_ms: tiempo de asentamiento del servo por paso.
        - samples: nº de medidas por ángulo (se aplica media recortada).
        - stream: si True, envía cada punto al GUI en tiempo real.

        Devuelve: lista [[dist_m, angle_gui_deg], ...] del barrido completo.
        """
        print("Iniciando escaneo de radar...")
        # Asegura TILT centrado para “mirar al frente”
        try:
            RPIservo.move(SERVO_TILT, 90)
        except Exception as e:
            print(f"[radarScan] Aviso al centrar TILT: {e}")
        time.sleep(0.2)

        # Parámetros y límites del HC-SR04
        MIN_M, MAX_M = 0.02, 4.00   # 2 cm – 4 m (teórico del sensor)
        full_scan_result = []

        # Secuencia ping-pong para suavizar
        forward = list(range(0, 181, step))
        backward = list(range(180, -1, -step))
        sweep = forward + backward[1:-1]  # evita duplicar 0/180 en extremos

        def trimmed_mean(values, trim=0.34):
            if not values:
                return None
            vals = sorted(v for v in values if v is not None)
            if not vals:
                return None
            k = int(len(vals) * trim)
            vals = vals[k: len(vals) - k] if len(vals) > 2*k else vals
            return sum(vals) / len(vals) if vals else None

        for angle in sweep:
            # Mueve PAN y espera asentamiento
            try:
                RPIservo.move(SERVO_PAN, angle)
            except Exception as e:
                print(f"[radarScan] Error moviendo PAN a {angle}: {e}")
            time.sleep(dwell_ms / 1000.0)

            # Varias lecturas por ángulo
            readings = []
            for _ in range(max(1, samples)):
                d = ultra.checkdist()  # en metros (según tu módulo)
                # Filtra por rango sensor
                if d is not None and MIN_M <= d <= MAX_M:
                    readings.append(d)
                time.sleep(0.02)  # pausa entre pings (20 ms)

            dist = trimmed_mean(readings)
            if dist is None:
                continue  # nada válido en este ángulo

            # Mapea a ángulo de visualización tipo “radar”
            display_angle = 180 - angle
            point = [dist, display_angle]
            full_scan_result.append(point)

            # Streaming al GUI
            if stream:
                self.send_radar_data(dist, display_angle)

        # Vuelve PAN al centro
        time.sleep(0.2)
        try:
            RPIservo.move(SERVO_PAN, 90)
        except Exception:
            pass

        print(f"Escaneo de radar finalizado. Puntos válidos: {len(full_scan_result)}")
        return full_scan_result

    def pause(self):
        self.functionMode = 'none'
        self.websocket = None
        self.event_loop = None
        move.stop()
        self.__flag.clear()
        print("Pausando todas las funciones activas.")

    def modeSet(self, mode):
        """
        Permite cambiar de modo desde fuera (compatibilidad con webServer).
        """
        if mode == 'trackLine':
            self.trackLine()
        elif mode == 'Automatic':
            self.automatic(self.websocket, self.event_loop)
        elif mode == 'none':
            self.pause()
        else:
            print(f"[Functions] Modo desconocido: {mode}")

    def resume(self):
        self.__flag.set()

    def automatic(self, websocket, loop):
        print("🤖 Activando modo 'Automático' con radar en vivo.")
        self.websocket = websocket
        self.event_loop = loop
        self.functionMode = 'Automatic'
        self.auto_state = "forward"
        self.resume()

    def trackLine(self):
        self.functionMode = 'trackLine'
        self.last_turn_direction = 'none' 
        self.qr_latched = False
        self.qr_last_try = 0.0

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
        CENTER = 95
        LEFT = 130
        RIGHT = 60
        VELOCIDAD = 50
        KP = 10   # Ganancia proporcional para el servo (ajusta sensibilidad)

        s_left = GPIO.input(line_pin_left)
        s_mid = GPIO.input(line_pin_middle)
        s_right = GPIO.input(line_pin_right)
        print(f"Sensores (I-M-D): {s_left}-{s_mid}-{s_right}")

        if s_left == 1 and s_mid == 1 and s_right == 1:
            # Línea perdida: centra y detén movimiento
            RPIservo.move(SERVO_STEERING, CENTER)
            move.stop()

            # Evitar relanzar scanQR en bucle:
            now = time.time()
            try:
                cam = Camera.get_instance()
                # Si ya está escaneando o acabamos de intentarlo, no reintentes
                already_scanning = (cam.modeSelect == 'scanQR') or getattr(cam.cv_thread, 'qr_scanning', False)
            except Exception:
                already_scanning = False

            if (not self.qr_latched) and (now - self.qr_last_try > self.qr_cooldown_s) and (not already_scanning):
                try:
                    print(f"[trackLine] Camera instance OK. Modo previo: {cam.modeSelect}")
                    cam.modeselect('scanQR')
                    print("[trackLine] Modo solicitado: scanQR")
                except Exception as e:
                    print(f"[trackLine] Aviso: no pude activar scanQR: {e}")
                # Latch + marca de tiempo para no repetir
                self.qr_latched = True
                self.qr_last_try = now

            # Mantén este return para salir del ciclo de seguimiento en esta iteración
            self.pause()
            return

        # Valor base del servo (recto)
        target_angle = CENTER  


        if s_mid == 1 and s_left == 0 and s_right == 0:
            # Línea centrada
            target_angle = CENTER
            self.last_turn_direction = 'center'

        elif s_left == 1 and s_mid == 0:
            # Línea a la izquierda → corregir proporcionalmente
            target_angle = CENTER + KP
            self.last_turn_direction = 'left'

        elif s_right == 1 and s_mid == 0:
            # Línea a la derecha → corregir proporcionalmente
            target_angle = CENTER - KP
            self.last_turn_direction = 'right'

        elif s_left == 1 and s_mid == 1 and s_right == 0:
            # Entre centro e izquierda
            target_angle = CENTER + KP // 2
            self.last_turn_direction = 'left'

        elif s_right == 1 and s_mid == 1 and s_left == 0:
            # Entre centro y derecha
            target_angle = CENTER - KP // 2
            self.last_turn_direction = 'right'


        else:
            # Línea perdida (000 o estado extraño)
            #print("⚠️ Línea perdida, buscando...")
            if self.last_turn_direction == 'left':
                target_angle = LEFT   # gira buscando izquierda
            elif self.last_turn_direction == 'right':
                target_angle = RIGHT  # gira buscando derecha
            else:
                target_angle = CENTER   # si no hay historial, mantener recto

        # Limitar ángulo entre 45° y 135°
        target_angle = max(RIGHT, min(LEFT, target_angle))
        RPIservo.move(SERVO_STEERING, target_angle)

        # Avanza siempre hacia delante
        move.forward(VELOCIDAD)
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
