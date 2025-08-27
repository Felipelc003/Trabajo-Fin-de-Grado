#!/usr/bin/python3
# File name   : functions.py
# Description : Control Functions using modern libraries
# Author      : William (Adaptado por Felipe y Gemini)
# Date        : 2025/08/22

import time
import RPi.GPIO as GPIO
import threading
import os
import json
import ultra
import Kalman_filter
import move
import RPIservo
import asyncio # <--- Importante añadir asyncio

# --- Definiciones de Servos para Claridad ---
SERVO_TILT = 0     # Servo de inclinación vertical de la cámara
SERVO_PAN = 1      # Servo de giro horizontal de la cámara
SERVO_STEERING = 2 # Servo de dirección de las ruedas

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
        self.websocket = None # Para guardar el canal de comunicación
        self.event_loop = None  # Para enviar mensajes de forma segura
        setup_line_pins()

        super(Functions, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    def send_radar_data(self, distance, angle):
        """ Envía un único punto de datos del radar al cliente. """
        if self.websocket and self.event_loop:
            try:
                # El formato es una lista que contiene una sola lista de [distancia, angulo]
                response = {'title': 'scanResult', 'data': [[distance, angle]]}
                # Enviamos el mensaje de forma segura desde el hilo
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

        for angle in range(0, 181, 10): # Pasos más grandes para un escaneo más rápido
            RPIservo.move(SERVO_PAN, angle)
            time.sleep(0.05)
            dist = ultra.checkdist()
            if dist < 5:
                full_scan_result.append([dist, angle])

        time.sleep(0.5)
        RPIservo.move(SERVO_PAN, 90)
        print("Escaneo de radar finalizado.")
        return full_scan_result

    def pause(self):
        self.functionMode = 'none'
        self.websocket = None # Limpiamos el websocket al pausar
        self.event_loop = None
        move.motorStop()
        self.__flag.clear()

    def resume(self):
        self.__flag.set()

    def automatic(self, websocket, loop):
        self.websocket = websocket # Guardamos el websocket y el loop
        self.event_loop = loop
        self.functionMode = 'Automatic'
        self.resume()

    def trackLine(self):
        self.functionMode = 'trackLine'
        self.resume()

    def automaticProcessing(self):
        """ Lógica de evasión de obstáculos CON envío de datos de radar. """
        # 1. Mirar al frente y medir
        current_angle = 90
        RPIservo.move(SERVO_PAN, current_angle)
        time.sleep(0.2)
        dist = ultra.checkdist()
        self.send_radar_data(dist, current_angle) # Enviamos el dato
        print(f"Distancia frontal: {dist*100:.2f} cm")

        if dist * 100 > 40: # Si hay más de 40 cm, avanza
            RPIservo.move(SERVO_STEERING, 90)
            move.motor_left(1, 0, 80)
            move.motor_right(1, 0, 80)
        elif dist * 100 > 20: # Si está entre 20 y 40 cm, decide a dónde girar
            move.motorStop()
            
            # 2. Mirar a la izquierda y medir
            left_angle = 160
            RPIservo.move(SERVO_PAN, left_angle)
            time.sleep(0.3)
            dist_left = ultra.checkdist()
            self.send_radar_data(dist_left, left_angle) # Enviamos el dato
            
            # 3. Mirar a la derecha y medir
            right_angle = 20
            RPIservo.move(SERVO_PAN, right_angle)
            time.sleep(0.3)
            dist_right = ultra.checkdist()
            self.send_radar_data(dist_right, right_angle) # Enviamos el dato

            RPIservo.move(SERVO_PAN, 90) # Volver al centro
            
            if dist_left > dist_right:
                print("Decisión: Girar a la izquierda")
                RPIservo.move(SERVO_STEERING, 45)
                move.motor_left(1, 1, 80)
                move.motor_right(1, 0, 80)
            else:
                print("Decisión: Girar a la derecha")
                RPIservo.move(SERVO_STEERING, 135)
                move.motor_left(1, 0, 80)
                move.motor_right(1, 1, 80)
            time.sleep(0.5)
        else: # Si está a menos de 20 cm, marcha atrás
            print("Decisión: Marcha atrás")
            move.motor_left(1, 1, 80)
            move.motor_right(1, 1, 80)
            time.sleep(0.5)
            move.motorStop()

    def trackLineProcessing(self):
        # ... (esta función no ha cambiado)
        status_right = GPIO.input(line_pin_right)
        status_middle = GPIO.input(line_pin_middle)
        status_left = GPIO.input(line_pin_left)

        if status_middle == 0:
            RPIservo.move(SERVO_STEERING, 90)
            move.motor_left(1, 0, 80); move.motor_right(1, 0, 80)
        elif status_left == 0:
            RPIservo.move(SERVO_STEERING, 45)
            move.motor_left(1, 0, 80); move.motor_right(1, 0, 80)
        elif status_right == 0:
            RPIservo.move(SERVO_STEERING, 135)
            move.motor_left(1, 0, 80); move.motor_right(1, 0, 80)
        else:
            move.motorStop()
        time.sleep(0.1)

    def functionGoing(self):
        if self.functionMode == 'none':
            self.pause()
        elif self.functionMode == 'Automatic':
            self.automaticProcessing()
        elif self.functionMode == 'trackLine':
            self.trackLineProcessing()
    
    def run(self):
        while True:
            self.__flag.wait()
            self.functionGoing()
