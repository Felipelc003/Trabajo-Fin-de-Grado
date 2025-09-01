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

        # Estados para el modo automático
        self.auto_state = "AVANZAR"
        self.turn_direction = "derecha" # Dirección por defecto para el escape
        self.dist_frente = 0 # Para guardar la última distancia frontal medida

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
        self.websocket = None # Limpiamos el websocket al pausar
        self.event_loop = None
        move.motorStop()
        self.__flag.clear()
        print("Pausando todas las funciones activas.")

    def resume(self):
        self.__flag.set()

    def automatic(self, websocket, loop):
        print("🤖 Activando modo 'Automático' con radar en vivo.")
        self.websocket = websocket # Guardamos el websocket y el loop
        self.event_loop = loop
        self.functionMode = 'Automatic'
        self.auto_state = "AVANZAR" # Reseteamos al estado inicial
        self.resume()

    def trackLine(self):
        self.functionMode = 'trackLine'
        self.resume()

    def automaticProcessing(self):
        """ Lógica de evasión de obstáculos basada en estados. """
        VELOCIDAD = 80
        DISTANCIA_SEGURA_AVANCE = 40 # cm
        DISTANCIA_MINIMA_OBSTACULO = 20 # cm

        # --- ESTADO 1: AVANZAR ---
        if self.auto_state == "AVANZAR":
            print("Estado: AVANZAR")
            RPIservo.move(SERVO_PAN, 90) # Mirar al frente
            time.sleep(0.1)
            dist = ultra.checkdist()
            if dist is None: dist = 0 # Protección contra lecturas fallidas
            dist_cm = dist * 100
            self.send_radar_data(dist, 90)
            print(f"Distancia frontal: {dist_cm:.2f} cm")

            if dist_cm > DISTANCIA_SEGURA_AVANCE:
                RPIservo.move(SERVO_STEERING, 90) # Dirección recta
                move.motor(1, 0, VELOCIDAD)
                #move.motor_right(1, 0, VELOCIDAD)
            else:
                print("Obstáculo detectado. Cambiando a estado DECIDIR_RUTA.")
                move.motorStop()
                self.dist_frente = dist_cm # Guardamos la distancia frontal para usarla después
                self.auto_state = "DECIDIR_RUTA"

        # --- ESTADO 2: DECIDIR RUTA ---
        elif self.auto_state == "DECIDIR_RUTA":
            print("Estado: DECIDIR_RUTA")
            if self.dist_frente < DISTANCIA_MINIMA_OBSTACULO:
                print("¡Demasiado cerca! Iniciando maniobra de escape.")
                self.auto_state = "MANIOBRA_ESCAPE"
                return # Salimos para ejecutar el escape en el siguiente ciclo

            # Escanear izquierda y derecha
            RPIservo.move(SERVO_PAN, 160); time.sleep(0.3)
            dist_left = ultra.checkdist(); self.send_radar_data(dist_left, 160)
            RPIservo.move(SERVO_PAN, 20); time.sleep(0.3)
            dist_right = ultra.checkdist(); self.send_radar_data(dist_right, 20)
            RPIservo.move(SERVO_PAN, 90) # Volver al centro

            if dist_left is None: dist_left = 0
            if dist_right is None: dist_right = 0
            
            if dist_left > dist_right:
                print(f"Ruta elegida: Izquierda (dist: {dist_left*100:.2f} cm)")
                self.turn_direction = "izquierda"
            else:
                print(f"Ruta elegida: Derecha (dist: {dist_right*100:.2f} cm)")
                self.turn_direction = "derecha"
            self.auto_state = "EJECUTAR_GIRO"

        # --- ESTADO 3: EJECUTAR GIRO ---
        elif self.auto_state == "EJECUTAR_GIRO":
            print(f"Estado: EJECUTAR_GIRO hacia la {self.turn_direction}")
            if self.turn_direction == "izquierda":
                RPIservo.move(SERVO_STEERING, 45) # Girar ruedas a la izquierda
            else: # Derecha
                RPIservo.move(SERVO_STEERING, 135) # Girar ruedas a la derecha
            
            # ¡Ahora avanzamos con las ruedas giradas!
            move.motor(1, 0, VELOCIDAD)
            #move.motor_right(1, 0, VELOCIDAD)
            time.sleep(0.8) # Tiempo para que el giro sea efectivo
            self.auto_state = "AVANZAR" # Volvemos a evaluar el camino

        # --- ESTADO 4: MANIOBRA DE ESCAPE ---
        elif self.auto_state == "MANIOBRA_ESCAPE":
            print("Estado: MANIOBRA_ESCAPE")
            # Primero, marcha atrás en línea recta para ganar espacio
            move.motor(1, 1, VELOCIDAD)
            #move.motor_right(1, 1, VELOCIDAD)
            time.sleep(1)

            # Luego, pivota sobre sí mismo para cambiar de dirección
            RPIservo.move(SERVO_STEERING, 45) # Ruedas a la izquierda
            move.motor(1, 1, VELOCIDAD) # Rueda izquierda atrás
            #move.motor_right(1, 0, VELOCIDAD) # Rueda derecha adelante
            time.sleep(0.8) # Tiempo de pivote
            self.auto_state = "AVANZAR" # Volvemos a evaluar el camino

    def trackLineProcessing(self):
        """
        Lógica de seguimiento de línea ADAPTADA para un solo motor de propulsión
        y un servo de dirección (Hardware PiCar-B).
        """
        VELOCIDAD = 70 # Puedes ajustar esta velocidad

        # --- LECTURA DE SENSORES ---
        status_right = GPIO.input(line_pin_right)
        status_middle = GPIO.input(line_pin_middle)
        status_left = GPIO.input(line_pin_left)
        
        # Recordatorio: 1 = Línea Negra, 0 = Superficie Blanca
        line_status = (status_left, status_middle, status_right)
        
        # --- LÓGICA DE DECISIÓN ---
        
        # Caso 1: Línea en el centro (0, 1, 0) -> Ruedas rectas y motor adelante
        if line_status == (0, 1, 0):
            RPIservo.move(SERVO_STEERING, 90)  # Dirección recta
            move.motor(1, 0, VELOCIDAD)
            self.last_turn_direction = 'none'

        # Caso 2: Se desvía a la derecha (1, 1, 0) o (1, 0, 0) -> Girar ruedas a la izquierda y avanzar
        elif line_status in [(1, 1, 0), (1, 0, 0)]:
            RPIservo.move(SERVO_STEERING, 45)  # Girar ruedas a la izquierda
            move.motor(1, 0, VELOCIDAD)
            self.last_turn_direction = 'left'

        # Caso 3: Se desvía a la izquierda (0, 1, 1) o (0, 0, 1) -> Girar ruedas a la derecha y avanzar
        elif line_status in [(0, 1, 1), (0, 0, 1)]:
            RPIservo.move(SERVO_STEERING, 135) # Girar ruedas a la derecha
            move.motor(1, 0, VELOCIDAD)
            self.last_turn_direction = 'right'

        # Caso 4: Línea perdida (0, 0, 0) -> Detener motor y girar ruedas para buscar
        elif line_status == (0, 0, 0):
            move.motorStop() # Detiene el avance
            # Gira las ruedas en la última dirección para buscar la línea
            if self.last_turn_direction == 'left':
                RPIservo.move(SERVO_STEERING, 45)
            elif self.last_turn_direction == 'right':
                RPIservo.move(SERVO_STEERING, 135)
        
        # Caso 5: Cruce o final (1, 1, 1) -> Parar completamente
        else: 
            move.motorStop()
