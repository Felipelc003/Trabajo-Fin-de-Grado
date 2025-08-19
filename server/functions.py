#!/usr/bin/python3
# File name   : functions.py
# Description : Control Functions using modern libraries
# Author      : William (Adaptado por Felipe y Gemini)
# Date        : 2025/08/11

import time
import RPi.GPIO as GPIO
import threading
import os
import json
import ultra
import Kalman_filter
import move
import RPIservo # Nuestro RPIservo.py moderno

# --- Definiciones de Servos para Claridad ---
SERVO_TILT = 0     # Servo de inclinación vertical de la cámara
SERVO_PAN = 1      # Servo de giro horizontal de la cámara
SERVO_STEERING = 2 # Servo de dirección de las ruedas

# --- Inicialización de Módulos ---
# NOTA: Se han eliminado las inicializaciones de 'ServoCtrl' y 'PCA9685'
# RPIservo.py se encarga de todo el hardware de servos.
move.setup()
kalman_filter_X = Kalman_filter.Kalman_filter(0.01, 0.1)

# La configuración de pines para el seguidor de línea se mantiene igual
line_pin_right = 20
line_pin_middle = 16
line_pin_left = 19

# MPU6050 (Giroscopio/Acelerómetro) - Puede necesitar instalación de librería
MPU_connection = 1
try:
    from mpu6050 import mpu6050
    sensor = mpu6050(0x68)
    print('mpu6050 conectado.')
except:
    MPU_connection = 0
    print('mpu6050 desconectado o librería no instalada. El modo "Steady" no funcionará.')

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
        setup_line_pins()

        super(Functions, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    def radarScan(self):
        """ Realiza un escaneo de radar moviendo el servo de paneo (PAN). """
        print("Iniciando escaneo de radar...")
        result = []
        RPIservo.move(SERVO_TILT, 90) # Pone el tilt en el centro
        time.sleep(0.5)

        # Barrido de 0 a 180 grados
        for angle in range(0, 181, 3): # Escanea en pasos de 3 grados
            RPIservo.move(SERVO_PAN, angle)
            time.sleep(0.03) # Pequeña pausa para que el servo llegue
            dist = ultra.checkdist()
            if dist < 5: # Ignora distancias muy lejanas (posible ruido)
                # El ángulo para el resultado es el mismo que el del servo
                result.append([dist, angle])

        # Vuelve al centro
        time.sleep(0.5)
        RPIservo.move(SERVO_PAN, 90)
        print("Escaneo de radar finalizado.")
        return result

    def pause(self):
        self.functionMode = 'none'
        move.motorStop()
        self.__flag.clear()

    def resume(self):
        self.__flag.set()

    def automatic(self):
        self.functionMode = 'Automatic'
        self.resume()

    def trackLine(self):
        self.functionMode = 'trackLine'
        self.resume()

    def keepDistance(self):
        self.functionMode = 'keepDistance'
        self.resume()

    def steady(self, goalPos):
        if MPU_connection == 0:
            print("Modo Steady no disponible, MPU6050 no conectado.")
            return
        self.functionMode = 'Steady'
        # 'goalPos' era un valor PWM, ahora necesitamos un ángulo. Usamos 90 (centro).
        self.steadyGoal = 90 
        self.resume()

    def trackLineProcessing(self):
        status_right = GPIO.input(line_pin_right)
        status_middle = GPIO.input(line_pin_middle)
        status_left = GPIO.input(line_pin_left)

        if status_middle == 0: # Línea en el centro
            RPIservo.move(SERVO_STEERING, 90) # Dirección recta
            move.motor_left(1, 0, 80)
            move.motor_right(1, 0, 80)
        elif status_left == 0: # Línea a la izquierda
            RPIservo.move(SERVO_STEERING, 45) # Gira a la izquierda
            move.motor_left(1, 0, 80)
            move.motor_right(1, 0, 80)
        elif status_right == 0: # Línea a la derecha
            RPIservo.move(SERVO_STEERING, 135) # Gira a la derecha
            move.motor_left(1, 0, 80)
            move.motor_right(1, 0, 80)
        else: # No hay línea
            move.motorStop()
        time.sleep(0.1)

    def automaticProcessing(self):
        """ Lógica de evasión de obstáculos. """
        RPIservo.move(SERVO_PAN, 90) # Mira al frente
        dist = ultra.checkdist() * 100 # a cm
        print(f"Distancia: {dist:.2f} cm")

        if dist > 40:
            RPIservo.move(SERVO_STEERING, 90) # Recto
            move.motor_left(1, 0, 80)
            move.motor_right(1, 0, 80)
        elif dist > 20:
            move.motorStop()
            RPIservo.move(SERVO_PAN, 160) # Mira a la izquierda
            time.sleep(0.3)
            dist_left = ultra.checkdist() * 100

            RPIservo.move(SERVO_PAN, 20) # Mira a la derecha
            time.sleep(0.3)
            dist_right = ultra.checkdist() * 100

            RPIservo.move(SERVO_PAN, 90) # Vuelve al centro
            if dist_left > dist_right:
                print("Girando a la izquierda")
                RPIservo.move(SERVO_STEERING, 45)
                move.motor_left(1, 1, 80) # Gira sobre sí mismo
                move.motor_right(1, 0, 80)
            else:
                print("Girando a la derecha")
                RPIservo.move(SERVO_STEERING, 135)
                move.motor_left(1, 0, 80) # Gira sobre sí mismo
                move.motor_right(1, 1, 80)
            time.sleep(0.5)
        else:
            print("Marcha atrás")
            move.motor_left(1, 1, 80)
            move.motor_right(1, 1, 80)
            time.sleep(0.5)
            move.motorStop()

    def steadyProcessing(self):
        if MPU_connection:
            accel_data = sensor.get_accel_data()
            x_accel = accel_data['x']
            x_filtered = kalman_filter_X.kalman(x_accel)

            # Mapea la aceleración a un ángulo de corrección
            # Este factor de '20' puede necesitar ajuste
            correction_angle = x_filtered * 20

            # Calcula el nuevo ángulo del servo
            new_angle = self.steadyGoal + correction_angle

            # Limita el ángulo para no forzar el servo
            if new_angle < 45: new_angle = 45
            if new_angle > 135: new_angle = 135

            RPIservo.move(SERVO_TILT, new_angle)
        time.sleep(0.05)

    def keepDisProcessing(self):
        distance_to_keep = 0.4 # Mantener 40 cm
        dist = ultra.checkdist()

        if dist > distance_to_keep + 0.1:
            move.motor_left(1, 0, 70)
            move.motor_right(1, 0, 70)
        elif dist < distance_to_keep - 0.1:
            move.motor_left(1, 1, 70)
            move.motor_right(1, 1, 70)
        else:
            move.motorStop()

    def functionGoing(self):
        if self.functionMode == 'none':
            self.pause()
        elif self.functionMode == 'Automatic':
            self.automaticProcessing()
        elif self.functionMode == 'Steady':
            self.steadyProcessing()
        elif self.functionMode == 'trackLine':
            self.trackLineProcessing()
        elif self.functionMode == 'keepDistance':
            self.keepDisProcessing()

    def run(self):
        while True:
            self.__flag.wait()
            self.functionGoing()

if __name__ == '__main__':
    # Bloque de prueba
    try:
        fuc = Functions()
        fuc.start()
        print("Iniciando prueba de seguimiento de línea.")
        fuc.trackLine()
        time.sleep(10)
        print("Deteniendo prueba.")
        fuc.pause()
        time.sleep(1)
    except KeyboardInterrupt:
        move.motorStop()
