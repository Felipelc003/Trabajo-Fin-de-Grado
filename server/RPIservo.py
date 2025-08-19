#!/usr/bin/python3
# File name : RPIservo.py
# Description : Control Servos using modern Adafruit CircuitPython libraries
# Author : Adaptado por Gemini
# Date : 2025/08/11

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Inicialización del Hardware ---
try:
    # Inicializa el bus I2C de la Raspberry Pi
    i2c = busio.I2C(board.SCL, board.SDA)

    # Crea una instancia del controlador PCA9685
    pca = PCA9685(i2c)
    pca.frequency = 50 # Frecuencia estándar para servos analógicos

    # Crea un array para acceder a los 16 servos fácilmente
    servos = [servo.Servo(pca.channels[i], min_pulse=500, max_pulse=2500) for i in range(16)]

    print("Controlador de servos PCA9685 inicializado correctamente.")

except Exception as e:
    print(f"Error: No se pudo inicializar la placa de servos PCA9685.")
    print(f"Detalle: {e}")
    exit()

# --- Funciones de Control ---

def move(servo_id, angle):
    """
    Mueve un servo a un ángulo específico (0 a 180).
    """
    if not 0 <= servo_id <= 15:
        print(f"Error: El ID del servo {servo_id} es inválido.")
        return
    if not 0 <= angle <= 180:
        print(f"Error: El ángulo {angle} es inválido.")
        return

    try:
        servos[servo_id].angle = angle
    except Exception as e:
        print(f"No se pudo mover el servo {servo_id}: {e}")

def stop(servo_id):
    """
    Libera un servo, deteniendo el envío de pulsos.
    """
    if not 0 <= servo_id <= 15:
        print(f"Error: El ID del servo {servo_id} es inválido.")
        return

    servos[servo_id].angle = None # Desactiva el servo

def cleanup():
    """
    Desactiva la placa PCA9685, liberando todos los servos.
    """
    print("Desactivando todos los servos.")
    pca.deinit()

# --- Bloque de prueba ---
if __name__ == '__main__':
    import time
    try:
        print("Iniciando prueba de servo 0. Presiona Ctrl+C para detener.")
        while True:
            move(0, 0)
            time.sleep(1)
            move(0, 90)
            time.sleep(1)
            move(0, 180)
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nPrueba detenida.")
    finally:
        cleanup()
