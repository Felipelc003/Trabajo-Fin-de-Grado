#!/usr/bin/python3
# File name   : ultra.py (Versión mejorada)
# Description : Detection distance and tracking with ultrasonic
# Website     : www.adeept.com
# Author      : William
# Date        : 2019/02/23

import RPi.GPIO as GPIO
import time

# --- Definición de Pines ---
Tr = 11
Ec = 8

# --- Configuración Inicial (se hace una sola vez) ---
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(Tr, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Ec, GPIO.IN)

print("Sensor ultrasónico inicializado.")

def checkdist():
    """Lee la distancia una vez. La configuración ya está hecha."""
    GPIO.output(Tr, GPIO.LOW)
    time.sleep(0.000002) # Pequeña pausa para asegurar un pulso limpio

    # Enviar el pulso de disparo
    GPIO.output(Tr, GPIO.HIGH)
    time.sleep(0.000015) # El pulso debe durar al menos 10µs
    GPIO.output(Tr, GPIO.LOW)

    # Medir el tiempo de la respuesta
    while not GPIO.input(Ec):
        pass
    t1 = time.time()

    while GPIO.input(Ec):
        pass
    t2 = time.time()

    # Calcular la distancia en metros
    distance = (t2 - t1) * 340 / 2
    return distance

# --- Bloque principal para pruebas ---
if __name__ == '__main__':
    try:
        while True:
            dist_m = checkdist()
            dist_cm = dist_m * 100
            print("Distancia: %.2f cm" % dist_cm)
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nPrograma detenido por el usuario.")
    finally:
        GPIO.cleanup() # Limpia los pines GPIO al salir
