#!/usr/bin/python3
# File name   : test_led_derecho.py
# Description : Testeo final y aislado para el LED delantero derecho.

import RPi.GPIO as GPIO
import time

# --- Definición de Pines (BCM) para el LED Derecho ---
PIN_R = 10
PIN_G = 9
PIN_B = 25

pins = [PIN_R, PIN_G, PIN_B]

# Lógica de encendido: LOW = ON, HIGH = OFF
ON = GPIO.LOW
OFF = GPIO.HIGH

def setup():
    """Configura los pines GPIO como salidas."""
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)

def all_off():
    """Apaga todos los colores del LED."""
    print("Apagando todos los colores...")
    for pin in pins:
        GPIO.output(pin, OFF)

print("=============================================")
print("===  Prueba Final del LED Delantero Derecho ===")
print("=============================================")

try:
    setup()
    all_off()
    time.sleep(2)

    print("--> Intentando encender ROJO (GPIO 10)")
    GPIO.output(PIN_R, ON)
    time.sleep(3)
    all_off()
    time.sleep(1)

    print("--> Intentando encender VERDE (GPIO 9)")
    GPIO.output(PIN_G, ON)
    time.sleep(3)
    all_off()
    time.sleep(1)

    print("--> Intentando encender AZUL (GPIO 25)")
    GPIO.output(PIN_B, ON)
    time.sleep(3)
    all_off()

    print("\nPrueba finalizada.")

except Exception as e:
    print(f"\nOcurrió un error durante la prueba: {e}")
finally:
    print("Limpiando pines GPIO.")
    GPIO.cleanup()
