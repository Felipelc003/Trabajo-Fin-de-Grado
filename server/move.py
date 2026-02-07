#!/usr/bin/env python3
# File name   : move.py
# Description : Control del Motor DC (Canal B - Tracción Trasera)

import time
import RPi.GPIO as GPIO

# ==========================================
# Configuración de Pines (BCM) - Motor B
# ==========================================
# Ojo aquí: Si ves que el coche va al revés cuando le dices "adelante",
# solo tienes que cambiar los números de PIN_IN1 y PIN_IN2.
PIN_EN   = 4   # Enable (PWM) -> Esto controla la potencia/velocidad
PIN_IN1  = 14  # Input 1
PIN_IN2  = 15  # Input 2

# Estado Global
_pwm = None
_current_speed = 70  # Velocidad por defecto (0-100)

def setup():
    """Inicializa los pines GPIO y el PWM."""
    global _pwm
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Configurar pines como salida
    GPIO.setup(PIN_EN, GPIO.OUT)
    GPIO.setup(PIN_IN1, GPIO.OUT)
    GPIO.setup(PIN_IN2, GPIO.OUT)
    
    # Arrancamos el PWM a 1000Hz. Es el pulso que le da fuerza al motor.
    try:
        _pwm = GPIO.PWM(PIN_EN, 1000)
        _pwm.start(0) # Empieza parado (ciclo 0)
    except Exception as e:
        print(f"[Move] Error iniciando PWM: {e}")

def speed_set(speed_value):
    """Actualiza la variable global de velocidad."""
    global _current_speed
    try:
        val = int(speed_value)
        # Limitamos la velocidad (0 a 100)
        _current_speed = max(0, min(100, val))
    except ValueError:
        pass
    return _current_speed

def forward(speed=None):
    """Mueve el motor hacia adelante."""
    global _current_speed
    
    # Si no se pasa velocidad, usar la global
    s = speed if speed is not None else _current_speed
    s = max(0, min(100, int(s)))
    
    # Pin_IN1 y Pin_IN2 determina la dirección hacia delante
    GPIO.output(PIN_IN1, GPIO.LOW)
    GPIO.output(PIN_IN2, GPIO.HIGH)
    
    if _pwm:
        _pwm.ChangeDutyCycle(s)

def backward(speed=None):
    """Mueve el motor hacia atrás."""
    global _current_speed
    
    s = speed if speed is not None else _current_speed
    s = max(0, min(100, int(s)))
    
    # Invertimos la polaridad de los pines: ahora IN1 es HIGH y IN2 es LOW
    GPIO.output(PIN_IN1, GPIO.HIGH)
    GPIO.output(PIN_IN2, GPIO.LOW)
    
    if _pwm:
        _pwm.ChangeDutyCycle(s) # Indica la potencia a la uqe debe ir el motor

def stop():
    """Detiene el motor."""
    GPIO.output(PIN_IN1, GPIO.LOW)
    GPIO.output(PIN_IN2, GPIO.LOW)
    if _pwm:
        _pwm.ChangeDutyCycle(0) # Potencia cero

# Alias para compatibilidad con webServer.py / functions.py
motorStop = stop

def destroy():
    """Libera los recursos GPIO al cerrar."""
    stop()
    if _pwm:
        try:
            _pwm.stop()
        except: pass
    GPIO.cleanup() # Libera los pines GPIO

if __name__ == '__main__':
    # Test simple si se ejecuta directamente
    try:
        setup()
        print("Test: Adelante 50%")
        forward(50)
        time.sleep(2)
        print("Test: Atrás 50%")
        backward(50)
        time.sleep(2)
        stop()
        destroy()
    except KeyboardInterrupt:
        destroy()