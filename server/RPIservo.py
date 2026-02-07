#!/usr/bin/env python3
# Nombre del archivo: RPIservo.py
# Descripción: Módulo de control para servomotores utilizando el controlador PWM PCA9685.
# Este script gestiona la comunicación I2C y proporciona una interfaz de alto nivel para posicionar los servos.

import time
# Librerías de CircuitPython para acceso al hardware (I2C y pines).
import board
import busio
# Librerías de Adafruit para el controlador PCA9685 y gestión de servos.
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Estado Global del Módulo ---
pca = None          # Instancia del controlador PCA9685.
servos = []         # Lista que almacena los objetos de control para cada canal de servo.
_initialized = False # Bandera para controlar la inicialización única del sistema.

def init():
    """
    Inicializa la comunicación I2C y configura el controlador PCA9685.
    
    Esta función establece el bus I2C, configura la frecuencia PWM a 50Hz (estándar para servos)
    e instancia los objetos de control para los 16 canales disponibles en la placa.
    Si ocurre un error durante la inicialización, se captura la excepción y se deshabilita el control.
    """
    global pca, servos, _initialized
    
    # Evitar reinicialización si ya está configurado.
    if _initialized:
        return

    try:
        # Inicialización del bus I2C utilizando los pines SDA y SCL definidos por la placa.
        i2c = busio.I2C(board.SCL, board.SDA)
        
        # Creación de la instancia del controlador PCA9685.
        pca = PCA9685(i2c)
        pca.frequency = 50  # Frecuencia de 50Hz requerida para servos analógicos y digitales estándar.
        
        # Generación de los objetos Servo para los 16 canales (0-15).
        # Los anchos de pulso (min_pulse=500, max_pulse=2500) están ajustados para servos comunes tipo SG90/MG90S.
        servos = [servo.Servo(pca.channels[i], min_pulse=500, max_pulse=2500) for i in range(16)]
        
        _initialized = True
        print("[RPIservo] Controlador PCA9685 iniciado correctamente.")
        
    except Exception as e:
        print(f"[RPIservo] Error Crítico de Inicialización: No se detecta PCA9685. Detalles: {e}")
        print("[RPIservo] Funcionalidad de servos deshabilitada.")
        servos = [] #probar si sirve
        _initialized = False

def move(servo_id, angle):
    """
    Mueve un servo específico a una posición angular determinada.
    
    Parámetros:
    - servo_id (int): Identificador del canal del servo (0-15).
    - angle (float/int): Ángulo objetivo en grados (típicamente 0-180).
    
    La función valida que el servo esté inicializado y que el ID sea válido.
    Además, restringe el ángulo dentro de los límites seguros (0-180) para evitar daños físicos.
    """
    if not _initialized or not servos:
        return

    if 0 <= servo_id <= 15:
        # Restricción (clamping) del ángulo para asegurar que esté en el rango [0, 180].
        val = max(0, min(180, angle))
        try:
            servos[servo_id].angle = val
        except OSError:
            # Captura de errores de comunicación I2C (ruido, timeouts) para evitar caída del programa.
            pass

def stop(servo_id):
    """
    Desactiva la señal de control (PWM) para un servo específico.
    
    Esto "libera" el servo, permitiendo que se mueva libremente si se aplica fuerza externa,
    y detiene el consumo de energía para mantener la posición.
    
    Parámetros:
    - servo_id (int): Identificador del canal del servo (0-15).
    """
    if not _initialized: return
    
    if 0 <= servo_id <= 15:
        try:
            # Establecer el ángulo a None deshabilita la señal PWM en el canal especificado.
            servos[servo_id].angle = None
        except: pass

def cleanup():
    """
    Libera los recursos de hardware asociados al controlador PCA9685.
    Se debe llamar al detener la aplicación para asegurar un cierre limpio del bus I2C.
    """
    global pca
    if pca:
        try:
            pca.deinit()
        except: pass
    print("[RPIservo] Recursos liberados y controlador finalizado.")

# --- Auto-inicialización ---
# Se intenta inicializar el módulo automáticamente al ser importado.
# Esto simplifica el uso en otros scripts, asegurando que el hardware esté listo.
init()

if __name__ == '__main__':
    """
    Bloque de prueba unitaria.
    Ejecuta una secuencia de movimientos en los primeros tres servos para verificar el funcionamiento.
    """
    print("Iniciando prueba de diagnóstico de servos (Canales 0-2)...")
    time.sleep(1)
    for s in [0, 1, 2]:
        print(f"Probando Servo ID: {s}")
        move(s, 90)  # Posición central
        time.sleep(0.5)
        move(s, 60)  # Izquierda / Abajo
        time.sleep(0.5)
        move(s, 120) # Derecha / Arriba
        time.sleep(0.5)
        move(s, 90)  # Retorno al centro
    cleanup()
    print("Prueba finalizada.")