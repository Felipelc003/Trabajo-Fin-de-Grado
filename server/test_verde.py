import RPi.GPIO as GPIO
import time

PIN_VERDE = 23

print(f"Iniciando prueba directa del Pin GPIO {PIN_VERDE}...")

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_VERDE, GPIO.OUT)

try:
    print("Intentando APAGAR el pin (GPIO.HIGH)...")
    GPIO.output(PIN_VERDE, GPIO.HIGH)
    time.sleep(4)

    print("Intentando ENCENDER el pin (GPIO.LOW)...")
    GPIO.output(PIN_VERDE, GPIO.LOW)
    time.sleep(4)

    print("Apagando de nuevo el pin...")
    GPIO.output(PIN_VERDE, GPIO.HIGH)
    GPIO.cleanup()
    print("Prueba finalizada.")

except Exception as e:
    print(f"Ocurrió un error: {e}")
    GPIO.cleanup()
