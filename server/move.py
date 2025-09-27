#!/usr/bin/env python3
# File name   : move.py
# Description : Control Motor
# Product     : GWR
# Website     : www.gewbot.com
# Author      : William
# Date        : 2019/07/24
import time
import RPi.GPIO as GPIO

# ========= Configuración =========
# En PiCar-B se usa UN solo canal del HAT de motor. Tu motor está en A.
SELECTED_CHANNEL = 'A'   # 'A' o 'B' (tú usas A)

# Pines (BCM) del driver:
Motor_A_EN    = 4
Motor_B_EN    = 17
Motor_A_Pin1  = 14
Motor_A_Pin2  = 15
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

# Direcciones:
Dir_forward   = 0
Dir_backward  = 1

# Estado interno:
_pwm = None          # PWM del canal elegido
_pwm_started = False
_EN  = None          # pin EN del canal elegido
_IN1 = None          # pin IN1 del canal elegido
_IN2 = None          # pin IN2 del canal elegido
_current_speed = 60  # % duty (0–100)

# ========= Utilidades internas =========
def _clamp_speed(v):
    try:
        v = int(v)
    except:
        v = _current_speed
    return max(0, min(100, v))

def _ensure_pwm_started():
    global _pwm_started
    if _pwm is not None and not _pwm_started:
        try:
            _pwm.start(0)
        except RuntimeError:
            # Si ya estaba iniciado en otro sitio, lo ignoramos
            pass
        _pwm_started = True

def _apply(status, direction, speed):
    """Aplica orden al ÚNICO motor: on/off + sentido + PWM."""
    if status == 0:
        # Parar y deshabilitar EN
        GPIO.output(_IN1, GPIO.LOW)
        GPIO.output(_IN2, GPIO.LOW)
        GPIO.output(_EN, GPIO.LOW)
        return

    s = _clamp_speed(speed)

    # Sentido
    if direction == Dir_backward:
        GPIO.output(_IN1, GPIO.HIGH)
        GPIO.output(_IN2, GPIO.LOW)
    else:  # Dir_forward
        GPIO.output(_IN1, GPIO.LOW)
        GPIO.output(_IN2, GPIO.HIGH)

    # PWM (habilita EN y aplica duty)
    GPIO.output(_EN, GPIO.HIGH)
    _ensure_pwm_started()
    if _pwm is not None:
        try:
            _pwm.ChangeDutyCycle(s)
        except RuntimeError:
            # Si el PWM no está listo por alguna razón, reintenta iniciar
            try:
                _pwm.start(0)
                _pwm.ChangeDutyCycle(s)
            except Exception as e:
                print(f"[move] Error aplicando PWM: {e}")

# ========= API pública =========
def setup():
    """Inicializa GPIO y selecciona canal A o B. Idempotente: se puede llamar varias veces."""
    global _pwm, _EN, _IN1, _IN2, _pwm_started
    GPIO.setwarnings(False)
    # No cambiamos el modo si ya estaba configurado por otra parte del programa.
    try:
        GPIO.getmode()
    except Exception:
        pass
    GPIO.setmode(GPIO.BCM)

    # Config de pines (idempotente)
    for p in (Motor_A_EN, Motor_B_EN, Motor_A_Pin1, Motor_A_Pin2, Motor_B_Pin1, Motor_B_Pin2):
        try:
            GPIO.setup(p, GPIO.OUT)
        except RuntimeError:
            # Si ya estaba configurado, seguimos
            pass

    # Resolver canal
    if SELECTED_CHANNEL.upper() == 'A':
        _EN, _IN1, _IN2 = Motor_A_EN, Motor_A_Pin1, Motor_A_Pin2
    else:
        _EN, _IN1, _IN2 = Motor_B_EN, Motor_B_Pin1, Motor_B_Pin2

    motorStop()

    # Crear PWM solo si no existe
    if _pwm is None:
        try:
            _pwm = GPIO.PWM(_EN, 1000)  # 1kHz
            _pwm_started = False
        except RuntimeError as e:
            # Otro PWM puede estar asociado ya; en ese caso asumimos que lo gestiona otro init
            print(f"[move] Aviso: PWM ya existente en GPIO { _EN }: {e}")
            _pwm = None
            _pwm_started = False
    else:
        # Si ya existe, aseguramos frecuencia y duty 0
        try:
            _pwm.ChangeFrequency(1000)
            _ensure_pwm_started()
            _pwm.ChangeDutyCycle(0)
        except Exception as e:
            print(f"[move] Aviso al reconfigurar PWM existente: {e}")

def motorStop():
    """Para el motor y deshabilita EN (sin detener el objeto PWM)."""
    try:
        GPIO.output(_IN1, GPIO.LOW)
        GPIO.output(_IN2, GPIO.LOW)
        GPIO.output(_EN, GPIO.LOW)
    except Exception:
        pass

def stop():
    """Alias claro."""
    motorStop()

def speed_set(v=None):
    """Ajusta/consulta la velocidad por defecto (0–100)."""
    global _current_speed
    if v is None:
        return _current_speed
    _current_speed = _clamp_speed(v)
    return _current_speed

def forward(speed=None):
    """Avanza (el servo de dirección se controla en otro módulo)."""
    s = _current_speed if speed is None else _clamp_speed(speed)
    _apply(1, Dir_forward, s)

def backward(speed=None):
    """Retrocede."""
    s = _current_speed if speed is None else _clamp_speed(speed)
    _apply(1, Dir_backward, s)

# ========= Compatibilidad con código existente =========
def motor(status, direction, speed=None):
    """
    Compat: motor(1|0, 0|1, [speed])
    - status: 1 on / 0 off
    - direction: 0 forward / 1 backward
    - speed: opcional, usa la global si es None
    """
    if status == 0:
        stop(); return
    s = _current_speed if speed is None else _clamp_speed(speed)
    _apply(1, direction, s)

def destroy():
    global _pwm, _pwm_started
    try:
        motorStop()
        if _pwm is not None:
            try:
                _pwm.ChangeDutyCycle(0)
            except Exception:
                pass
            try:
                _pwm.stop()
            except Exception:
                pass
            _pwm = None
            _pwm_started = False
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass

if __name__ == '__main__':
    try:
        setup()
        speed_set(60)
        forward(); time.sleep(1.0)
        stop();   time.sleep(0.2)
        backward(); time.sleep(1.0)
        stop()
        destroy()
    except KeyboardInterrupt:
        destroy()
