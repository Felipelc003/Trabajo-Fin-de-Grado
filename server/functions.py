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
import asyncio
from camera_opencv import Camera

# --- Definiciones de Servos (canales) ---
SERVO_TILT = 0
SERVO_PAN = 1
SERVO_STEERING = 2

# --- Ángulos del SERVO de dirección (ruedas) ---
STEER_RIGHT  = 60   # derecha
STEER_CENTER = 90   # recto
STEER_LEFT   = 130  # izquierda

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
        self.websocket = None
        self.event_loop = None
        setup_line_pins()

        # --- Estado seguidor de línea / giros ---
        self.last_turn_direction = 'none'   # 'left' | 'right' | 'center' | 'none'

        # --- Estados para QR ---
        self.awaiting_qr = False
        self.qr_ignore_until = 0.0          # ventana para ignorar cruce tras decidir QR

        self.qr_launch_ts = 0.0           # instante en el que se lanzó scanQR
        self.qr_launch_retry_done = False # si ya hicimos un único reintento de lanzamiento
        self.qr_turn_boost_until = 0.0


        # --- Estados para el modo automático (no usado aquí) ---
        self.auto_state = "AVANZAR"
        self.turn_direction = "derecha"
        self.dist_frente = 0

        super(Functions, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    # ------------------ Utilidades de RADAR (ultrasonidos con PAN) ------------------
    def send_radar_data(self, distance, angle):
        if self.websocket and self.event_loop:
            try:
                response = {'title': 'scanResult', 'data': [[distance, angle]]}
                asyncio.run_coroutine_threadsafe(
                    self.websocket.send(json.dumps(response)),
                    self.event_loop
                )
            except Exception as e:
                print(f"Error al enviar datos del radar: {e}")

    def radarScan(self, step=5, dwell_ms=80, samples=3, stream=True):
        print("Iniciando escaneo de radar...")
        try:
            RPIservo.move(SERVO_TILT, 90)
        except Exception as e:
            print(f"[radarScan] Aviso al centrar TILT: {e}")
        time.sleep(0.2)

        MIN_M, MAX_M = 0.02, 4.00
        full_scan_result = []

        forward = list(range(0, 181, step))
        backward = list(range(180, -1, -step))
        sweep = forward + backward[1:-1]

        def trimmed_mean(values, trim=0.34):
            if not values: return None
            vals = sorted(v for v in values if v is not None)
            if not vals: return None
            k = int(len(vals) * trim)
            vals = vals[k: len(vals) - k] if len(vals) > 2*k else vals
            return sum(vals) / len(vals) if vals else None

        for angle in sweep:
            try:
                RPIservo.move(SERVO_PAN, angle)
            except Exception as e:
                print(f"[radarScan] Error moviendo PAN a {angle}: {e}")
            time.sleep(dwell_ms / 1000.0)

            readings = []
            for _ in range(max(1, samples)):
                d = ultra.checkdist()
                if d is not None and MIN_M <= d <= MAX_M:
                    readings.append(d)
                time.sleep(0.02)

            dist = trimmed_mean(readings)
            if dist is None:
                continue

            display_angle = 180 - angle
            point = [dist, display_angle]
            full_scan_result.append(point)

            if stream:
                self.send_radar_data(dist, display_angle)

        time.sleep(0.2)
        try:
            RPIservo.move(SERVO_PAN, 90)
        except Exception:
            pass

        print(f"Escaneo de radar finalizado. Puntos válidos: {len(full_scan_result)}")
        return full_scan_result

    # ------------------ Control de modos ------------------
    def pause(self):
        self.functionMode = 'none'
        self.websocket = None
        self.event_loop = None
        try:
            move.stop()
        except Exception:
            try: move.motorStop()
            except: pass
        self.__flag.clear()
        print("Pausando todas las funciones activas.")

    def modeSet(self, mode):
        if mode == 'trackLine':
            self.trackLine()
        elif mode == 'Automatic':
            self.automatic(self.websocket, self.event_loop)
        elif mode == 'none':
            self.pause()
        else:
            print(f"[Functions] Modo desconocido: {mode}")

    def resume(self):
        self.__flag.set()

    def automatic(self, websocket, loop):
        print("🤖 Activando modo 'Automático' con radar en vivo.")
        self.websocket = websocket
        self.event_loop = loop
        self.functionMode = 'Automatic'
        self.auto_state = "forward"
        self.resume()

    def trackLine(self):
        self.functionMode = 'trackLine'
        self.last_turn_direction = 'none'
        self.qr_latched = False
        self.qr_last_try = 0.0
        self.resume()

    # ------------------ (Opcional) Maniobra QR directa — no usada en el enfoque actual ------------------
    def _execute_qr_maneuver(self, direction: str):
        """Mantener por compatibilidad; no se invoca en el enfoque 'QR como sesgo'."""
        VELOCIDAD = 55
        d = str(direction).strip().lower()
        try:
            cam = Camera.get_instance()
            cam.modeselect('none')
            cam.cv_thread.qr_scanning = False
            cam.cv_thread.last_qr_result = None
        except Exception:
            pass

        try:
            move.stop()
        except Exception:
            try: move.motorStop()
            except: pass
        RPIservo.move(SERVO_STEERING, STEER_CENTER)
        time.sleep(0.05)

        if d in ('left', 'izquierda'):
            RPIservo.move(SERVO_STEERING, STEER_LEFT)
            move.forward(VELOCIDAD)
            time.sleep(0.75)
        elif d in ('right', 'derecha'):
            RPIservo.move(SERVO_STEERING, STEER_RIGHT)
            move.forward(VELOCIDAD)
            time.sleep(0.75)
        elif d in ('forward', 'center', 'recto'):
            RPIservo.move(SERVO_STEERING, STEER_CENTER)
            move.forward(VELOCIDAD)
            time.sleep(0.6)

        # No recentramos aquí a propósito

    def _reset_qr_state(self):
        """Resetea todo el estado relacionado con QR y cámara."""
        self.awaiting_qr = False
        self.qr_ignore_until = 0.0
        self.qr_hold_until_exit_111 = False
        self.qr_lock_drive_until = 0.0
        self.qr_lock_angle = STEER_CENTER
        self.qr_wait_deadline = 0.0
        self.qr_launch_ts = 0.0
        self.qr_launch_retry_done = False
        self.qr_turn_boost_until = 0.0
        self.last_turn_direction = 'none'
        # Apagar modo scan en la cámara y limpiar último resultado
        try:
            cam = Camera.get_instance()
            cam.modeselect('none')
            cam.cv_thread.qr_scanning = False
            cam.cv_thread.last_qr_result = None
        except Exception:
            pass


    def automaticProcessing(self):
        # (Tu lógica automática si la usas)
        pass

    # ------------------ Seguimiento de línea + cruces con QR ------------------
    def trackLineProcessing(self):
        """
        - Sigue línea con 3 sensores.
        - Se para en bifurcaciones (L y R = 1), lanza scanQR y espera hasta qr_max_wait_s.
        - Si QR llega: mapea a 'last_turn_direction' como si fuese pérdida de línea por el lado opuesto.
        - Si no hay QR (timeout): salida por defecto (recto si centro=1, si no izquierda).
        - Activa ventana 'qr_ignore_until' para no quedarse atrapado en 111.
        """
        CENTER = STEER_CENTER
        LEFT   = STEER_LEFT
        RIGHT  = STEER_RIGHT
        VELOCIDAD = 50
        KP = 10

        # Asegurar flags para el reintento único del barrido QR
        if not hasattr(self, 'qr_launch_ts'):
            self.qr_launch_ts = 0.0
        if not hasattr(self, 'qr_launch_retry_done'):
            self.qr_launch_retry_done = False

        # Estado de cámara / escaneo
        try:
            cam = Camera.get_instance()
            scanning = bool(getattr(cam.cv_thread, 'qr_scanning', False))
        except Exception:
            cam = None
            scanning = False

        # Si está escaneando, PARADO y ruedas centradas
        if scanning:
            try:
                move.stop()
            except Exception:
                try: move.motorStop()
                except: pass
            RPIservo.move(SERVO_STEERING, CENTER)
            time.sleep(0.02)
            return

        # ---------- Esperando QR (sin fallback: se queda parado) ----------
        if self.awaiting_qr:
            # ¿Hay resultado de QR?
            data = None
            try:
                data = getattr(cam.cv_thread, 'last_qr_result', None)
            except Exception:
                pass
            choice = (str(data).strip().lower() if data else '')

            if choice:
                print(f"[trackLine] QR leído: {choice}")

                # Cerrar escaneo y limpiar
                try:
                    if cam:
                        cam.modeselect('none')
                        cam.cv_thread.last_qr_result = None
                        cam.cv_thread.qr_scanning = False
                except Exception:
                    pass

                # Ventana para NO re-parar en el cruce y poder salir
                self.qr_ignore_until = time.time() + 1.0
                self.awaiting_qr = False

                # MAPEO: usar QR de forma DIRECTA (misma dirección)
                if ('left' in choice) or ('izquierda' in choice):
                    self.last_turn_direction = 'left'
                    self.qr_turn_boost_until = time.time() + 0.9   # <<— BOOST
                    print("[trackLine] QR 'left' -> girar IZQUIERDA (last_turn_direction='left')")
                elif ('right' in choice) or ('derecha' in choice):
                    self.last_turn_direction = 'right'
                    self.qr_turn_boost_until = time.time() + 0.9   # <<— BOOST
                    print("[trackLine] QR 'right' -> girar DERECHA (last_turn_direction='right')")
                elif ('forward' in choice) or ('center' in choice) or ('recto' in choice):
                    self.last_turn_direction = 'center'
                    print("[trackLine] QR 'forward' -> recto (last_turn_direction='center')")
                else:
                    # QR raro: quedarse parado esperando otro
                    self.awaiting_qr = True
                    move.stop()
                    RPIservo.move(SERVO_STEERING, STEER_CENTER)
                    return

                return  # salimos; el seguidor aplicará el sesgo en la siguiente iteración

            # No hay QR aún y NO se está escaneando: posible final de barrido sin QR o que no arrancó
            if (not scanning) and (not self.qr_launch_retry_done):
                # Un único reintento si han pasado >0.8 s desde el lanzamiento
                if (time.time() - self.qr_launch_ts) > 0.8:
                    try:
                        if cam:
                            cam.modeselect('scanQR')
                            self.qr_launch_retry_done = True
                            print("[trackLine] Reintento único de scanQR (no arrancó el barrido)")
                    except Exception as e:
                        print(f"[trackLine] no se pudo relanzar scanQR: {e}")

            # En cualquier caso, sin QR -> PARADO y centrado
            try:
                move.stop()
            except Exception:
                try: move.motorStop()
                except: pass
            RPIservo.move(SERVO_STEERING, CENTER)
            return
        # ------------- LECTURA DE SENSORES -------------
        s_left = GPIO.input(line_pin_left)
        s_mid  = GPIO.input(line_pin_middle)
        s_right= GPIO.input(line_pin_right)
        print(f"Sensores (I-M-D): {s_left}-{s_mid}-{s_right}")

        # Ventana para ignorar cruce tras decidir QR
        now = time.time()
        ignore_cruce = now < getattr(self, 'qr_ignore_until', 0.0)

        # ------------- DETECCIÓN DE CRUCE -------------
        # Cruce: L y R a 1 (T o +), independientemente del centro
        is_cruce = (s_left == 1 and s_right == 1)

        if is_cruce and not ignore_cruce:
            move.stop()
            RPIservo.move(SERVO_STEERING, CENTER)

            # Lanzar escaneo si aún no estamos esperando QR
            if not self.awaiting_qr:
                try:
                    if cam:
                        cam.modeselect('scanQR')
                        print("[trackLine] scanQR lanzado en cruce (L y R = 1)")
                    self.awaiting_qr = True
                    self.qr_launch_ts = time.time()
                    self.qr_launch_retry_done = False  # habilita un ÚNICO reintento si no arranca

                except Exception as e:
                    print(f"[trackLine] no se pudo lanzar scanQR: {e}")
                return

            return  # parado en cruce mientras esperamos/gestionamos QR

        # ------------- CONTROL PROPORCIONAL DE DIRECCIÓN -------------
        target_angle = CENTER

        if s_mid == 1 and s_left == 0 and s_right == 0:
            target_angle = CENTER
            self.last_turn_direction = 'center'

        elif s_left == 1 and s_mid == 0:
            target_angle = min(LEFT, CENTER + KP)
            self.last_turn_direction = 'left'

        elif s_right == 1 and s_mid == 0:
            target_angle = max(RIGHT, CENTER - KP)
            self.last_turn_direction = 'right'

        elif s_left == 1 and s_mid == 1 and s_right == 0:
            target_angle = min(LEFT, CENTER + (KP // 2))
            self.last_turn_direction = 'left'

        elif s_right == 1 and s_mid == 1 and s_left == 0:
            target_angle = max(RIGHT, CENTER - (KP // 2))
            self.last_turn_direction = 'right'

        else:
            # Línea perdida o combinación rara (incluye 000 y también 111 cuando ignore_cruce=True)
            if self.last_turn_direction == 'left':
                target_angle = LEFT
            elif self.last_turn_direction == 'right':
                target_angle = RIGHT
            else:
                target_angle = CENTER

        # Clamp por seguridad
        target_angle = max(RIGHT, min(LEFT, target_angle))
        RPIservo.move(SERVO_STEERING, target_angle)

        # ------------- AVANCE -------------
        # Avanza solo si NO esperamos QR y NO estamos en cruce real (cuando no estamos en ventana de ignorar)
        if (not self.awaiting_qr) and (not (is_cruce and not ignore_cruce)):
            speed_now = 70 if time.time() < getattr(self, 'qr_turn_boost_until', 0.0) else VELOCIDAD
            move.forward(speed_now)
        time.sleep(0.05)

    # ------------------ Loop del hilo ------------------
    def functionGoing(self):
        if self.functionMode == 'none':
            pass
        elif self.functionMode == 'Automatic':
            self.automaticProcessing()
        elif self.functionMode == 'trackLine':
            self.trackLineProcessing()

    def run(self):
        while True:
            self.__flag.wait()
            if self.functionMode != 'none':
                self.functionGoing()
