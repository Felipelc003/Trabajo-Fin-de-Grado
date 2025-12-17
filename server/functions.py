#!/usr/bin/python3
# File name   : functions.py
# Description : Control loop (servo dirección + motores) consumiendo visión (5 ventanas) de camera_opencv
# Author      : Felipe (optimizado y modular)

import time
import threading

import move
import RPIservo
from camera_opencv import Camera
import numpy as np  # <-- NECESARIO para np.sign

# -----------------------------
# Constantes de dirección/marcha
# -----------------------------
SERVO_TILT = 0
SERVO_PAN  = 1
SERVO_STEERING = 2

# Dirección (ajusta a tu coche)
STEER_RIGHT        = 50
STEER_CENTER       = 88.5   # calibra "recto" real
STEER_LEFT         = 120
K_STEER            = 0.065  # deg/px (invierte a -0.08 si gira al revés)
KD_STEER           = 0.15  # Ganancia Derivativa
KI_STEER           = 0.01  # Ganancia Integral

# --- CONFIGURACIÓN SERVO PAN (CÁMARA) ---
PAN_CENTER         = 85     # Ángulo central (mirando al frente)
PAN_MAX_LEFT       = 130    # Límite físico
PAN_MAX_RIGHT      = 40

# --- AJUSTES DE SUAVIZADO PAN ---
PAN_GAIN           = 0.05   # Ganancia visual
PAN_CENTER_FORCE   = 0.03   # Fuerza de retorno al centro
PAN_MAX_STEP       = 4.0    # Máx grados por ciclo
PAN_DEADBAND       = 8      # Deadband visual para la cámara

# Velocidades
DRIVE_BASE_SPEED    = 60    # Velocidad base en curvas cerradas
DRIVE_MAX_SPEED     = 65    # Velocidad en rectas (default)
MIN_MOVE_SPEED      = 30    # vencer rozamiento

# Velocidades por color
SPEED_BLACK_BOOST   = 70    # Recta negra a tope (Braver)
SPEED_WHITE_NORMAL  = 50    # Normal
SPEED_YELLOW_MAX    = 40    # Velocidad máxima amarilla
YELLOW_CENTER_BIAS  = -5    # Desplazamiento del centro a la derecha en curvas amarillas
RED_STOP_TIME       = 2.0   # Tiempo de parada en rojo
RED_IGNORE_TIME     = 3.0   # Tiempo para ignorar rojo tras parar (cooldown)

# Maniobras (Curvas y Reversa)
SEARCH_TURN_SPEED      = 40   # Más fuerza al girar buscando
SEARCH_REVERSE_SPEED   = 50   # Más fuerza marcha atrás

# Ganancia de giro específica por color
STEER_GAIN_BLACK       = 0.15 # Antes 1/3 (~0.33). Reducido a 0.15 para suavizar
BLACK_DEADBAND_PX      = 40   # Ignorar errores menores en recta negra

# Penalización de velocidad por descentramiento en bandas bajas (2,3,4)
BOTTOM_CENTER_SLOW_THRESH_PX = 110   # a partir de ~110 px ya recorta
BOTTOM_CENTER_CUT_FRAC       = 0.25  # Menos frenada en curvas (Braver: 25%)
BOTTOM_CENTER_DEADBAND_PX    = 8     # tolerancia: ignora offsets muy pequeños

# Lógica de velocidad vs error (basada en NEAR o error mixto si NEAR falla)
ERR_SLOW_THRESH_PX  = 100   # |err| > esto → recorta velocidad
BLACK_SLOW_THRESH_PX = 180  # Más permisivo en recta negra (frena menos)
ERR_STOP_THRESH_PX  = 180   # |err| > esto → casi parar (más permisivo)

# Parada por pérdida de línea
NO_LINE_STOP_FRAMES = 5

# Rampa y frecuencia de órdenes
# RAMP_STEP           = 7   <-- BORRA O COMENTA ESTA LÍNEA
RAMP_STEP_UP          = 10   # Aceleración RÁPIDA (Braver)
RAMP_STEP_DOWN        = 10  # Frenada RÁPIDA
RAMP_HZ_LIMIT         = 30.0  # Hz máximos de envío de órdenes al motor

# --- Mezcla de 5 ventanas (0=top ... 4=bottom) ---
# Más peso a la banda inferior (near) como pediste
WIN_WEIGHTS = [0.0, 0.0, 0.0, 0.40, 0.60]  # suma ≈ 1.0
PRED_GAIN   = 0.35   # anticipación usando gradiente bottom-top

# --- Recortes suaves por MID/FAR (opcional) ---
MID_ERR_SLOW_THRESH = 100
FAR_ERR_SLOW_THRESH = 130
CUT_MID_FRAC = 0.15  # máx 15% extra por mid
CUT_FAR_FRAC = 0.30  # máx 15% extra por far

# --- Pre-giro anticipado por bandas ---
FAR_PRESTEER_DEG  = 6   # cuando SOLO far ve curva → gira ±10°
MID_PRESTEER_DEG  = 6   # si mid también la ve → suma ±10°
NEAR_PRESTEER_DEG = 8   # cuando near ya la ve → suma ligera (consolidación)

# Limitador de velocidad de giro del servo (suaviza cambios bruscos)
STEER_SLEW_DEG_PER_SEC = 60  # máx grados/seg que puede cambiar el servo
STEER_SLEW_RATE_BLACK  = 100  # máx grados/seg específico para línea negra


FRESH_TIMEOUT_SEC = 1.0   # antes 0.5; más permisivo para no perder frames

ANY_BAND_MIN_SPEED = 38

SEARCH_DEBOUNCE_FRAMES = 3      # nº de frames buenos para soltar latch
NEAR_EDGE_THRESH_PX    = 100    # opcional: exigir que se perdió estando "en el borde"
SEARCH_FORWARD_TIMEOUT_S   = 5.0  # Tiempo para la búsqueda hacia adelante

# --- Constantes para la maniobra de reversa ---
SEARCH_REACQUIRE_CENTER_PX = 40   # Umbral (en píxeles) para considerar la línea "centrada"

# Pesos de fallback near/mid/far (si no podemos mezclar 5)
NEAR_W, MID_W, FAR_W     = 0.70, 0.20, 0.10

# --- Suavizado del error ---
ERR_EMA_ALPHA     = 0.40   # 0..1 (más alto = responde más rápido)
ERR_EMA_ALPHA_BLACK = 0.15 # Muy bajo para suavizar recta negra
ERR_DEADBAND_PX   = 6      # ignora errores pequeños ±6 px
TANH_SCALE_PX     = 140    # compresión suave en saturaciones (opcional)
USE_TANH_SHAPING  = True   # activa compresión no lineal del error


class Functions(threading.Thread):
    """
    Hilo de CONTROL: consume el estado de línea publicado por camera_opencv (modo 'lineBlack')
    y gobierna servo de dirección + motores con rampa.
    """

    def __init__(self, *args, **kwargs):
        super(Functions, self).__init__(*args, **kwargs)

        # Estado de modo/ejecución
        self.functionMode = 'none'

        # Estado del seguidor (control)
        self.line_follow_active = False
        self._target_speed = 0
        self._current_speed = 0
        self._no_line_frames = 0
        self._last_drive_time = 0.0
        self._line_last_seq = None
        self._last_debug_log = 0.0
        self._ignore_red_until = 0.0 # Cooldown para no parar en bucle

        self._pan_angle = float(PAN_CENTER)
        self._pan_active = False

        # Evento para pausar/reanudar el bucle del hilo
        self.__flag = threading.Event()
        self.__flag.clear()

        self._servo_last_angle = STEER_CENTER
        self._servo_last_time  = time.time()

        # -------- Estado para el latch de búsqueda --------
        self.last_near_err  = None       # último err_near válido
        self.last_near_side = None       # 'left' | 'right' | None
        self.last_near_color = None
        self.search_latch   = None       # 'left' | 'right' | None
        self.search_debounce = 0         # frames de mejora hacia el centro

        self._err_ema = None   # último error filtrado para la mezcla

        # --- ESTADO PID ---
        self._pid_last_err = 0.0
        self._pid_integral = 0.0
        self._pid_last_time = time.time()


        # Motores listos
        try:
            RPIservo.move(SERVO_PAN, int(self._pan_angle))
            RPIservo.move(SERVO_TILT, 40)
        except:
            pass
        
        try:
            if hasattr(move, "setup"):
                move.setup()
        except Exception as e:
            print("[Functions] Aviso: move.setup() falló:", e)

    # --------------- Helpers de movimiento ---------------

    def _update_pan_servo(self, error_px):
        """
        Mueve la cámara persiguiendo el error visual con FUERZA DE RETORNO SUAVE.
        Devuelve el offset en grados desde el centro.
        """
        if error_px is None:
            # Si no hay línea o no queremos seguirla, volver al centro lentamente
            center_delta = (PAN_CENTER - self._pan_angle) * 0.1
            self._pan_angle += center_delta
        else:
            # 1. Calcular fuerza visual
            vision_delta = 0
            if abs(float(error_px)) > PAN_DEADBAND:
                # Error Positivo (Izq) -> Delta Positivo (Aumentar ángulo hacia Izq)
                vision_delta = float(error_px) * PAN_GAIN 

            # 2. Calcular fuerza de retorno (Elasticidad)
            center_delta = (PAN_CENTER - self._pan_angle) * PAN_CENTER_FORCE
            
            # 3. Sumar fuerzas y limitar (Slew Rate)
            total_delta = vision_delta + center_delta
            if total_delta > PAN_MAX_STEP: total_delta = PAN_MAX_STEP
            elif total_delta < -PAN_MAX_STEP: total_delta = -PAN_MAX_STEP

            self._pan_angle += total_delta

        # Clamp y aplicar
        self._pan_angle = max(PAN_MAX_RIGHT, min(PAN_MAX_LEFT, self._pan_angle))
        
        try:
            RPIservo.move(SERVO_PAN, int(self._pan_angle))
        except:
            pass

        return (self._pan_angle - PAN_CENTER)

    def _motor_stop(self):
        try:
            if hasattr(move, "motorStop"):
                move.motorStop()
            else:
                move.stop()
        except Exception:
            pass
        self._current_speed = 0

    def _set_target_speed(self, spd: int):
        spd = int(max(0, min(100, spd)))
        self._target_speed = spd

    def _ramp_and_drive(self):
        now = time.time()
        if now - self._last_drive_time < 1.0 / RAMP_HZ_LIMIT:
            return
        self._last_drive_time = now

        if self._current_speed < self._target_speed:
            # ESTAMOS ACELERANDO
            
            # 1. Si estamos por debajo de la mínima (o parados), saltamos a la mínima
            if self._current_speed < MIN_MOVE_SPEED:
                self._current_speed = MIN_MOVE_SPEED
            
            # 2. Si ya nos movemos, subimos suavemente (Rampa Lenta)
            else:
                self._current_speed = min(self._current_speed + RAMP_STEP_UP, self._target_speed)
        
        elif self._current_speed > self._target_speed:
            # ESTAMOS FRENANDO: Usamos paso grande (Rampa Rápida)
            self._current_speed = max(self._current_speed - RAMP_STEP_DOWN, self._target_speed)

        # --- APLICAR AL MOTOR ---
        v = int(self._current_speed)
        print("[Functions] Velocidad actual:", v, " Velocidad objetivo:", self._target_speed)

        # Si la velocidad es muy baja (menor que la mínima para moverse) y el target es 0, paramos.
        # (Mantenemos un pequeño margen para no cortar en seco si estamos frenando suave)
        if (not self.line_follow_active) or (v < MIN_MOVE_SPEED and self._target_speed == 0):
            self._motor_stop()
            return

        try:
            move.forward(v)
        except Exception as e:
            self._motor_stop()

    def _steer_from_err(self, err_px: int):
        """
        Mapea error de píxeles (izq:+ / der:-) a ángulo de servo.
        'shim' de compatibilidad: simple, fiable y rápido.
        """
        target = int(STEER_CENTER + K_STEER * int(err_px))
        target = max(STEER_RIGHT, min(STEER_LEFT, target))
        try:
            RPIservo.move(SERVO_STEERING, target)
        except Exception:
            pass
        return target

    def _steer_command(self, target_deg: int, slew_rate=STEER_SLEW_DEG_PER_SEC):
        """
        Aplica un limitador de pendiente (slew-rate) al servo para que no gire
        más rápido de slew_rate. Devuelve el ángulo realmente enviado.
        """
        now = time.time()
        dt = max(1e-3, now - self._servo_last_time)
        max_step = slew_rate * dt

        # clamp a los límites físicos
        target_deg = max(STEER_RIGHT, min(STEER_LEFT, int(target_deg)))

        # aplicar slew
        delta = target_deg - self._servo_last_angle
        if abs(delta) > max_step:
            target_deg = int(self._servo_last_angle + max_step * (1 if delta > 0 else -1))

        # enviar al servo
        try:
            RPIservo.move(SERVO_STEERING, target_deg)
        except Exception:
            pass

        self._servo_last_angle = target_deg
        self._servo_last_time  = now
        return target_deg

    def _filter_mix_err(self, err, alpha=ERR_EMA_ALPHA):
        """
        Aplica deadband, EMA y compresión suave (tanh) al error combinado.
        """
        if err is None:
            return None

        # Deadband: elimina micro-oscilaciones
        if abs(err) < ERR_DEADBAND_PX:
            err = 0.0

        # EMA (suavizado exponencial)
        if self._err_ema is None:
            self._err_ema = float(err)
        else:
            self._err_ema = (1.0 - alpha) * self._err_ema + alpha * float(err)
        e = self._err_ema

        # Compresión no lineal (evita órdenes extremas de golpe)
        if USE_TANH_SHAPING and TANH_SCALE_PX > 1:
            e = TANH_SCALE_PX * np.tanh(e / float(TANH_SCALE_PX))

        return e



    # --------------- API de modos (llamadas desde webServer/GUI) ---------------

    def pause(self):
        """
        Detiene de forma segura cualquier modo activo:
        - Para motores y congela rampas.
        - Centra la dirección (si hay servo).
        - Desactiva overlays / modos de cámara.
        - Pone el hilo de funciones en pausa.
        """
        self.functionMode = 'none'
        self.line_follow_active = False

        # Parar motores (con rampa si está disponible)
        try:
            self._set_target_speed(0)
            self._ramp_and_drive()
        except Exception:
            try:
                if hasattr(move, "motorStop"):
                    move.motorStop()
                else:
                    move.stop()
            except Exception:
                pass

        self._target_speed = 0
        self._current_speed = 0
        self._no_line_frames = 0
        self._line_last_seq = None

        # Centrar la dirección
        try:
            RPIservo.move(SERVO_STEERING, STEER_CENTER)
        except Exception:
            pass

        # Quitar overlays / modos de cámara
        try:
            cam = Camera.get_instance()
            cam.modeselect('none')
        except Exception:
            pass

        # Pausar hilo
        try:
            self.__flag.clear()
        except Exception:
            pass

        print("Pausa: motores OFF, cámara en 'none', dirección centrada.")

    def modeSet(self, mode: str):
        # Unifica Automatic y trackLine al mismo flujo por cámara
        if mode in ('trackLine', 'Automatic', 'automatic'):
            self.trackLine()
        elif mode == 'none':
            self.pause()
        else:
            print(f"[Functions] Modo desconocido: {mode}")

    def resume(self):
        self.__flag.set()

    def trackLine(self):
        """
        Seguir línea con cámara:
        - La cámara publica métricas (5 ventanas) en cv_thread.line_state (modo 'lineBlack').
        - Este hilo controla servo + motores en base a dichas métricas.
        """
        # Seguridad básica
        try:
            if hasattr(move, "motorStop"):
                move.motorStop()
            else:
                move.stop()
        except Exception:
            pass
        try:
            RPIservo.move(SERVO_STEERING, STEER_CENTER)
        except Exception:
            pass

        # Activar visión
        try:
            cam = Camera.get_instance()
            cam.modeselect('lineBlack')
            print("[trackLine] Cámara en 'lineBlack'. Control en functions.py")
        except Exception as e:
            print(f"[trackLine] No se pudo activar lineBlack: {e}")

        # Estado de control
        self._target_speed = 0
        self._current_speed = 0
        self._no_line_frames = 0
        self._line_last_seq = None
        self._last_debug_log = 0.0
        self.line_follow_active = True

        # Reset latch
        self.last_near_err = None
        self.last_near_side = None
        self.search_latch = None
        self.search_debounce = 0
        self._err_ema = None

        self.functionMode = 'trackLine'
        self.resume()

        # Pequeña tolerancia inicial: espera a la PRIMERA medición fresca
        try:
            cvp = Camera.get_instance().cv_thread
            t0 = time.time()
            while time.time() - t0 < 0.5:  # hasta 0.5s de margen
                st, seq = cvp.get_line_state(wait_new=True, last_seq=None, timeout=0.2)
                if (time.time() - st.get('timestamp', 0)) < FRESH_TIMEOUT_SEC:
                    break
        except Exception:
            pass


    # --------------- Lazo de control (se llama periódicamente desde run) ---------------

    def trackLineProcessing(self):
        """
        Dirección = mezcla ponderada de 5 ventanas + anticipación + pre-giro por bandas.
        Velocidad = avanza SIEMPRE que al menos una banda vea línea.
        Con latch de búsqueda: si se pierde NEAR, gira hacia el último lado visto
        hasta que NEAR reaparezca y su |error| empiece a bajar.
        """
        # 1) Productor (cámara)
        try:
            cam = Camera.get_instance()
            cvp = cam.cv_thread
        except Exception:
            time.sleep(0.1)
            return

        # 2) Esperar medición NUEVA
        st, seq = cvp.get_line_state(wait_new=True, last_seq=self._line_last_seq, timeout=0.25)
        self._line_last_seq = seq

        # 3) Datos
        now = time.time()
        fresh = (now - st.get('timestamp', 0)) < FRESH_TIMEOUT_SEC

        errs = st.get('errs')        # lista 5 (0=top..4=bottom)
        hasl = st.get('has_list')    # lista booleana 5
        en   = st.get('err_near', st.get('err', None))
        em   = st.get('err_mid',  None)
        ef   = st.get('err_far',  None)
        hn   = st.get('has_near', en is not None)
        hm   = st.get('has_mid',  em is not None)
        hf   = st.get('has_far',  ef is not None)


        color_list = st.get('color_list', [None]*5) 
        current_band_color = color_list[4] if (hasl and hasl[4]) else None
        last_color = None
        if current_band_color != last_color:
            print("[", current_band_color, "]")
            last_color = current_band_color

        any_band = isinstance(hasl, list) and any(hasl)

        # Variables de la SEGUNDA banda más cercana (índice 3)
        en3 = errs[3] if (isinstance(errs, list) and len(errs) > 3) else None
        hn3 = hasl[3] if (isinstance(hasl, list) and len(hasl) > 3) else False
        color3 = color_list[3] if (isinstance(color_list, list) and len(color_list) > 3) else None

        # 3.a) Actualizar "último lado" visto por la BANDA 3 (antes NEAR)
        # Recordatorio: err = center_x - cx  → err>0 => línea a la IZQUIERDA (cx a la izquierda)
        if hn3 and en3 is not None:
            self.last_near_side = 'left' if float(en3) > 0 else 'right'
            self.last_near_err  = float(en3)  # asegura que se actualiza
            self.last_near_color = color3

        # 3.b) Lógica de LATCH de búsqueda (Base: BANDA 3)
        # Activa latch sólo cuando NO hay BANDA 3 y la última vez estaba en el BORDE
        if (self.search_latch is None) and (self.last_near_side in ('left', 'right')):
            cond_perdida = (not hn3) or (not any_band)
            if cond_perdida:
                # Eliminamos la condición del "borde" (NEAR_EDGE_THRESH_PX)
                if (self.last_near_err is None) or (abs(self.last_near_err) >= NEAR_EDGE_THRESH_PX):
                    if self.last_near_color == 'yellow':
                        self.search_latch = 'search_forward_right'
                    elif self.last_near_side == 'left':
                        self.search_latch = 'search_forward_left'
                    else:
                        self.search_latch = 'search_forward_right'

                    self.search_debounce = 0
                    self.search_started_t = now  # para límite de tiempo

        # 3.c) Lógica de LATCH (TRANSICIÓN Y SALIDA - Base BANDA 3)
        if self.search_latch is not None:
            
            # Condición de SALIDA (Línea centrada en BANDA 3)
            if hn3 and en3 is not None and (abs(float(en3)) < SEARCH_REACQUIRE_CENTER_PX):
                self.search_debounce += 1
                if self.search_debounce >= SEARCH_DEBOUNCE_FRAMES:
                    self.search_latch = None  # ¡LATCH SUELTO! (Sale de la búsqueda)
                    self.search_debounce = 0
                    self._motor_stop() # Paramos motores inmediatamente
                    
            # Condición de TRANSICIÓN (Timeout Etapa 1 -> Etapa 2)
            elif self.search_latch in ('search_forward_left', 'search_forward_right'):

                timeout_limit = SEARCH_FORWARD_TIMEOUT_S
                if self.last_near_color == 'yellow':
                    #timeout_limit = 12.0 # 12s de búsqueda si era amarilla
                    pass
                else:
                    timeout_limit = SEARCH_FORWARD_TIMEOUT_S

                    if (now - self.search_started_t) > timeout_limit:
                        # Se acabó el tiempo, pasa a Etapa 2 (Reversa)
                        if self.search_latch == 'search_forward_left':
                            self.search_latch = 'search_reverse_left'
                        else:
                            self.search_latch = 'search_reverse_right'
                                
                # Si vemos la línea pero no está centrada, reseteamos debounce
                if hn3:
                    self.search_debounce = 0
                    
            # Si estamos buscando (en cualquier etapa) y no vemos la línea
            elif not hn3:
                self.search_debounce = 0

        # 4) Dirección base
        servo_base = None
        mix_err = None

        # 4.a) Si hay latch activo → giro fijo hacia ese lado (skip mezcla y pre-giro)
        if self.search_latch is not None:

           # Etapa 1: Búsqueda ADELANTE (Gira HACIA el lado perdido)
            if self.search_latch == 'search_forward_right':
                servo_cmd = STEER_RIGHT
            elif self.search_latch == 'search_forward_left':
                servo_cmd = STEER_LEFT
                
            # Etapa 2: Búsqueda REVERSA (Gira OPUESTO al lado perdido)
            elif self.search_latch == 'search_reverse_right':
                servo_cmd = STEER_LEFT
            elif self.search_latch == 'search_reverse_left':
                servo_cmd = STEER_RIGHT
                
            servo_pos = self._steer_command(servo_cmd)
        else:

            # 4.b) Dirección por mezcla de 5 ventanas con pesos WIN_WEIGHTS
            if isinstance(errs, list) and isinstance(hasl, list) and len(errs) == len(hasl) == 5 and any_band:
                acc = 0.0; wsum = 0.0
                for i, e in enumerate(errs):
                    if e is not None and hasl[i]:
                        acc  += WIN_WEIGHTS[i] * float(e)
                        wsum += WIN_WEIGHTS[i]
                if wsum > 0:
                    mix_err = acc / wsum
                    # anticipación: diferencia bottom-top (4 - 0)
                    e_top    = float(errs[0]) if (hasl[0] and errs[0] is not None) else mix_err
                    e_bottom = float(errs[4]) if (hasl[4] and errs[4] is not None) else mix_err
                    grad = e_bottom - e_top
                    mix_err = mix_err + PRED_GAIN * grad
                    
                    # Usar alpha suave si estamos en negro
                    alpha_use = ERR_EMA_ALPHA_BLACK if (current_band_color == 'black') else ERR_EMA_ALPHA
                    mix_err = self._filter_mix_err(mix_err, alpha=alpha_use)

            if mix_err is None:
                servo_base = STEER_CENTER
            else:
                now = time.time()
                dt = max(1e-3, now - self._pid_last_time)

                # --- LÓGICA PID ---
                error = float(mix_err) # Error P

                if current_band_color == 'black' and abs(error) < BLACK_DEADBAND_PX:
                    error = 0.0

                # Término I (con "anti-windup": si no hay línea, resetea)
                if not any_band:
                    self._pid_integral = 0.0
                else:
                    self._pid_integral += error * dt
                    self._pid_integral = max(-100.0, min(100.0, self._pid_integral)) # Límite

                # Término D
                derivative = (error - self._pid_last_err) / dt

                # Guardar para la próxima
                self._pid_last_err = error
                self._pid_last_time = now

                # Cálculo final de dirección
                kd_use = KD_STEER
                ki_use = KI_STEER
                pid_out = (K_STEER * error) + (ki_use * self._pid_integral) + (kd_use * derivative)

                # --- LÓGICA DE MOVIMIENTO DE CÁMARA (ACTIVE GAZE) ---
                pan_offset = 0
                if current_band_color == 'yellow':
                    # Persigue la línea amarilla
                    pan_offset = self._update_pan_servo(mix_err)
                    if pan_offset >= 75: pan_offset = 75
                elif current_band_color == 'white':
                    # Persigue la línea blanca 
                    pan_offset = self._update_pan_servo(mix_err)
                else:
                    # Vuelve al centro para negro/otros
                    self._update_pan_servo(None)
                    pan_offset = 0

                # --- CÁLCULO FINAL DE DIRECCIÓN ---
                if current_band_color == 'black':
                    servo_base = int(STEER_CENTER + pid_out)
                    if servo_base >= 91.5:
                        servo_base = 91.5
                    elif servo_base <= 85.5:
                        servo_base = 85.5
                if current_band_color == 'yellow' or current_band_color == 'white':
                    # Active Gaze: Dirección = Centro + OffsetCámara + PID
                    # Copiamos el giro de la cámara + corrección fina
                    servo_base = int(STEER_CENTER + pan_offset + pid_out)
                    
                else:
                    # Lógica estándar
                    servo_base = int(STEER_CENTER + pid_out)

            # 4.c) PRE-GIRO escalonado por bandas (empieza con FAR)
            pre_bias = 0
            if hf and ef is not None:
                pre_bias += int(np.sign(float(ef)) * FAR_PRESTEER_DEG)
            if hm and em is not None:
                pre_bias += int(np.sign(float(em)) * MID_PRESTEER_DEG)
            if hn and en is not None:
                pre_bias += int(np.sign(float(en)) * NEAR_PRESTEER_DEG)

            if servo_base is None:
                servo_base = STEER_CENTER

            servo_cmd = max(STEER_RIGHT, min(STEER_LEFT, servo_base + pre_bias))
            rate_use = STEER_SLEW_RATE_BLACK if (current_band_color == 'black') else STEER_SLEW_DEG_PER_SEC
            servo_pos = self._steer_command(servo_cmd, slew_rate=rate_use)

        # 5) Velocidad
        # A) PARADA EN ROJO
        if current_band_color == 'red' and self.search_latch is None and (time.time() > self._ignore_red_until):
            print("[Func] ROJO DETECTADO: Parando...")
            self._motor_stop()
            time.sleep(RED_STOP_TIME)
            self._set_target_speed(60)
            self._ramp_and_drive()
            self._ignore_red_until = time.time() + RED_IGNORE_TIME
            return

        if self.search_latch is not None:
            # --- MODO BÚSQUEDA ---
            if self.search_latch in ('search_forward_right', 'search_forward_left'):
                if self.last_near_color == 'yellow':
                    self._set_target_speed(SPEED_YELLOW_MAX)
                else:
                    self._set_target_speed(SEARCH_TURN_SPEED) # 40
                self._ramp_and_drive()
            else:
                try: # REVERSA FUERTE
                    move.backward(SEARCH_REVERSE_SPEED) # 38
                    self._current_speed = -SEARCH_REVERSE_SPEED
                except Exception: self._motor_stop()

        elif self.line_follow_active and fresh and any_band:
            # --- MODO NORMAL ---
            
            # Selección de velocidad MAX según color
            # Selección de velocidad MAX: Prioridad a AMARILLO (frenar si se ve en CUALQUIER ventana)
            is_yellow_any = False
            is_white_any = False
            if isinstance(color_list, list) and isinstance(hasl, list) and len(color_list) == len(hasl):
                for i in range(len(color_list)):
                    if hasl[i]:
                        if color_list[i] == 'yellow': is_yellow_any = True
                        if color_list[i] == 'white':  is_white_any = True
            
            target_max = DRIVE_MAX_SPEED
            
            if is_yellow_any:
                target_max = SPEED_YELLOW_MAX # 45 - Prioridad absoluta: si veo amarillo, freno
                if self._current_speed > target_max:
                    self._current_speed = float(target_max)

            elif is_white_any:
                target_max = SPEED_WHITE_NORMAL # 50
            elif current_band_color == 'black':
                target_max = SPEED_BLACK_BOOST # 70 - Solo si NO hay amarillo/blanco y estoy en negro

            ref_err = float(en) if (hn and en) else (float(mix_err) if mix_err else 0.0)
            abs_ref = abs(ref_err)
            
            if abs_ref >= ERR_STOP_THRESH_PX:
                base = max(0, int(DRIVE_BASE_SPEED * 0.2))
            else:
                thresh_use = BLACK_SLOW_THRESH_PX if (current_band_color == 'black') else ERR_SLOW_THRESH_PX
                k = max(0.0, min(1.0, abs_ref / float(thresh_use)))
                base = int(DRIVE_BASE_SPEED + (target_max - DRIVE_BASE_SPEED) * (1.0 - k))
                base = max(base, MIN_MOVE_SPEED)

            cut_mid = cut_far = 0
            if hm and em is not None:
                k_mid = max(0.0, min(1.0, abs(float(em)) / float(MID_ERR_SLOW_THRESH)))
                cut_mid = int(CUT_MID_FRAC * (DRIVE_MAX_SPEED - MIN_MOVE_SPEED) * k_mid)
            if hf and ef is not None:
                k_far = max(0.0, min(1.0, abs(float(ef)) / float(FAR_ERR_SLOW_THRESH)))
                cut_far = int(CUT_FAR_FRAC * (DRIVE_MAX_SPEED - MIN_MOVE_SPEED) * k_far)

            errs_abs = []
            if isinstance(errs, list) and isinstance(hasl, list) and len(errs) >= 5 and len(hasl) >= 5: # Asume 5 o 10 bandas
                num_bands_to_check = len(errs) // 2 # Chequea la mitad inferior
                start_index = len(errs) - num_bands_to_check
                for i in range(start_index, len(errs)):
                    if hasl[i] and errs[i] is not None:
                        off = abs(float(errs[i]))
                        off = max(0.0, off - float(BOTTOM_CENTER_DEADBAND_PX))
                        errs_abs.append(off)

            extra_cut = 0
            if errs_abs:
                max_off  = max(errs_abs)
                k_center = max(0.0, min(1.0, max_off / float(BOTTOM_CENTER_SLOW_THRESH_PX)))
                extra_cut = int(BOTTOM_CENTER_CUT_FRAC * (DRIVE_MAX_SPEED - MIN_MOVE_SPEED) * k_center)

            desired = base - (cut_mid + cut_far + extra_cut)

            if not hn:
                desired = max(desired, ANY_BAND_MIN_SPEED)
            
            self._no_line_frames = 0
            self._set_target_speed(int(max(MIN_MOVE_SPEED, desired)))
            # move.forward(self._target_speed)
            self._ramp_and_drive() # ¡Aquí SÍ usamos la rampa!

        else:
            # --- MODO PARADA (Sin línea Y sin búsqueda) ---
            self._no_line_frames += 1
            if self._no_line_frames >= NO_LINE_STOP_FRAMES:
                RPIservo.move(SERVO_STEERING, STEER_RIGHT)
                self._set_target_speed(40)
                move.forward(self._target_speed)
                # self._ramp_and_drive() 

        # 6) Memorias para próxima iteración

        try:
            cam = Camera.get_instance()
            cvp = cam.cv_thread
        except Exception:
            pass


    # --------------- Bucle del hilo ---------------

    def functionGoing(self):
        if self.functionMode == 'trackLine':
            self.trackLineProcessing()
        # (otros modos en el futuro)

    def run(self):
        while True:
            self.__flag.wait()          # espera a que haya algún modo activo
            if self.functionMode != 'none':
                self.functionGoing()
