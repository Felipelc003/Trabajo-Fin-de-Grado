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
SERVO_PAN = 1
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
PAN_MAX_LEFT       = 170    # Límite físico
PAN_MAX_RIGHT      = 10

# --- AJUSTES DE SUAVIZADO ---
PAN_KP             = 0.025  # BAJAMOS de 0.06 a 0.035 (Más suave)
PAN_RETURN_SPEED   = 2
PAN_TO_STEER_GAIN  = 0.8   # Bajamos un poco el acople con las ruedas
PAN_DEADBAND       = 15     # NUEVO: Si el error es < 10px, la cámara NO se mueve (evita temblores)
PAN_MAX_STEP       = 1.0    # NUEVO: Máx grados que puede girar por ciclo (evita saltos bruscos)

# Velocidades
DRIVE_BASE_SPEED    = 35
DRIVE_MAX_SPEED     = 40
MIN_MOVE_SPEED      = 30    # vencer rozamiento

# Penalización de velocidad por descentramiento en bandas bajas (2,3,4)
BOTTOM_CENTER_SLOW_THRESH_PX = 110   # a partir de ~110 px ya recorta
BOTTOM_CENTER_CUT_FRAC       = 0.45  # hasta un 45% del margen (MAX - MIN)
BOTTOM_CENTER_DEADBAND_PX    = 8     # tolerancia: ignora offsets muy pequeños

# Lógica de velocidad vs error (basada en NEAR o error mixto si NEAR falla)
ERR_SLOW_THRESH_PX  = 100   # |err| > esto → recorta velocidad
ERR_STOP_THRESH_PX  = 160   # |err| > esto → casi parar

# Parada por pérdida de línea
NO_LINE_STOP_FRAMES = 5

# Rampa y frecuencia de órdenes
# RAMP_STEP           = 7   <-- BORRA O COMENTA ESTA LÍNEA
RAMP_STEP_UP          = 1   # Aceleración LENTA (1 unidad por ciclo)
RAMP_STEP_DOWN        = 10  # Frenada RÁPIDA (10 unidades por ciclo, casi instantánea)
RAMP_HZ_LIMIT         = 15.0  # Hz máximos de envío de órdenes al motor

# Depuración (sube para menos logs; 0 = silencio)
DEBUG_DRIVE_LOG_EVERY = 0.0

# --- Mezcla de 5 ventanas (0=top ... 4=bottom) ---
# Más peso a la banda inferior (near) como pediste
WIN_WEIGHTS = [0.15, 0.25, 0.30, 0.20, 0.10]  # suma ≈ 1.0
PRED_GAIN   = 0.35   # anticipación usando gradiente bottom-top

# --- Recortes suaves por MID/FAR (opcional) ---
MID_ERR_SLOW_THRESH = 100
FAR_ERR_SLOW_THRESH = 130
CUT_MID_FRAC = 0.15  # máx 15% extra por mid
CUT_FAR_FRAC = 0.30  # máx 15% extra por far

# --- Mínimo seguro si sólo tenemos bandas superiores ---
SAFE_MIN_WHEN_FAR_ONLY = 32

# --- Pre-giro anticipado por bandas ---
FAR_PRESTEER_DEG  = 6   # cuando SOLO far ve curva → gira ±10°
MID_PRESTEER_DEG  = 6   # si mid también la ve → suma ±10°
NEAR_PRESTEER_DEG = 8   # cuando near ya la ve → suma ligera (consolidación)

# Limitador de velocidad de giro del servo (suaviza cambios bruscos)
STEER_SLEW_DEG_PER_SEC = 60  # máx grados/seg que puede cambiar el servo

FRESH_TIMEOUT_SEC = 1.0   # antes 0.5; más permisivo para no perder frames

ANY_BAND_MIN_SPEED = 38

# --- cómo buscar cuando estamos en latch ---
SEARCH_TURN_DEG        = 14   # giro suave hacia el lado perdido
SEARCH_TURN_SPEED      = 32   # velocidad lenta mientras buscamos
#SEARCH_TURN_MAX_TIME_S = 2.0  # seguridad
SERACH_TURN_MAX_TIME_S = 20.0
SEARCH_DEBOUNCE_FRAMES = 3      # nº de frames buenos para soltar latch
SEARCH_HYST_PX         = 1.0    # margen de histéresis para considerar "mejora" de |err|
NEAR_EDGE_THRESH_PX    = 140    # opcional: exigir que se perdió estando "en el borde"
SEARCH_FORWARD_TIMEOUT_S   = 5.0  # Tiempo para la búsqueda hacia adelante

# --- Constantes para la maniobra de reversa ---
SEARCH_REVERSE_SPEED       = 28   # Velocidad de marcha atrás (ajusta según tu motor)
SEARCH_REACQUIRE_CENTER_PX = 40   # Umbral (en píxeles) para considerar la línea "centrada"

# Pesos de fallback near/mid/far (si no podemos mezclar 5)
NEAR_W, MID_W, FAR_W     = 0.70, 0.20, 0.10

# --- Suavizado del error ---
ERR_EMA_ALPHA     = 0.40   # 0..1 (más alto = responde más rápido)
ERR_DEADBAND_PX   = 6      # ignora errores pequeños ±6 px
TANH_SCALE_PX     = 140    # compresión suave en saturaciones (opcional)
USE_TANH_SHAPING  = True   # activa compresión no lineal del error

# Recorte adicional por distancia al centro (todas las bandas)
CENTER_DIST_SLOW_THRESH_PX = 120   # a partir de ~120 px de offset del centro empieza a recortar
CENTER_DIST_CUT_FRAC       = 0.30  # recorta hasta el 30% del margen (DRIVE_MAX_SPEED - MIN_MOVE_SPEED)

# --- disparo del latch sólo si NEAR se perdió en el borde ---
NEAR_EDGE_THRESH_PX      = 100  # “muy lejos del centro” para considerar que se perdió en el borde

# --- cómo buscar cuando estamos en latch ---
SEARCH_TURN_DEG          = 14   # ya lo tienes; giro suave hacia el lado perdido
SEARCH_TURN_SPEED        = 32   # velocidad lenta mientras buscamos
SEARCH_TURN_MAX_TIME_S   = 2.0  # (opcional) por seguridad, máx. tiempo de búsqueda continua


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

        self._pan_angle = float(PAN_CENTER)
        self._pan_active = False  # False = Fija en 90, True = Siguiendo línea
        self._pan_trigger_count = 0

        # Evento para pausar/reanudar el bucle del hilo
        self.__flag = threading.Event()
        self.__flag.clear()

        self._servo_last_angle = STEER_CENTER
        self._servo_last_time  = time.time()

        # -------- Estado para el latch de búsqueda --------
        self.last_near_err  = None       # último err_near válido
        self.last_near_side = None       # 'left' | 'right' | None
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
            RPIservo.move(SERVO_TILT, 50)
        except:
            pass

        try:
            if hasattr(move, "setup"):
                move.setup()
        except Exception as e:
            print("[Functions] Aviso: move.setup() falló:", e)

    # --------------- Helpers de movimiento ---------------

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

    def _steer_command(self, target_deg: int):
        """
        Aplica un limitador de pendiente (slew-rate) al servo para que no gire
        más rápido de STEER_SLEW_DEG_PER_SEC. Devuelve el ángulo realmente enviado.
        """
        now = time.time()
        dt = max(1e-3, now - self._servo_last_time)
        max_step = STEER_SLEW_DEG_PER_SEC * dt

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

    def _update_pan_servo(self, error_px):
        """
        Mueve la cámara basándose en el error visual.
        Actúa como un integrador: Mientras haya error, la cámara sigue girando.
        Si error es 0, la cámara mantiene su ángulo actual (NO vuelve al centro).
        """
        # Constantes locales para ajuste fino
        PAN_GAIN = 0.04      # Velocidad de giro de la cámara
        PAN_MAX_STEP = 3.0   # Máximo grados por frame (suavizado)
        PAN_DEADBAND = 10    # Si el error es pequeño, no mover

        if error_px is None:
            # Si no hay línea, NO HACEMOS NADA. 
            # Nos quedamos mirando donde estábamos (Memoria).
            return (self._pan_angle - PAN_CENTER)

        # 1. Zona muerta (Deadband)
        if abs(float(error_px)) < PAN_DEADBAND:
            return (self._pan_angle - PAN_CENTER)

        # 2. Calcular cuánto queremos girar
        # Error positivo (línea a la izq) -> Sumar ángulo
        delta = float(error_px) * PAN_GAIN

        # 3. Limitar la velocidad (Slew Rate)
        if delta > PAN_MAX_STEP: delta = PAN_MAX_STEP
        elif delta < -PAN_MAX_STEP: delta = -PAN_MAX_STEP

        # 4. Aplicar al ángulo actual (ACUMULATIVO)
        self._pan_angle += delta

        # 5. Límites físicos del servo
        self._pan_angle = max(PAN_MAX_RIGHT, min(PAN_MAX_LEFT, self._pan_angle))

        # 6. Mover servo
        try:
            RPIservo.move(SERVO_PAN, int(self._pan_angle))
        except:
            pass

        # Devolvemos cuánto estamos girados respecto al centro
        return (self._pan_angle - PAN_CENTER)

    def _filter_mix_err(self, err):
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
            a = ERR_EMA_ALPHA
            self._err_ema = (1.0 - a) * self._err_ema + a * float(err)
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

        self._pan_angle = float(PAN_CENTER) # Reseteamos la variable interna
        try:
            RPIservo.move(SERVO_PAN, int(PAN_CENTER)) # Movemos el físico
        except:
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
        Lógica: Cámara Fija Resistente.
        Solo activa seguimiento si la línea está en el borde (>110px) durante 3 frames seguidos.
        """
        # 1) Productor
        try:
            cam = Camera.get_instance()
            cvp = cam.cv_thread
        except Exception:
            time.sleep(0.1)
            return

        # 2) Esperar medición
        st, seq = cvp.get_line_state(wait_new=True, last_seq=self._line_last_seq, timeout=0.25)
        self._line_last_seq = seq

        # 3) Datos
        now = time.time()
        fresh = (now - st.get('timestamp', 0)) < FRESH_TIMEOUT_SEC
        errs, hasl = st.get('errs'), st.get('has_list')
        en, hn = st.get('err_near', st.get('err', None)), st.get('has_near', False)
        any_band = isinstance(hasl, list) and any(hasl)

        # 3.a) Actualizar memoria visual
        if hn and en is not None:
            self.last_near_err = float(en)
            self.last_near_side = 'left' if float(en) > 0 else 'right'

        # ---------------------------------------------------------
        # 3.b) DISPARO DE EMERGENCIA (MANIOBRA REVERSA)
        # ---------------------------------------------------------
        if self.search_latch is None:
            panic_mode = None
            # Límites físicos de giro (35-135)
            if self._pan_angle > 135: 
                panic_mode = 'search_reverse_left'
            elif self._pan_angle < 35:
                panic_mode = 'search_reverse_right'

            if panic_mode is not None:
                print(f"[Functions] PÁNICO FÍSICO ({self._pan_angle:.0f}°). REVERSA.")
                self.search_latch = panic_mode
                self.search_debounce = 0
                self.search_started_t = now
            
            elif not any_band:
                # Si perdemos la línea totalmente, buscamos hacia donde mira la cámara
                side = 'left' if self._pan_angle > PAN_CENTER else 'right'
                self.search_latch = f'search_forward_{side}' 
                self.search_debounce = 0
                self.search_started_t = now

        # ---------------------------------------------------------
        # 3.c) SALIDA DE EMERGENCIA
        # ---------------------------------------------------------
        if self.search_latch is not None:
            # Salida: Cámara segura (80-100) y línea visible
            if (80 < self._pan_angle < 100) and any_band:
                self.search_debounce += 1
                if self.search_debounce >= SEARCH_DEBOUNCE_FRAMES:
                    self.search_latch = None 
                    self._motor_stop()
                    # Resetear estado híbrido al recuperar
                    self._pan_active = False 
                    self._pan_trigger_count = 0
                    self._pan_angle = PAN_CENTER
            else:
                self.search_debounce = 0
                
            if self.search_latch and (now - self.search_started_t) > 5.0:
                 self.search_latch = None; self._motor_stop()

        # ---------------------------------------------------------
        # 4) CÁLCULO DE ÁNGULOS (Lógica Híbrida Estricta)
        # ---------------------------------------------------------
        
        # A) Error Visual
        mix_err = None
        if isinstance(errs, list) and isinstance(hasl, list) and len(errs) == 5 and any_band:
            acc = 0.0; wsum = 0.0
            for i, e in enumerate(errs):
                if e is not None and hasl[i]:
                    acc  += WIN_WEIGHTS[i] * float(e)
                    wsum += WIN_WEIGHTS[i]
            if wsum > 0:
                mix_err = acc / wsum

        # B) GESTIÓN DEL SERVO PAN
        # Umbrales más estrictos para que NO se mueva en curvas suaves
        PAN_TRIGGER_PX = 110  # Solo activa si error > 110 (muy al borde)
        PAN_RESET_PX   = 30   # Desactiva si error < 30 (bien centrado)
        TRIGGER_FRAMES = 3    # Debe mantenerse en el borde 3 frames seguidos
        
        err_val = float(mix_err) if mix_err is not None else 0.0

        if self.search_latch is not None:
            # MODO BÚSQUEDA: Cámara siempre activa
            self._update_pan_servo(mix_err)
            
        else:
            # MODO CONDUCCIÓN
            if not self._pan_active:
                # --- ESTADO FIJO (90°) ---
                
                # Solo activamos si el error es GRANDE y PERSISTENTE
                if abs(err_val) > PAN_TRIGGER_PX:
                    self._pan_trigger_count += 1
                else:
                    self._pan_trigger_count = 0 # Reset si vuelve a zona segura
                
                if self._pan_trigger_count >= TRIGGER_FRAMES:
                    self._pan_active = True
                    # print(">> CÁMARA LIBERADA (Línea perdiéndose)")
                
                # Mantener clavada en 90 mientras no esté activa
                if not self._pan_active:
                    if self._pan_angle != PAN_CENTER:
                        self._pan_angle = PAN_CENTER
                        try: RPIservo.move(SERVO_PAN, int(PAN_CENTER))
                        except: pass
            
            else:
                # --- ESTADO SEGUIMIENTO ---
                # Condición de salida: Línea centrada Y Cámara mirando al frente
                angle_ok = (82 < self._pan_angle < 95)
                line_ok  = (abs(err_val) < PAN_RESET_PX)
                
                if angle_ok and line_ok:
                    self._pan_active = False
                    self._pan_trigger_count = 0
                    self._pan_angle = PAN_CENTER
                    try: RPIservo.move(SERVO_PAN, int(PAN_CENTER))
                    except: pass
                    # print("<< CÁMARA BLOQUEADA (Recuperada)")
                else:
                    # Seguimos rastreando
                    self._update_pan_servo(mix_err)

        # C) MOVER RUEDAS
        servo_cmd = STEER_CENTER 

        if self.search_latch is not None:
            if 'reverse' in self.search_latch:
                if 'left' in self.search_latch: servo_cmd = STEER_RIGHT
                else:                           servo_cmd = STEER_LEFT
            elif 'forward' in self.search_latch:
                if 'left' in self.search_latch: servo_cmd = STEER_LEFT
                else:                           servo_cmd = STEER_RIGHT
        
        else:
            # Acople: Si la cámara está Fija, pan_offset es 0, así que no afecta.
            pan_offset = self._pan_angle - PAN_CENTER
            pan_correction = pan_offset * 0.6
            
            pid_out = 0
            if mix_err is not None:
                dt = max(1e-3, now - self._pid_last_time)
                error = float(mix_err)
                self._pid_integral = max(-100, min(100, self._pid_integral + error * dt))
                derivative = (error - self._pid_last_err) / dt
                self._pid_last_err = error; self._pid_last_time = now
                pid_out = (K_STEER * error) + (KI_STEER * self._pid_integral) + (KD_STEER * derivative)

            servo_cmd = int(STEER_CENTER + pid_out + pan_correction)

        servo_cmd = max(STEER_RIGHT, min(STEER_LEFT, servo_cmd))
        self._steer_command(servo_cmd)
        
        # Debug para verificar estado
        # status_cam = "ACT" if self._pan_active else "FIX"
        # print(f"CAM: {self._pan_angle:.0f}° [{status_cam}] | ERR: {err_val:.0f}")

        # ---------------------------------------------------------
        # 5) VELOCIDAD
        # ---------------------------------------------------------
        if self.search_latch is not None:
            self._no_line_frames = 0
            if 'reverse' in self.search_latch:
                try: move.backward(SEARCH_REVERSE_SPEED); self._current_speed = -SEARCH_REVERSE_SPEED
                except: self._motor_stop()
            elif 'forward' in self.search_latch:
                self._set_target_speed(SEARCH_TURN_SPEED); self._ramp_and_drive()

        elif self.line_follow_active and fresh and any_band:
            ref_err = float(en) if (hn and en) else (float(mix_err) if mix_err else 0.0)
            abs_ref = abs(ref_err)
            if abs_ref >= ERR_STOP_THRESH_PX: base = int(DRIVE_BASE_SPEED * 0.2)
            else:
                k = max(0.0, min(1.0, abs_ref / float(ERR_SLOW_THRESH_PX)))
                base = int(DRIVE_BASE_SPEED + (DRIVE_MAX_SPEED - DRIVE_BASE_SPEED) * (1.0 - k))
            
            desired = max(MIN_MOVE_SPEED, base)
            self._no_line_frames = 0
            self._set_target_speed(int(desired))
            self._ramp_and_drive()

        else:
            self._no_line_frames += 1
            if self._no_line_frames >= NO_LINE_STOP_FRAMES:
                self._set_target_speed(0); self._ramp_and_drive()

        # 6) Actualizar HUD
        try:
            Camera.get_instance().cv_thread.set_vehicle_status(self._current_speed, self._servo_last_angle)
        except: pass

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
