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
SERVO_STEERING = 2

# Dirección (ajusta a tu coche)
STEER_RIGHT        = 60
STEER_CENTER       = 90   # calibra "recto" real
STEER_LEFT         = 120
K_STEER            = 0.10  # deg/px (invierte a -0.08 si gira al revés)
KD_STEER           = 0.04  # Ganancia Derivativa
KI_STEER           = 0.01  # Ganancia Integral

# Velocidades
DRIVE_BASE_SPEED    = 27
DRIVE_MAX_SPEED     = 30
MIN_MOVE_SPEED      = 25    # vencer rozamiento

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
RAMP_STEP           = 7
RAMP_HZ_LIMIT       = 12.0  # Hz máximos de envío de órdenes al motor

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
CUT_FAR_FRAC = 0.15  # máx 15% extra por far

# --- Mínimo seguro si sólo tenemos bandas superiores ---
SAFE_MIN_WHEN_FAR_ONLY = 32

# --- Pre-giro anticipado por bandas ---
FAR_PRESTEER_DEG  = 6   # cuando SOLO far ve curva → gira ±10°
MID_PRESTEER_DEG  = 6   # si mid también la ve → suma ±10°
NEAR_PRESTEER_DEG = 8   # cuando near ya la ve → suma ligera (consolidación)

# Limitador de velocidad de giro del servo (suaviza cambios bruscos)
STEER_SLEW_DEG_PER_SEC = 90  # máx grados/seg que puede cambiar el servo

FRESH_TIMEOUT_SEC = 1.0   # antes 0.5; más permisivo para no perder frames

ANY_BAND_MIN_SPEED = 38

# --- cómo buscar cuando estamos en latch ---
SEARCH_TURN_DEG        = 14   # giro suave hacia el lado perdido
SEARCH_TURN_SPEED      = 32   # velocidad lenta mientras buscamos
#SEARCH_TURN_MAX_TIME_S = 2.0  # seguridad
SERACH_TURN_MAX_TIME_S = 5.0
SEARCH_DEBOUNCE_FRAMES = 3      # nº de frames buenos para soltar latch
SEARCH_HYST_PX         = 1.0    # margen de histéresis para considerar "mejora" de |err|
NEAR_EDGE_THRESH_PX    = 140    # opcional: exigir que se perdió estando "en el borde"

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
NEAR_EDGE_THRESH_PX      = 140  # “muy lejos del centro” para considerar que se perdió en el borde

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

        # rampa hacia el objetivo
        if self._current_speed < self._target_speed:
            self._current_speed = min(self._current_speed + RAMP_STEP, self._target_speed)
        elif self._current_speed > self._target_speed:
            self._current_speed = max(self._current_speed - RAMP_STEP, self._target_speed)

        v = int(self._current_speed)

        if (not self.line_follow_active) or v <= 0:
            self._motor_stop()
            return
        try:
            move.forward(v)  # tu move.py usa forward(speed)
        except Exception as e:
            # Evita spam si el motor no está listo en algún ciclo
            # print("[drive] forward() falló:", e)
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

        any_band = isinstance(hasl, list) and any(hasl)

        # 3.a) Actualizar "último lado" visto por NEAR
        # Recordatorio: err = center_x - cx  → err>0 => línea a la IZQUIERDA (cx a la izquierda)
        if hn and en is not None:
            self.last_near_side = 'left' if float(en) > 0 else 'right'
            self.last_near_err  = float(en)  # asegura que se actualiza

        # 3.b) Lógica de LATCH de búsqueda
        # Activa latch sólo cuando NO hay NEAR y la última vez estaba en el BORDE
        if (self.search_latch is None) and (self.last_near_side in ('left', 'right')):
            cond_perdida = (not hn) or (not any_band)
            if cond_perdida:
                # opcional: exige que la última vez estuviera "al borde"
                if (self.last_near_err is None) or (abs(self.last_near_err) >= NEAR_EDGE_THRESH_PX):
                    self.search_latch = self.last_near_side
                    self.search_debounce = 0
                    self.search_started_t = now  # para límite de tiempo

        # Si hay NEAR y estamos en latch, soltamos sólo cuando |err_near| mejora durante N frames
        if hn and self.search_latch is not None and en is not None and self.last_near_err is not None:
            if abs(float(en)) < abs(float(self.last_near_err)) - SEARCH_HYST_PX:
                self.search_debounce += 1
            else:
                self.search_debounce = 0
            if self.search_debounce >= SEARCH_DEBOUNCE_FRAMES:
                self.search_latch = None
                self.search_debounce = 0

        # 4) Dirección base
        servo_base = None
        mix_err = None

        # 4.a) Si hay latch activo → giro fijo hacia ese lado (skip mezcla y pre-giro)
        if self.search_latch is not None:
            # Signo: steer>0 izquierda, steer<0 derecha
            #steer_bias_deg = (+SEARCH_TURN_DEG) if self.search_latch == 'left' else (-SEARCH_TURN_DEG)
            #servo_cmd = max(STEER_RIGHT, min(STEER_LEFT, STEER_CENTER + int(steer_bias_deg)))
            if self.search_latch == 'left':
                servo_cmd = STEER_LEFT  # Gira a tope izquierda (120)
            else:
                servo_cmd = STEER_RIGHT # Gira a tope derecha (60)

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
                    mix_err = self._filter_mix_err(mix_err)

            if mix_err is None:
                servo_base = STEER_CENTER
            else:
                now = time.time()
                dt = max(1e-3, now - self._pid_last_time)

                # --- LÓGICA PID ---
                error = float(mix_err) # Error P

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
                pid_out = (K_STEER * error) + (KI_STEER * self._pid_integral) + (KD_STEER * derivative)

                # Quita el cálculo antiguo
                # servo_base = int(STEER_CENTER + K_STEER * int(mix_err))
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
            servo_pos = self._steer_command(servo_cmd)

        # 5) Velocidad — AVANZA si hay al menos una banda, aunque near no esté
        if self.line_follow_active and fresh and (any_band or self.search_latch is not None):
            # referencia para modular velocidad: near si existe, si no mix_err, si no 0
            ref_err = None
            if hn and en is not None:
                ref_err = float(en)
            elif mix_err is not None:
                ref_err = float(mix_err)
            else:
                ref_err = 0.0

            abs_ref = abs(ref_err)
            if abs_ref >= ERR_STOP_THRESH_PX:
                base = max(0, int(DRIVE_BASE_SPEED * 0.2))
            else:
                k = max(0.0, min(1.0, abs_ref / float(ERR_SLOW_THRESH_PX)))
                base = int(DRIVE_BASE_SPEED +
                        (DRIVE_MAX_SPEED - DRIVE_BASE_SPEED) * (1.0 - k))
                base = max(base, MIN_MOVE_SPEED)

            # recortes suaves por mid/far (opcionales)
            cut_mid = cut_far = 0
            if hm and em is not None:
                k_mid = max(0.0, min(1.0, abs(float(em)) / float(MID_ERR_SLOW_THRESH)))
                cut_mid = int(CUT_MID_FRAC * (DRIVE_MAX_SPEED - MIN_MOVE_SPEED) * k_mid)
            if hf and ef is not None:
                k_far = max(0.0, min(1.0, abs(float(ef)) / float(FAR_ERR_SLOW_THRESH)))
                cut_far = int(CUT_FAR_FRAC * (DRIVE_MAX_SPEED - MIN_MOVE_SPEED) * k_far)

            # --- recorte por "descentramiento" de cualquiera de las 3 bandas bajas (2,3,4) ---
            errs_abs = []
            if isinstance(errs, list) and isinstance(hasl, list) and len(errs) >= 5 and len(hasl) >= 5:
                for i in (2, 3, 4):  # far-medio-cerca inferiores
                    if hasl[i] and errs[i] is not None:
                        off = abs(float(errs[i]))
                        # deadband para no castigar micro-desalineaciones
                        off = max(0.0, off - float(BOTTOM_CENTER_DEADBAND_PX))
                        errs_abs.append(off)

            extra_cut = 0
            if errs_abs:
                # si cualquiera se aleja, recortamos (uso el máximo para ser conservador)
                max_off  = max(errs_abs)
                k_center = max(0.0, min(1.0, max_off / float(BOTTOM_CENTER_SLOW_THRESH_PX)))
                extra_cut = int(BOTTOM_CENTER_CUT_FRAC * (DRIVE_MAX_SPEED - MIN_MOVE_SPEED) * k_center)

            desired = base - (cut_mid + cut_far + extra_cut)

            if self.search_latch is not None:
                # (opcional) corta la búsqueda si excede el tiempo máx.
                if hasattr(self, 'search_started_t') and (now - self.search_started_t) > SEARCH_TURN_MAX_TIME_S:
                     self.search_latch = None  # suelta por timeout de seguridad
                desired = min(desired, SEARCH_TURN_SPEED)


            # Si NO hay NEAR pero sí otras → aseguremos avance mínimo
            if not hn:
                desired = max(desired, ANY_BAND_MIN_SPEED)

            self._no_line_frames = 0
            self._set_target_speed(int(max(MIN_MOVE_SPEED, desired)))
            self._ramp_and_drive()
        else:
            # Sin línea visible:
            if self.search_latch is not None:
                # Seguimos avanzando lento mientras buscamos
                self._no_line_frames = 0
                self._set_target_speed(int(max(MIN_MOVE_SPEED, SEARCH_TURN_SPEED)))
                self._ramp_and_drive()
            else:
                # Parada segura tras N frames sin nada que ver
                self._no_line_frames += 1
                if self._no_line_frames >= NO_LINE_STOP_FRAMES:
                    self._set_target_speed(0)
                    self._ramp_and_drive()


        # 6) Memorias para próxima iteración
        if hn and en is not None:
             self.last_near_err = float(en)

        # ... justo después de self._ramp_and_drive() (o al final del bloque de velocidad)
        try:
            cam = Camera.get_instance()
            cvp = cam.cv_thread
            # usa la velocidad actual y el último ángulo enviado (ya guardas self._servo_last_angle)
            cvp.set_vehicle_status(self._current_speed, self._servo_last_angle)
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
