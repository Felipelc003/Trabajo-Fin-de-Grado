#!/usr/bin/python3
#
# This file is part of  Diseño Modular e Incremental para la 
# Navegación Autónoma Contextual en Sistemas Embebidos de Bajo Coste
#
# Copyright 2026 Felipe López Castro <i12locaf@uco.es>
#
# Diseño Modular e Incremental para la Navegación Autónoma Contextual 
# en Sistemas Embebidos de Bajo Coste is free software: you can redistribute it 
# and/or modify it under the terms of the GNU General Public License 
# as published by the Free Software Foundation, either version 3 of the License, 
# or  (at your option) any later version.
# 
# Diseño Modular e Incremental para la Navegación Autónoma Contextual 
# en Sistemas Embebidos de Bajo Coste is distributed in the hope that it will be useful, 
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Diseño Modular e Incremental para la Navegación Autónoma Contextual 
# en Sistemas Embebidos de Bajo Coste.
# If not, see <https://www.gnu.org/licenses/>.
#

# Nombre del archivo: functions.py
# Descripción: Bucle de control principal (dirección + tracción) basado en visión artificial.
#               Implementa lógica PID, máquinas de estado para manejo de QR,
#               control de velocidad adaptativo y maniobras de recuperación.

import time
import threading

import move
import RPIservo
from camera_opencv import Camera
import numpy as np  

# -----------------------------
# Constantes de Actuadores y Calibración
# -----------------------------
SERVO_TILT = 0     # Canal del servo de inclinación (Tilt)
SERVO_PAN  = 1     # Canal del servo de paneo (Pan)
SERVO_STEERING = 2 # Canal del servo de dirección

# --- Calibración de Dirección ---
# Valores PWM mapeados a ángulos físicos del servo de dirección.
STEER_RIGHT        = 50
STEER_CENTER       = 88.5   # Centro óptico/mecánico calibrado (ajustar si el coche deriva)
STEER_LEFT         = 120
# Ganancias del Controlador PID de Dirección
K_STEER            = 0.065  # Proporcional: Reacción directa al error (deg/px)
KD_STEER           = 0.15   # Derivativo: Amortiguación de oscilaciones
KI_STEER           = 0.01   # Integral: Corrección de error estacionario

# --- Configuración Servo Pan (Cámara) ---
# Seguimiento visual activo: la cámara gira en algunos casos para mantener la línea en el centro.
PAN_CENTER         = 85     # Posición central mirando al frente
PAN_MAX_LEFT       = 130    # Límite físico izquierdo
PAN_MAX_RIGHT      = 40     # Límite físico derecho

# Dinámica del seguimiento de cámara
PAN_GAIN           = 0.05   # Sensibilidad del movimiento de cámara al error
PAN_CENTER_FORCE   = 0.01   # "Resorte" virtual que devuelve la cámara al centro
PAN_MAX_STEP       = 6.0    # Máxima velocidad de giro por ciclo (grados)
PAN_DEADBAND       = 8      # Zona muerta en píxeles para evitar 'jitter' (vibración)

# -----------------------------
# Constantes de Velocidad y Tracción
# -----------------------------
DRIVE_BASE_SPEED    = 60    # Velocidad crucero base (0-100)
DRIVE_MAX_SPEED     = 65    # Límite absoluto de velocidad automática
MIN_MOVE_SPEED      = 30    # PWM mínimo necesario para vencer la fricción estática

# --- Perfiles de Velocidad según Tipo de Línea ---
SPEED_BLACK_BOOST   = 80    # Velocidad alta para rectas negras (confianza alta)
SPEED_WHITE_NORMAL  = 55    # Velocidad moderada para terreno blanco
SPEED_YELLOW_MAX    = 45    # Velocidad reducida para zonas de precaución (amarillo)

# --- Parámetros de Búsqueda y Recuperación ---
COLOR_SEARCH_SPEED  = 55    # Velocidad durante maniobras de búsqueda de línea
YELLOW_CENTER_BIAS  = -5    # Desplazamiento intencional para seguir el carril amarillo
RED_STOP_TIME       = 2.0   # Tiempo de espera (s) al detectar señal de STOP (rojo)
RED_IGNORE_TIME     = 3.0   # Tiempo de gracia (s) después de un STOP para evitar falsos positivos

# --- Secuencias de Navegación (Misiones) ---
# Definen el orden esperado de colores en el circuito para navegación lógica.
WHITE_SEQUENCE = ["white", "black"]
YELLOW_SEQUENCE = ["yellow", "black", "yellow", "black"]
USE_YELLOW_SEQUENCE = True
USE_WHITE_SEQUENCE = True

# Maniobras de emergencia
SEARCH_TURN_SPEED      = 40   # Velocidad de giro
SEARCH_REVERSE_SPEED   = 50   # Velocidad de retroceso

# Ajustes específicos de dirección por color
STEER_GAIN_BLACK       = 0.15 
BLACK_DEADBAND_PX      = 40   # Mayor tolerancia en línea negra para evitar correcciones nerviosas

# --- Reducción de Velocidad en Curvas ---
# El coche frena automáticamente si detecta que la línea se aleja del centro (curva cerrada)
BOTTOM_CENTER_SLOW_THRESH_PX = 110   # Umbral de error lateral para iniciar frenado
BOTTOM_CENTER_CUT_FRAC       = 0.10  # Porcentaje máximo de reducción de velocidad (10%)
BOTTOM_CENTER_DEADBAND_PX    = 8     

# Umbrales críticos
ERR_SLOW_THRESH_PX  = 100   # Error para comenzar a reducir velocidad (general)
BLACK_SLOW_THRESH_PX = 180  # Error tolerado en rectas
ERR_STOP_THRESH_PX  = 180   # Error crítico: Posible salida de pista

# Seguridad
NO_LINE_STOP_FRAMES = 5     # Número de frames sin línea antes de detenerse/buscar

# --- Rampa de Aceleración (Soft Start/Stop) ---
RAMP_STEP_UP          = 10   # Incremento de velocidad por ciclo
RAMP_STEP_DOWN        = 10   # Decremento de velocidad por ciclo
RAMP_HZ_LIMIT         = 30.0 # Tasa máxima de actualización de motores (Hz)

# --- Ponderación de Ventanas de Visión ---
# La imagen se divide en 5 bandas. Los pesos determinan cuánto influye cada banda en la dirección.
# Pesos bajos arriba (lejos) y altos abajo (cerca).
WIN_WEIGHTS = [0.02, 0.03, 0.05, 0.20, 0.70]  
PRED_GAIN   = 0.15   # Ganancia predictiva: Usa la pendiente de la línea para anticipar curvas

# Reducción anticipada de velocidad (Look-ahead)
MID_ERR_SLOW_THRESH = 150
FAR_ERR_SLOW_THRESH = 200
CUT_MID_FRAC = 0.05  # Reducción leve si la curva empieza a media distancia
CUT_FAR_FRAC = 0.05  # Reducción leve si la curva se ve a lo lejos

# --- Pre-giro (Feed-forward) ---
# Aplica un giro extra basado en la curvatura futura de la línea
FAR_PRESTEER_DEG  = 0    
MID_PRESTEER_DEG  = 3    
NEAR_PRESTEER_DEG = 10   

# Búsqueda Activa (Active Search)
COLOR_SEARCH_SPEED_ACTIVE = 40        
COLOR_SEARCH_STEER = 60        

# Slew Rate: Limitador de velocidad del servo para movimientos más naturales
STEER_SLEW_DEG_PER_SEC = 60   
STEER_SLEW_RATE_BLACK  = 100  # Respuesta más rápida en carril rápido

FRESH_TIMEOUT_SEC = 1.0   # Tiempo máximo para considerar un dato de visión como válido

ANY_BAND_MIN_SPEED = 38

SEARCH_DEBOUNCE_FRAMES = 3      # Frames consecutivos necesarios para confirmar recuperación de línea
NEAR_EDGE_THRESH_PX    = 100    # Umbral de borde para activar latch de búsqueda
SEARCH_FORWARD_TIMEOUT_S   = 5.0 # Tiempo máximo intentando buscar hacia adelante antes de retroceder

# Constantes para maniobra de reversa
SEARCH_REACQUIRE_CENTER_PX = 40   

# Pesos de respaldo (si falla la mezcla de ponderada)
NEAR_W, MID_W, FAR_W     = 0.70, 0.20, 0.10

# --- Filtrado de Señal de Error ---
ERR_EMA_ALPHA     = 0.50   # Factor de suavizado (Exponential Moving Average)
ERR_EMA_ALPHA_BLACK = 0.15 # Suavizado más agresivo en rectas
ERR_DEADBAND_PX   = 6      # Ruido a ignorar
TANH_SCALE_PX     = 140    # Escala para la función de modelado no lineal (tanh)
USE_TANH_SHAPING  = True   # Activa respuesta no lineal (más suave al centro, saturada en extremos)


class Functions(threading.Thread):
    """
    Hilo de CONTROL PRINCIPAL (Control Loop).
    
    Orquesta la percepción y la acción:
    1. Recibe el estado del entorno procesado por `cv_processor`.
    2. Ejecuta lógica de alto nivel (Misiones, QR, Recuperación).
    3. Calcula comandos de bajo nivel (PID Dirección, Perfiles Velocidad).
    4. Envía órdenes a los actuadores (Servos y Motores DC).
    """

    def __init__(self, *args, **kwargs):
        """Inicialización de variables de estado y controladores."""
        super(Functions, self).__init__(*args, **kwargs)

        # Estado de modo de operación
        self.functionMode = 'none'

        # Variables de control de tracción
        self.line_follow_active = False
        self._target_speed = 0        # Velocidad deseada
        self._current_speed = 0       # Velocidad actual (para rampa)
        self._no_line_frames = 0      # Contador de pérdida de señal visual
        self._last_drive_time = 0.0
        self._line_last_seq = None    # Sincronización con thread de visión
        self._last_debug_log = 0.0
        self._ignore_red_until = 0.0  # Temporizador inercial tras STOP

        # Estado del servo de cámara (Pan)
        self._pan_angle = float(PAN_CENTER)
        self._pan_active = False

        # Sincronización de hilos (Pause/Resume)
        self.__flag = threading.Event()
        self.__flag.clear()

        # Estado del servo de dirección
        self._servo_last_angle = STEER_CENTER
        self._servo_last_time  = time.time()

        # --- Variables Latch para Recuperación ---
        # "Latch" memoriza el último estado válido conocido para saber hacia dónde girar
        # si se pierde la línea.
        self.last_near_err  = None       
        self.last_near_side = None       
        self.last_near_color = None
        self.search_latch   = None       # Estado actual de la maniobra de búsqueda
        self.search_debounce = 0         

        self._err_ema = None   # Estado del filtro EMA

        # Estado del controlador PID
        self._pid_last_err = 0.0
        self._pid_integral = 0.0
        self._pid_last_time = time.time()

        # Estado de secuencia de colores (Lógica de Misión)
        self.white_sequence_index = 0  
        self.yellow_sequence_index = 0  
        self.target_color = None  
        self.active_sequence = None  
        
        # Sistema de lectura de códigos QR
        self.qr_initial_color = None    
        self.qr_mode = None              
        self.qr_cycles_total = 0         
        self.qr_cycles_done = 0          
        self.qr_current_color = None     
        self.qr_needs_read = True        
        
        # Estado de búsqueda activa de color (Post-QR)
        self.color_search_mode = False   
        self.color_search_target = None  
        self.color_search_direction = None  
        self.color_search_forced = False  
        self.color_search_frames = 0      

        # Variables para BARRIDO DE CÁMARA (Visual Sweep)
        self.sweep_start_time = 0.0
        self.sweep_direction = 1 # 1=Right, -1=Left 

        # Inicialización de hardware (Posición segura)
        try:
            RPIservo.move(SERVO_PAN, int(self._pan_angle))
            RPIservo.move(SERVO_TILT, 40)
        except:
            pass
        
        try:
            if hasattr(move, "setup"):
                move.setup()
        except Exception as e:
            print("[Functions] Error en inicialización de motores:", e)

    # ---------------------------------------------------------
    # Métodos Auxiliares de Movimiento y Control
    # ---------------------------------------------------------

    def _update_telemetry(self, speed=None, steer=None):
        """
        Envía la telemetría actual (velocidad y dirección) al sistema de cámara
        para que se muestre en el HUD.
        """
        # Usar valores cacheados si no se especifican
        s = int(self._current_speed) if speed is None else int(speed)
        t = int(self._servo_last_angle) if steer is None else int(steer)
        
        try:
            # Obtener instancia de cámara (Singleton) y actualizar procesador
            cam = Camera.get_instance()
            if cam and cam.cv_thread:
                cam.cv_thread.set_vehicle_status(s, t)
        except Exception:
            pass # Evitar bloquear el hilo de control si la cámara falla


    def _update_pan_servo(self, error_px):
        """
        Sistema de "Mirada Activa" (Active Gaze).
        
        Controla el servo Pan para mantener la línea centrada en la imagen, independiente
        del chasis del robot. Ayuda a no perder la línea en curvas cerradas.
        
        Retorna:
            El offset angular aplicado (útil para sumar a la dirección).
        """
        if error_px is None:
            # Retorno elástico al centro si no hay referencia visual
            center_delta = (PAN_CENTER - self._pan_angle) * 0.1
            self._pan_angle += center_delta
        else:
            # Corrección Proporcional basada en el error visual
            vision_delta = 0
            if abs(float(error_px)) > PAN_DEADBAND:
                vision_delta = float(error_px) * PAN_GAIN 

            # Fuerza de centrado (para evitar que la cámara se quede girada permanentemente)
            center_delta = (PAN_CENTER - self._pan_angle) * PAN_CENTER_FORCE
            
            # Limitador de velocidad (Slew Rate Limiter) para movimiento suave
            total_delta = vision_delta + center_delta
            if total_delta > PAN_MAX_STEP: total_delta = PAN_MAX_STEP
            elif total_delta < -PAN_MAX_STEP: total_delta = -PAN_MAX_STEP

            self._pan_angle += total_delta

        # Saturación a límites mecánicos
        self._pan_angle = max(PAN_MAX_RIGHT, min(PAN_MAX_LEFT, self._pan_angle))
        
        try:
            RPIservo.move(SERVO_PAN, int(self._pan_angle))
        except:
            pass

        return (self._pan_angle - PAN_CENTER)

    def _motor_stop(self):
        """Detiene los motores DC inmediatamente."""
        try:
            if hasattr(move, "motorStop"):
                move.motorStop()
            else:
                move.stop()
        except Exception:
            pass
        self._current_speed = 0

    def _set_target_speed(self, spd: int):
        """Establece la consigna de velocidad objetivo."""
        spd = int(max(0, min(100, spd)))
        self._target_speed = spd

    def _ramp_and_drive(self):
        """
        Controlador de Potencia de Motores con Rampa de Aceleración.
        
        Suaviza la aceleración y frenado para:
        1. Evitar picos de corriente que reinicien la Raspberry Pi.
        2. Reducir el deslizamiento de las ruedas (tracción mecánica).
        3. Proporcionar un movimiento más suave.
        """
        now = time.time()
        # Limitador de frecuencia de actualización
        if now - self._last_drive_time < 1.0 / RAMP_HZ_LIMIT:
            return
        self._last_drive_time = now

        # Lógica de Rampa
        if self._current_speed < self._target_speed:
            # Aceleración
            if self._current_speed < MIN_MOVE_SPEED:
                self._current_speed = MIN_MOVE_SPEED # Kick-start
            else:
                self._current_speed = min(self._current_speed + RAMP_STEP_UP, self._target_speed)
        
        elif self._current_speed > self._target_speed:
            # Frenado
            self._current_speed = max(self._current_speed - RAMP_STEP_DOWN, self._target_speed)

        # Aplicación al hardware
        v = int(self._current_speed)
        print(f"[Functions] Vel: {v:3d} | Objetivo: {self._target_speed:3d}", end='\r', flush=True)

        if (not self.line_follow_active) or (v < MIN_MOVE_SPEED and self._target_speed == 0):
            self._motor_stop()
            self._update_telemetry(speed=0) # Actualizar telemetría a 0
            return

        try:
            move.forward(v)
            self._update_telemetry(speed=v) # Actualizar telemetría de velocidad
        except Exception:
            self._motor_stop()

    def _steer_from_err(self, err_px: int):
        """Convierte error visual (px) directamente a ángulo de servo (deg) [Modo Simple]."""
        target = int(STEER_CENTER + K_STEER * int(err_px))
        target = max(STEER_RIGHT, min(STEER_LEFT, target))
        try:
            RPIservo.move(SERVO_STEERING, target)
        except Exception:
            pass
        return target

    def _steer_command(self, target_deg: int, slew_rate=STEER_SLEW_DEG_PER_SEC):
        """
        Envía comando al servo de dirección con limitación de velocidad (Slew Rate).
        Evita movimientos bruscos que puedan desestabilizar el vehículo.
        """
        now = time.time()
        dt = max(1e-3, now - self._servo_last_time)
        max_step = slew_rate * dt

        # Saturación física
        target_deg = max(STEER_RIGHT, min(STEER_LEFT, int(target_deg)))

        # Limitación de paso
        delta = target_deg - self._servo_last_angle
        if abs(delta) > max_step:
            target_deg = int(self._servo_last_angle + max_step * (1 if delta > 0 else -1))

        try:
            RPIservo.move(SERVO_STEERING, target_deg)
        except Exception:
            pass

        self._servo_last_angle = target_deg
        self._servo_last_time  = now
        
        self._update_telemetry(steer=target_deg) # Actualizar telemetría de dirección
        return target_deg

    def _filter_mix_err(self, err, alpha=ERR_EMA_ALPHA):
        """
        Preprocesamiento avanzado de la señal de error para el PID.
        Pipeline: Deadband -> Filtro Paso Bajo (EMA) -> Conformado No Lineal (Tanh).
        """
        if err is None:
            return None

        # 1. Zona muerta: Ignorar pequeños errores
        if abs(err) < ERR_DEADBAND_PX:
            err = 0.0

        # 2. Filtro EMA (Media Móvil Exponencial) para reducir ruido
        if self._err_ema is None:
            self._err_ema = float(err)
        else:
            self._err_ema = (1.0 - alpha) * self._err_ema + alpha * float(err)
        e = self._err_ema

        # 3. Función de transferencia no lineal (Tanh)
        # Permite alta sensibilidad en el centro y satura suavemente en los extremos
        if USE_TANH_SHAPING and TANH_SCALE_PX > 1:
            e = TANH_SCALE_PX * np.tanh(e / float(TANH_SCALE_PX))

        return e

    # ---------------------------------------------------------
    # Gestión de Modos y Estados
    # ---------------------------------------------------------

    def pause(self):
        """Transición a estado seguro: Motores OFF, Servos centrados."""
        self.functionMode = 'none'
        self.line_follow_active = False

        try:
            self._set_target_speed(0)
            self._ramp_and_drive()
        except Exception:
            try:
                if hasattr(move, "motorStop"): move.motorStop()
                else: move.stop()
            except Exception: pass

        self._target_speed = 0
        self._current_speed = 0
        self._no_line_frames = 0
        self._line_last_seq = None

        try: RPIservo.move(SERVO_STEERING, STEER_CENTER)
        except Exception: pass

        try:
            # Desactiva procesamiento de visión para ahorrar CPU
            cam = Camera.get_instance()
            cam.modeselect('none')
        except Exception: pass

        try: self.__flag.clear()
        except Exception: pass

        print("[Functions] Sistema pausado.")

    def modeSet(self, mode: str):
        """Selector principal de modo de operación."""
        if mode == 'trackLine':
            self.trackLine()
        elif mode == 'none':
            self.pause()
        else:
            print(f"[Functions] Modo desconocido: {mode}")

    def resume(self):
        """Libera el bloqueo del thread principal."""
        self.__flag.set()

    def trackLine(self):
        """
        Inicializa y arranca el modo de Seguidor de Línea.
        Resetea todos los controladores y estados internos para un arranque limpio.
        """
        try:
            if hasattr(move, "motorStop"): move.motorStop()
            else: move.stop()
        except Exception: pass
        
        try: RPIservo.move(SERVO_STEERING, STEER_CENTER)
        except Exception: pass

        try:
            cam = Camera.get_instance()
            cam.modeselect('trackLine')
            print("[trackLine] Visión activada.")
        except Exception as e:
            print(f"[trackLine] Error al activar visión: {e}")

        # Reinicio de variables de control
        self._target_speed = 0
        self._current_speed = 0
        self._no_line_frames = 0
        self._line_last_seq = None
        self._last_debug_log = 0.0
        self.line_follow_active = True
        self._ignore_red_until = 0.0

        # Reinicio de variables de búsqueda (Latch)
        self.last_near_err = None
        self.last_near_side = None
        self.last_near_color = None
        self.search_latch = None
        self.search_debounce = 0
        self._err_ema = None
        
        # Reinicio estado QR
        self.qr_initial_color = None
        self.qr_mode = None
        self.qr_cycles_total = 0
        self.qr_cycles_done = 0
        self.qr_current_color = None
        self.qr_needs_read = True
        
        # Reinicio búsqueda activa
        self.color_search_mode = False
        self.color_search_target = None
        self.color_search_direction = None
        self.color_search_forced = False
        self.color_search_frames = 0
        
        # Reinicio Secuencias de color
        self.white_sequence_index = 0
        self.yellow_sequence_index = 0
        self.target_color = None
        self.active_sequence = None
        
        # Reinicio PID
        self._pid_last_err = 0.0
        self._pid_integral = 0.0
        self._pid_last_time = time.time()
        
        self.functionMode = 'trackLine'
        self.resume()

        # Espera activa para estabilización de la cámara y primeros frames válidos
        try:
            cvp = Camera.get_instance().cv_thread
            t0 = time.time()
            while time.time() - t0 < 0.5:
                # Polling rápido para vaciar buffers viejos
                st, seq = cvp.get_line_state(wait_new=True, last_seq=None, timeout=0.2)
                if (time.time() - st.get('timestamp', 0)) < FRESH_TIMEOUT_SEC:
                    break
        except Exception:
            pass

    # ---------------------------------------------------------
    # Bucle Principal de Control (Llamado desde run)
    # ---------------------------------------------------------

    def trackLineProcessing(self):
        """
        CICLO DE CONTROL CRÍTICO:
        Ejecuta la lógica de conducción autónoma en tiempo real.
        
        Etapas:
        1. Adquisición de Datos: Obtiene geometría de líneas y colores desde `cv_processor`.
        2. Gestión de Misión (QR): Decide qué color seguir y cuándo detenerse.
        3. Recuperación: Detecta si se perdió la línea e inicia maniobras de búsqueda.
        4. Navegación (Steering): Calcula el ángulo de giro usando PID ponderado y Feed-forward.
        5. Tracción (Throttle): Ajusta la velocidad según la curvatura y condiciones de seguridad.
        """
        # 1. Adquisición de instancia de cámara
        try:
            cam = Camera.get_instance()
            cvp = cam.cv_thread
        except Exception:
            time.sleep(0.1)
            return

        # 2. Sincronización bloqueante con el siguiente frame de visión
        # Evita ejecutar el lazo PID más rápido que la cámara (ahorro de CPU)
        st, seq = cvp.get_line_state(wait_new=True, last_seq=self._line_last_seq, timeout=0.25)
        self._line_last_seq = seq

        # 3. Desempaquetado de datos del entorno
        now = time.time()
        fresh = (now - st.get('timestamp', 0)) < FRESH_TIMEOUT_SEC

        errs = st.get('errs')        # Lista de errores desde arriba (0) hacia abajo (4)
        hasl = st.get('has_list')    # Banderas booleanas si se detectó línea en esa banda
        en   = st.get('err_near', st.get('err', None)) # Error cercano (fundamental para control)
        em   = st.get('err_mid',  None) # Error medio (anticipación)
        ef   = st.get('err_far',  None) # Error lejano (anticipación)
        hn   = st.get('has_near', en is not None)
        hm   = st.get('has_mid',  em is not None)
        hf   = st.get('has_far',  ef is not None)

        color_list = st.get('color_list', [None]*5)
        raw_band_color = color_list[4] if (hasl and hasl[4]) else None
        
        # -------------------------------------------------------
        # Filtrado Lógico de Colores (QR / Misiones)
        # -------------------------------------------------------
        if self.qr_current_color is not None and not self.qr_needs_read:
            # Si tenemos una misión activa (ej. QR ordena seguir AMARILLO):
            # Filtramos todos los detecciones que NO sean de ese color.
            
            active_before = [(i, color_list[i]) for i in range(len(hasl)) if hasl[i]]
            
            filtered_hasl = list(hasl) if hasl else [False]*5
            filtered_errs = list(errs) if errs else [None]*5
            filtered_count = 0
            
            for i in range(len(filtered_hasl)):
                if filtered_hasl[i]:
                    band_color = color_list[i]
                    
                    # Máscara de exclusión lógica
                    if self.qr_current_color == 'yellow':
                        if band_color == 'white':
                            filtered_hasl[i] = False
                            filtered_errs[i] = None
                            filtered_count += 1
                    elif self.qr_current_color == 'white':
                        if band_color == 'yellow':
                            filtered_hasl[i] = False
                            filtered_errs[i] = None
                            filtered_count += 1
            
            hasl = filtered_hasl
            errs = filtered_errs
            
            # --- Detección de pérdida de camino correcto ---
            any_correct_color = any(hasl)
            
            if not any_correct_color and self.qr_current_color is not None:
                # No vemos el color correcto, ¿Vemos el color incorrecto? (ej. cruce o bifurcación)
                wrong_color = 'white' if self.qr_current_color == 'yellow' else 'yellow'
                
                sees_wrong_color = any(color_list[i] == wrong_color for i in range(len(active_before)) 
                                      if i < len(filtered_hasl))
                
                if sees_wrong_color:
                    # Si vemos el color incorrecto, activamos el modo de BÚSQUEDA DEL COLOR CORRECTO
                    if not self.color_search_mode:
                        print(f"[Color Search] Detectado {wrong_color.upper()}, buscando {self.qr_current_color.upper()}")
                        self.color_search_mode = True
                        self.color_search_target = self.qr_current_color
            else:
                # Recuperación: Si volvemos a ver el color correcto, desactivamos búsqueda
                if self.color_search_mode:
                    if self.color_search_forced:
                        self.color_search_frames += 1
                        if raw_band_color == self.color_search_target:
                            self.color_search_mode = False
                            self.color_search_target = None
                            self.color_search_direction = None
                            self.color_search_forced = False
                            self.color_search_frames = 0
                        elif self.color_search_frames > 50: # Timeout de búsqueda
                            self.color_search_mode = False
                            self.color_search_frames = 0
                    else:
                        self.color_search_mode = False
                        self.color_search_target = None
                        self.color_search_direction = None
                        self.color_search_forced = False
                        self.color_search_frames = 0
        
        # -------------------------------------------------------
        # Lógica de Secuencias Predefinidas (Fallback sin QR)
        # -------------------------------------------------------
        elif self.target_color is not None and (USE_WHITE_SEQUENCE or USE_YELLOW_SEQUENCE):
            if self.active_sequence is None and raw_band_color in ['white', 'yellow']:
                if raw_band_color == 'white' and USE_WHITE_SEQUENCE:
                    self.active_sequence = 'white'
                    self.target_color = WHITE_SEQUENCE[0]
                elif raw_band_color == 'yellow' and USE_YELLOW_SEQUENCE:
                    self.active_sequence = 'yellow'
                    self.target_color = YELLOW_SEQUENCE[0]
            
            # Filtrado similar al modo QR
            filtered_hasl = list(hasl) if hasl else [False]*5
            for i in range(len(filtered_hasl)):
                if filtered_hasl[i]:
                    band_color = color_list[i]
                    if self.active_sequence == 'white' and self.target_color == 'white':
                        if band_color == 'yellow':
                            filtered_hasl[i] = False
                    elif self.active_sequence == 'yellow' and self.target_color == 'yellow':
                        if band_color == 'white':
                            RPIservo.move(SERVO_PAN, 70) # Ayuda visual
                            filtered_hasl[i] = False
            hasl = filtered_hasl
        
        current_band_color = color_list[4] if (hasl and hasl[4]) else None
        any_band = isinstance(hasl, list) and any(hasl)

        # Variables de banda auxiliar (índice 3) para robustez
        en3 = errs[3] if (isinstance(errs, list) and len(errs) > 3) else None
        hn3 = hasl[3] if (isinstance(hasl, list) and len(hasl) > 3) else False
        color3 = color_list[3] if (isinstance(color_list, list) and len(color_list) > 3) else None

        # Actualización de memoria de última posición conocida (Latch)
        if hn3 and en3 is not None:
            self.last_near_side = 'left' if float(en3) > 0 else 'right'
            self.last_near_err  = float(en3) 
            self.last_near_color = color3

        # -------------------------------------------------------
        # Lógica de Recuperación (State Machine)
        # -------------------------------------------------------
        if (self.search_latch is None) and (self.last_near_side in ('left', 'right')):
            cond_perdida = (not hn3) or (not any_band)
            if cond_perdida:
                # Si se pierde la línea, decidimos la maniobra basada en el último dato
                if (self.last_near_err is None) or (abs(self.last_near_err) >= NEAR_EDGE_THRESH_PX):
                    if self.last_near_color == 'yellow':
                        self.search_latch = 'search_forward_right'
                    elif self.last_near_side == 'left':
                         # Si la línea se fue por la izquierda, giramos izquierda buscándola
                        self.search_latch = 'search_forward_left'
                    else:
                        self.search_latch = 'search_forward_right'

                    self.search_debounce = 0
                    self.search_started_t = now

        # Ejecución de la máquina de estados de recuperación
        if self.search_latch is not None:
            
            # Condición de ÉXITO: recuperamos la línea centrada
            if hn3 and en3 is not None and (abs(float(en3)) < SEARCH_REACQUIRE_CENTER_PX):
                self.search_debounce += 1
                if self.search_debounce >= SEARCH_DEBOUNCE_FRAMES:
                    self.search_latch = None 
                    self.search_debounce = 0
                    self._motor_stop() # Breve parada para estabilizar
                    
            # Timeout de búsqueda frontal -> Pasar a búsqueda en REVERSA
            elif self.search_latch in ('search_forward_left', 'search_forward_right'):
                timeout_limit = SEARCH_FORWARD_TIMEOUT_S
                if (now - self.search_started_t) > timeout_limit:
                    if self.search_latch == 'search_forward_left':
                        self.search_latch = 'search_reverse_left'
                    else:
                        self.search_latch = 'search_reverse_right'
                    
                    # Avance forzado de secuencia si aplica
                    if self.active_sequence == 'white' and USE_WHITE_SEQUENCE:
                        if self.white_sequence_index < len(WHITE_SEQUENCE) - 1:
                            self.white_sequence_index += 1
                            self.target_color = WHITE_SEQUENCE[self.white_sequence_index]
                            self.search_latch = None
                    elif self.active_sequence == 'yellow' and USE_YELLOW_SEQUENCE:
                        if self.yellow_sequence_index < len(YELLOW_SEQUENCE) - 1:
                            self.yellow_sequence_index += 1
                            self.target_color = YELLOW_SEQUENCE[self.yellow_sequence_index]
                            self.search_latch = None
                                
            elif not hn3:
                self.search_debounce = 0

            # Lógica Específica de BARRIDO DE CÁMARA
            if self.search_latch == 'search_sweep':
                # Si hemos encontrado cualquier indicio de línea (any_band), intentamos centrar y salir
                if any_band:
                   # Dejar que el PID y Active Gaze hagan su trabajo
                   self.search_latch = None
                   self._motor_stop()
                   return

                # Ejecución del movimiento de barrido (Ping-Pong)
                # Oscila entre PAN_MAX_LEFT (130) y PAN_MAX_RIGHT (40)
                sweep_speed = 3.0 # Rad/s
                dt_sweep = now - self.sweep_start_time
                
                # Timeout del barrido: Si tarda mucho, pasamos a MARCHA ATRÁS
                if dt_sweep > 4.0: # 4 segundos de búsqueda
                    print("[Sweep] Barrido fallido. Iniciando retroceso.")
                    self.search_latch = 'search_reverse_smart'
                    # Centrar cámara para retroceder
                    RPIservo.move(SERVO_PAN, PAN_CENTER)
                    self.sweep_start_time = now # Reusamos timer para el reverso
                else:
                    # Movimiento Sinusoidal para suavidad
                    # Centro 85, amplitud 45 -> [40, 130]
                    amplitude = 45
                    angle = PAN_CENTER + amplitude * np.sin(sweep_speed * dt_sweep)
                    try:
                        RPIservo.move(SERVO_PAN, int(angle))
                    except: pass

            # Timeout para Reverse Smart
            elif self.search_latch == 'search_reverse_smart':
                if (now - self.sweep_start_time) > 2.0: # 2 segundos de marcha atrás
                     # Si falla, volvemos a intentar barrido o paramos?
                     # Paramos para no irnos al infinito
                     print("[Recovery] Recuperación fallida total.")
                     self.search_latch = None
                     self._motor_stop()

        # -------------------------------------------------------
        # 4. Cálculo de Dirección (Steering)
        # -------------------------------------------------------
        servo_base = None
        mix_err = None

        if self.color_search_mode:
            # Modo búsqueda de color activa: forzar giro
            if self.color_search_direction == 'right':
                servo_cmd = STEER_RIGHT
            elif self.color_search_direction == 'left':
                servo_cmd = STEER_LEFT
            else:
                servo_cmd = STEER_RIGHT
            
            self._set_target_speed(COLOR_SEARCH_SPEED)
            servo_pos = self._steer_command(servo_cmd)

        elif self.search_latch is not None:
           # Modo recuperación: giro predefinido según estado
            if self.search_latch == 'search_forward_right':
                servo_cmd = STEER_RIGHT
            elif self.search_latch == 'search_forward_left':
                servo_cmd = STEER_LEFT
            elif self.search_latch == 'search_reverse_right':
                servo_cmd = STEER_LEFT  # Invertido al retroceder
            elif self.search_latch == 'search_reverse_left':
                servo_cmd = STEER_RIGHT # Invertido al retroceder
            else:
                servo_cmd = STEER_CENTER

                
            servo_pos = self._steer_command(servo_cmd)
        else:
            # Modo Seguimiento Normal: PID Ponderado
            if isinstance(errs, list) and isinstance(hasl, list) and len(errs) == len(hasl) == 5 and any_band:
                # Calcular error promedio ponderado
                acc = 0.0; wsum = 0.0
                for i, e in enumerate(errs):
                    if e is not None and hasl[i]:
                        acc  += WIN_WEIGHTS[i] * float(e)
                        wsum += WIN_WEIGHTS[i]
                if wsum > 0:
                    mix_err = acc / wsum
                    # Término D predictivo espacial (Gradiente visual)
                    e_top    = float(errs[0]) if (hasl[0] and errs[0] is not None) else mix_err
                    e_bottom = float(errs[4]) if (hasl[4] and errs[4] is not None) else mix_err
                    grad = e_bottom - e_top
                    mix_err = mix_err + PRED_GAIN * grad
                    
                    # Filtrado temporal
                    alpha_use = ERR_EMA_ALPHA_BLACK if (current_band_color == 'black') else ERR_EMA_ALPHA
                    mix_err = self._filter_mix_err(mix_err, alpha=alpha_use)

            if mix_err is None:
                servo_base = STEER_CENTER
            else:
                now = time.time()
                dt = max(1e-3, now - self._pid_last_time)

                # --- CONTROLADOR PID ---
                error = float(mix_err) # Proporcional

                if current_band_color == 'black' and abs(error) < BLACK_DEADBAND_PX:
                    error = 0.0 # Zona muerta específica

                # Integral (con limitador Anti-windup)
                if not any_band:
                    self._pid_integral = 0.0
                else:
                    self._pid_integral += error * dt
                    self._pid_integral = max(-100.0, min(100.0, self._pid_integral))

                # Derivativo (dError/dt)
                derivative = (error - self._pid_last_err) / dt

                self._pid_last_err = error
                self._pid_last_time = now

                # Salida del PID
                pid_out = (K_STEER * error) + (KI_STEER * self._pid_integral) + (KD_STEER * derivative)

                # Active Gaze: Mover cámara en curvas cerradas
                pan_offset = 0
                if current_band_color == 'yellow':
                    pan_offset = self._update_pan_servo(mix_err)
                    if pan_offset >= 75: pan_offset = 75
                elif current_band_color == 'white':
                    pan_offset = self._update_pan_servo(mix_err)
                else:
                    self._update_pan_servo(None) # Centrar cámara
                    pan_offset = 0

                # Aplicación de salida
                if current_band_color == 'black':
                    servo_base = int(STEER_CENTER + pid_out)
                    # Limitación suave para alta velocidad
                    servo_base = max(85.5, min(91.5, servo_base))
                if current_band_color == 'yellow' or current_band_color == 'white':
                    # Compensación de Pan en curvas lentas
                    servo_base = int(STEER_CENTER + pan_offset + pid_out)
                else:
                    servo_base = int(STEER_CENTER + pid_out)

            # Pre-giro (Feed-forward) basado en visión lejana
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
            
            # Ajuste de velocidad del servo según urgencia
            rate_use = STEER_SLEW_RATE_BLACK if (current_band_color == 'black') else STEER_SLEW_DEG_PER_SEC
            servo_pos = self._steer_command(servo_cmd, slew_rate=rate_use)

        # -------------------------------------------------------
        # 5. Control de Velocidad (Throttle)
        # -------------------------------------------------------
        
        # Detección de STOP (Línea roja) y gestión de Misión QR
        if current_band_color == 'red' and self.search_latch is None and (time.time() > self._ignore_red_until):
            
            # Caso A: Se requiere lectura de un QR
            if self.qr_needs_read:
                print("[QR] Deteniendo para escaneo de código...")
                self._motor_stop()
                
                try: # Pausar visión principal
                    cam = Camera.get_instance()
                    cvp = cam.cv_thread
                    cvp.pause() 
                except Exception: pass
                    
                # Posicionamiento de cámara para leer QR
                try:
                    RPIservo.move(SERVO_TILT, 90) # Mirar arriba
                    RPIservo.move(SERVO_PAN, 85)
                    cvp.start_qr_scan() # Iniciar hilo scanner
                except Exception: pass

                # Espera bloqueante hasta leer código o timeout
                qr_timeout = 60.0
                qr_start_time = time.time()
                qr_found = False
                
                while (time.time() - qr_start_time) < qr_timeout:
                    try:
                        st, seq = cvp.get_line_state(wait_new=False)
                        if st.get('qr_valid', False):
                            color = st.get('qr_color')
                            mode = st.get('qr_mode')
                            cycles = st.get('qr_cycles')
                            qr_found = True
                            print(f"[QR] Datos recibidos: Color={color}, Modo={mode}, Ciclos={cycles}")
                            break
                    except Exception: pass
                    time.sleep(0.1)
                
                try: cvp.stop_qr_scan()
                except: pass
                
                # Configuración de la nueva misión
                self.qr_initial_color = color
                self.qr_mode = mode
                self.qr_cycles_total = cycles
                self.qr_cycles_done = 0
                self.qr_current_color = color
                self.qr_needs_read = False
                
                
                # Configuración de búsqueda forzada para encontrar el nuevo color objetivo
                self.color_search_mode = True
                self.color_search_target = color
                self.color_search_forced = True
                self.color_search_frames = 0
                
                # Orientación inicial heurística según el color objetivo
                if color == 'white':
                    self.color_search_direction = 'left' 
                    RPIservo.move(SERVO_PAN, 100)
                    try: cvp.set_ignore_colors(['yellow'])
                    except: pass
                elif color == 'yellow':
                    self.color_search_direction = 'right'
                    RPIservo.move(SERVO_PAN, 70)
                    try: cvp.set_ignore_colors(['white'])
                    except: pass
                else:
                    self.color_search_direction = 'left'
                    try: cvp.set_ignore_colors([])
                    except: pass
                
                try: cvp.resume()
                except: pass

                # Restauración de actuadores para conducción
                try:
                    RPIservo.move(SERVO_TILT, 40)
                    RPIservo.move(SERVO_PAN, 85)
                    RPIservo.move(SERVO_STEERING, 88.5)
                except: pass
                
                # Limpiar flag de lectura
                try:
                    with cvp._line_lock:
                        cvp._line_state['qr_valid'] = False
                except: pass
                
                time.sleep(RED_STOP_TIME) # Espera semafórica
                
                self._set_target_speed(50)
                self._ramp_and_drive()
                self._ignore_red_until = time.time() + RED_IGNORE_TIME
                return
                
            # Caso B: Si ya tenemos misión, contamos ciclos (vueltas/paradas)
            else:
                self.qr_cycles_done += 1
                
                # Lógica Modo Único
                if self.qr_mode == 'U':
                    if self.qr_cycles_done >= self.qr_cycles_total:
                        self.qr_needs_read = True # Misión cumplida, esperar nuevo QR
                        print("[QR] Misión completada.")
                        self._motor_stop()
                        return
                    else:
                        self._ignore_red_until = time.time() + RED_IGNORE_TIME
                        return
                
                # Lógica Modo Alternado (Cambio de carril cada parada)
                elif self.qr_mode == 'A':
                    total_stops = self.qr_cycles_total * 2
                    
                    if self.qr_cycles_done >= total_stops:
                        self.qr_needs_read = True
                        self._motor_stop()
                        return
                    else:
                        # Cambio de color objetivo (Toggle)
                        if self.qr_current_color == 'yellow':
                            self.qr_current_color = 'white'
                        else:
                            self.qr_current_color = 'yellow'
                        
                        try:
                            cam = Camera.get_instance()
                            cvp = cam.cv_thread
                            if self.qr_current_color == 'white':
                                cvp.set_ignore_colors(['yellow'])
                            elif self.qr_current_color == 'yellow':
                                cvp.set_ignore_colors(['white'])
                        except Exception: pass
                
                        self._ignore_red_until = time.time() + RED_IGNORE_TIME
                        return

        # Aplicación de velocidad según estado de Navegación
        if self.search_latch is not None:
            # Velocidad reducida durante recuperación
            if self.search_latch in ('search_forward_right', 'search_forward_left'):
                if self.last_near_color == 'yellow':
                    self._set_target_speed(SPEED_YELLOW_MAX)
                else:
                    self._set_target_speed(SEARCH_TURN_SPEED) 
                self._ramp_and_drive()
            
            elif self.search_latch == 'search_sweep':
                # Durante el barrido, mantenemos los motores detenidos
                self._motor_stop()
            
            elif self.last_near_color == 'red' and self.qr_current_color == 'white':
                try:
                    RPIservo.move(SERVO_PAN, PAN_MAX_LEFT)
                except Exception:
                    pass
            
            else:
                # Reverso (search_reverse_left/right/smart)
                try: 
                    move.backward(SEARCH_REVERSE_SPEED) 
                    self._current_speed = -SEARCH_REVERSE_SPEED
                except Exception: self._motor_stop()

        elif self.line_follow_active and fresh and any_band:
            # Conducción Normal: Determinar velocidad base
            is_yellow_any = False
            is_white_any = False
            if isinstance(color_list, list) and isinstance(hasl, list) and len(color_list) == len(hasl):
                for i in range(len(color_list)):
                    if hasl[i]:
                        if color_list[i] == 'yellow': is_yellow_any = True
                        if color_list[i] == 'white':  is_white_any = True
            
            target_max = DRIVE_MAX_SPEED
            
            # Prioridad de reducción de velocidad
            if is_yellow_any:
                target_max = SPEED_YELLOW_MAX # Precaución
                if self._current_speed > target_max:
                    self._current_speed = float(target_max)
            elif is_white_any:
                target_max = SPEED_WHITE_NORMAL
            elif current_band_color == 'black':
                target_max = SPEED_BLACK_BOOST

            # Cálculo de frenado inteligente en curvas
            ref_err = float(en) if (hn and en) else (float(mix_err) if mix_err else 0.0)
            abs_ref = abs(ref_err)
            
            if abs_ref >= ERR_STOP_THRESH_PX:
                # Curva muy crítica: Velocidad mínima
                base = max(0, int(DRIVE_BASE_SPEED * 0.5))
            else:
                # Interpolación lineal de velocidad basada en error
                thresh_use = BLACK_SLOW_THRESH_PX if (current_band_color == 'black') else ERR_SLOW_THRESH_PX
                k = max(0.0, min(1.0, abs_ref / float(thresh_use)))
                base = int(DRIVE_BASE_SPEED + (target_max - DRIVE_BASE_SPEED) * (1.0 - k))
                base = max(base, MIN_MOVE_SPEED)

            # Reducciones adicionales predictivas
            cut_mid = cut_far = 0
            if hm and em is not None:
                k_mid = max(0.0, min(1.0, abs(float(em)) / float(MID_ERR_SLOW_THRESH)))
                cut_mid = int(CUT_MID_FRAC * (DRIVE_MAX_SPEED - MIN_MOVE_SPEED) * k_mid)
            if hf and ef is not None:
                k_far = max(0.0, min(1.0, abs(float(ef)) / float(FAR_ERR_SLOW_THRESH)))
                cut_far = int(CUT_FAR_FRAC * (DRIVE_MAX_SPEED - MIN_MOVE_SPEED) * k_far)

            # Reducción por descentramiento acumulado
            errs_abs = []
            if isinstance(errs, list) and len(errs) >= 5:
                # Verificar desviaciones en la mitad inferior de la imagen
                num_bands_to_check = len(errs) // 2 
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
            self._ramp_and_drive() 

        else:
            # Parada de Seguridad por pérdida de señal sostenida
            self._no_line_frames += 1
            if self._no_line_frames >= NO_LINE_STOP_FRAMES:
                self._motor_stop()
                
                # Si no estamos ya en una maniobra, iniciamos BARRIDO
                #Pendiente de prueba
                if self.search_latch is None:
                    print("[Lost] Línea perdida. Iniciando barrido visual...")
                    self.search_latch = 'search_sweep'
                    self.sweep_start_time = time.time()

    # ---------------------------------------------------------
    # Ejecución del Thread
    # ---------------------------------------------------------

    def functionGoing(self):
        if self.functionMode == 'trackLine':
            self.trackLineProcessing()

    def run(self):
        while True:
            self.__flag.wait()   # Espera hasta resume()       
            if self.functionMode != 'none':
                self.functionGoing()
