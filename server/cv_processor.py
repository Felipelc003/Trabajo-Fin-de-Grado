# cv_processor.py
# Descripción: Módulo orquestador de visión artificial.
# Este componente implementa una arquitectura Productor-Consumidor utilizando multiprocesamiento para separar
# la captura de video del análisis intensivo de imágenes (detección de líneas).
# Además, gestiona un hilo dedicado para el escaneo de códigos QR, asegurando un rendimiento fluido.
# Características:
# - Multiprocesamiento para evitar bloqueo por GIL (Global Interpreter Lock).
# - Hilo secundario para lectura de códigos QR con la librería pyzbar.
# - Comunicación Inter-Procesos (IPC) mediante Colas.

import time
import threading
import multiprocessing as mp
from typing import Optional

import cv2
import numpy as np
from pyzbar import pyzbar

class CVProcessor:
    """
    Clase principal de procesamiento de visión (Controlador).
    
    Actúa como interfaz entre la cámara y los algoritmos de análisis. Recibe los fotogramas,
    los submuestrea y los envía a un proceso "worker" independiente para detectar líneas.
    Paralelamente, puede activar un escáner de códigos QR en segundo plano.
    
    Atributos:
        mode (str): Modo de operación actual (ej. 'trackLine', 'none').
        _line_state (dict): Diccionario compartido con el estado actual de la detección (error, centro, etc.).
        _vision_proc (Process): Proceso independiente encargado de ejecutar `vision_line.py`.
    """
    def __init__(self):
        """
        Constructor de la clase CVProcessor.
        Inicializa las estructuras de datos, bloqueos de hilos (Locks) y el proceso de trabajo (Worker).
        """
        self.mode: str = "none"
        self.draw_overlays: bool = True
        self.frame = None  # Buffer para almacenar el frame actual accesible por el hilo QR.
        self._paused = False

        # --- Estado Compartido de la Detección de Línea ---
        # Almacena métricas críticas como el error de alineación (err) y coordenadas (cx).
        # Se protege con un Lock para evitar condiciones de carrera entre el hilo que recibe datos y el que los lee.
        self._line_state = {
            "has_line": False, "err": 0, "cx": None,
            "img_w": 0, "img_h": 0, "timestamp": 0.0,
            # Datos específicos del subsistema QR
            "qr_data": None,        
            "qr_color": None,       
            "qr_mode": None,        
            "qr_cycles": None,      
            "qr_valid": False,      
            "qr_timestamp": 0.0,    
        }
        self._line_lock = threading.Lock()
        self._line_seq = 0 # Secuencial para sincronización y detección de nuevos datos.
        
        # --- Configuración del Escáner QR ---
        self._qr_scanning = False
        self._qr_scan_thread = None
        
        # Filtros dinámicos de color (ignorados por el algoritmo de visión)
        self._ignore_colors = []  
        self._ignore_lock = threading.Lock()

        # --- Multiprocesamiento ---
        # Configuración del método de inicio del proceso. 'fork' es preferible en Unix/Linux
        # por su eficiencia en clonar el espacio de memoria (Copy-on-Write).
        start_method = "fork" if "fork" in mp.get_all_start_methods() else "spawn"
        ctx = mp.get_context(start_method)

        # Colas de comunicación Inter-Procesos (IPC)
        # _qin: Envía frames reducidos del proceso principal al worker.
        # _qout: Recibe tuplas (Estado, Overlay) del worker al proceso principal.
        self._qin = ctx.Queue(maxsize=1)   
        self._qout = ctx.Queue(maxsize=1)  
        self._algo_size = (320, 240)       # Resolución optimizada para procesamiento rápido.
        self._every = 3                    # Factor de submuestreo temporal (procesar 1 de cada 3 frames).

        self._vehicle_status = {'speed': 0, 'steer': 88.5}
        self._status_lock = threading.Lock()

        # Inicialización y arranque del Proceso de Visión
        self._vision_proc = ctx.Process(
            target=_vision_worker_main,
            args=(self._qin, self._qout, self._algo_size),
            daemon=True, # El proceso morirá si el programa principal termina.
        )
        self._vision_proc.start()

        self._last_overlay: Optional[np.ndarray] = None
        self._frame_i = 0 # Contador de frames para el submuestreo

        # Optimización de OpenCV para desactivar su propio threading interno y evitar conflictos.
        try: cv2.setNumThreads(1)
        except Exception: pass

    # ---------------------------------------------------------
    # Métodos de Acceso al Estado (Thread-Safe)
    # ---------------------------------------------------------

    def _publish_line_state(self, st: dict):
        """Actualiza el estado interno con los resultados recibidos del worker."""
        with self._line_lock:
            self._line_state = st
            self._line_seq += 1

    def get_line_state(self, wait_new: bool = False, last_seq: Optional[int] = None,
                       timeout: float = 0.25):
        """
        Recupera el estado actual de la visión.
        
        Parámetros:
        - wait_new (bool): Si es True, bloquea hasta recibir un dato más reciente que last_seq.
        - last_seq (int): Último número de secuencia procesado por el cliente.
        - timeout (float): Tiempo máximo de espera en segundos.
        
        Retorna:
        - Tupla (estado, secuencia actual).
        """
        if wait_new and last_seq is not None:
            t0 = time.time()
            while (time.time() - t0) < timeout:
                with self._line_lock:
                    if self._line_seq != last_seq:
                        break
                time.sleep(0.004)
        with self._line_lock:
            return dict(self._line_state), self._line_seq

    def set_vehicle_status(self, speed: int, steer: int):
        """Actualiza el estado conocido del vehículo (telemetría)."""
        with self._status_lock:
            self._vehicle_status = {'speed': int(speed), 'steer': int(steer)}

    def get_vehicle_status(self):
        """Devuelve una copia del estado del vehículo."""
        with self._status_lock:
            return dict(self._vehicle_status)
    
    # ---------------------------------------------------------
    # Control de Flujo del Procesamiento
    # ---------------------------------------------------------

    def pause(self):
        """Pausa temporalmente el envío de frames al worker."""
        self._paused = True
        print("[CVProcessor] Procesamiento pausado.")
    
    def resume(self):
        """Reanuda el procesamiento de visión."""
        self._paused = False
        print("[CVProcessor] Procesamiento reanudado.")
    
    def set_ignore_colors(self, colors: list):
        """Actualiza la lista de colores a ignorar durante la segmentación."""
        with self._ignore_lock:
            self._ignore_colors = list(colors) if colors else []
    
    def get_ignore_colors(self):
        """Obtiene la lista actual de colores ignorados."""
        with self._ignore_lock:
            return list(self._ignore_colors)
    
    # ---------------------------------------------------------
    # Sistema de Escaneo QR (Segundo Plano)
    # ---------------------------------------------------------

    def start_qr_scan(self):
        """Inicia el hilo dedicado a la detección de códigos QR."""
        if self._qr_scanning:
            return
        
        self._qr_scanning = True
        self._qr_scan_thread = threading.Thread(target=self._qr_scan_worker, daemon=True)
        self._qr_scan_thread.start()
        print("[CVProcessor] Iniciando escaneo QR.")
    
    def stop_qr_scan(self):
        """Detiene el hilo de escaneo QR."""
        self._qr_scanning = False
        print("[CVProcessor] Deteniendo escaneo QR.")

    def _qr_scan_worker(self):
        """
        Lógica del hilo de decodificación QR.
        
        Analiza periódicamente el frame actual almacenado (`self.frame`) utilizando la librería pyzbar.
        Si detecta un código válido con formato 'Color-Modo-Ciclos' (ej. 'Yellow-A-3'), actualiza
        el estado compartido y detiene el escaneo automáticamente.
        """
        print("[CVProcessor QR] Worker iniciado.")
        
        while self._qr_scanning:
            try:
                if self.frame is None:
                    time.sleep(0.1)
                    continue
                
                # Copia para evitar conflictos de lectura/escritura con el hilo principal
                frame = self.frame.copy()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                decoded_objects = pyzbar.decode(gray)
                
                if decoded_objects:
                    for obj in decoded_objects:
                        qr_data = obj.data.decode('utf-8').strip().upper()
                        
                        # Validación del formato esperado: C-M-N
                        parts = qr_data.split('-')
                        if len(parts) == 3:
                            color_code, mode_code, cycles_str = parts
                            
                            # Validaciones específicas de negocio
                            valid_color = False
                            if color_code == 'Y':
                                color = 'yellow'; valid_color = True
                            elif color_code == 'W':
                                color = 'white'; valid_color = True
                            
                            valid_mode = mode_code in ['A', 'U']
                            
                            try:
                                cycles = int(cycles_str)
                                valid_cycles = cycles > 0
                            except ValueError:
                                valid_cycles = False
                            
                            if valid_color and valid_mode and valid_cycles:
                                print(f"[CVProcessor QR] QR Válido detectado: {color}, {mode_code}, {cycles}")
                                
                                with self._line_lock:
                                    self._line_state['qr_data'] = qr_data
                                    self._line_state['qr_color'] = color
                                    self._line_state['qr_mode'] = mode_code
                                    self._line_state['qr_cycles'] = cycles
                                    self._line_state['qr_valid'] = True
                                    self._line_state['qr_timestamp'] = time.time()
                                    self._line_seq += 1
                                
                                self._qr_scanning = False
                                return
                
                time.sleep(0.1) # Tasa de refresco moderada para no saturar CPU
                
            except Exception as e:
                print(f"[CVProcessor QR] Excepción en worker: {e}")
                time.sleep(0.1)
        
        print("[CVProcessor QR] Worker finalizado.")

    # ---------------------------------------------------------
    # Integración con Pipeline de Cámara
    # ---------------------------------------------------------

    def draw_elements_on_frame(self, frame_bgr: np.ndarray) -> np.ndarray:
        """
        Método de orquestación llamado cíclicamente por el hilo de la cámara.
        
        1. Actualiza el buffer de imagen para el escáner QR.
        2. Envía frames al proceso worker (aplicando submuestreo temporal).
        3. Recoge resultados asíncronos del worker.
        4. Superpone gráficos de depuración (overlays) si están habilitados.
        
        Retorna:
            El frame original (posiblemente modificado con superposiciones).
        """
        self.frame = frame_bgr # Actualización buffer QR
        
        if self.mode == "trackLine" and not self._paused:
            self._frame_i += 1
            
            # Submuestreo y Envío al proceso worker
            if (self._frame_i % self._every) == 0:
                try:
                    small = cv2.resize(frame_bgr, self._algo_size, interpolation=cv2.INTER_AREA)
                    ignore_colors = self.get_ignore_colors()
                    
                    # Manejo de la cola llena: Descartes el frame más antiguo si es necesario.
                    if self._qin.full():
                        try: self._qin.get_nowait()
                        except Exception: pass
                    
                    self._qin.put_nowait((small, ignore_colors))
                except Exception:
                    pass

            # Recogida de resultados (Non-blocking)
            # Se intenta vaciar la cola para obtener siempre el resultado más fresco disponible.
            got = False
            st = ov = None
            while True:
                try:
                    st, ov = self._qout.get_nowait()
                    got = True
                except Exception:
                    break

            if got:
                self._publish_line_state(st)
                self._last_overlay = ov if self.draw_overlays else None

        else:
            self._last_overlay = None

        # Renderizado de Overlay (Visualización de depuración)
        if self._last_overlay is not None:
            try:
                # Escalar el overlay pequeño al tamaño original del frame
                ov = cv2.resize(
                    self._last_overlay,
                    (frame_bgr.shape[1], frame_bgr.shape[0]),
                    interpolation=cv2.INTER_LINEAR
                )
                frame_bgr = ov # Reemplazo directo (o superposición lógica según necesidad)
            except Exception:
                pass

        return frame_bgr


# =========================================================
# Función del Proceso Worker (Espacio de memoria separado)
# =========================================================

def _vision_worker_main(qin: mp.Queue, qout: mp.Queue, algo_size: tuple[int, int]):
    """
    Punto de entrada para el proceso secundario de visión.
    
    Este bucle infinito se ejecuta en un núcleo separado. Consume imágenes, ejecuta
    la lógica pesada de `run_line_auto` (en vision_line.py) y devuelve los resultados.
    Aísla los cálculos intensivos del hilo principal del servidor web.
    """
    try: cv2.setNumThreads(1)
    except Exception: pass

    from vision_line import run_line_auto 

    while True:
        try:
            # Espera bloqueante hasta recibir datos
            data = qin.get()
            if isinstance(data, tuple) and len(data) == 2:
                small, ignore_colors = data
            else:
                small = data
                ignore_colors = []
        except (EOFError, KeyboardInterrupt):
            break # Terminación limpia
        except Exception:
            continue

        try:
            # Ejecución del algoritmo de visión
            state, overlay = run_line_auto(small, draw_overlays=True, ignore_colors=ignore_colors)
        except Exception as e:
            # Fallback seguro: Retornar estado vacío/neutro en caso de error algorítmico,
            # evitando que el proceso muera.
            h, w = small.shape[:2]
            state, overlay = ({
                "has_list": [False]*5, "cxs":[None]*5, "errs":[None]*5, "bands":[(0,0)]*5,
                "has_near": False, "err_near": 0, "cx_near": w//2, "band_near": (0,0),
                "has_mid":  False, "err_mid":  0, "cx_mid":  w//2, "band_mid":  (0,0),
                "has_far":  False, "err_far":  0, "cx_far":  w//2, "band_far":  (0,0),
                "has_line": False, "err": 0, "cx": w//2,
                "img_w": w, "img_h": h, "timestamp": time.time(),
                "color_list": [None]*5,
            }, None)

        # Envío de resultados: Se vacía la cola de salida para asegurar que solo
        # se emite el resultado más reciente, evitando latencia acumulada (backpressure).
        try:
            while True:
                qout.get_nowait()
        except Exception:
            pass
        qout.put((state, overlay))
