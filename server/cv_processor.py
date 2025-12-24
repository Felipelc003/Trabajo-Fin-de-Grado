# cv_processor.py
# Orquesta el modo lineBlack en OTRO PROCESO y publica estado + overlay.

import time
import threading
import multiprocessing as mp
from typing import Optional

import cv2
import numpy as np
from pyzbar import pyzbar

class CVProcessor:
    def __init__(self):
        self.mode: str = "none"
        self.draw_overlays: bool = True
        self.frame = None  # Frame actual para lectura QR
        self._paused = False

        self._line_state = {
            "has_line": False, "err": 0, "cx": None,
            "img_w": 0, "img_h": 0, "timestamp": 0.0,
            # Datos QR
            "qr_data": None,        # String del QR leído (ej: "Y-A-2")
            "qr_color": None,       # 'yellow' o 'white'
            "qr_mode": None,        # 'A' o 'U'
            "qr_cycles": None,      # número entero
            "qr_valid": False,      # True si se leyó un QR válido
            "qr_timestamp": 0.0,    # Cuándo se leyó el QR
        }
        self._line_lock = threading.Lock()
        self._line_seq = 0
        
        # Estado de scaneo QR
        self._qr_scanning = False
        self._qr_scan_thread = None

        # Multiproceso (fork en RPi para evitar re-imports de GPIO)
        start_method = "fork" if "fork" in mp.get_all_start_methods() else "spawn"
        ctx = mp.get_context(start_method)

        self._qin = ctx.Queue(maxsize=1)   # Frame pequeño -> worker
        self._qout = ctx.Queue(maxsize=1)  # (state, overlay) <- worker
        self._algo_size = (320, 240)
        self._every = 3   # procesa 1 de cada N frames

        self._vehicle_status = {'speed': 0, 'steer': 90}
        self._status_lock = threading.Lock()

        self._vision_proc = ctx.Process(
            target=_vision_worker_main,
            args=(self._qin, self._qout, self._algo_size),
            daemon=True,
        )
        self._vision_proc.start()

        self._last_overlay: Optional[np.ndarray] = None
        self._frame_i = 0

        try: cv2.setNumThreads(1)
        except Exception: pass

    # ----- estado público -----
    def _publish_line_state(self, st: dict):
        with self._line_lock:
            self._line_state = st
            self._line_seq += 1

    def get_line_state(self, wait_new: bool = False, last_seq: Optional[int] = None,
                       timeout: float = 0.25):
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
        with self._status_lock:
            self._vehicle_status = {'speed': int(speed), 'steer': int(steer)}

    def get_vehicle_status(self):
        with self._status_lock:
            return dict(self._vehicle_status)

    def pause(self):
        """Pausa el procesamiento de bandas temporalmente"""
        self._paused = True
        print("[CVProcessor] Procesamiento de bandas PAUSADO")
    
    def resume(self):
        """Reanuda el procesamiento de bandas"""
        self._paused = False
        print("[CVProcessor] Procesamiento de bandas REANUDADO")
    
    # ----- QR Scanning -----
    def start_qr_scan(self):
        """Inicia el escaneo de QR en un thread separado"""
        if self._qr_scanning:
            print("[CVProcessor] QR scan ya está activo")
            return
        
        self._qr_scanning = True
        self._qr_scan_thread = threading.Thread(target=self._qr_scan_worker, daemon=True)
        self._qr_scan_thread.start()
        print("[CVProcessor] Iniciando escaneo QR...")
    
    def stop_qr_scan(self):
        """Detiene el escaneo de QR"""
        self._qr_scanning = False
        print("[CVProcessor] Deteniendo escaneo QR...")
    
    def _qr_scan_worker(self):
        """Thread worker que escanea QR codes continuamente"""
        print("[CVProcessor QR] Worker iniciado")
        
        while self._qr_scanning:
            try:
                # Usar el frame actual
                if self.frame is None:
                    time.sleep(0.1)
                    continue
                
                frame = self.frame.copy()
                
                # Convertir a escala de grises
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Detectar códigos QR
                decoded_objects = pyzbar.decode(gray)
                
                if decoded_objects:
                    for obj in decoded_objects:
                        # Decodificar el texto del QR
                        qr_data = obj.data.decode('utf-8').strip().upper()
                        print(f"[CVProcessor QR] Código detectado: {qr_data}")
                        
                        # Parsear formato C-M-N
                        parts = qr_data.split('-')
                        if len(parts) == 3:
                            color_code, mode_code, cycles_str = parts
                            
                            # Validar color
                            if color_code == 'Y':
                                color = 'yellow'
                            elif color_code == 'W':
                                color = 'white'
                            else:
                                print(f"[CVProcessor QR] Color inválido: {color_code}")
                                continue
                            
                            # Validar modo
                            if mode_code not in ['A', 'U']:
                                print(f"[CVProcessor QR] Modo inválido: {mode_code}")
                                continue
                            
                            # Validar ciclos
                            try:
                                cycles = int(cycles_str)
                                if cycles <= 0:
                                    print(f"[CVProcessor QR] Ciclos inválido: {cycles}")
                                    continue
                            except ValueError:
                                print(f"[CVProcessor QR] Ciclos no numérico: {cycles_str}")
                                continue
                            
                            # QR válido encontrado - publicar en line_state
                            print(f"[CVProcessor QR] ✓ QR válido: Color={color}, Modo={mode_code}, Ciclos={cycles}")
                            
                            with self._line_lock:
                                self._line_state['qr_data'] = qr_data
                                self._line_state['qr_color'] = color
                                self._line_state['qr_mode'] = mode_code
                                self._line_state['qr_cycles'] = cycles
                                self._line_state['qr_valid'] = True
                                self._line_state['qr_timestamp'] = time.time()
                                self._line_seq += 1
                            
                            # Detener escaneo después de leer un QR válido
                            self._qr_scanning = False
                            print("[CVProcessor QR] QR válido leído - deteniendo escaneo")
                            return
                        else:
                            print(f"[CVProcessor QR] Formato inválido: {qr_data}")
                
                # Pequeña pausa
                time.sleep(0.1)
                
            except Exception as e:
                print(f"[CVProcessor QR] Error: {e}")
                time.sleep(0.1)
        
        print("[CVProcessor QR] Worker finalizado")


    # ----- llamado desde la cámara para pegar overlay y alimentar worker -----
    def draw_elements_on_frame(self, frame_bgr: np.ndarray) -> np.ndarray:
        # Guardar frame actual para lectura QR
        self.frame = frame_bgr
        
        if self.mode == "trackLine" and not self._paused:
            self._frame_i += 1
            if (self._frame_i % self._every) == 0:
                try:
                    small = cv2.resize(frame_bgr, self._algo_size, interpolation=cv2.INTER_AREA)
                    if self._qin.full():
                        try: self._qin.get_nowait()
                        except Exception: pass
                    self._qin.put_nowait(small)
                except Exception:
                    pass

            # drenar resultados
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
                # guardamos overlay solo si queremos dibujarlo
                self._last_overlay = ov if self.draw_overlays else None

        else:
            self._last_overlay = None

        # === RENDER ===
        # Si tenemos overlay (ya es el frame BGR con los cuadrados encima), lo usamos DIRECTO.
        # ¡Importante!: Nada de addWeighted aquí para evitar "lavar/blanquear" la imagen.
        if self._last_overlay is not None:
            try:
                ov = cv2.resize(
                    self._last_overlay,
                    (frame_bgr.shape[1], frame_bgr.shape[0]),
                    interpolation=cv2.INTER_LINEAR
                )
                frame_bgr = ov
            except Exception:
                pass

        return frame_bgr


# ====================== worker (otro proceso) ======================

def _vision_worker_main(qin: mp.Queue, qout: mp.Queue, algo_size: tuple[int, int]):
    try: cv2.setNumThreads(1)
    except Exception: pass

    from vision_line import run_line_auto  # importa aquí (sólo en el hijo)

    while True:
        try:
            small = qin.get()
        except (EOFError, KeyboardInterrupt):
            break
        except Exception:
            continue

        try:
            # Pedimos a vision_line que pinte los cuadrados sobre una COPIA del frame (BGR)
            state, overlay = run_line_auto(small, draw_overlays=True)
        except Exception:
            h, w = small.shape[:2]
            state, overlay = ({
                "has_list": [False]*5, "cxs":[None]*5, "errs":[None]*5, "bands":[(0,0)]*5,
                "has_near": False, "err_near": 0, "cx_near": w//2, "band_near": (0,0),
                "has_mid":  False, "err_mid":  0, "cx_mid":  w//2, "band_mid":  (0,0),
                "has_far":  False, "err_far":  0, "cx_far":  w//2, "band_far":  (0,0),
                "has_line": False, "err": 0, "cx": w//2,
                "img_w": w, "img_h": h, "timestamp": time.time(),
            }, None)

        try:
            while True:
                qout.get_nowait()
        except Exception:
            pass
        qout.put((state, overlay))
