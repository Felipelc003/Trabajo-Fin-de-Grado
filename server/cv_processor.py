# cv_processor.py
# Orquesta el modo lineBlack en OTRO PROCESO y publica estado + overlay.

import time
import threading
import multiprocessing as mp
from typing import Optional

import cv2
import numpy as np


class CVProcessor:
    def __init__(self):
        self.mode: str = "none"
        self.draw_overlays: bool = True

        self._line_state = {
            "has_line": False, "err": 0, "cx": None,
            "img_w": 0, "img_h": 0, "timestamp": 0.0,
        }
        self._line_lock = threading.Lock()
        self._line_seq = 0

        # Multiproceso (fork en RPi para evitar re-imports de GPIO)
        start_method = "fork" if "fork" in mp.get_all_start_methods() else "spawn"
        ctx = mp.get_context(start_method)

        self._qin = ctx.Queue(maxsize=1)   # Frame pequeño -> worker
        self._qout = ctx.Queue(maxsize=1)  # (state, overlay) <- worker
        self._algo_size = (320, 240)
        self._every = 1   # procesa 1 de cada N frames

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

    # ----- llamado desde la cámara para pegar overlay y alimentar worker -----
    def draw_elements_on_frame(self, frame_bgr: np.ndarray) -> np.ndarray:
        if self.mode == "lineBlack":
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
                self._last_overlay = ov if self.draw_overlays else None


        else:
            self._last_overlay = None

        if self._last_overlay is not None:
            try:
                ov = cv2.resize(self._last_overlay, (frame_bgr.shape[1], frame_bgr.shape[0]),
                                interpolation=cv2.INTER_NEAREST)
                frame_bgr = cv2.addWeighted(frame_bgr, 1.0, ov, 0.8, 0.0)
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
