# vision_line.py
# Lógica "pura" del seguidor de línea negra con 5 bandas y corredores,
# basada en tu _line_black_overlay original, pero sin 'self' ni I/O.
# Devuelve (state, overlay_bgr).

import time
from typing import Tuple, List

import cv2
import numpy as np

# ===== Memoria por banda (se mantiene dentro del PROCESO de visión) =====
# Se resetea al arrancar el worker.
_SW_PREV_CX = [None, None, None, None, None]  # 0..4


def run_line_black(frame: np.ndarray, draw_overlays: bool = True) -> tuple[dict, np.ndarray | None]:
    """
    Procesa un frame BGR PEQUEÑO (p. ej. 320x240) y devuelve:
      - state: dict con has_list/cxs/errs/bands + near/mid/far + err/cx principales
      - overlay: imagen BGR con dibujos o None si draw_overlays=False
    """
    h, w = frame.shape[:2]
    center_x = w // 2
    overlay = frame.copy() if draw_overlays else None

    # --- RANGO HSV (negro del usuario) ---
    HSV_LOWER = np.array([60,   0,   0], dtype=np.uint8)
    HSV_UPPER = np.array([179, 255, 132], dtype=np.uint8)

    USE_OTSU_LEVELS = [True, True, False, False, False]  # 0..4

    y_fracs = [
        (0.00, 0.20),  # 0 top
        (0.20, 0.40),  # 1
        (0.40, 0.60),  # 2
        (0.60, 0.80),  # 3
        (0.80, 1.00),  # 4 bottom (near)
    ]
    N = 5
    BAND_ENABLED = [False, True, True, True, True]  # desactiva sólo la superior (0)

    MIN_AREAS = [140, 160, 180, 200, 220]
    KS = [3, 3, 3, 3, 3]  # kernel por banda

    CORRIDOR_HALVES = [50, 45, 40, 35, None]  # bottom sin corredor

    MAX_STEP_X = [30, 25, 22, 18, 9999]
    W_EMA      = 0.6
    WIDEN_STEP = 20
    MAX_WIDEN  = 60

    BORDER = 2
    COLORS = [
        (0, 0, 255),     # 0 top - rojo
        (0, 165, 255),   # 1 naranja
        (0, 255, 255),   # 2 amarillo/cian
        (0, 255, 165),   # 3 verde claro
        (0, 255, 0),     # 4 near - verde
    ]

    FORCE_NEAR_VERTICAL = True
    NEAR_LEN   = 140
    NEAR_MIN_W = 10
    NEAR_MAX_W = 70

    hsv_full  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def mask_hsv_or_otsu(roi_hsv, roi_gray, k, use_otsu=False):
        m_hsv = cv2.inRange(roi_hsv, HSV_LOWER, HSV_UPPER)
        if use_otsu:
            blur = cv2.GaussianBlur(roi_gray, (5, 5), 0)
            m_otsu = cv2.threshold(
                blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
            )[1]
            m = cv2.bitwise_or(m_hsv, m_otsu)
        else:
            m = m_hsv
        ker = np.ones((k, k), np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  ker, iterations=1)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ker, iterations=1)
        return m

    def detect_one_band(idx, corridor_center=None, corridor_half=None):
        y0_frac, y1_frac = y_fracs[idx]
        y1 = int(h * y0_frac)
        y2 = int(h * y1_frac)
        k = KS[idx]
        min_area = MIN_AREAS[idx]
        color = COLORS[idx]
        use_otsu = USE_OTSU_LEVELS[idx]

        if corridor_center is not None and corridor_half is not None:
            xL = max(0, int(corridor_center - corridor_half))
            xR = min(w, int(corridor_center + corridor_half))
            roi_hsv  = hsv_full[y1:y2, xL:xR, :]
            roi_gray = gray_full[y1:y2, xL:xR]
            x_off = xL
        else:
            roi_hsv  = hsv_full[y1:y2, :, :]
            roi_gray = gray_full[y1:y2, :]
            x_off = 0

        m = mask_hsv_or_otsu(roi_hsv, roi_gray, k, use_otsu=use_otsu)

        cnts = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        best = None
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) >= min_area:
                best = c

        cx_abs = cy_abs = None
        box = None
        used_w = None

        if best is not None:
            rect = cv2.minAreaRect(best)
            (cx_r, cy_r), (rw, rh), ang = rect

            if FORCE_NEAR_VERTICAL and idx == 4:
                line_w = max(NEAR_MIN_W, min(NEAR_MAX_W, float(min(rw, rh))))
                half_w = line_w / 2.0
                half_L = float(NEAR_LEN) / 2.0
                box = np.array([
                    [cx_r - half_w, cy_r - half_L],
                    [cx_r + half_w, cy_r - half_L],
                    [cx_r + half_w, cy_r + half_L],
                    [cx_r - half_w, cy_r + half_L],
                ], dtype=np.float32).astype(int)
                used_w = line_w
            else:
                box = cv2.boxPoints(rect).astype(int)

            box[:, 0] += x_off
            box[:, 1] += y1

            cx_abs = int(cx_r) + x_off
            cy_abs = int(cy_r) + y1

            if draw_overlays and overlay is not None:
                cv2.drawContours(overlay, [box], 0, color, BORDER)
                cv2.circle(overlay, (cx_abs, cy_abs), 3, color, -1)

        if draw_overlays and overlay is not None:
            cv2.rectangle(overlay, (0, y1), (w - 1, y2 - 1), color, 1)
            if corridor_center is not None and corridor_half is not None:
                xL = max(0, int(corridor_center - corridor_half))
                xR = min(w - 1, int(corridor_center + corridor_half))
                cv2.line(overlay, (xL, y1), (xL, y2 - 1), (0, 255, 255), 1)
                cv2.line(overlay, (xR, y1), (xR, y2 - 1), (0, 255, 255), 1)

        return (cx_abs is not None), cx_abs, cy_abs, y1, y2, box, used_w

    results = [None] * N
    corridor_center = None
    widen = 0
    order = [4, 3, 2, 1, 0]

    for i in order:
        if not BAND_ENABLED[i]:
            y1 = int(h * y_fracs[i][0]); y2 = int(h * y_fracs[i][1])
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
            continue

        expected = corridor_center if corridor_center is not None else _SW_PREV_CX[i]
        half = CORRIDOR_HALVES[i]
        half_eff = None
        if half is not None:
            half_eff = min(half + widen, (MAX_WIDEN if i != 4 else half))

        found, cx, cy, y1, y2, box, used_w = detect_one_band(
            i,
            corridor_center=expected if half_eff is not None else None,
            corridor_half=half_eff
        )

        if not found:
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
            widen += WIDEN_STEP
            continue

        if _SW_PREV_CX[i] is not None and cx is not None:
            dx = cx - _SW_PREV_CX[i]
            max_dx = MAX_STEP_X[i]
            if abs(dx) > max_dx:
                cx = int(_SW_PREV_CX[i] + max_dx * (1 if dx > 0 else -1))

        if _SW_PREV_CX[i] is None:
            _SW_PREV_CX[i] = float(cx)
        else:
            _SW_PREV_CX[i] = (1.0 - W_EMA) * float(_SW_PREV_CX[i]) + W_EMA * float(cx)

        results[i] = {'found': True, 'cx': int(_SW_PREV_CX[i]), 'cy': cy, 'band': (y1, y2), 'w_used': used_w}

        corridor_center = results[i]['cx']
        widen = 0

    if draw_overlays and overlay is not None:
        cv2.line(overlay, (center_x, 0), (center_x, h - 1), (0, 255, 255), 1)

    has_list = [r['found'] for r in results]
    cx_list  = [r['cx']    for r in results]
    bands    = [r['band']  for r in results]
    err_list = [(center_x - cx) if cx is not None else None for cx in cx_list]

    has_near, err_near, cx_near = has_list[4], err_list[4], cx_list[4]
    has_mid,  err_mid,  cx_mid  = has_list[2], err_list[2], cx_list[2]
    has_far,  err_far,  cx_far  = has_list[1], err_list[1], cx_list[1]

    state = {
        'has_list': has_list,
        'cxs': cx_list,
        'errs': err_list,
        'bands': bands,

        'has_near': has_near, 'err_near': err_near, 'cx_near': cx_near, 'band_near': bands[4],
        'has_mid':  has_mid,  'err_mid':  err_mid,  'cx_mid':  cx_mid,  'band_mid':  bands[2],
        'has_far':  has_far,  'err_far':  err_far,  'cx_far':  cx_far,  'band_far':  bands[1],

        'has_line': any(has_list),
        'err': err_near, 'cx': cx_near,

        'timestamp': time.time(), 'img_w': w, 'img_h': h,
    }

    if draw_overlays and overlay is not None:
        status = " ".join([f"{i}:{'OK' if has_list[i] else '--'}" for i in range(N-1, -1, -1)])
        top_text_y = max(20, bands[0][0] - 10)
        cv2.putText(overlay, f"            lineBlack | {status}", (10, top_text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (50, 220, 50), 2, cv2.LINE_AA)

    return state, (overlay if draw_overlays else None)
