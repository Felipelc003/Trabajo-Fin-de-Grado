# vision_line.py
# Seguimiento de línea AUTO (negra o blanca) con filtros geométricos y de contraste.
# Mantiene compatibilidad con run_line_black / run_line_white.

import cv2
import numpy as np
import time

# ==========================
# Perfiles HSV + Otsu
# ==========================
BLACK_PROFILE = {
    "hsv_lower": (28, 0, 0),
    "hsv_upper": (179, 187, 73),
    "otsu_invert": True,   # negro: fondo claro → binario invertido
}

WHITE_PROFILE = {
    # Blanco ≈ saturación baja, valor alto (ajusta según tu iluminación)
    "hsv_lower": (117, 0, 162),
    "hsv_upper": (179, 255, 255),
    "otsu_invert": False,  # blanco: binario normal
}

# ==========================
# Parámetros por bandas
# ==========================
# 5 bandas de abajo (near) a arriba (far)
Y_FRACS = [(0.00, 0.20), (0.20, 0.40), (0.40, 0.60), (0.60, 0.80), (0.80, 1.00)]
N_BANDS = 5
BAND_ENABLED = [False, True, True, True, True]  # activa las 4 superiores + near
MIN_AREAS = [140, 160, 180, 200, 220]
KERNEL_SIZES = [3, 3, 3, 3, 3]

# Corredor por banda (mitad de ancho). Ahora también en la banda inferior.
CORRIDOR_HALVES = [50, 45, 40, 35, 55]
MAX_STEP_X = [30, 25, 22, 18, 9999]  # límite de salto lateral por banda (px)

# Suavizado y ensanchado del corredor heredado
W_EMA = 0.6
WIDEN_STEP = 20
MAX_WIDEN = 60

# Forzar rectángulo vertical en near
FORCE_NEAR_VERTICAL = True
NEAR_LEN, NEAR_MIN_W, NEAR_MAX_W = 140, 10, 70

# Overlays
BORDER = 3
COLORS = [(0, 0, 255), (0, 165, 255), (0, 255, 255), (0, 255, 165), (0, 255, 0)]

# Filtros adicionales
FILL_RATIO_MAX = 0.32        # para evitar que el suelo (gran mancha) gane
ELONG_MIN_NON_NEAR = 1.6     # forma alargada (h/ w) en bandas no-near
CONTRAST_MIN_WHITE = 8.0     # antes 12; el adaptativo ya ayuda
CONTRAST_MIN_BLACK = 6.0     # negro suele tener menos margen con fondo oscuro

FILL_RATIO_MAX_WHITE = 0.45  # blanco en suelo oscuro puede salir “grueso”
FILL_RATIO_MAX_BLACK = 0.60  # negro tolera más grosor sin confundir suelo
ELONG_MIN_NON_NEAR = 1.4     # baja un poco para no tirar líneas gordas

# Estados previos (para EMA / histeresis de color)
_SW_PREV_CX = [None, None, None, None, None]     # centro x previo por banda
_POLARITY_PREV = [None, None, None, None, None]  # 'black' | 'white'


# ==========================
# Helpers
# ==========================
def _mask_white(roi_bgr, roi_hsv, roi_gray, ksize=3):
    # HSV blanco (S bajo, V alto) – relajado
    m_hsv = cv2.inRange(
        roi_hsv,
        np.array((0, 0, 190), np.uint8),   # Vmin=190 (ajusta 170-210 según luz)
        np.array((179, 90, 255), np.uint8) # Smax=90
    )
    # Adaptativo “claro” contra fondo local
    m_adapt = cv2.adaptiveThreshold(
        roi_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
        31, -5  # bloque 31, C=-5 → resalta regiones claras
    )
    m = cv2.bitwise_or(m_hsv, m_adapt)

    ker = np.ones((ksize, ksize), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  ker, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ker, iterations=1)
    return m


def _mask_black(roi_bgr, roi_hsv, roi_gray, ksize=3):
    # HSV negro – solo techo en V (controla “oscuridad”)
    m_hsv = cv2.inRange(
        roi_hsv,
        np.array((0, 0, 0), np.uint8),
        np.array((179, 255, 120), np.uint8)   # Vmax=120 (ajusta 100–140)
    )
    # Otsu invertido (oscuras=255)
    blur = cv2.GaussianBlur(roi_gray, (5, 5), 0)
    m_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
    # Adaptativo “oscuro”
    m_adapt = cv2.adaptiveThreshold(
        roi_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,
        31, 5   # C=+5 → favorece regiones oscuras
    )
    # Intersección para limpiar fondo/sombras sueltas
    m = cv2.bitwise_and(m_hsv, cv2.bitwise_and(m_otsu, m_adapt))

    ker = np.ones((ksize, ksize), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  ker, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ker, iterations=1)
    return m


def _largest_contour(mask):
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    if not cnts:
        return None
    return max(cnts, key=cv2.contourArea)


def _mean_gray_in_contour(gray_roi, contour):
    mask = np.zeros_like(gray_roi, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    return float(cv2.mean(gray_roi, mask=mask)[0])


def _score_contour(contour, band_area, corridor_center=None, cx_rect=None):
    """Puntuación base por tamaño relativo y penalización por distancia al corredor."""
    if contour is None:
        return 0.0
    area = float(cv2.contourArea(contour))
    if area <= 0:
        return 0.0
    score = max(0.0, min(1.0, area / max(1.0, float(band_area))))
    if corridor_center is not None and cx_rect is not None:
        dist = abs(float(cx_rect) - float(corridor_center))
        score *= 1.0 / (1.0 + dist / 40.0)
    return score


# ==========================
# Núcleo AUTO por banda
# ==========================
def _detect_band_auto(idx, frame, hsv_full, gray_full, w, h,
                      corridor_center=None, corridor_half=None,
                      draw_overlays=True, overlay=None):
    """Detecta línea negra o blanca en una banda, con filtros y selección auto."""
    y1 = int(h * Y_FRACS[idx][0])
    y2 = int(h * Y_FRACS[idx][1])
    roi_bgr = frame[y1:y2, :]
    roi_hsv = hsv_full[y1:y2, :]
    roi_gray = gray_full[y1:y2, :]

    k = KERNEL_SIZES[idx]
    m_white = _mask_white(roi_bgr, roi_hsv, roi_gray, ksize=k)
    m_black = _mask_black(roi_bgr, roi_hsv, roi_gray, ksize=k)

    # Mejor contorno de cada polaridad
    c_b = _largest_contour(m_black)
    c_w = _largest_contour(m_white)

    # Fondo de la banda y contraste
    bg_med = float(np.median(roi_gray)) if roi_gray.size else 0.0
    ok_b, ok_w = False, False
    mu_b = _mean_gray_in_contour(roi_gray, c_b) if c_b is not None else None
    mu_w = _mean_gray_in_contour(roi_gray, c_w) if c_w is not None else None
    ok_b = (mu_b is not None) and ((bg_med - mu_b) >= CONTRAST_MIN_BLACK)  # negro más oscuro que fondo
    ok_w = (mu_w is not None) and ((mu_w - bg_med) >= CONTRAST_MIN_WHITE)  # blanco más claro que fondo

    # Si no pasan contraste, anula scores
    if not ok_b:
        c_b = None
    if not ok_w:
        c_w = None

    # Rects y forma / elongación / fill-ratio
    def process_contour(c, is_near):
        if c is None:
            return None, None, None, None, 0.0, None
        rect = cv2.minAreaRect(c)
        (cx_r, cy_r), (rw, rh), ang = rect

        # Normaliza dimensiones (h_rect es el mayor lado)
        w_rect, h_rect = (min(rw, rh), max(rw, rh))
        elong = (h_rect + 1e-6) / (w_rect + 1e-6)

        # Filtro de forma en NO-near
        if not is_near and elong < ELONG_MIN_NON_NEAR:
            return None, None, None, None, 0.0, None

        # Fill ratio máximo (usa área del corredor si existe)
        if corridor_center is not None and corridor_half is not None:
            xL = max(0, int(corridor_center - corridor_half))
            xR = min(w, int(corridor_center + corridor_half))
            band_area = float(max(1, (y2 - y1) * (xR - xL)))
        else:
            band_area = float(max(1, (y2 - y1) * w))

        area = float(cv2.contourArea(c))
        if (area / band_area) > FILL_RATIO_MAX:
            return None, None, None, None, 0.0, None

        # Caja para dibujar
        if FORCE_NEAR_VERTICAL and is_near:
            line_w = max(NEAR_MIN_W, min(NEAR_MAX_W, float(min(rw, rh))))
            half_w = line_w / 2.0
            half_L = float(NEAR_LEN) / 2.0
            box = np.array([
                [cx_r - half_w, cy_r - half_L],
                [cx_r + half_w, cy_r - half_L],
                [cx_r + half_w, cy_r + half_L],
                [cx_r - half_w, cy_r + half_L],
            ], dtype=np.float32).astype(int)
        else:
            box = cv2.boxPoints(rect).astype(int)

        # Puntuación (penaliza distancia al corredor si lo hay)
        score = _score_contour(c, band_area, corridor_center, cx_r)
        return rect, box, int(cx_r), int(cy_r), score, area

    rect_b, box_b, cx_b, cy_b, score_b, area_b = process_contour(c_b, is_near=(idx == 4))
    rect_w, box_w, cx_w, cy_w, score_w, area_w = process_contour(c_w, is_near=(idx == 4))

    # Histeresis de polaridad
    prev = _POLARITY_PREV[idx]
    chosen = None
    if (rect_b is None) and (rect_w is None):
        chosen = None
    elif (rect_b is not None) and (rect_w is None):
        chosen = 'black'
    elif (rect_b is None) and (rect_w is not None):
        chosen = 'white'
    else:
        chosen = 'black' if score_b >= score_w else 'white'
        if prev is not None:
            s_chosen = score_b if chosen == 'black' else score_w
            s_prev = score_b if prev == 'black' else score_w
            if s_prev > 0 and (s_chosen / max(1e-6, s_prev)) < 1.2:
                chosen = prev
    _POLARITY_PREV[idx] = chosen

    # Nada válido
    if chosen is None:
        if draw_overlays and overlay is not None:
            cv2.rectangle(overlay, (0, y1), (w - 1, y2 - 1), COLORS[idx], 1)
        return (False, None, None, y1, y2, None, None, None)

    # Datos de la opción elegida
    if chosen == 'black':
        rect, box, cx, cy, score = rect_b, box_b, cx_b, cy_b, score_b
        color_draw = (0, 180, 255)  # naranja
    else:
        rect, box, cx, cy, score = rect_w, box_w, cx_w, cy_w, score_w
        color_draw = (255, 255, 255)  # blanco

    # Si por cualquier motivo no hay geometría, descarta
    if rect is None or box is None:
        if draw_overlays and overlay is not None:
            cv2.rectangle(overlay, (0, y1), (w - 1, y2 - 1), COLORS[idx], 1)
        return (False, None, None, y1, y2, None, None, chosen)

    # Trasladar a coords absolutas
    box[:, 1] += y1
    cx_abs = int(cx)
    cy_abs = int(cy) + y1

    # Dibujo
    if draw_overlays and overlay is not None:
        cv2.drawContours(overlay, [box], 0, color_draw, BORDER)
        cv2.circle(overlay, (cx_abs, cy_abs), 3, color_draw, -1)
        cv2.rectangle(overlay, (0, y1), (w - 1, y2 - 1), COLORS[idx], 1)
        # Corredor (si existe)
        if corridor_center is not None and corridor_half is not None:
            xL = max(0, int(corridor_center - corridor_half))
            xR = min(w - 1, int(corridor_center + corridor_half))
            cv2.rectangle(overlay, (xL, y1), (xR, y2 - 1), (50, 220, 50), 1)

    used_w = None if not FORCE_NEAR_VERTICAL or idx != 4 else min(rect[1])
    return (True, cx_abs, cy_abs, y1, y2, box, used_w, chosen)


# ==========================
# Función principal AUTO
# ==========================
def run_line_auto(frame: np.ndarray, draw_overlays: bool = True):
    h, w = frame.shape[:2]
    center_x = w // 2
    overlay = frame.copy() if draw_overlays else None

    hsv_full = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    results = [None] * N_BANDS
    color_list = [None] * N_BANDS
    corridor_center = None
    widen = 0
    order = [4, 3, 2, 1, 0]  # de abajo a arriba

    for i in order:
        y1 = int(h * Y_FRACS[i][0])
        y2 = int(h * Y_FRACS[i][1])

        if not BAND_ENABLED[i]:
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
            color_list[i] = None
            continue

        expected = corridor_center if corridor_center is not None else _SW_PREV_CX[i]
        half = CORRIDOR_HALVES[i]
        half_eff = None
        if half is not None:
            # Ensancha si la banda previa falló; limita ensanchado total
            half_eff = min(half + widen, (MAX_WIDEN if i != 4 else half))

        found, cx, cy, y1, y2, box, used_w, chosen = _detect_band_auto(
            i, frame, hsv_full, gray_full, w, h,
            corridor_center=expected if half_eff is not None else None,
            corridor_half=half_eff,
            draw_overlays=draw_overlays, overlay=overlay
        )
        color_list[i] = chosen

        if not found:
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
            widen += WIDEN_STEP  # ampliar corredor para la siguiente banda
            continue

        # Limitar salto lateral y suavizado
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
        widen = 0  # resetea ensanchado al encontrar línea

    # Overlays globales
    if draw_overlays and overlay is not None:
        cv2.line(overlay, (center_x, 0), (center_x, h - 1), (0, 255, 255), 1)
        status = " ".join([f"{i}:{'B' if color_list[i]=='black' else ('W' if color_list[i]=='white' else '--')}"
                           for i in range(N_BANDS - 1, -1, -1)])
        cv2.putText(overlay, f"         lineAuto | {status}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (50, 220, 50), 2, cv2.LINE_AA)

    has_list = [r['found'] for r in results]
    cx_list = [r['cx'] for r in results]
    bands = [r['band'] for r in results]
    err_list = [(center_x - cx) if cx is not None else None for cx in cx_list]

    state = {
        'has_list': has_list,
        'cxs': cx_list,
        'errs': err_list,
        'bands': bands,
        'has_near': has_list[4], 'err_near': err_list[4], 'cx_near': cx_list[4], 'band_near': bands[4],
        'has_mid': has_list[2], 'err_mid': err_list[2], 'cx_mid': cx_list[2], 'band_mid': bands[2],
        'has_far': has_list[1], 'err_far': err_list[1], 'cx_far': cx_list[1], 'band_far': bands[1],
        'has_line': any(has_list),
        'err': err_list[4], 'cx': cx_list[4],
        'timestamp': time.time(), 'img_w': w, 'img_h': h,
        'color_list': color_list,  # telemetría opcional
    }
    return state, (overlay if draw_overlays else None)


# ==========================
# Wrappers específicos (compat)
# ==========================
def _run_line_with_profile(frame: np.ndarray, profile: dict, draw_overlays: bool = True):
    """Versión simple forzada a un color (útil para pruebas)."""
    h, w = frame.shape[:2]
    center_x = w // 2
    overlay = frame.copy() if draw_overlays else None

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    results = [None] * N_BANDS
    corridor_center = None
    widen = 0
    order = [4, 3, 2, 1, 0]

    for i in order:
        y1 = int(h * Y_FRACS[i][0])
        y2 = int(h * Y_FRACS[i][1])

        if not BAND_ENABLED[i]:
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
            continue

        roi_hsv = hsv[y1:y2, :]
        roi_gray = gray[y1:y2, :]

        k = KERNEL_SIZES[i]
        mask = _mask_from_profile(roi_hsv, roi_gray, profile, ksize=k)
        c = _largest_contour(mask)

        if c is None:
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
            widen += WIDEN_STEP
            if draw_overlays and overlay is not None:
                cv2.rectangle(overlay, (0, y1), (w - 1, y2 - 1), COLORS[i], 1)
            continue

        rect = cv2.minAreaRect(c)
        (cx_r, cy_r), (rw, rh), ang = rect

        # Filtros básicos (forma y fill-ratio); más laxos que en AUTO
        w_rect, h_rect = (min(rw, rh), max(rw, rh))
        elong = (h_rect + 1e-6) / (w_rect + 1e-6)
        if i != 4 and elong < 1.4:
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
            widen += WIDEN_STEP
            continue

        # Limitar salto lateral y EMA
        cx_abs = int(cx_r)
        if _SW_PREV_CX[i] is not None:
            dx = cx_abs - _SW_PREV_CX[i]
            max_dx = MAX_STEP_X[i]
            if abs(dx) > max_dx:
                cx_abs = int(_SW_PREV_CX[i] + max_dx * (1 if dx > 0 else -1))
            _SW_PREV_CX[i] = (1.0 - W_EMA) * float(_SW_PREV_CX[i]) + W_EMA * float(cx_abs)
        else:
            _SW_PREV_CX[i] = float(cx_abs)

        if FORCE_NEAR_VERTICAL and i == 4:
            line_w = max(NEAR_MIN_W, min(NEAR_MAX_W, float(min(rw, rh))))
            half_w = line_w / 2.0
            half_L = float(NEAR_LEN) / 2.0
            box = np.array([
                [cx_r - half_w, cy_r - half_L],
                [cx_r + half_w, cy_r - half_L],
                [cx_r + half_w, cy_r + half_L],
                [cx_r - half_w, cy_r + half_L],
            ], dtype=np.float32).astype(int)
        else:
            box = cv2.boxPoints(rect).astype(int)

        box[:, 1] += y1
        cy_abs = int(cy_r) + y1

        if draw_overlays and overlay is not None:
            cv2.drawContours(overlay, [box], 0, (255, 255, 255), BORDER)
            cv2.circle(overlay, (int(_SW_PREV_CX[i]), cy_abs), 3, (255, 255, 255), -1)
            cv2.rectangle(overlay, (0, y1), (w - 1, y2 - 1), COLORS[i], 1)

        results[i] = {'found': True, 'cx': int(_SW_PREV_CX[i]), 'cy': cy_abs, 'band': (y1, y2), 'w_used': None}
        corridor_center = results[i]['cx']
        widen = 0

    if draw_overlays and overlay is not None:
        cv2.line(overlay, (center_x, 0), (center_x, h - 1), (0, 255, 255), 1)
        cv2.putText(overlay, "         lineProfile", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (50, 220, 50), 2, cv2.LINE_AA)

    has_list = [r['found'] for r in results]
    cx_list = [r['cx'] for r in results]
    bands = [r['band'] for r in results]
    err_list = [(center_x - cx) if cx is not None else None for cx in cx_list]

    state = {
        'has_list': has_list,
        'cxs': cx_list,
        'errs': err_list,
        'bands': bands,
        'has_near': has_list[4], 'err_near': err_list[4], 'cx_near': cx_list[4], 'band_near': bands[4],
        'has_mid': has_list[2], 'err_mid': err_list[2], 'cx_mid': cx_list[2], 'band_mid': bands[2],
        'has_far': has_list[1], 'err_far': err_list[1], 'cx_far': cx_list[1], 'band_far': bands[1],
        'has_line': any(has_list),
        'err': err_list[4], 'cx': cx_list[4],
        'timestamp': time.time(), 'img_w': w, 'img_h': h,
    }
    return state, (overlay if draw_overlays else None)


def run_line_black(frame: np.ndarray, draw_overlays: bool = True):
    return _run_line_with_profile(frame, BLACK_PROFILE, draw_overlays)


def run_line_white(frame: np.ndarray, draw_overlays: bool = True):
    return _run_line_with_profile(frame, WHITE_PROFILE, draw_overlays)
