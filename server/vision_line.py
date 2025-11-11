# vision_line.py
# Seguimiento de línea AUTO (negra o blanca) con filtros geométricos y de contraste.
# Mantiene compatibilidad con run_line_black / run_line_white.

import cv2
import numpy as np
import time

# ==========================
# Perfiles HSV + Otsu (para wrappers)
# ==========================
BLACK_PROFILE = {
    "hsv_lower": (40, 0, 0),
    "hsv_upper": (179, 255, 20),
    "otsu_invert": True,
}

WHITE_PROFILE = {
    "hsv_lower": (0, 90, 80),
    "hsv_upper": (179, 255, 255),
    "otsu_invert": False,
}

RED_PROFILE = {
    "hsv_lower": (0, 193, 0),
    "hsv_upper": (15, 255, 255),
    "otsu_invert": False,
}

YELLOW_PROFILE = {
    "hsv_lower": (0, 0, 60),
    "hsv_upper": (55, 255, 255),
    "otsu_invert": False, # Las líneas claras no invierten Otsu
}

# ==========================
# Parámetros por bandas
# ==========================
Y_FRACS = [(0.00, 0.20), (0.20, 0.40), (0.40, 0.60), (0.60, 0.80), (0.80, 1.00)]
N_BANDS = 5
BAND_ENABLED = [False, False, True, True, True]
MIN_AREAS = [140, 160, 180, 200, 220]
KERNEL_SIZES = [3, 3, 3, 3, 3]

CORRIDOR_HALVES = [50, 45, 40, 35, 55]
MAX_STEP_X = [30, 25, 22, 18, 9999]

W_EMA = 0.6
WIDEN_STEP = 20
MAX_WIDEN = 60

FORCE_NEAR_VERTICAL = True
NEAR_LEN, NEAR_MIN_W, NEAR_MAX_W = 140, 10, 70

# Overlays (contornos clásicos)
BORDER = 3
COLORS = [(0, 0, 255), (0, 165, 255), (0, 255, 255), (0, 255, 165), (0, 255, 0)]

# Filtros / thresholds
FILL_RATIO_MAX = 0.32
ELONG_MIN_NON_NEAR = 1.4
CONTRAST_MIN_WHITE = 8.0
CONTRAST_MIN_BLACK = 6.0
FILL_RATIO_MAX_WHITE = 0.45
FILL_RATIO_MAX_BLACK = 0.60

# Texto HUD
TEXT_FONT_SCALE = 0.4
TEXT_THICKNESS = 1
TEXT_Y = 18
TEXT_COLOR = (50, 220, 50)

# Estados previos
_SW_PREV_CX = [None] * N_BANDS
_POLARITY_PREV = [None] * N_BANDS

# ==========================
# Overlay en modo "filas" (rectángulo que se mueve en X)
# ==========================
ROW_MODE = True
ROW_PAD = 4
ROW_BOX_W = 90
ROW_THICK = 2
ROW_USE_POLARITY_COLOR = True
ROW_FILLED = True
ROW_FILL_ALPHA = 0.25

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _fill_rect_alpha(dst, pt1, pt2, color, alpha=0.25):
    """Rellena un rectángulo con transparencia sobre 'dst' (BGR)."""
    lay = dst.copy()
    cv2.rectangle(lay, pt1, pt2, color, -1)
    cv2.addWeighted(lay, alpha, dst, 1.0 - alpha, 0, dst)

def _row_color(i, polarity):
    # Evitamos blanco puro para no "blanquear" visualmente
    if ROW_USE_POLARITY_COLOR and polarity in ("black", "white", "red", "yellow"):
        return (0, 180, 255) if polarity == "black" else \
               (200, 200, 200) if polarity == "white" else \
               (0, 0, 255) if polarity == "red" else \
               (0, 255, 255) # Añadido: BGR para amarillo
    return COLORS[i]

def _draw_row_box(overlay, i, y1, y2, cx, img_w, color, box_w=None, filled=False, alpha=0.25):
    if box_w is None:
        box_w = ROW_BOX_W
    row_h = (y2 - y1)
    box_h = max(8, row_h - 2 * ROW_PAD)
    cy = (y1 + y2) // 2
    half_w = int(box_w // 2)
    xL = _clamp(int(cx - half_w), 0, img_w - 1)
    xR = _clamp(int(cx + half_w), 0, img_w - 1)
    yT = _clamp(int(cy - box_h // 2), y1, y2 - 1)
    yB = _clamp(int(cy + box_h // 2), y1, y2 - 1)
    if filled:
        _fill_rect_alpha(overlay, (xL, yT), (xR, yB), color, alpha)
    cv2.rectangle(overlay, (xL, yT), (xR, yB), color, ROW_THICK)

# ==========================
# Helpers de máscaras
# ==========================
def _mask_white(roi_bgr, roi_hsv, roi_gray, ksize=3):
    m_hsv = cv2.inRange(
        roi_hsv,
        np.array((0, 0, 190), np.uint8),
        np.array((179, 90, 255), np.uint8)
    )
    m_adapt = cv2.adaptiveThreshold(
        roi_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, -5
    )
    m = cv2.bitwise_or(m_hsv, m_adapt)
    ker = np.ones((ksize, ksize), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, ker, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ker, iterations=1)
    return m

def _mask_black(roi_bgr, roi_hsv, roi_gray, ksize=3):
    m_hsv = cv2.inRange(
        roi_hsv,
        np.array((0, 0, 0), np.uint8),
        np.array((179, 255, 120), np.uint8)
    )
    blur = cv2.GaussianBlur(roi_gray, (5, 5), 0)
    m_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
    m_adapt = cv2.adaptiveThreshold(
        roi_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 5
    )
    m = cv2.bitwise_and(m_hsv, cv2.bitwise_and(m_otsu, m_adapt))
    ker = np.ones((ksize, ksize), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, ker, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ker, iterations=1)
    return m

def _mask_from_profile(roi_hsv, roi_gray, profile, ksize=3):
    lower = np.array(profile["hsv_lower"], np.uint8)
    upper = np.array(profile["hsv_upper"], np.uint8)
    m_hsv = cv2.inRange(roi_hsv, lower, upper)
    blur = cv2.GaussianBlur(roi_gray, (5, 5), 0)
    th_flag = cv2.THRESH_BINARY_INV if profile.get("otsu_invert", False) else cv2.THRESH_BINARY
    m_otsu = cv2.threshold(blur, 0, 255, th_flag + cv2.THRESH_OTSU)[1]
    m = cv2.bitwise_or(m_hsv, m_otsu)
    ker = np.ones((ksize, ksize), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, ker, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ker, iterations=1)
    return m

def _mask_red(roi_bgr, roi_hsv, roi_gray, ksize=3):
    return _mask_from_profile(roi_hsv, roi_gray, RED_PROFILE, ksize=ksize)

def _mask_yellow(roi_bgr, roi_hsv, roi_gray, ksize=3):
    return _mask_from_profile(roi_hsv, roi_gray, YELLOW_PROFILE, ksize=ksize)

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
    y1 = int(h * Y_FRACS[idx][0])
    y2 = int(h * Y_FRACS[idx][1])
    roi_bgr = frame[y1:y2, :]
    roi_hsv = hsv_full[y1:y2, :]
    roi_gray = gray_full[y1:y2, :]

    k = KERNEL_SIZES[idx]
    m_white = _mask_white(roi_bgr, roi_hsv, roi_gray, ksize=k)
    m_black = _mask_black(roi_bgr, roi_hsv, roi_gray, ksize=k)
    m_red = _mask_red(roi_bgr, roi_hsv, roi_gray, ksize=k)
    m_yellow = _mask_yellow(roi_bgr, roi_hsv, roi_gray, ksize=k)

    c_b = _largest_contour(m_black)
    c_w = _largest_contour(m_white)
    c_r = _largest_contour(m_red)
    c_y = _largest_contour(m_yellow)

    # Contraste
    bg_med = float(np.median(roi_gray)) if roi_gray.size else 0.0
    mu_b = _mean_gray_in_contour(roi_gray, c_b) if c_b is not None else None
    mu_w = _mean_gray_in_contour(roi_gray, c_w) if c_w is not None else None
    ok_b = (mu_b is not None) and ((bg_med - mu_b) >= CONTRAST_MIN_BLACK)
    ok_w = (mu_w is not None) and ((mu_w - bg_med) >= CONTRAST_MIN_WHITE)
    if not ok_b:
        c_b = None
    if not ok_w:
        c_w = None

    def process_contour(c, is_near, is_black):
        if c is None:
            return None, None, None, None, 0.0, None, None
        rect = cv2.minAreaRect(c)
        (cx_r, cy_r), (rw, rh), ang = rect

        w_rect, h_rect = (min(rw, rh), max(rw, rh))
        elong = (h_rect + 1e-6) / (w_rect + 1e-6)
        if not is_near and elong < ELONG_MIN_NON_NEAR:
            return None, None, None, None, 0.0, None, None

        if corridor_center is not None and corridor_half is not None:
            xL = max(0, int(corridor_center - corridor_half))
            xR = min(w, int(corridor_center + corridor_half))
            band_area = float(max(1, (y2 - y1) * (xR - xL)))
        else:
            band_area = float(max(1, (y2 - y1) * w))

        area = float(cv2.contourArea(c))
        limit = FILL_RATIO_MAX_BLACK if is_black else FILL_RATIO_MAX_WHITE
        if (area / band_area) > limit:
            return None, None, None, None, 0.0, None, None

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

        score = _score_contour(c, band_area, corridor_center, cx_r)
        return rect, box, int(cx_r), int(cy_r), score, area, band_area

    rect_b, box_b, cx_b, cy_b, score_b, area_b, _ = process_contour(c_b, is_near=(idx == 4), is_black=True)
    rect_w, box_w, cx_w, cy_w, score_w, area_w, _ = process_contour(c_w, is_near=(idx == 4), is_black=False)
    rect_r, box_r, cx_r, cy_r, score_r, area_r, _ = process_contour(c_r, is_near=(idx == 4), is_black=False)
    rect_y, box_y, cx_y, cy_y, score_y, area_y, _ = process_contour(c_y, is_near=(idx == 4), is_black=False)

    prev = _POLARITY_PREV[idx]

    # construir lista de candidatos presentes
    cand = []
    if rect_b is not None:
        cand.append(('black', score_b))
    if rect_w is not None:
        cand.append(('white', score_w))
    if rect_r is not None:
        cand.append(('red', score_r))
    if rect_y is not None:
        cand.append(('yellow', score_y))
    if not cand:
        chosen = None
    else:
        # elige el de mayor score
        chosen = max(cand, key=lambda t: t[1])[0]
        if prev is not None:
            s_chosen = dict(cand).get(chosen, 0.0)
            s_prev = dict(cand).get(prev, 0.0)
            # si el ganador no mejora lo suficiente, mantenemos el anterior
            if s_prev > 0 and (s_chosen / max(1e-6, s_prev)) < 1.2:
                chosen = prev

    _POLARITY_PREV[idx] = chosen

    if chosen is None:
        if draw_overlays and overlay is not None and not ROW_MODE:
            cv2.rectangle(overlay, (0, y1), (w - 1, y2 - 1), COLORS[idx], 1)
        return (False, None, None, y1, y2, None, None, None)

    if chosen == 'black':
        rect, box, cx, cy, score = rect_b, box_b, cx_b, cy_b, score_b
        color_draw = (0, 0, 0)
    elif chosen == 'white':
        rect, box, cx, cy, score = rect_w, box_w, cx_w, cy_w, score_w
        color_draw = (255, 255, 255)
    elif chosen == 'red': # El 'else' original ahora es 'elif'
        rect, box, cx, cy, score = rect_r, box_r, cx_r, cy_r, score_r
        color_draw = (0, 255, 255)
    elif chosen == 'yellow':
        rect, box, cx, cy, score = rect_y, box_y, cx_y, cy_y, score_y
        color_draw = (0, 255, 255) # BGR para amarillo

    used_w = None if not FORCE_NEAR_VERTICAL or idx != 4 or rect is None else min(rect[1])

    if rect is None or box is None:
        if draw_overlays and overlay is not None and not ROW_MODE:
            cv2.rectangle(overlay, (0, y1), (w - 1, y2 - 1), COLORS[idx], 1)
        return (False, None, None, y1, y2, None, None, chosen)

    box[:, 1] += y1
    cx_abs = int(cx)
    cy_abs = int(cy) + y1

    if draw_overlays and overlay is not None and not ROW_MODE:
        cv2.drawContours(overlay, [box], 0, color_draw, BORDER)
        cv2.circle(overlay, (cx_abs, cy_abs), 3, color_draw, -1)
        cv2.rectangle(overlay, (0, y1), (w - 1, y2 - 1), COLORS[idx], 1)
        if corridor_center is not None and corridor_half is not None:
            xL = max(0, int(corridor_center - corridor_half))
            xR = min(w - 1, int(corridor_center + corridor_half))
            cv2.rectangle(overlay, (xL, y1), (xR, y2 - 1), (50, 220, 50), 1)

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

    # Guías de filas
    if draw_overlays and overlay is not None and ROW_MODE:
        for i in range(len(Y_FRACS)):
            y1g = int(h * Y_FRACS[i][0])
            cv2.line(overlay, (0, y1g), (w - 1, y1g), (70, 70, 70), 1)
        cv2.line(overlay, (0, h - 1), (w - 1, h - 1), (70, 70, 70), 1)

    results = [None] * N_BANDS
    color_list = [None] * N_BANDS
    corridor_center = None
    widen = 0
    order = [4, 3, 2, 1, 0]

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
            widen += WIDEN_STEP
            if draw_overlays and overlay is not None and ROW_MODE:
                cx_draw = int(_SW_PREV_CX[i]) if _SW_PREV_CX[i] is not None else (w // 2)
                color = _row_color(i, None)
                _draw_row_box(overlay, i, y1, y2, cx_draw, w, color,
                              box_w=None, filled=ROW_FILLED, alpha=ROW_FILL_ALPHA)
            continue

        # Suavizado lateral
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

        # VISUAL por filas
        if draw_overlays and overlay is not None and ROW_MODE:
            cx_draw = results[i]['cx']
            pol = color_list[i]
            color = _row_color(i, pol)
            dyn_w = None
            if results[i]['w_used'] is not None and i == 4:
                dyn_w = int(_clamp(results[i]['w_used'] * 1.2, 30, max(40, (y2 - y1) * 0.9)))
            _draw_row_box(overlay, i, y1, y2, cx_draw, w, color,
                          box_w=dyn_w, filled=ROW_FILLED, alpha=ROW_FILL_ALPHA)

    # HUD
    if draw_overlays and overlay is not None:
        cv2.line(overlay, (center_x, 0), (center_x, h - 1), (0, 255, 255), 1)
        status = " ".join([
            f"{i}:{'B' if color_list[i] == 'black' else ('W' if color_list[i] == 'white' else ('R' if color_list[i] == 'red' else ('Y' if color_list[i] == 'yellow' else '--')))}"
            for i in range(N_BANDS - 1, -1, -1)
        ])
        cv2.putText(overlay, f"lineAuto | {status}", (10, TEXT_Y),
                    cv2.FONT_HERSHEY_SIMPLEX, TEXT_FONT_SCALE, TEXT_COLOR, TEXT_THICKNESS, cv2.LINE_AA)

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
        'color_list': color_list,
    }
    return state, (overlay if draw_overlays else None)


# ==========================
# Wrappers (compat)
# ==========================
def _run_line_with_profile(frame: np.ndarray, profile: dict, draw_overlays: bool = True):
    h, w = frame.shape[:2]
    center_x = w // 2
    overlay = frame.copy() if draw_overlays else None

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    results = [None] * N_BANDS
    color_list = [None] * N_BANDS
    corridor_center = None
    widen = 0
    order = [4, 3, 2, 1, 0]

    for i in order:
        y1 = int(h * Y_FRACS[i][0]); y2 = int(h * Y_FRACS[i][1])
        if not BAND_ENABLED[i]:
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
            color_list[i] = None
            continue

        roi_hsv = hsv[y1:y2, :]; roi_gray = gray[y1:y2, :]
        k = KERNEL_SIZES[i]
        mask = _mask_from_profile(roi_hsv, roi_gray, profile, ksize=k)
        c = _largest_contour(mask)

        if c is None:
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
            widen += WIDEN_STEP
            if draw_overlays and overlay is not None and ROW_MODE:
                cx_draw = int(_SW_PREV_CX[i]) if _SW_PREV_CX[i] is not None else (w // 2)
                color = _row_color(i, None)
                _draw_row_box(overlay, i, y1, y2, cx_draw, w, color,
                              box_w=None, filled=ROW_FILLED, alpha=ROW_FILL_ALPHA)
            continue

        rect = cv2.minAreaRect(c)
        (cx_r, cy_r), (rw, rh), _ang = rect

        w_rect, h_rect = (min(rw, rh), max(rw, rh))
        elong = (h_rect + 1e-6) / (w_rect + 1e-6)
        if i != 4 and elong < 1.4:
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
            widen += WIDEN_STEP
            continue

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

        results[i] = {'found': True, 'cx': int(_SW_PREV_CX[i]), 'cy': cy_abs, 'band': (y1, y2), 'w_used': None}
        corridor_center = results[i]['cx']
        widen = 0

        if draw_overlays and overlay is not None:
            if ROW_MODE:
                cx_draw = results[i]['cx']
                color = _row_color(i, None)
                _draw_row_box(overlay, i, y1, y2, cx_draw, w, color,
                              box_w=None, filled=ROW_FILLED, alpha=ROW_FILL_ALPHA)
            else:
                cv2.drawContours(overlay, [box], 0, (255, 255, 255), BORDER)
                cv2.circle(overlay, (int(_SW_PREV_CX[i]), cy_abs), 3, (255, 255, 255), -1)
                cv2.rectangle(overlay, (0, y1), (w - 1, y2 - 1), COLORS[i], 1)

    if draw_overlays and overlay is not None:
        cv2.line(overlay, (center_x, 0), (center_x, h - 1), (0, 255, 255), 1)
        if not ROW_MODE:
            cv2.putText(overlay, "lineProfile", (10, TEXT_Y),
                        cv2.FONT_HERSHEY_SIMPLEX, TEXT_FONT_SCALE, TEXT_COLOR, TEXT_THICKNESS, cv2.LINE_AA)

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

def run_line_red(frame: np.ndarray, draw_overlays: bool = True):
    return _run_line_with_profile(frame, RED_PROFILE, draw_overlays)

def run_line_yellow(frame: np.ndarray, draw_overlays: bool = True):
    return _run_line_with_profile(frame, YELLOW_PROFILE, draw_overlays)

