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

# Nombre del archivo: vision_line.py
# Descripción: Módulo avanzado de visión artificial para la detección y seguimiento de líneas.
# Implementa un sistema de análisis por bandas horizontales para robustez algorítmica.
# Soporta múltiples colores (blanco, negro, rojo, amarillo) simultáneamente y utiliza
# una combinación de filtrado por color (HSV), umbralización adaptativa (Otsu) y análisis geométrico de contornos.

import cv2
import numpy as np
import time

# ==========================
# Perfiles de Color y Umbralización
# ==========================
# Definición de rangos HSV y estrategias de binarización para cada color de interés.
# Otsu Invertido se usa para objetos oscuros sobre fondo claro (línea negra).
# En el caso de que la iluminación cambie dráticamente, se debe ajustar los valores nuevamente.

BLACK_PROFILE = {
    "hsv_lower": (0, 0, 0),
    "hsv_upper": (179, 120, 255),
    "otsu_invert": True,
}

WHITE_PROFILE = {
    "hsv_lower": (88, 20, 240),
    "hsv_upper": (113, 255, 255),
    "otsu_invert": False,
}

RED_PROFILE = {
    "hsv_lower": (0, 0, 0),
    "hsv_upper": (179, 220, 255),
    "otsu_invert": True, 
}

YELLOW_PROFILE = {
    "hsv_lower": (30, 30, 50),
    "hsv_upper": (40, 170, 255),
    "otsu_invert": False,
}

# ==========================
# Configuración del Sistema de Bandas (ROIs)
# =========================
# La imagen se divide en 5 bandas horizontales para analizar la curvarura de la línea.
# Y_FRACS define la posición vertical relativa (0.0 a 1.0) de cada banda [inicio, fin].
Y_FRACS = [(0.00, 0), (0, 0), (0, 0), (0.25, 0.45), (0.45, 1.00)]
N_BANDS = 5
# BAND_ENABLED determina qué bandas se procesan activamente (por defecto, las inferiores).
BAND_ENABLED = [False, False, False, True, True]

# Parámetros de filtrado geométrico por banda
MIN_AREAS = [210, 240, 240, 300, 330]   # Área mínima del contorno para ser considerado válido.
KERNEL_SIZES = [3, 3, 3, 3, 4]         # Tamaño del kernel para operaciones morfológicas (erosión/dilatación).

# Parámetros de seguimiento ("Corridor Logic")
# Define el ancho esperado del camino para guiar la búsqueda en la siguiente banda.
CORRIDOR_HALVES = [30, 35, 40, 45, 65]
MAX_STEP_X = [30, 25, 22, 18, 9999]    # Desplazamiento lateral máximo permitido entre frames consecutivos.

# Constantes de control
W_EMA = 0.6            # Peso del promedio móvil exponencial para suavizado de coordenadas (0.0 - 1.0).
WIDEN_STEP = 40        # Incremento del área de búsqueda si no se encuentra línea en la banda anterior.
MAX_WIDEN = 120        # Límite máximo de ampliación de búsqueda.
MAX_BAND_JUMP = 100    # Salto lateral máximo permitido entre bandas adyacentes (coherencia espacial).
FORCE_NEAR_VERTICAL = False
NEAR_LEN, NEAR_MIN_W, NEAR_MAX_W = 140, 10, 70

# Configuración Visual (Overlays)
BORDER = 3
COLORS = [(0, 0, 255), (0, 165, 255), (0, 255, 255), (0, 255, 165), (0, 255, 0)]

# Filtros de Calidad de Detección
FILL_RATIO_MAX = 0.32          # Máxima relación de llenado (área contorno / área banda) para evitar falsos positivos grandes.
ELONG_MIN_NON_NEAR = 1.4       # Elongación mínima requerida para considerar que un blob es una línea (y no una mancha circular).
CONTRAST_MIN_WHITE = 8.0       # Diferencia mínima de brillo (gris) entre línea blanca y fondo.
CONTRAST_MIN_BLACK = 10.0      # Diferencia mínima de brillo entre fondo y línea negra.
FILL_RATIO_MAX_WHITE = 0.45
FILL_RATIO_MAX_BLACK = 0.60

# Configuración de Texto en Pantalla (HUD)
TEXT_FONT_SCALE = 0.4
TEXT_THICKNESS = 1
TEXT_Y = 18
TEXT_COLOR = (50, 220, 50)

# Almacenamiento de estado entre frames (para suavizado)
_SW_PREV_CX = [None] * N_BANDS     # Última posición X conocida por banda.
_POLARITY_PREV = [None] * N_BANDS  # Último color detectado por banda.

# ==========================
# Configuración Visual ROW_MODE
# ==========================
ROW_MODE = True
ROW_PAD = 4
ROW_BOX_W = 45
ROW_THICK = 2
ROW_USE_POLARITY_COLOR = True
ROW_FILLED = True
ROW_FILL_ALPHA = 0.25

def _clamp(v, lo, hi):
    """Restringe un valor dentro de un rango [lo, hi]."""
    return max(lo, min(hi, v))

def _fill_rect_alpha(dst, pt1, pt2, color, alpha=0.25):
    """Dibuja un rectángulo con transparencia sobre la imagen destino."""
    lay = dst.copy()
    cv2.rectangle(lay, pt1, pt2, color, -1)
    cv2.addWeighted(lay, alpha, dst, 1.0 - alpha, 0, dst)

def _row_color(i, polarity):
    """Selecciona el color de visualización basado en la polaridad detectada (tipo de línea)."""
    if ROW_USE_POLARITY_COLOR and polarity in ("black", "white", "red", "yellow"):
        return (0, 180, 255) if polarity == "black" else \
               (200, 200, 200) if polarity == "white" else \
               (0, 0, 255) if polarity == "red" else \
               (0, 255, 255)
    return COLORS[i]

def _draw_row_box(overlay, i, y1, y2, cx, img_w, color, box_w=None, filled=False, alpha=0.25):
    """Función auxiliar para dibujar las cajas de seguimiento en el overlay."""
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
# Funciones de Máscara (Segmentación)
# ==========================

def _mask_white(roi_bgr, roi_hsv, roi_gray, ksize=3):
    """Genera máscara binaria para líneas blancas utilizando rango HSV."""
    lower = np.array(WHITE_PROFILE["hsv_lower"], np.uint8)
    upper = np.array(WHITE_PROFILE["hsv_upper"], np.uint8)
    
    m = cv2.inRange(roi_hsv, lower, upper)
    
    # Limpieza morfológica (Open -> Close) para eliminar ruido y cerrar huecos
    ker = np.ones((ksize, ksize), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, ker, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ker, iterations=1)
    return m

def _mask_black(roi_bgr, roi_hsv, roi_gray, ksize=3):
    """
    Genera máscara binaria para líneas negras.
    Combina múltiples técnicas para robustez: HSV + Otsu (Invertido) + Threshold Adaptativo.
    """
    # 1. Filtrado HSV (Rango de grises oscuros)
    m_hsv = cv2.inRange(
        roi_hsv,
        np.array((47, 152, 0), np.uint8),
        np.array((179, 255, 170), np.uint8) 
    )
    # 2. Binarización de Otsu sobre imagen suavizada
    blur = cv2.GaussianBlur(roi_gray, (5, 5), 0)
    m_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
    
    # 3. Umbralización Adaptativa (Gaussian) para manejar iluminación variable
    m_adapt = cv2.adaptiveThreshold(
        roi_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 5
    )
    
    # Intersección de las tres máscaras (AND) -> Solo píxeles que cumplan todos los criterios
    m = cv2.bitwise_and(m_hsv, cv2.bitwise_and(m_otsu, m_adapt))
    
    ker = np.ones((ksize, ksize), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, ker, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ker, iterations=1)
    return m


def _mask_red(roi_bgr, roi_hsv, roi_gray, ksize=3):
    """
    Genera máscara binaria para líneas rojas.
    Maneja el matiz rojo que se divide en dos rangos en el espectro HSV (0-10 y 160-180).
    """
    # Rango Rojo Bajo (0-10)
    lower1 = np.array([0, 80, 50])
    upper1 = np.array([10, 255, 255])
    m1 = cv2.inRange(roi_hsv, lower1, upper1)
    
    # Rango Rojo Alto (160-180)
    lower2 = np.array([160, 80, 50])
    upper2 = np.array([180, 255, 255])
    m2 = cv2.inRange(roi_hsv, lower2, upper2)
    
    # Unión de rangos (OR)
    m_color = cv2.bitwise_or(m1, m2)

    # Limpieza
    ker = np.ones((ksize, ksize), np.uint8)
    m = cv2.morphologyEx(m_color, cv2.MORPH_OPEN, ker, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ker, iterations=1)
    return m

def _mask_yellow(roi_bgr, roi_hsv, roi_gray, ksize=3):
    """Genera máscara binaria para líneas amarillas."""
    lower = np.array([45, 32, 165])
    upper = np.array([80, 110, 255])
    
    m_hsv = cv2.inRange(roi_hsv, lower, upper)
    
    ker = np.ones((ksize, ksize), np.uint8)
    m = cv2.morphologyEx(m_hsv, cv2.MORPH_OPEN, ker, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ker, iterations=1)
    
    return m

def _mask_from_profile(roi_hsv, roi_gray, profile, ksize=3):
    """Función genérica para crear máscaras basadas en perfiles de configuración."""
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

def _largest_contour(mask):
    """Retorna el contorno más grande encontrado en la máscara binaria."""
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    if not cnts:
        return None
    return max(cnts, key=cv2.contourArea)

def _mean_gray_in_contour(gray_roi, contour):
    """Calcula el valor medio de gris dentro de un contorno específico."""
    mask = np.zeros_like(gray_roi, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    return float(cv2.mean(gray_roi, mask=mask)[0])

def _score_contour(contour, band_area, corridor_center=None, cx_rect=None):
    """
    Asigna una puntuación (score) a un contorno candidato.
    Factores: Tamaño relativo al área de la banda y centralidad (si aplica lógica de corredor).
    """
    if contour is None:
        return 0.0
    area = float(cv2.contourArea(contour))
    if area <= 0:
        return 0.0
    score = max(0.0, min(1.0, area / max(1.0, float(band_area))))
    
    # Penalización por distancia al centro esperado (Logic de Corredor)
    if corridor_center is not None and cx_rect is not None:
        dist = abs(float(cx_rect) - float(corridor_center))
        score *= 1.0 / (1.0 + dist / 40.0)
    return score

# ==========================
# Núcleo de Detección por Banda
# ==========================
def _detect_band_auto(idx, frame, hsv_full, gray_full, w, h,
                      corridor_center=None, corridor_half=None,
                      draw_overlays=True, overlay=None, ignore_colors=None):
    """
    Analiza una banda horizontal específica en busca de líneas de cualquier color (Multicolor).
    
    Aplica concurrentemente la detección de Negro, Blanco, Rojo y Amarillo,
    y selecciona el mejor candidato basado en puntuación y consistencia.
    
    Args:
        idx (int): Índice de la banda (0-4).
        ignore_colors (list): Colores a excluir explícitamente del análisis.
    """
    if ignore_colors is None:
        ignore_colors = []
    
    # Definición de la Región de Interés (ROI) vertical
    y1 = int(h * Y_FRACS[idx][0])
    y2 = int(h * Y_FRACS[idx][1])
    roi_bgr = frame[y1:y2, :]
    roi_hsv = hsv_full[y1:y2, :]
    roi_gray = gray_full[y1:y2, :]

    k = KERNEL_SIZES[idx]
    
    # Generación de máscaras para colores permitidos
    m_white = _mask_white(roi_bgr, roi_hsv, roi_gray, ksize=k) if 'white' not in ignore_colors else np.zeros_like(roi_gray)
    m_black = _mask_black(roi_bgr, roi_hsv, roi_gray, ksize=k) if 'black' not in ignore_colors else np.zeros_like(roi_gray)
    m_red = _mask_red(roi_bgr, roi_hsv, roi_gray, ksize=k) if 'red' not in ignore_colors else np.zeros_like(roi_gray)
    m_yellow = _mask_yellow(roi_bgr, roi_hsv, roi_gray, ksize=k) if 'yellow' not in ignore_colors else np.zeros_like(roi_gray)

    # Extracción de contornos principales
    c_b = _largest_contour(m_black)
    c_w = _largest_contour(m_white)
    c_r = _largest_contour(m_red)
    c_y = _largest_contour(m_yellow)

    # Validación de Contraste (necesaria para diferenciar línea de suelo similar)
    bg_med = float(np.median(roi_gray)) if roi_gray.size else 0.0
    mu_b = _mean_gray_in_contour(roi_gray, c_b) if c_b is not None else None
    mu_w = _mean_gray_in_contour(roi_gray, c_w) if c_w is not None else None
    
    # El negro debe ser significativamente más oscuro que el fondo
    ok_b = (mu_b is not None) and ((bg_med - mu_b) >= CONTRAST_MIN_BLACK)
    # El blanco debe ser significativamente más brillante que el fondo
    ok_w = (mu_w is not None) and ((mu_w - bg_med) >= CONTRAST_MIN_WHITE)
    
    if not ok_b: c_b = None
    if not ok_w: c_w = None

    def process_contour(c, is_near, is_black):
        """Analiza geometría del contorno: Rectángulo mínimo, elongación y área."""
        if c is None:
            return None, None, None, None, 0.0, None, None
        rect = cv2.minAreaRect(c)
        (cx_r, cy_r), (rw, rh), ang = rect

        w_rect, h_rect = (min(rw, rh), max(rw, rh))
        elong = (h_rect + 1e-6) / (w_rect + 1e-6)
        
        # Filtro de forma: Descartar si es muy cuadrado/circular (salvo banda cercana)
        if not is_near and elong < ELONG_MIN_NON_NEAR:
            return None, None, None, None, 0.0, None, None

        # Cálculo de área de la banda efectiva (para ratio de llenado)
        if corridor_center is not None and corridor_half is not None:
            xL = max(0, int(corridor_center - corridor_half))
            xR = min(w, int(corridor_center + corridor_half))
            band_area = float(max(1, (y2 - y1) * (xR - xL)))
        else:
            band_area = float(max(1, (y2 - y1) * w))

        area = float(cv2.contourArea(c))
        limit = FILL_RATIO_MAX_BLACK if is_black else FILL_RATIO_MAX_WHITE
        
        # Filtro de tamaño excesivo (evitar detectar toda la carretera como línea)
        if (area / band_area) > limit:
            return None, None, None, None, 0.0, None, None

        box = cv2.boxPoints(rect).astype(int)
        score = _score_contour(c, band_area, corridor_center, cx_r)
        return rect, box, int(cx_r), int(cy_r), score, area, band_area

    # Procesamiento de todos los candidatos
    is_near_band = (idx == 4)
    rect_b, box_b, cx_b, cy_b, score_b, area_b, _ = process_contour(c_b, is_near=is_near_band, is_black=True)
    rect_w, box_w, cx_w, cy_w, score_w, area_w, _ = process_contour(c_w, is_near=is_near_band, is_black=False)
    rect_r, box_r, cx_r, cy_r, score_r, area_r, _ = process_contour(c_r, is_near=is_near_band, is_black=False)
    rect_y, box_y, cx_y, cy_y, score_y, area_y, _ = process_contour(c_y, is_near=is_near_band, is_black=False)

    # --- Lógica de Selección de Mejor Candidato ---
    prev = _POLARITY_PREV[idx]
    cand = []
    if rect_b is not None: cand.append(('black', score_b))
    if rect_w is not None: cand.append(('white', score_w))
    if rect_r is not None: cand.append(('red', score_r))
    if rect_y is not None: cand.append(('yellow', score_y))

    if not cand:
        chosen = None
    else:
        # Elegir el que tenga mayor puntuación
        chosen = max(cand, key=lambda t: t[1])[0]
        # Histéresis: Si ya seguíamos un color, preferimos mantenerlo salvo que el nuevo sea mucho mejor
        if prev is not None:
            s_chosen = dict(cand).get(chosen, 0.0)
            s_prev = dict(cand).get(prev, 0.0)
            if s_prev > 0 and (s_chosen / max(1e-6, s_prev)) < 1.2:
                chosen = prev

    _POLARITY_PREV[idx] = chosen # Actualizar memoria de estado

    if chosen is None:
        if draw_overlays and overlay is not None and not ROW_MODE:
            cv2.rectangle(overlay, (0, y1), (w - 1, y2 - 1), COLORS[idx], 1)
        return (False, None, None, y1, y2, None, None, None)

    # Asignación de variables según el ganador
    if chosen == 'black':
        rect, box, cx, cy, score = rect_b, box_b, cx_b, cy_b, score_b
        color_draw = (0, 0, 0)
    elif chosen == 'white':
        rect, box, cx, cy, score = rect_w, box_w, cx_w, cy_w, score_w
        color_draw = (255, 255, 255)
    elif chosen == 'red': 
        rect, box, cx, cy, score = rect_r, box_r, cx_r, cy_r, score_r
        color_draw = (0, 255, 255)
    elif chosen == 'yellow':
        rect, box, cx, cy, score = rect_y, box_y, cx_y, cy_y, score_y
        color_draw = (0, 0, 255)

    used_w = None if not FORCE_NEAR_VERTICAL or idx != 4 or rect is None else min(rect[1])

    if rect is None or box is None:
        return (False, None, None, y1, y2, None, None, chosen)

    # Ajuste de coordenadas al marco de referencia de la imagen completa
    box[:, 1] += y1
    cx_abs = int(cx)
    cy_abs = int(cy) + y1

    # Dibujado de resultado en overlay (Modo Contorno)
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
# Función Principal AUTO
# ==========================
def run_line_auto(frame: np.ndarray, draw_overlays: bool = True, ignore_colors: list = None):
    """
    Algoritmo Principal de Detección Automática.
    
    Ejecuta el pipeline completo de visión:
    1. Preprocesamiento (Conversión BGR -> HSV/Gray).
    2. Iteración por bandas (de abajo hacia arriba, más cercana a más lejana).
    3. Mantenimiento del "corredor" (predicción de dónde debe estar la línea en la siguiente banda).
    4. Suavizado temporal de las coordenadas detectadas.
    5. Generación de estructura de datos de estado y overlays visuales.
    
    Args:
        frame: Imagen de entrada BGR.
        draw_overlays: Booleano para activar graficos de debug.
        ignore_colors: Lista de colores a excluir del análisis.
    
    Returns:
        Tupla (diccionario_estado, frame_con_overlay).
    """
    if ignore_colors is None:
        ignore_colors = []
    
    h, w = frame.shape[:2]
    gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv_full = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    center_x = w // 2
    overlay = frame.copy() if draw_overlays else None

    # Dibujado de líneas guía (Grid)
    if draw_overlays and overlay is not None and ROW_MODE:
        for i in range(len(Y_FRACS)):
            y1g = int(h * Y_FRACS[i][0])
            cv2.line(overlay, (0, y1g), (w - 1, y1g), (70, 70, 70), 1)
        cv2.line(overlay, (0, h - 1), (w - 1, h - 1), (70, 70, 70), 1)

    results = [None] * N_BANDS
    color_list = [None] * N_BANDS
    corridor_center = None
    widen = 0
    # Orden de procesamiento: De abajo (Cercano, Index 4) hacia arriba (Lejano, Index 0)
    order = [4, 3, 2, 1, 0]

    for i in order:
        y1 = int(h * Y_FRACS[i][0])
        y2 = int(h * Y_FRACS[i][1])

        if not BAND_ENABLED[i]:
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
            color_list[i] = None
            continue

        # Lógica de Corredor: ¿Dónde esperamos encontrar la línea basándonos en la banda anterior?
        expected = corridor_center if corridor_center is not None else _SW_PREV_CX[i]
        half = CORRIDOR_HALVES[i]
        half_eff = None
        if half is not None:
            # Ampliamos la búsqueda si fallamos en la banda previa (widen)
            half_eff = min(half + widen, (MAX_WIDEN if i != 4 else half))

        found, cx, cy, y1, y2, box, used_w, chosen = _detect_band_auto(
            i, frame, hsv_full, gray_full, w, h,
            corridor_center=expected if half_eff is not None else None,
            corridor_half=half_eff,
            draw_overlays=draw_overlays, overlay=overlay,
            ignore_colors=ignore_colors
        )

        # Validación de coherencia espacial (Salto brusco)
        if found and corridor_center is not None and cx is not None:
            dist = abs(cx - corridor_center)
            if dist > MAX_BAND_JUMP:
                found = False # Descartamos por incoherencia física
                chosen = None
        color_list[i] = chosen

        if not found:
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
            widen += WIDEN_STEP # Incrementamos área de búsqueda para la siguiente banda
            if draw_overlays and overlay is not None and ROW_MODE:
                cx_draw = int(_SW_PREV_CX[i]) if _SW_PREV_CX[i] is not None else (w // 2)
                color = _row_color(i, None)
                _draw_row_box(overlay, i, y1, y2, cx_draw, w, color,
                              box_w=None, filled=ROW_FILLED, alpha=ROW_FILL_ALPHA)
            continue

        # Suavizado de coordenadas (Low-Pass Filter) para evitar jitter
        if _SW_PREV_CX[i] is not None and cx is not None:
            dx = cx - _SW_PREV_CX[i]
            max_dx = MAX_STEP_X[i]
            # Clamping de la velocidad de movimiento lateral máxima
            if abs(dx) > max_dx:
                cx = int(_SW_PREV_CX[i] + max_dx * (1 if dx > 0 else -1))

        if _SW_PREV_CX[i] is None:
            _SW_PREV_CX[i] = float(cx)
        else:
            _SW_PREV_CX[i] = (1.0 - W_EMA) * float(_SW_PREV_CX[i]) + W_EMA * float(cx)

        results[i] = {'found': True, 'cx': int(_SW_PREV_CX[i]), 'cy': cy, 'band': (y1, y2), 'w_used': used_w}
        corridor_center = results[i]['cx'] # Actualizamos centro para la siguiente banda (arriba)
        widen = 0 # Reseteamos ampliación de búsqueda al encontrar línea

        # Visualización en modo Filas (Cajas)
        if draw_overlays and overlay is not None and ROW_MODE:
            cx_draw = results[i]['cx']
            pol = color_list[i]
            color = _row_color(i, pol)
            dyn_w = None
            if results[i]['w_used'] is not None and i == 4:
                dyn_w = int(_clamp(results[i]['w_used'] * 1.2, 30, max(40, (y2 - y1) * 0.9)))
            _draw_row_box(overlay, i, y1, y2, cx_draw, w, color,
                          box_w=dyn_w, filled=ROW_FILLED, alpha=ROW_FILL_ALPHA)

    # Información HUD General
    if draw_overlays and overlay is not None:
        cv2.line(overlay, (center_x, 0), (center_x, h - 1), (0, 255, 255), 1)
        status = " ".join([
            f"{i}:{'B' if color_list[i] == 'black' else ('W' if color_list[i] == 'white' else ('R' if color_list[i] == 'red' else ('Y' if color_list[i] == 'yellow' else '-')))}"
            for i in range(N_BANDS - 1, -1, -1)
        ])
        cv2.putText(overlay, f"lineAuto | {status}", (10, TEXT_Y),
                    cv2.FONT_HERSHEY_SIMPLEX, TEXT_FONT_SCALE, TEXT_COLOR, TEXT_THICKNESS, cv2.LINE_AA)

    # Compilación de resultados finales para el controlador
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
        'err': err_list[4], 'cx': cx_list[4], # Error principal basado en banda cercana
        'timestamp': time.time(), 'img_w': w, 'img_h': h,
        'color_list': color_list,
    }
    return state, (overlay if draw_overlays else None)


# ==========================
# Wrappers de Compatibilidad
# ==========================
def _run_line_with_profile(frame: np.ndarray, profile: dict, draw_overlays: bool = True):
    """
    Versión simplificada heredada para ejecutar detección con un solo perfil de color.
    Mantenida por compatibilidad hacia atrás con controladores antiguos.
    """
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
        y1 = int(h * Y_FRACS[i][0]); y2 = int(h * Y_FRACS[i][1])
        if not BAND_ENABLED[i]:
            results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
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

        if corridor_center is not None:
            dist = abs(cx_abs - corridor_center)
            if dist > MAX_BAND_JUMP:
                results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
                widen += WIDEN_STEP
                continue

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
