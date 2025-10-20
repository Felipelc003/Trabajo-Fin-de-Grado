# camera_opencv.py — Detección de línea con 5 ventanas (sliding windows) y publicación para control
import cv2
import numpy as np
import threading
import time
import imutils
from base_camera import BaseCamera
import RPIservo

# (opcional, si usas el modo de QR)
try:
    from pyzbar import pyzbar
    HAS_QR = True
except Exception:
    HAS_QR = False

# ----------------- Servos (por claridad) -----------------
SERVO_TILT = 0
SERVO_PAN = 1
SERVO_STEERING = 2  # servo de dirección (ruedas)

# Calibración dirección (si lo usas aquí; normalmente se maneja desde functions.py)
STEER_CENTER = 95
STEER_LEFT   = 130
STEER_RIGHT  = 60


class CVProcessor(threading.Thread):
    """
    Hilo que procesa frames:
      - findColor / watchDog / scanQR: se procesa bajo demanda (set_mode).
      - lineBlack: se dibuja inline desde draw_elements_on_frame para máxima fluidez.
    Publica estado de la línea (5 ventanas) mediante get_line_state().
    """

    def __init__(self):
        super(CVProcessor, self).__init__()
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # Estado
        self.mode = 'none'
        self.is_processing = False
        self.img_to_process = None
        self.drawing_elements = {}

        # Estado para seguidor de línea (publicado a Functions)
        self.line_state = {
            'has_line': False, 'err': None, 'cx': None, 'bbox': None,
            'timestamp': 0.0, 'y_band': (0, 0), 'img_w': 0, 'img_h': 0
        }
        self._line_lock = threading.Lock()
        self._line_event = threading.Event()
        self._line_seq = 0

        # Trackers para estabilizar corredores por ventana (0=top..4=bottom)
        self.sw_prev_cx = [None] * 5

        # Rango HSV inicial (si usas findColor)
        self.color_lower = np.array([24, 100, 100])
        self.color_upper = np.array([44, 255, 255])

        # Estado de pan/tilt (solo si usas findColor/QR)
        self.pan_angle  = 90.0
        self.tilt_angle = 90.0

        self.avg_background = None  # watch_dog
        self.last_qr_result = None
        self.qr_scanning = False
        self.qr_pan = 0
        self.qr_step = 5
        self.qr_reported = False

        # Sincronización del hilo
        self._flag = threading.Event()
        self._flag.clear()

        # Mostrar/ocultar overlays para rendimiento
        self.draw_overlays = True

        self.start()

    # ----------------- Publicación de estado de línea -----------------
    def _publish_line_state(self, st: dict):
        st = dict(st)
        st.setdefault('timestamp', time.time())
        with self._line_lock:
            self.line_state = st
            self._line_seq += 1
            self._line_event.set()

    def get_line_state(self, wait_new: bool = False, last_seq: int | None = None, timeout: float = 0.2):
        """
        Consumidor (Functions.py): devuelve (state_copy, seq).
        Si wait_new=True, espera a que haya seq > last_seq (con timeout).
        """
        if wait_new:
            deadline = time.time() + timeout
            while time.time() < deadline:
                with self._line_lock:
                    if last_seq is None or self._line_seq != last_seq:
                        return dict(self.line_state), self._line_seq
                self._line_event.wait(max(0.0, deadline - time.time()))
        with self._line_lock:
            return dict(self.line_state), self._line_seq

    # ----------------- Detección multi-ventana (5 recuadros) -----------------
    def _line_black_overlay(self, frame):
        """
        Detección de línea con columna de 5 ventanas apiladas (0=arriba..4=abajo),
        corredores centrados en la detección inferior inmediata, memoria por banda
        y ensanchamiento progresivo si una banda falla. Publica errs/cxs/bands.
        """
        h, w = frame.shape[:2]
        center_x = w // 2

        # --- RANGO HSV (negro del usuario) ---
        HSV_LOWER = np.array([60,   0,   0], dtype=np.uint8)
        HSV_UPPER = np.array([179, 255, 132], dtype=np.uint8)

        # OTSU por nivel (iluminación complicada arriba, menos abajo)
        USE_OTSU_LEVELS = [True, True, False, False, False]  # 0..4

        # Ventanas que cubren todo el alto (top->bottom)
        y_fracs = [
            (0.00, 0.20),  # 0 top
            (0.20, 0.40),  # 1
            (0.40, 0.60),  # 2
            (0.60, 0.80),  # 3
            (0.80, 1.00),  # 4 bottom (near)
        ]
        N = 5

        BAND_ENABLED = [False, True, True, True, True]  # ← desactiva SOLO la superior (índice 0)

        # Áreas mínimas (arriba más fino -> umbral menor)
        MIN_AREAS = [140, 160, 180, 200, 220]
        KS = [3, 3, 3, 3, 3]  # kernel morfológico por banda

        # Corredores (px a cada lado del centro esperado); bottom no usa corredor
        CORRIDOR_HALVES = [50, 45, 40, 35, None]

        # Estabilización y límites
        MAX_STEP_X = [30, 25, 22, 18, 9999]  # salto lateral máximo por banda (px)
        W_EMA      = 0.6                     # suavizado de memoria
        WIDEN_STEP = 20                      # ensanchar corredor si falla una banda
        MAX_WIDEN  = 60
        # Dibujo
        BORDER = 2
        COLORS = [
            (0, 0, 255),     # 0 top - rojo
            (0, 165, 255),   # 1 naranja
            (0, 255, 255),   # 2 cian/amarillo
            (0, 255, 165),   # 3 verde claro
            (0, 255, 0),     # 4 near - verde
        ]

        # Forzar la banda bottom (near) a rectángulo vertical estable
        FORCE_NEAR_VERTICAL = True
        NEAR_LEN   = 140   # altura del rect en banda 4
        NEAR_MIN_W = 10
        NEAR_MAX_W = 70

        # Precalcular HSV y GRAY
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
            """Detecta centro X en banda idx. Devuelve found,cx_abs,cy_abs,y1,y2,box,used_w"""
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
                rect = cv2.minAreaRect(best)  # ((cx,cy),(rw,rh),ang) en coords ROI
                (cx_r, cy_r), (rw, rh), ang = rect

                if FORCE_NEAR_VERTICAL and idx == 4:
                    # forzar rect vertical con ancho acotado y altura fija
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

                # A coordenadas globales
                box[:, 0] += x_off
                box[:, 1] += y1
                if self.draw_overlays:
                    cv2.drawContours(frame, [box], 0, color, BORDER)

                cx_abs = int(cx_r) + x_off
                cy_abs = int(cy_r) + y1
                if self.draw_overlays:
                    cv2.circle(frame, (cx_abs, cy_abs), 3, color, -1)

            # Marco de banda y corredor
            if self.draw_overlays:
                cv2.rectangle(frame, (0, y1), (w - 1, y2 - 1), color, 1)
                if corridor_center is not None and corridor_half is not None:
                    xL = max(0, int(corridor_center - corridor_half))
                    xR = min(w - 1, int(corridor_center + corridor_half))
                    cv2.line(frame, (xL, y1), (xL, y2 - 1), (0, 255, 255), 1)
                    cv2.line(frame, (xR, y1), (xR, y2 - 1), (0, 255, 255), 1)

            return (cx_abs is not None), cx_abs, cy_abs, y1, y2, box, used_w

        # Ejecutar bottom->top para centrar corredores
        results = [None] * N
        corridor_center = None
        widen = 0
        order = [4, 3, 2, 1, 0]

        for i in order:
            # Si la banda está deshabilitada, publica "no hay" y sigue
            if not BAND_ENABLED[i]:
                results[i] = {
                    'found': False, 'cx': None, 'cy': None,
                    'band': (int(h*y_fracs[i][0]), int(h*y_fracs[i][1])),
                    'w_used': None
                }
            # OJO: no modificamos 'corridor_center' ni 'widen' aquí
                continue

            # Centro esperado: el de la inferior, si existe; si no, memoria propia
            expected = corridor_center if corridor_center is not None else self.sw_prev_cx[i]

            half = CORRIDOR_HALVES[i]
            half_eff = None
            if half is not None:
                half_eff = half + widen

            found, cx, cy, y1, y2, box, used_w = detect_one_band(
                i,
                corridor_center=expected if half_eff is not None else None,
                corridor_half=half_eff
            )

            if not found:
                results[i] = {'found': False, 'cx': None, 'cy': None, 'band': (y1, y2), 'w_used': None}
                widen += WIDEN_STEP
                continue

            # Limitar salto lateral respecto a memoria
            if self.sw_prev_cx[i] is not None and cx is not None:
                dx = cx - self.sw_prev_cx[i]
                max_dx = MAX_STEP_X[i]
                if abs(dx) > max_dx:
                    cx = int(self.sw_prev_cx[i] + max_dx * (1 if dx > 0 else -1))

            # EMA para estabilizar memoria
            if self.sw_prev_cx[i] is None:
                self.sw_prev_cx[i] = float(cx)
            else:
                self.sw_prev_cx[i] = (1.0 - W_EMA) * float(self.sw_prev_cx[i]) + W_EMA * float(cx)

            results[i] = {'found': True, 'cx': int(self.sw_prev_cx[i]), 'cy': cy, 'band': (y1, y2), 'w_used': used_w}

            corridor_center = results[i]['cx']  # guía para la banda superior
            widen = 0  # al encontrar, resetea ensanchamiento

        # Línea vertical central (guía)
        if self.draw_overlays:
            cv2.line(frame, (center_x, 0), (center_x, h - 1), (0, 255, 255), 1)

        # Listas de 5 elementos (0..4)
        has_list = [r['found'] for r in results]
        cx_list  = [r['cx']    for r in results]
        bands    = [r['band']  for r in results]
        center_x = w // 2
        err_list = [ (center_x - cx) if cx is not None else None for cx in cx_list ]

        # near/mid conservan 4 y 2; far pasa a ser la 1 (top desactivada)
        has_near, err_near, cx_near = has_list[4], err_list[4], cx_list[4]
        has_mid,  err_mid,  cx_mid  = has_list[2], err_list[2], cx_list[2]
        has_far,  err_far,  cx_far  = has_list[1], err_list[1], cx_list[1]   # <—— AQUÍ

        state = {
            'has_list': has_list,
            'cxs': cx_list,
            'errs': err_list,
            'bands': bands,

            'has_near': has_near, 'err_near': err_near, 'cx_near': cx_near, 'band_near': bands[4],
            'has_mid':  has_mid,  'err_mid':  err_mid,  'cx_mid':  cx_mid,  'band_mid':  bands[2],
            'has_far':  has_far,  'err_far':  err_far,  'cx_far':  cx_far,  'band_far':  bands[1],  # <—— AQUÍ

            'has_line': any(has_list),
            'err': err_near, 'cx': cx_near,   # compat con lo que ya usabas
            'timestamp': time.time(), 'img_w': w, 'img_h': h
        }
        self._publish_line_state(state)

        # Etiqueta de estado
        if self.draw_overlays:
            status = " ".join([f"{i}:{'OK' if has_list[i] else '--'}" for i in range(N-1, -1, -1)])
            top_text_y = max(20, bands[0][0] - 10)  # ← usa 'bands'
            cv2.putText(frame, f"            lineBlack | {status}", (10, top_text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (50, 220, 50), 2, cv2.LINE_AA)

        return frame

    # ----------------- Otros modos opcionales -----------------
    def find_color(self, frame):
        KP = 0.1
        MIN_AREA = 500
        (h, w) = frame.shape[:2]
        center_x, center_y = w // 2, h // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        self.drawing_elements = {}
        target_found = False

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_AREA:
                target_found = True
                (x, y, w, h) = cv2.boundingRect(c)
                obj_center_x = x + w // 2
                obj_center_y = y + h // 2

                if self.draw_overlays:
                    self.drawing_elements['rect'] = (x, y, x + w, y + h)
                    self.drawing_elements['text'] = 'Target Locked'

                error_x = center_x - obj_center_x
                error_y = center_y - obj_center_y

                self.pan_angle += error_x * KP
                self.tilt_angle += error_y * KP

                self.pan_angle = max(0, min(180, self.pan_angle))
                self.tilt_angle = max(0, min(180, self.tilt_angle))

                RPIservo.move(SERVO_PAN, int(self.pan_angle))
                RPIservo.move(SERVO_TILT, int(self.tilt_angle))

        if not target_found and self.draw_overlays:
            self.drawing_elements['text'] = 'Target Detecting'

    def watch_dog(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        if self.avg_background is None:
            self.avg_background = gray.copy().astype("float")
            return
        cv2.accumulateWeighted(gray, self.avg_background, 0.5)
        frame_delta = cv2.absdiff(gray, cv2.convertScaleAbs(self.avg_background))
        thresh = cv2.threshold(frame_delta, 5, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        self.drawing_elements.pop('motion_rect', None)
        for c in cnts:
            if cv2.contourArea(c) < 5000:
                continue
            (x, y, w, h) = cv2.boundingRect(c)
            if self.draw_overlays:
                self.drawing_elements['motion_rect'] = (x, y, x + w, y + h)

    def scan_qr(self, frame):
        if not HAS_QR:
            return
        if not self.qr_scanning:
            self.qr_scanning = True
            self.qr_pan = 0
            self.qr_step = 5
            self.qr_reported = False
            self.last_qr_result = None
            self.drawing_elements = {}
            try:
                RPIservo.move(SERVO_TILT, 90)
                RPIservo.move(SERVO_PAN, 0)
            except Exception:
                pass

        self.drawing_elements = {}

        barcodes = pyzbar.decode(frame) if HAS_QR else []
        if barcodes:
            barcode = max(barcodes, key=lambda b: b.rect[2] * b.rect[3])
            data = barcode.data.decode("utf-8")
            self.last_qr_result = data

            (x, y, w, h) = barcode.rect
            if self.draw_overlays:
                self.drawing_elements['qr_rect'] = (x, y, x + w, y + h)
                self.drawing_elements['qr_text'] = data

            if not self.qr_reported:
                try:
                    RPIservo.move(SERVO_PAN, 90)
                except Exception:
                    pass
                self.qr_reported = True

            # Fin del modo
            self.qr_scanning = False
            self.mode = 'none'
            try:
                Camera.get_instance().modeSelect = 'none'
            except Exception:
                pass
            return

        # Avanzar PAN si no hay QR
        if self.qr_pan <= 180:
            try:
                RPIservo.move(SERVO_PAN, int(self.qr_pan))
            except Exception:
                pass
            self.qr_pan += self.qr_step
        else:
            # Fin sin QR
            self.qr_scanning = False
            self.mode = 'none'
            try:
                Camera.get_instance().modeSelect = 'none'
            except Exception:
                pass
            try:
                RPIservo.move(SERVO_PAN, 90)
            except Exception:
                pass

    # ----------------- Render de overlays en el frame actual -----------------
    def draw_elements_on_frame(self, frame):
        if self.mode == 'findColor':
            if 'text' in self.drawing_elements and self.draw_overlays:
                cv2.putText(frame, self.drawing_elements['text'], (40, 60),
                            self.font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            if 'rect' in self.drawing_elements and self.draw_overlays:
                x1, y1, x2, y2 = self.drawing_elements['rect']
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        elif self.mode == 'lineBlack':
            frame = self._line_black_overlay(frame)

        elif self.mode == 'watchDog':
            if 'motion_rect' in self.drawing_elements and self.draw_overlays:
                x1, y1, x2, y2 = self.drawing_elements['motion_rect']
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 2)

        elif self.mode == 'scanQR' and HAS_QR:
            if 'qr_rect' in self.drawing_elements and self.draw_overlays:
                x1, y1, x2, y2 = self.drawing_elements['qr_rect']
                text = self.drawing_elements.get('qr_text', '')
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, text, (x1, y1 - 10),
                            self.font, 0.7, (0, 255, 0), 2)
        return frame

    # ----------------- Hilo run: sólo para modos por demanda -----------------
    def run(self):
        while True:
            self._flag.wait()
            if self.mode == 'none':
                self.is_processing = False
                self._flag.clear()
                continue

            self.is_processing = True
            if self.img_to_process is not None:
                if self.mode == 'findColor':
                    self.find_color(self.img_to_process)
                elif self.mode == 'watchDog':
                    self.watch_dog(self.img_to_process)
                elif self.mode == 'scanQR' and HAS_QR:
                    self.scan_qr(self.img_to_process)
            self.is_processing = False
            self._flag.clear()

    def pause(self):
        self.pan_angle = 90
        self.tilt_angle = 90
        self.mode = 'none'
        self.drawing_elements = {}
        self._flag.clear()

    def resume(self):
        self._flag.set()

    def set_mode(self, new_mode, image):
        if new_mode != self.mode:
            self.last_qr_result = None
        self.mode = new_mode
        self.img_to_process = image
        self.resume()


class Camera(BaseCamera):
    """
    Singleton que provee frames JPEG y gestiona el hilo CVProcessor.
    """
    _instance = None
    _picam2 = None
    _picam2_lock = threading.Lock()

    def __init__(self):
        if Camera._instance is not None:
            raise RuntimeError("Camera is a singleton, use get_instance()")

        Camera._instance = self
        self.modeSelect = 'none'
        self.cv_thread = CVProcessor()
        super(Camera, self).__init__()

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            print("Creando la instancia del objeto Camera por primera vez...")
            cls._instance = Camera()
        return cls._instance

    def modeselect(self, mode: str):
        """
        Cambiar de modo sin reabrir cámara. Ej: Camera.get_instance().modeselect('lineBlack')
        """
        print(f"[Camera] modeselect -> {mode}")
        self.modeSelect = mode

        if self.modeSelect in ('findColor', 'watchDog', 'scanQR'):
            self.cv_thread.set_mode(self.modeSelect, None)
        elif self.modeSelect == 'lineBlack':
            # Importante: fija modo directo para que draw_elements_on_frame entre en 'lineBlack'
            self.cv_thread.mode = 'lineBlack'
            self.cv_thread.img_to_process = None
        elif self.modeSelect == 'none':
            self.cv_thread.pause()

    def start_background_feed(self):
        """
        Consumidor en segundo plano que itera frames() (opcional).
        """
        if getattr(self, "_bg_thread", None):
            return
        def _pump():
            try:
                for _ in self.frames():
                    time.sleep(0.01)
            except Exception as e:
                print(f"[Camera] background feed stopped: {e}")
        t = threading.Thread(target=_pump, daemon=True)
        t.start()
        self._bg_thread = t
        print("[Camera] background feed started")

    def colorFindSet(self, invarH, invarS, invarV):
        """
        Definición de ventana HSV (para findColor). No afecta lineBlack.
        """
        H_LOWER = max(invarH - 10, 0)
        H_UPPER = min(invarH + 10, 179)
        S_LOWER = 100; S_UPPER = 255
        V_LOWER = 80;  V_UPPER = 255
        self.cv_thread.color_lower = np.array([H_LOWER, S_LOWER, V_LOWER])
        self.cv_thread.color_upper = np.array([H_UPPER, S_UPPER, V_UPPER])
        print("--- Rango HSV (findColor) ---")
        print("LOWER:", self.cv_thread.color_lower, "UPPER:", self.cv_thread.color_upper)

    @staticmethod
    def frames():
        # Esperar singleton
        while Camera._instance is None:
            time.sleep(0.01)
        cam_instance = Camera._instance

        # Inicializar Picamera2 una vez
        try:
            from picamera2 import Picamera2
            if Camera._picam2 is None:
                print("Inicializando hardware de Picamera2 (singleton).")
                _p2 = Picamera2()
                # Puedes bajar a (480, 360) si necesitas más FPS
                config = _p2.create_preview_configuration(main={"size": (640, 480)}, controls={"FrameRate": 60.0}, buffer_count=4)
                _p2.configure(config)
                _p2.set_controls({"FrameRate": 60.0})
                _p2.start()
                Camera._picam2 = _p2
                print("Picamera2 inicializada correctamente (singleton).")
            picam2 = Camera._picam2
        except Exception as e:
            print(f"Error al iniciar Picamera2: {e}")
            error_img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(error_img, "Camera Error", (180, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            while True:
                yield cv2.imencode('.jpg', error_img)[1].tobytes()

        # --- Estado para FPS ---
        fps_prev_t = time.time()
        fps_ema = 0.0  # media exponencial para que no salte


        # Bucle de captura y render
        while True:
            try:
                img_rgb = picam2.capture_array("main")
            except Exception as e:
                print(f"[frames] capture_array() fallo: {e}")
                black = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(black, "Capture Error", (160, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                yield cv2.imencode('.jpg', black)[1].tobytes()
                continue

            img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

            # Procesar por demanda (findColor/watchDog/scanQR)
            try:
                if (cam_instance.modeSelect in ('findColor', 'watchDog', 'scanQR')
                        and not cam_instance.cv_thread.is_processing):
                    cam_instance.cv_thread.set_mode(cam_instance.modeSelect, img)
            except Exception:
                pass

            # Dibujo de overlays — lineBlack se hace inline aquí
            try:
                img = cam_instance.cv_thread.draw_elements_on_frame(img)
            except Exception:
                pass

            # --- Calcular y dibujar FPS (siempre visible) ---
            now = time.time()
            dt = max(1e-6, now - fps_prev_t)
            inst_fps = 1.0 / dt
            fps_ema = inst_fps if fps_ema == 0.0 else (0.9 * fps_ema + 0.1 * inst_fps)
            fps_prev_t = now

            # color simple por rango de FPS (opcional)
            color = (40, 255, 40) if fps_ema >= 15 else ((40, 220, 220) if fps_ema >= 8 else (0, 0, 255))
            cv2.putText(img, f"FPS: {fps_ema:.1f}", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)


            # JPEG (baja calidad si quieres más FPS: calidad 60)
            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
            if not ok:
                buf = cv2.imencode('.jpg', np.zeros_like(img))[1]
            yield buf.tobytes()
