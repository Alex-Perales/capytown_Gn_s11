#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════╗
║    CapyTown — Semana 11 · RC-2 · "Las 3 Vueltas del Jirón"            ║
║    Lane Following: Visión HSV + IPM + Control PID                      ║
║    Universidad ESAN · Robótica 2026-I · Grupo 4                        ║
╚══════════════════════════════════════════════════════════════════════════╝

ARCHIVO ÚNICO que incluye:
  - Todas las variables configurables explicadas al inicio
  - Calibrador HSV interactivo (modo --calibrar)
  - LaneDetector: detecta líneas y calcula error lateral en metros
  - LaneController: PID que convierte el error en velocidad del robot
  - Contador de vueltas por odometría

MODOS DE USO:
  Normal   : ros2 run capytown_esan_pkg lane_node
  Calibrar : python3 lane_node.py --calibrar --source 0
"""

# ═══════════════════════════════════════════════════════════════════════════════
#  SECCIÓN 1 — VARIABLES CONFIGURABLES
#  Modifica estos valores para ajustar el comportamiento del robot.
#  Cada variable tiene comentarios de qué pasa si subes (+) o bajas (-).
# ═══════════════════════════════════════════════════════════════════════════════

# ── MISIÓN ────────────────────────────────────────────────────────────────────

NUM_VUELTAS = 3
# Número de vueltas autónomas que el robot debe completar antes de detenerse.
# (+) Más vueltas → el robot corre por más tiempo antes de frenar.
# (-) Menos vueltas → el robot se detiene antes. Mínimo recomendado: 3 (RC-2).

METROS_POR_VUELTA = 6.4
# Perímetro aproximado de UNA vuelta al Escenario A (m).
# Escenario A = 2.0 × 2.0 m. Contorno ≈ 6.4 m.
# (+) Subir → el robot espera recorrer más antes de contar la vuelta.
#     Usar si cuenta vueltas antes de terminar el circuito.
# (-) Bajar → cuenta las vueltas antes de terminar.

# ── VELOCIDAD ─────────────────────────────────────────────────────────────────

LINEAR_SPEED = 0.20
# Velocidad lineal de crucero en metros por segundo (m/s).
# Mínimo aceptable para RC-2: 0.15 m/s.
# (+) Más rápido → completa las vueltas en menos tiempo (mejor tiempo Grand Prix),
#     pero el PID tiene menos tiempo para corregir → puede salirse en curvas.
# (-) Más lento → más fácil de controlar, más estable en curvas, pero más lento.

MAX_ANGULAR = 2.0
# Velocidad angular máxima en radianes por segundo (rad/s). Límite de giro.
# (+) Permite giros más bruscos → útil en curvas cerradas, pero puede oscilar.
# (-) Giros más suaves → trayectoria más fluida, pero puede no girar
#     lo suficiente en curvas cerradas y salirse.

# ── CONTROL PID ───────────────────────────────────────────────────────────────
# El PID calcula cuánto debe girar el robot según el error lateral.
# Fórmula: omega = Kp*e + Ki*integral(e) + Kd*de/dt
# REGLA DE ORO: ajusta UN parámetro a la vez. Primero Kp, luego Kd, al final Ki.

KP = 2.5
# Ganancia Proporcional — reacciona al error ACTUAL.
# (+) Reacciona más fuerte → corrige más rápido, pero si es muy alto
#     el robot zigzaguea (oscila) porque sobre-corrige constantemente.
# (-) Reacciona más suave → el robot va más recto pero tarda en corregir
#     las curvas y puede salirse del carril lentamente.
# Rango típico: 1.0 – 5.0.

KI = 0.0
# Ganancia Integral — corrige el error acumulado en el tiempo.
# Útil si el robot siempre se va hacia el mismo lado en las rectas.
# (+) Corrige más el sesgo sostenido, pero si es muy alto
#     causa sobreoscilación y "wind-up" (sobre-corrige en curvas).
# (-) No corrige el sesgo. Dejar en 0.0 mientras Kp y Kd no estén ajustados.
# Si se activa, usar valores pequeños: 0.01 – 0.1.

KD = 0.3
# Ganancia Derivativa — amortigua el cambio del error (anticipa la corrección).
# (+) Amortigua más las oscilaciones de Kp alto → reduce el zigzag.
#     Si es muy alto amplifica el ruido de la cámara → movimientos bruscos.
# (-) Menos amortiguación → el robot puede oscilar más en rectas largas.
# Rango típico: 0.1 – 1.0.

INTEGRAL_LIMIT = 0.5
# Límite anti-windup del término integral (metros). Evita que Ki "explote".
# (+) Permite mayor acumulación → más corrección de sesgo,
#     pero más riesgo de sobreoscilación si Ki > 0.
# (-) Limita más el integral → más seguro, pero Ki tiene menos efecto.

ERROR_TIMEOUT = 0.5
# Segundos sin recibir /lane_error antes de frenar por seguridad.
# (+) Más tolerante a pérdidas momentáneas de detección → no frena tan fácil.
#     Riesgo: si se sale del carril, tarda más en frenar.
# (-) Frena más rápido si pierde las líneas → más seguro, pero puede frenar
#     en curvas donde momentáneamente solo ve una línea.

CONTROL_RATE = 30.0
# Frecuencia del lazo de control en Hz (veces por segundo que calcula el PID).
# (+) Más frecuente → reacciona más rápido, más preciso, pero usa más CPU.
# (-) Menos frecuente → reacciona más lento, puede perder detalles a alta velocidad.

# ── VISIÓN — COLORES HSV ──────────────────────────────────────────────────────
# HSV = Tono (H: 0-180), Saturación (S: 0-255), Valor/Brillo (V: 0-255).
# IMPORTANTE: recalibrar con modo --calibrar al inicio de cada sesión de lab.

# Blanco — borde derecho del carril
WHITE_H_MIN = 0
WHITE_H_MAX = 180
# H del blanco no importa (puede ser cualquier tono) → dejar en 0-180.

WHITE_S_MIN = 0
WHITE_S_MAX = 30
# Saturación del blanco es MUY baja (color casi sin tono).
# (+) Subir S_max → detecta grises claros. Riesgo: piso gris = "blanco".
# (-) Bajar S_max → solo blanco puro. Riesgo: línea no detectada (NaN).

WHITE_V_MIN = 180
WHITE_V_MAX = 255
# Brillo del blanco es MUY alto.
# (+) Subir V_min → solo superficies muy brillantes. Más selectivo.
# (-) Bajar V_min → detecta blancos sombreados. Riesgo: confundir piso gris.

# Amarillo — eje central discontinuo
YELLOW_H_MIN = 20
YELLOW_H_MAX = 35
# Tono del amarillo en OpenCV: aprox 20-35 (escala 0-180).
# (+) Ampliar rango → más tolerante a variaciones de luz.
#     Riesgo: captura naranja (H<20) o verde-amarillento (H>40).
# (-) Estrechar rango → más selectivo, menos falsos positivos.

YELLOW_S_MIN = 100
YELLOW_S_MAX = 255
# Saturación del amarillo es alta (color vivo).
# (+) Subir S_min → exige amarillo más puro. Riesgo: línea desaparece si luz lava el color.
# (-) Bajar S_min → detecta amarillos más pálidos. Riesgo: captura piso amarillento.

YELLOW_V_MIN = 100
YELLOW_V_MAX = 255
# Brillo del amarillo.
# (+) Subir V_min → exige más brillo. Puede perder línea en sombras.
# (-) Bajar V_min → detecta amarillos más oscuros. Riesgo: confundir con piso.

# ── VISIÓN — GEOMETRÍA / IPM ──────────────────────────────────────────────────

MIN_AREA = 150
# Área mínima en píxeles para aceptar una detección como real (filtro de ruido).
# (+) Ignorar manchas más grandes → menos falsos positivos, pero puede ignorar
#     la línea cuando está lejos o se ve delgada.
# (-) Acepta manchas más pequeñas → detecta líneas lejanas, pero más
#     sensible a reflejos y ruido → saltos en /lane_error.

LANE_WIDTH_M = 0.21
# Ancho del carril CapyTown en metros (especificación oficial del tile).
# (+) Subir → asume carril más ancho → estima el centro más alejado.
# (-) Bajar → estimaciones más conservadoras del centro del carril.

PX_PER_METER = 600.0
# Escala de la vista IPM: píxeles por metro en la imagen de pájaro.
# (+) Subir → errores laterales más pequeños → reacciona menos.
# (-) Bajar → errores laterales más grandes → reacciona más fuerte.

LOOK_AHEAD_ROW = 0.6
# Fila de la imagen IPM donde se mide el error: 0.0=arriba (lejos), 1.0=abajo (cerca).
# (+) Subir → más reactivo pero puede oscilar en rectas largas.
# (-) Bajar → anticipa más las curvas → trayectoria más suave pero reacciona más tarde.

# ── CÁMARA / IPM — TRAPECIO DE PERSPECTIVA ────────────────────────────────────

IPM_TOP_Y = 0.50
# Altura del borde SUPERIOR del trapecio (fracción de h).
# (+) Subir → captura menos piso, cámara más horizontal.
# (-) Bajar → captura más piso. Ideal para cámara inclinada hacia abajo.
# Original del enunciado: 0.62. Ajustado para cámara más inclinada: 0.50.

IPM_TOP_LEFT_X  = 0.18
IPM_TOP_RIGHT_X = 0.82
# Ancho del borde superior del trapecio.
# (+) Acercar los puntos → trapecio más estrecho arriba.
# (-) Alejar los puntos → captura más campo lateral.

IPM_BOTTOM_Y = 0.98
# Altura del borde INFERIOR del trapecio. Normalmente cerca del fondo (0.98).


# ═══════════════════════════════════════════════════════════════════════════════
#  SECCIÓN 2 — IMPORTS
# ═══════════════════════════════════════════════════════════════════════════════

import argparse
import math
import sys
import time

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


# ═══════════════════════════════════════════════════════════════════════════════
#  SECCIÓN 3 — FUNCIONES COMPARTIDAS
# ═══════════════════════════════════════════════════════════════════════════════

def build_ipm(w, h):
    """
    Construye la matriz de homografía para la IPM (vista de pájaro).

    Convierte la vista frontal de la cámara a una vista desde arriba donde
    las distancias en píxeles son proporcionales a distancias reales en el piso.
    """
    src = np.float32([
        [IPM_TOP_LEFT_X  * w, IPM_TOP_Y    * h],
        [IPM_TOP_RIGHT_X * w, IPM_TOP_Y    * h],
        [1.00            * w, IPM_BOTTOM_Y * h],
        [0.00            * w, IPM_BOTTOM_Y * h],
    ])
    dst = np.float32([
        [0.30 * w, 0.0],
        [0.70 * w, 0.0],
        [0.70 * w, h  ],
        [0.30 * w, h  ],
    ])
    return cv2.getPerspectiveTransform(src, dst)


def centroid_x(mask, min_area):
    """
    Calcula la coordenada X del centroide de una máscara binaria.
    Retorna None si el área es menor que min_area (se descarta como ruido).
    """
    m = cv2.moments(mask, binaryImage=True)
    if m['m00'] < max(min_area, 1e-3):
        return None
    return m['m10'] / m['m00']


def apply_hsv_masks(frame, white_lo, white_hi, yellow_lo, yellow_hi, M):
    """
    Aplica la IPM y genera las máscaras de color para blanco y amarillo.

    Pasos:
      1. Transforma la imagen a vista de pájaro (IPM).
      2. Convierte de BGR a HSV.
      3. Aplica umbrales de color para blanco y amarillo.
      4. Limpia el ruido con morfología MORPH_OPEN.
    """
    h, w = frame.shape[:2]
    warp   = cv2.warpPerspective(frame, M, (w, h))
    hsv    = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), np.uint8)
    mask_w = cv2.morphologyEx(cv2.inRange(hsv, white_lo,  white_hi),  cv2.MORPH_OPEN, kernel)
    mask_y = cv2.morphologyEx(cv2.inRange(hsv, yellow_lo, yellow_hi), cv2.MORPH_OPEN, kernel)
    return warp, mask_w, mask_y


def compute_lane_error(mask_white, mask_yellow, w, h):
    """
    Calcula el error lateral en metros respecto al centro del carril.

    Casos:
      Ambas líneas → centro = promedio entre blanca y amarilla.
      Solo amarillo → centro = amarillo + media calzada a la derecha.
      Solo blanco   → centro = blanco - media calzada a la izquierda.
      Ninguna       → NaN → el controlador frena por seguridad.

    Convención de signo:
      error > 0 → centro a la DERECHA → girar derecha (omega < 0).
      error < 0 → centro a la IZQUIERDA → girar izquierda (omega > 0).
    """
    row  = int(LOOK_AHEAD_ROW * h)
    band = slice(max(0, row - 8), min(h, row + 8))

    x_white  = centroid_x(mask_white[band,  :], MIN_AREA)
    x_yellow = centroid_x(mask_yellow[band, :], MIN_AREA)

    half_px = (LANE_WIDTH_M / 2.0) * PX_PER_METER

    if x_white is not None and x_yellow is not None:
        center_px = (x_white + x_yellow) / 2.0
    elif x_yellow is not None:
        center_px = x_yellow + half_px
    elif x_white is not None:
        center_px = x_white - half_px
    else:
        center_px = None

    if center_px is None:
        error_m = float('nan')
    else:
        error_m = (center_px - w / 2.0) / PX_PER_METER

    return error_m, x_white, x_yellow, center_px, row


# ═══════════════════════════════════════════════════════════════════════════════
#  SECCIÓN 4 — CALIBRADOR HSV INTERACTIVO
#  Uso: python3 lane_node.py --calibrar --source 0
#  Abre ventanas con sliders para ajustar los valores HSV en tiempo real.
#  Presiona 's' para guardar el YAML, 'q' para salir.
# ═══════════════════════════════════════════════════════════════════════════════

def _nothing(_):
    pass


def _make_trackbars(window, h_min, h_max, s_min, s_max, v_min, v_max):
    """Crea una ventana con 6 sliders HSV."""
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.createTrackbar('H min', window, h_min, 180, _nothing)
    cv2.createTrackbar('H max', window, h_max, 180, _nothing)
    cv2.createTrackbar('S min', window, s_min, 255, _nothing)
    cv2.createTrackbar('S max', window, s_max, 255, _nothing)
    cv2.createTrackbar('V min', window, v_min, 255, _nothing)
    cv2.createTrackbar('V max', window, v_max, 255, _nothing)


def _read_trackbars(window):
    """Lee los 6 valores HSV actuales de los sliders."""
    return tuple(cv2.getTrackbarPos(k, window)
                 for k in ('H min', 'H max', 'S min', 'S max', 'V min', 'V max'))


def _yaml_block(white, yellow):
    """Genera el bloque YAML listo para copiar a hsv_params.yaml."""
    return f"""lane_detector:
  ros__parameters:
    white_h_min: {white[0]}
    white_h_max: {white[1]}
    white_s_min: {white[2]}
    white_s_max: {white[3]}
    white_v_min: {white[4]}
    white_v_max: {white[5]}
    yellow_h_min: {yellow[0]}
    yellow_h_max: {yellow[1]}
    yellow_s_min: {yellow[2]}
    yellow_s_max: {yellow[3]}
    yellow_v_min: {yellow[4]}
    yellow_v_max: {yellow[5]}
    min_area: {MIN_AREA}
    lane_width_m: {LANE_WIDTH_M}
    px_per_meter: {PX_PER_METER}
    look_ahead_row: {LOOK_AHEAD_ROW}
    publish_debug: true
"""


def run_calibrator(source):
    """
    Calibrador HSV interactivo con sliders en tiempo real.

    Abre la cámara, aplica la IPM y muestra:
      - Vista de pájaro con centroides W (blanco) e Y (amarillo) marcados.
      - Máscara del blanco.
      - Máscara del amarillo.

    Cada segundo imprime en consola el YAML con los valores actuales.

    Controles:
      q / ESC : salir e imprimir el YAML final.
      s       : guardar hsv_params_calibrado.yaml.
    """
    cap = None
    static_img = None
    if source.isdigit():
        cap = cv2.VideoCapture(int(source))
    else:
        img = cv2.imread(source)
        if img is not None:
            static_img = img
        else:
            cap = cv2.VideoCapture(source)

    if static_img is None and (cap is None or not cap.isOpened()):
        raise SystemExit(f'No se pudo abrir la fuente: {source}')

    _make_trackbars('Blanco (borde)',
                    WHITE_H_MIN, WHITE_H_MAX, WHITE_S_MIN,
                    WHITE_S_MAX, WHITE_V_MIN, WHITE_V_MAX)
    _make_trackbars('Amarillo (eje)',
                    YELLOW_H_MIN, YELLOW_H_MAX, YELLOW_S_MIN,
                    YELLOW_S_MAX, YELLOW_V_MIN, YELLOW_V_MAX)

    M = None
    last_print = 0.0
    white = yellow = None

    print('\n[CALIBRADOR] Ajusta los sliders hasta que las líneas se vean claras.')
    print('[CALIBRADOR] Presiona "s" para guardar el YAML, "q" para salir.\n')

    while True:
        frame = static_img.copy() if static_img is not None else None
        if frame is None:
            ok, frame = cap.read()
            if not ok:
                break

        frame = cv2.resize(frame, (640, 480))
        h, w = frame.shape[:2]
        if M is None:
            M = build_ipm(w, h)

        white  = _read_trackbars('Blanco (borde)')
        yellow = _read_trackbars('Amarillo (eje)')

        white_lo  = np.array([white[0],  white[2],  white[4]])
        white_hi  = np.array([white[1],  white[3],  white[5]])
        yellow_lo = np.array([yellow[0], yellow[2], yellow[4]])
        yellow_hi = np.array([yellow[1], yellow[3], yellow[5]])

        warp, mask_w, mask_y = apply_hsv_masks(
            frame, white_lo, white_hi, yellow_lo, yellow_hi, M)

        row  = int(LOOK_AHEAD_ROW * h)
        band = slice(max(0, row - 8), min(h, row + 8))
        xw = centroid_x(mask_w[band, :], MIN_AREA)
        xy = centroid_x(mask_y[band, :], MIN_AREA)

        view = warp.copy()
        cv2.line(view, (0, row), (w, row), (0, 255, 0), 1)
        for x, color, label in [(xw, (255,255,255), 'W'), (xy, (0,255,255), 'Y')]:
            if x is not None:
                cv2.circle(view, (int(x), row), 6, color, -1)
                cv2.putText(view, label, (int(x)-5, row-12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow('IPM + centroides (W=blanco Y=amarillo)', view)
        cv2.imshow('Mascara blanco',   mask_w)
        cv2.imshow('Mascara amarillo', mask_y)

        if time.time() - last_print > 1.0:
            print('--- Valores actuales ---')
            print(_yaml_block(white, yellow))
            last_print = time.time()

        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('s'):
            with open('hsv_params_calibrado.yaml', 'w') as f:
                f.write(_yaml_block(white, yellow))
            print('[CALIBRADOR] Guardado en hsv_params_calibrado.yaml')

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()

    if white and yellow:
        print('\n=== YAML FINAL (copiar a config/hsv_params.yaml) ===')
        print(_yaml_block(white, yellow))


# ═══════════════════════════════════════════════════════════════════════════════
#  SECCIÓN 5 — NODO DETECTOR DE CARRIL (ROS2)
#  Suscribe: /camera/image_raw
#  Publica:  /lane_error        → error lateral en metros
#            /lane/debug_image  → imagen de pájaro con líneas marcadas
# ═══════════════════════════════════════════════════════════════════════════════

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.bridge = CvBridge()

        self.white_lo  = np.array([WHITE_H_MIN,  WHITE_S_MIN,  WHITE_V_MIN])
        self.white_hi  = np.array([WHITE_H_MAX,  WHITE_S_MAX,  WHITE_V_MAX])
        self.yellow_lo = np.array([YELLOW_H_MIN, YELLOW_S_MIN, YELLOW_V_MIN])
        self.yellow_hi = np.array([YELLOW_H_MAX, YELLOW_S_MAX, YELLOW_V_MAX])

        self.M = None

        self.sub     = self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.pub_err = self.create_publisher(Float32, '/lane_error', 10)
        self.pub_dbg = self.create_publisher(Image,   '/lane/debug_image', 10)
        self.get_logger().info('lane_detector listo.')

    def on_image(self, msg):
        """
        Callback por cada frame de la cámara.

        1. Convierte mensaje ROS a imagen OpenCV.
        2. Construye la IPM en el primer frame.
        3. Aplica IPM y máscaras HSV.
        4. Calcula el error lateral.
        5. Publica error e imagen debug.
        """
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w  = frame.shape[:2]

        if self.M is None:
            self.M = build_ipm(w, h)

        warp, mask_w, mask_y = apply_hsv_masks(
            frame, self.white_lo, self.white_hi,
            self.yellow_lo, self.yellow_hi, self.M)

        error_m, x_white, x_yellow, center_px, row = compute_lane_error(
            mask_w, mask_y, w, h)

        out      = Float32()
        out.data = float(error_m)
        self.pub_err.publish(out)

        self._publish_debug(warp, row, x_white, x_yellow, center_px, msg)

    def _publish_debug(self, warp, row, xw, xy, xc, header_msg):
        """
        Publica imagen de pájaro en /lane/debug_image con:
          Línea verde  = fila de muestreo.
          Punto blanco = centroide borde blanco.
          Punto cian   = centroide eje amarillo.
          Punto rojo   = centro estimado del carril.
        """
        dbg = warp.copy()
        cv2.line(dbg, (0, row), (dbg.shape[1], row), (0, 255, 0), 1)
        for x, color in [(xw, (255,255,255)), (xy, (0,255,255)), (xc, (0,0,255))]:
            if x is not None:
                cv2.circle(dbg, (int(x), row), 5, color, -1)
        out        = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
        out.header = header_msg.header
        self.pub_dbg.publish(out)


# ═══════════════════════════════════════════════════════════════════════════════
#  SECCIÓN 6 — NODO CONTROLADOR PID (ROS2)
#  Suscribe: /lane_error, /odom_raw
#  Publica:  /cmd_vel → velocidad lineal y angular del robot
# ═══════════════════════════════════════════════════════════════════════════════

class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')

        self.error      = None
        self.last_error = 0.0
        self.integral   = 0.0
        self.last_stamp = self.get_clock().now()
        self.last_rx    = self.get_clock().now()

        self.laps_done       = 0
        self.total_dist      = 0.0
        self.last_odom_x     = None
        self.last_odom_y     = None
        self.mision_completa = False

        self.sub_err  = self.create_subscription(Float32,  '/lane_error', self.on_error, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom_raw',   self.on_odom,  10)
        self.pub      = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer    = self.create_timer(1.0 / CONTROL_RATE, self.control_loop)

        self.get_logger().info('lane_controller listo.')
        self.get_logger().info(
            f'Mision: {NUM_VUELTAS} vueltas x {METROS_POR_VUELTA} m = '
            f'{NUM_VUELTAS * METROS_POR_VUELTA:.1f} m totales.')

    def on_error(self, msg):
        """Recibe el error lateral. Solo actualiza si no es NaN."""
        if not math.isnan(msg.data):
            self.error   = msg.data
            self.last_rx = self.get_clock().now()

    def on_odom(self, msg):
        """
        Acumula distancia recorrida y cuenta vueltas.
        Cuando se completan NUM_VUELTAS, activa mision_completa.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_odom_x is not None:
            self.total_dist += math.hypot(x - self.last_odom_x,
                                          y - self.last_odom_y)
            vueltas = int(self.total_dist / METROS_POR_VUELTA)
            if vueltas > self.laps_done:
                self.laps_done = vueltas
                self.get_logger().info(
                    f'Vuelta {self.laps_done}/{NUM_VUELTAS} completada '
                    f'({self.total_dist:.1f} m recorridos)')
                if self.laps_done >= NUM_VUELTAS:
                    self.mision_completa = True
                    self.get_logger().info('Mision completa. Frenando.')

        self.last_odom_x = x
        self.last_odom_y = y

    def control_loop(self):
        """
        Lazo PID a CONTROL_RATE Hz.

        Si mision completa    → frena definitivamente.
        Si sin error reciente → frena por seguridad (timeout).
        Si error válido       → calcula omega con PID y publica /cmd_vel.

        PID:
          P = KP * error
          I = KI * integral (con anti-windup ±INTEGRAL_LIMIT)
          D = KD * (error - last_error) / dt
          omega saturado a ±MAX_ANGULAR
        """
        now = self.get_clock().now()
        dt  = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now
        if dt <= 0.0:
            return

        if self.mision_completa:
            self.pub.publish(Twist())
            return

        age = (now - self.last_rx).nanoseconds * 1e-9
        if self.error is None or age > ERROR_TIMEOUT:
            self.pub.publish(Twist())
            self.integral = 0.0
            return

        error  = self.error
        p_term = KP * error

        self.integral += error * dt
        self.integral  = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, self.integral))
        i_term = KI * self.integral

        d_term = KD * (error - self.last_error) / dt

        w = max(-MAX_ANGULAR, min(MAX_ANGULAR, p_term + i_term + d_term))
        self.last_error = error

        cmd           = Twist()
        cmd.linear.x  = LINEAR_SPEED
        cmd.angular.z = w
        self.pub.publish(cmd)


# ═══════════════════════════════════════════════════════════════════════════════
#  SECCIÓN 7 — PUNTO DE ENTRADA
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    """
    Modo calibrar (--calibrar --source 0):
      Abre el calibrador HSV interactivo sin ROS2.
      Ajusta los sliders y presiona 's' para guardar el YAML.

    Modo normal (sin argumentos):
      Lanza LaneDetector + LaneController en paralelo.
      El robot sigue el carril y se detiene al completar NUM_VUELTAS.
    """
    parser = argparse.ArgumentParser(description='CapyTown Lane Node RC-2')
    parser.add_argument('--calibrar', action='store_true',
                        help='Modo calibracion HSV interactivo (sin ROS2)')
    parser.add_argument('--source', default='0',
                        help='Indice de camara (0,1,...) o ruta a video/imagen')
    parsed, _ = parser.parse_known_args()

    if parsed.calibrar:
        run_calibrator(parsed.source)
        return

    if not ROS_AVAILABLE:
        print('ERROR: ROS2 no disponible. Usa --calibrar para calibrar HSV.')
        sys.exit(1)

    rclpy.init(args=args)
    detector   = LaneDetector()
    controller = LaneController()

    executor = MultiThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.pub.publish(Twist())
        detector.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
