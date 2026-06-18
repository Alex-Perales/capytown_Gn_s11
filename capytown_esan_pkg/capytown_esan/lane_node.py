#!/usr/bin/env python3
"""
==============================================================================
  CapyTown - Semana 11 - RC-2 - "Las 3 Vueltas del Jiron"
  Lane Following: Vision HSV + IPM + Control PID  (ARCHIVO UNICO)
  Universidad ESAN - Robotica 2026-I - Grupo 4
==============================================================================

Reescrito para replicar el funcionamiento del repo de referencia que YA corre
correctamente en pista (github.com/23-Andres-QC/Reto02), en un solo archivo.

QUE INCLUYE:
  - Posicionamiento AUTOMATICO de la camara (publica /servo_s2 = -45 al
    arrancar). >>> Este era el problema: antes la camara no quedaba apuntando
    a la pista y por eso no detectaba las lineas. Ahora se posiciona sola. <<<
  - LaneDetector robusto: deteccion amarillo+blanco en HSV sobre vista de
    pajaro (IPM), filtro de forma por PCA, 3 bandas horizontales, suavizado EMA
    y publicacion de error (/lane_error) y pendiente (/lane_slope).
  - LaneController: PID + anticipacion de curva (feed-forward por pendiente) +
    giro dedicado en esquinas + espera de arranque. Frena si no ve ninguna linea.
  - Contador de vueltas por odometria (mision de NUM_VUELTAS).
  - Calibrador HSV interactivo (modo --calibrar).

MODOS DE USO:
  Normal   : python3 lane_node.py        (o: ros2 run capytown_esan_pkg lane_node)
  Calibrar : python3 lane_node.py --calibrar --source 0

CONVENCION DE SIGNO:
  error > 0 -> centro a la DERECHA  -> girar derecha (w < 0)
  error < 0 -> centro a la IZQUIERDA -> girar izquierda (w > 0)
"""

# ============================================================================
#  SECCION 1 - VARIABLES CONFIGURABLES
# ============================================================================

# ============================================================================
# -- MISION--
NUM_VUELTAS = 3            # vueltas autonomas antes de frenar
METROS_POR_VUELTA = 6.4    # perimetro aprox. de una vuelta (m)

# -- POSICION DE CAMARA (servo) --
SERVO_S2 = -45             # grados; se publica en /servo_s2 al arrancar
SERVO_INIT_DELAY = 0.5     # s antes de publicar la posicion de camara

# -- VELOCIDAD / CONTROL -----------------------------------------------------
LINEAR_SPEED = 0.34        # m/s de crucero (>=0.2 requerido)
CURVE_SPEED_FACTOR = 0.6   # reduce velocidad 40% siempre (margen de reaccion)
MAX_ANGULAR = 2.0          # rad/s tope de giro
CONTROL_RATE = 30.0        # Hz del lazo de control
START_DELAY = 5.0          # s quieto tras la primera deteccion
ERROR_TIMEOUT = 0.5        # s sin color -> frena

# -- PID --
KP = 2.2
KI = 0.12
KD = 0.25
KFF = 0.6                  # feed-forward de anticipacion por pendiente
INTEGRAL_LIMIT = 0.5

# -- ESQUINAS (giro dedicado) -------------------------------------------------
SLOPE_CURVE_THRESHOLD = 0.04        # pendiente minima para anticipar curva
SHARP_TURN_SLOPE_THRESHOLD = 0.13   # pendiente que indica esquina ~90 grados
SHARP_TURN_KP_SLOPE = 3.0
SHARP_TURN_KP_E = 2.5
SHARP_TURN_MAX_W = 0.80
SHARP_TURN_SPEED_FACTOR = 0.3
MAX_ANTICIPATION_TIME = 0.8
CALIB_TOLERANCE = 0.025

# -- VISION: COLORES HSV 
WHITE_H_MIN, WHITE_H_MAX = 0, 180
WHITE_S_MIN, WHITE_S_MAX = 0, 65
WHITE_V_MIN, WHITE_V_MAX = 170, 255
WHITE_MIN_ELONG = 5.0
WHITE_MIN_AREA = 1000
WHITE_MAX_AREA = 8000

YELLOW_H_MIN, YELLOW_H_MAX = 15, 45
YELLOW_S_MIN, YELLOW_S_MAX = 45, 255
YELLOW_V_MIN, YELLOW_V_MAX = 80, 255
YELLOW_MIN_ELONG = 8.0
YELLOW_MIN_AREA = 500
YELLOW_MAX_AREA = 20000

# -- VISION: GEOMETRIA / IPM --
LANE_WIDTH_M = 0.22        # ancho de carril (m)
PX_PER_METER = 600.0       # escala de la vista IPM
WHITE_BIAS_M = 0.02        # desplaza el centro objetivo hacia el amarillo
EMA_ALPHA = 0.5            # suavizado de los centroides

# Trapecio de perspectiva (fracciones de w,h) - igual que la referencia
IPM_TOP_Y = 0.55
IPM_TOP_LEFT_X = 0.20
IPM_TOP_RIGHT_X = 0.80
IPM_BOTTOM_Y = 0.97
IPM_DST_LEFT = 0.25
IPM_DST_RIGHT = 0.75


# ============================================================================
#  SECCION 2 - IMPORTS
# ============================================================================
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
    from sensor_msgs.msg import Image, Imu
    from std_msgs.msg import Float32, Int32
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    Node = object  # permite importar el archivo (modo --calibrar) sin ROS2 instalado


# ============================================================================
#  SECCION 3 - FUNCIONES COMPARTIDAS DE VISION
# ============================================================================
def build_ipm(w, h):
    """Homografia de vista frontal -> vista de pajaro (IPM)."""
    src = np.float32([
        [IPM_TOP_LEFT_X  * w, IPM_TOP_Y    * h],
        [IPM_TOP_RIGHT_X * w, IPM_TOP_Y    * h],
        [1.00            * w, IPM_BOTTOM_Y * h],
        [0.00            * w, IPM_BOTTOM_Y * h],
    ])
    dst = np.float32([
        [IPM_DST_LEFT  * w, 0.0],
        [IPM_DST_RIGHT * w, 0.0],
        [IPM_DST_RIGHT * w, h  ],
        [IPM_DST_LEFT  * w, h  ],
    ])
    return cv2.getPerspectiveTransform(src, dst)


def centroid_x(mask):
    m = cv2.moments(mask, binaryImage=True)
    if m['m00'] < 1e-3:
        return None
    return m['m10'] / m['m00']


def component_filter(mask, min_area, max_area, min_elong):
    """Conserva componentes grandes y alargados (cintas) por elongacion PCA,
    rechaza manchas/reflejos redondeados."""
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = np.zeros_like(mask)
    for i in range(1, num):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_area or area > max_area:
            continue
        ys, xs = np.where(labels == i)
        if len(xs) < 10:
            continue
        pts = np.column_stack((xs, ys)).astype(np.float32)
        _, _, eigval = cv2.PCACompute2(pts, mean=None)
        elong = float(eigval[0, 0] / (eigval[1, 0] + 1e-6))
        if elong >= min_elong:
            out[labels == i] = 255
    return out


# ============================================================================
#  SECCION 4 - NODO DETECTOR DE CARRIL (ROS2)
#  Suscribe: /image_raw      Publica: /lane_error, /lane_slope,
#            /lane/debug_image, y /servo_s2 (posicion de camara, una vez)
# ============================================================================
class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.bridge = CvBridge()

        self.white_lo = np.array([WHITE_H_MIN, WHITE_S_MIN, WHITE_V_MIN], dtype=np.uint8)
        self.white_hi = np.array([WHITE_H_MAX, WHITE_S_MAX, WHITE_V_MAX], dtype=np.uint8)
        self.yellow_lo = np.array([YELLOW_H_MIN, YELLOW_S_MIN, YELLOW_V_MIN], dtype=np.uint8)
        self.yellow_hi = np.array([YELLOW_H_MAX, YELLOW_S_MAX, YELLOW_V_MAX], dtype=np.uint8)

        self.M = None
        self.warp_size = None

        self.x_yellow_f = None
        self.x_white_f = None
        self.x_center_f = None

        self.sub = self.create_subscription(Image, '/image_raw', self.on_image, 10)
        self.pub_err = self.create_publisher(Float32, '/lane_error', 10)
        self.pub_slope = self.create_publisher(Float32, '/lane_slope', 10)
        self.pub_dbg = self.create_publisher(Image, '/lane/debug_image', 10)
        self.pub_servo = self.create_publisher(Int32, '/servo_s2', 10)

        # Posicion de camara: se publica una vez al arrancar
        self._servo_sent = False
        self._servo_timer = self.create_timer(SERVO_INIT_DELAY, self._init_servo)

        self.get_logger().info('lane_detector listo.')

    def _init_servo(self):
        if self._servo_sent:
            return
        msg = Int32()
        msg.data = int(SERVO_S2)
        self.pub_servo.publish(msg)
        self.get_logger().info(f'Camara: servo s2 -> {SERVO_S2} grados')
        self._servo_sent = True
        self._servo_timer.cancel()

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        h, w = frame.shape[:2]
        if self.M is None:
            self.M = build_ipm(w, h)
            self.warp_size = (w, h)

        warp = cv2.warpPerspective(frame, self.M, self.warp_size)
        hsv = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)
        mask_y_raw = cv2.inRange(hsv, self.yellow_lo, self.yellow_hi)
        mask_w_raw = cv2.inRange(hsv, self.white_lo, self.white_hi)

        open_k = np.ones((3, 3), np.uint8)
        close_k = np.ones((7, 7), np.uint8)
        mask_y_raw = cv2.morphologyEx(mask_y_raw, cv2.MORPH_OPEN, open_k)
        mask_y_raw = cv2.morphologyEx(mask_y_raw, cv2.MORPH_CLOSE, close_k)
        mask_w_raw = cv2.morphologyEx(mask_w_raw, cv2.MORPH_OPEN, open_k)
        mask_w_raw = cv2.morphologyEx(mask_w_raw, cv2.MORPH_CLOSE, close_k)
        mask_w_raw = cv2.bitwise_and(mask_w_raw, cv2.bitwise_not(mask_y_raw))

        mask_yellow = component_filter(mask_y_raw, YELLOW_MIN_AREA, YELLOW_MAX_AREA, YELLOW_MIN_ELONG)
        mask_white = component_filter(mask_w_raw, WHITE_MIN_AREA, WHITE_MAX_AREA, WHITE_MIN_ELONG)

        band_rows = [h // 6, h // 2, (5 * h) // 6]
        band_slices = [slice(0, h // 3), slice(h // 3, (2 * h) // 3), slice((2 * h) // 3, h)]

        lane_width_px = LANE_WIDTH_M * PX_PER_METER
        white_bias_px = WHITE_BIAS_M * PX_PER_METER

        def band_center(sl):
            xy = centroid_x(mask_yellow[sl, :])
            xw = centroid_x(mask_white[sl, :])
            if xy is not None and xw is not None:
                dist = xw - xy
                if dist <= 0 or dist < lane_width_px * 0.6 or dist > lane_width_px * 1.3:
                    xw = None
            if xy is not None and xw is not None:
                return xy, xw, (xy + xw) / 2.0 - white_bias_px
            elif xy is not None:
                return xy, None, xy + lane_width_px / 2.0
            elif xw is not None:
                return None, xw, xw - lane_width_px / 2.0 - white_bias_px
            return None, None, None

        band_points = [band_center(sl) for sl in band_slices]
        trajectory_pts = [(c, r) for (_, _, c), r in zip(band_points, band_rows) if c is not None]

        slope_m = self._inferior_slope(mask_yellow, band_slices[2])
        x_yellow_raw, x_white_raw, center_raw = band_points[2]

        x_yellow = self._ema('x_yellow_f', x_yellow_raw)
        x_white = self._ema('x_white_f', x_white_raw)
        center_px = self._ema('x_center_f', center_raw)

        error_m = (center_px - w / 2.0) / PX_PER_METER if center_px is not None else float('nan')

        if center_px is not None:
            safety_margin_px = lane_width_px * 0.30
            angle_px = 0.0 if math.isnan(slope_m) else slope_m * PX_PER_METER
            look_ahead_gain = 0.7
            boost_gain = 1.2
            max_error_m = 0.20
            if x_yellow is not None:
                d_y = (w / 2.0) - x_yellow
                margin_y = safety_margin_px + max(0.0, -angle_px) * look_ahead_gain
                if d_y < margin_y:
                    error_m += ((margin_y - d_y) / PX_PER_METER) * boost_gain
            if x_white is not None:
                d_w = x_white - (w / 2.0)
                margin_w = safety_margin_px + max(0.0, angle_px) * look_ahead_gain
                if d_w < margin_w:
                    error_m -= ((margin_w - d_w) / PX_PER_METER) * boost_gain
            error_m = max(-max_error_m, min(max_error_m, error_m))

        if x_yellow is not None:
            target_cm = LANE_WIDTH_M * 100.0 / 2.0
            sep_cm = ((w / 2.0) - x_yellow) / PX_PER_METER * 100.0
            err_cm = sep_cm - target_cm
            estado = ('se ACERCA al amarillo' if err_cm < -0.3
                      else 'se ALEJA del amarillo' if err_cm > 0.3
                      else f'separacion correcta ({target_cm:.1f}cm)')
            self.get_logger().info(
                f'Amarillo={x_yellow:.0f}px sep={sep_cm:.1f}cm err={err_cm:+.1f}cm -> {estado}',
                throttle_duration_sec=0.5)
        elif x_white is not None:
            self.get_logger().info(f'Sin amarillo, usando BLANCO={x_white:.0f}px',
                                   throttle_duration_sec=0.5)
        else:
            self.get_logger().info('Sin amarillo ni blanco — frena', throttle_duration_sec=0.5)

        out = Float32(); out.data = float(error_m); self.pub_err.publish(out)
        sout = Float32(); sout.data = float(slope_m); self.pub_slope.publish(sout)

        self._publish_debug(warp, mask_white, mask_yellow, band_rows,
                            x_white, x_yellow, center_px, msg, trajectory_pts)

    def _inferior_slope(self, mask_yellow, sl):
        ys, xs = np.where(mask_yellow[sl, :] > 0)
        if len(xs) < 20:
            return float('nan')
        pts = np.column_stack((xs, ys)).astype(np.float32)
        vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        if abs(vy) < 1e-6:
            return float('nan')
        y_bot = (sl.stop - sl.start) - 1
        x_top = x0 + (0 - y0) * (vx / vy)
        x_bot = x0 + (y_bot - y0) * (vx / vy)
        return (x_top - x_bot) / PX_PER_METER

    def _ema(self, attr, value):
        prev = getattr(self, attr)
        if value is None:
            setattr(self, attr, None)
            return None
        if prev is None:
            setattr(self, attr, value)
            return value
        filtered = (1.0 - EMA_ALPHA) * prev + EMA_ALPHA * value
        setattr(self, attr, filtered)
        return filtered

    def _publish_debug(self, warp, mask_white, mask_yellow, band_rows,
                       xw, xy, xc, header_msg, trajectory_pts=None):
        h, w = warp.shape[:2]
        overlay = warp.copy()
        overlay[mask_white > 0] = (255, 255, 255)
        overlay[mask_yellow > 0] = (0, 255, 255)
        dbg = cv2.addWeighted(overlay, 0.55, warp, 0.45, 0)
        for r in band_rows:
            cv2.line(dbg, (0, r), (w, r), (0, 255, 0), 1)
        cv2.line(dbg, (w // 2, 0), (w // 2, h), (128, 128, 128), 1)
        if trajectory_pts and len(trajectory_pts) >= 2:
            pts = np.array([[int(x), int(y)] for x, y in trajectory_pts], dtype=np.int32)
            cv2.polylines(dbg, [pts], False, (255, 0, 255), 2)
            for x, y in trajectory_pts:
                cv2.circle(dbg, (int(x), int(y)), 4, (255, 0, 255), -1)
        mid_row = band_rows[1]
        for x, color, label in ((xw, (200, 200, 200), 'W'),
                                (xy, (0, 255, 255), 'Y'),
                                (xc, (0, 0, 255), 'C')):
            if x is not None:
                cv2.circle(dbg, (int(x), mid_row), 6, color, -1)
                cv2.putText(dbg, label, (int(x) + 8, mid_row - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        out = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
        out.header = header_msg.header
        self.pub_dbg.publish(out)


# ============================================================================
#  SECCION 5 - NODO CONTROLADOR PID (ROS2)
#  Suscribe: /lane_error, /lane_slope, /imu, /odom_raw   Publica: /cmd_vel
# ============================================================================
class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')

        self.error = None
        self.slope = 0.0
        self.last_error = 0.0
        self.smooth_w = 0.0
        self.integral = 0.0
        self.initialized = False
        self.start_time = None
        self.last_stamp = self.get_clock().now()
        self.last_rx = self.get_clock().now()
        self.anticipation_timer = 0.0
        self.in_sharp_turn = False

        # Mision por odometria
        self.laps_done = 0
        self.total_dist = 0.0
        self.last_odom_x = None
        self.last_odom_y = None
        self.mision_completa = False

        self.sub_err = self.create_subscription(Float32, '/lane_error', self.on_error, 10)
        self.sub_slope = self.create_subscription(Float32, '/lane_slope', self.on_slope, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom_raw', self.on_odom, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0 / CONTROL_RATE, self.control_loop)

        self.get_logger().info('lane_controller listo.')
        self.get_logger().info(
            f'Mision: {NUM_VUELTAS} vueltas x {METROS_POR_VUELTA} m = '
            f'{NUM_VUELTAS * METROS_POR_VUELTA:.1f} m totales.')

    def on_error(self, msg):
        if not math.isnan(msg.data):
            self.error = msg.data
            self.last_rx = self.get_clock().now()
            if not self.initialized:
                self.initialized = True
                self.start_time = self.last_rx
                self.get_logger().info(
                    f'Color detectado — esperando {START_DELAY:.0f}s antes de avanzar...')

    def on_slope(self, msg):
        if not math.isnan(msg.data):
            self.slope = msg.data

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_odom_x is not None:
            self.total_dist += math.hypot(x - self.last_odom_x, y - self.last_odom_y)
            vueltas = int(self.total_dist / METROS_POR_VUELTA)
            if vueltas > self.laps_done:
                self.laps_done = vueltas
                self.get_logger().info(
                    f'Vuelta {self.laps_done}/{NUM_VUELTAS} completada '
                    f'({self.total_dist:.1f} m)')
                if self.laps_done >= NUM_VUELTAS:
                    self.mision_completa = True
                    self.get_logger().info('Mision completa. Frenando.')
        self.last_odom_x = x
        self.last_odom_y = y

    def _smooth(self, target, alpha=0.25):
        self.smooth_w = (1.0 - alpha) * self.smooth_w + alpha * target
        return self.smooth_w

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now
        if dt <= 0.0:
            return

        if self.mision_completa:
            self.pub.publish(Twist())
            return

        if not self.initialized:
            self.pub.publish(Twist())
            return

        if (now - self.start_time).nanoseconds * 1e-9 < START_DELAY:
            self.pub.publish(Twist())
            return

        age = (now - self.last_rx).nanoseconds * 1e-9
        cmd = Twist()

        if age > ERROR_TIMEOUT:
            self.integral = 0.0
            self.smooth_w = 0.0
            self.in_sharp_turn = False
            self.anticipation_timer = 0.0
            self.pub.publish(Twist())
            return

        e = self.error
        if abs(e) < 0.01:
            e = 0.0

        # Esquina real
        if abs(self.slope) > SLOPE_CURVE_THRESHOLD:
            self.anticipation_timer += dt
        else:
            self.anticipation_timer = 0.0
        if (abs(self.slope) > SHARP_TURN_SLOPE_THRESHOLD
                or self.anticipation_timer > MAX_ANTICIPATION_TIME):
            self.in_sharp_turn = True
            self.anticipation_timer = 0.0

        if self.in_sharp_turn:
            w_target = -(SHARP_TURN_KP_SLOPE * self.slope + SHARP_TURN_KP_E * e)
            w_target = max(-SHARP_TURN_MAX_W, min(SHARP_TURN_MAX_W, w_target))
            cmd.angular.z = self._smooth(w_target, alpha=0.10)
            cmd.linear.x = LINEAR_SPEED * SHARP_TURN_SPEED_FACTOR
            self.pub.publish(cmd)
            if abs(self.slope) < SLOPE_CURVE_THRESHOLD and abs(e) < CALIB_TOLERANCE:
                self.in_sharp_turn = False
            return

        # PID + feed-forward
        P = KP * e
        self.integral += e * dt
        self.integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, self.integral))
        I = KI * self.integral
        D = KD * (e - self.last_error) / dt
        FF = KFF * self.slope if abs(self.slope) > SLOPE_CURVE_THRESHOLD else 0.0

        w_pid = -(P + I + D + FF)
        w_pid = max(-MAX_ANGULAR, min(MAX_ANGULAR, w_pid))

        cmd.linear.x = LINEAR_SPEED * CURVE_SPEED_FACTOR
        cmd.angular.z = self._smooth(w_pid, alpha=0.12)
        self.pub.publish(cmd)
        self.last_error = e


# ============================================================================
#  SECCION 6 - CALIBRADOR HSV INTERACTIVO  (python3 lane_node.py --calibrar)
# ============================================================================
def _nothing(_):
    pass


def _make_trackbars(window, vals):
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    names = ('H min', 'H max', 'S min', 'S max', 'V min', 'V max')
    maxs = (180, 180, 255, 255, 255, 255)
    for n, v, mx in zip(names, vals, maxs):
        cv2.createTrackbar(n, window, v, mx, _nothing)


def _read_trackbars(window):
    return tuple(cv2.getTrackbarPos(k, window)
                 for k in ('H min', 'H max', 'S min', 'S max', 'V min', 'V max'))


def _yaml_block(white, yellow):
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
    lane_width_m: {LANE_WIDTH_M}
    px_per_meter: {PX_PER_METER}
    servo_s2: {SERVO_S2}
    publish_debug: true
"""


def run_calibrator(source):
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
                    (WHITE_H_MIN, WHITE_H_MAX, WHITE_S_MIN, WHITE_S_MAX, WHITE_V_MIN, WHITE_V_MAX))
    _make_trackbars('Amarillo (eje)',
                    (YELLOW_H_MIN, YELLOW_H_MAX, YELLOW_S_MIN, YELLOW_S_MAX, YELLOW_V_MIN, YELLOW_V_MAX))

    M = None
    last_print = 0.0
    white = yellow = None
    print('\n[CALIBRADOR] Ajusta los sliders. "s" guarda YAML, "q" sale.\n')

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

        white = _read_trackbars('Blanco (borde)')
        yellow = _read_trackbars('Amarillo (eje)')
        warp = cv2.warpPerspective(frame, M, (w, h))
        hsv = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)
        mask_w = cv2.inRange(hsv, np.array([white[0], white[2], white[4]]),
                             np.array([white[1], white[3], white[5]]))
        mask_y = cv2.inRange(hsv, np.array([yellow[0], yellow[2], yellow[4]]),
                             np.array([yellow[1], yellow[3], yellow[5]]))

        view = warp.copy()
        xw = centroid_x(mask_w)
        xy = centroid_x(mask_y)
        for x, color, label in ((xw, (255, 255, 255), 'W'), (xy, (0, 255, 255), 'Y')):
            if x is not None:
                cv2.circle(view, (int(x), h // 2), 6, color, -1)
                cv2.putText(view, label, (int(x) - 5, h // 2 - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.imshow('IPM (W=blanco Y=amarillo)', view)
        cv2.imshow('Mascara blanco', mask_w)
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


# ============================================================================
#  SECCION 7 - PUNTO DE ENTRADA
# ============================================================================
def main(args=None):
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
    detector = LaneDetector()
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
