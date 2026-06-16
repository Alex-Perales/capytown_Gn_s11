#!/usr/bin/env python3
"""CapyTown lane_detector - Semana 11 (RC-2) - Grupo 4.

Segmenta el borde blanco y el eje amarillo por HSV, aplica una vista de
pajaro (IPM) y publica el error lateral en metros sobre /lane_error.

TODO 1 RESUELTO: calculo del centro del carril (center_px) y del error
lateral (error_m), manejando los 3 casos: se ven ambas lineas, solo
blanco, solo amarillo, o ninguna (NaN -> el controlador frena por
seguridad, ver lane_controller.py).
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.bridge = CvBridge()

        # --- Declaracion de parametros (cargados desde hsv_params.yaml) ---
        self.declare_parameters('', [
            ('white_h_min', 0), ('white_h_max', 180),
            ('white_s_min', 0), ('white_s_max', 30),
            ('white_v_min', 180), ('white_v_max', 255),
            ('yellow_h_min', 20), ('yellow_h_max', 35),
            ('yellow_s_min', 100), ('yellow_s_max', 255),
            ('yellow_v_min', 100), ('yellow_v_max', 255),
            ('min_area', 150),
            ('lane_width_m', 0.21),
            ('px_per_meter', 600.0),
            ('look_ahead_row', 0.6),
            ('publish_debug', True),
        ])

        gp = self.get_parameter
        self.white_lo = np.array([gp('white_h_min').value,
                                   gp('white_s_min').value,
                                   gp('white_v_min').value])
        self.white_hi = np.array([gp('white_h_max').value,
                                   gp('white_s_max').value,
                                   gp('white_v_max').value])
        self.yellow_lo = np.array([gp('yellow_h_min').value,
                                    gp('yellow_s_min').value,
                                    gp('yellow_v_min').value])
        self.yellow_hi = np.array([gp('yellow_h_max').value,
                                    gp('yellow_s_max').value,
                                    gp('yellow_v_max').value])
        self.min_area = gp('min_area').value
        self.lane_width_m = gp('lane_width_m').value
        self.px_per_meter = gp('px_per_meter').value
        self.look_ahead_row = gp('look_ahead_row').value
        self.publish_debug = gp('publish_debug').value

        self.M = None          # homografia IPM (se calcula al primer frame)
        self.warp_size = None

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, 10)
        self.pub_err = self.create_publisher(Float32, '/lane_error', 10)
        self.pub_dbg = self.create_publisher(Image, '/lane/debug_image', 10)
        self.get_logger().info('lane_detector listo.')

    # --- IPM: homografia de vista frontal a vista de pajaro ---
    def build_ipm(self, w, h):
        # 4 puntos del trapecio del piso en la imagen original (ajustar al lab)
        src = np.float32([[0.18 * w, 0.62 * h], [0.82 * w, 0.62 * h],
                           [1.00 * w, 0.98 * h], [0.00 * w, 0.98 * h]])
        dst = np.float32([[0.30 * w, 0.0], [0.70 * w, 0.0],
                           [0.70 * w, h], [0.30 * w, h]])
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.warp_size = (w, h)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]
        if self.M is None:
            self.build_ipm(w, h)

        # 1) Vista de pajaro (IPM)
        warp = cv2.warpPerspective(frame, self.M, self.warp_size)
        hsv = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)

        # 2) Mascaras de color
        mask_white = cv2.inRange(hsv, self.white_lo, self.white_hi)
        mask_yellow = cv2.inRange(hsv, self.yellow_lo, self.yellow_hi)
        kernel = np.ones((3, 3), np.uint8)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)

        # 3) Fila de muestreo (look-ahead)
        row = int(self.look_ahead_row * h)
        band = slice(max(0, row - 8), min(h, row + 8))

        # Filtro de area minima: si la mascara tiene muy pocos pixeles
        # encendidos en la banda, se considera "no detectado" (ruido).
        x_white = self._centroid_x(mask_white[band, :], self.min_area)
        x_yellow = self._centroid_x(mask_yellow[band, :], self.min_area)

        # 4) Centro del carril (px) -> error lateral (m)
        # half_px = distancia (en px) entre el eje amarillo central y el
        # borde blanco del carril = mitad del ancho del carril.
        half_px = (self.lane_width_m / 2.0) * self.px_per_meter

        if x_white is not None and x_yellow is not None:
            # Caso normal: se ven ambas lineas -> el centro del carril
            # es el punto medio entre el borde blanco y el eje amarillo.
            center_px = (x_white + x_yellow) / 2.0
        elif x_yellow is not None:
            # Solo se ve el eje amarillo (p.ej. en una curva cerrada donde
            # el borde blanco sale del cuadro). El blanco esta a la
            # DERECHA del amarillo (mismo orden que en build_ipm/dst),
            # por lo que el centro del carril esta media calzada a la
            # derecha del amarillo.
            center_px = x_yellow + half_px
        elif x_white is not None:
            # Solo se ve el borde blanco -> el centro esta media calzada
            # a la IZQUIERDA del blanco.
            center_px = x_white - half_px
        else:
            # No se detecta ninguna linea -> NaN. lane_controller.py
            # interpreta NaN/timeout como "frenar por seguridad".
            center_px = None

        if center_px is None:
            error_m = float('nan')
        else:
            # Convenio de signo (sec. 4 del enunciado):
            #   error_m > 0  => el centro del carril esta a la DERECHA
            #                   del centro de la imagen (robot desviado
            #                   a la izquierda) => el controlador debe
            #                   girar a la derecha (omega < 0).
            #   error_m < 0  => centro del carril a la izquierda del
            #                   centro de la imagen (robot desviado a la
            #                   derecha) => omega > 0.
            error_m = (center_px - w / 2.0) / self.px_per_meter

        out = Float32()
        out.data = float(error_m)
        self.pub_err.publish(out)

        if self.publish_debug:
            self._publish_debug(warp, row, x_white, x_yellow, center_px, msg)

    @staticmethod
    def _centroid_x(mask, min_area):
        m = cv2.moments(mask, binaryImage=True)
        # m['m00'] = numero de pixeles "encendidos" en la mascara (area).
        # Si el area es menor que min_area, se descarta como ruido y se
        # reporta "no detectado" (None) en vez de un centroide falso.
        if m['m00'] < max(min_area, 1e-3):
            return None
        return m['m10'] / m['m00']

    def _publish_debug(self, warp, row, xw, xy, xc, header_msg):
        dbg = warp.copy()
        cv2.line(dbg, (0, row), (dbg.shape[1], row), (0, 255, 0), 1)
        for x, color in ((xw, (255, 255, 255)), (xy, (0, 255, 255)),
                         (xc, (0, 0, 255))):
            if x is not None:
                cv2.circle(dbg, (int(x), row), 5, color, -1)
        out = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
        out.header = header_msg.header
        self.pub_dbg.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
