#!/usr/bin/env python3
"""hsv_calibrate.py - CapyTown G4 (Semana 11, RC-2)

ARCHIVO PRINCIPAL DE CALIBRACION HSV (blanco del borde + amarillo del
eje central). Solo nos importan estos dos colores: todo lo demas
(verde del tile, gris del piso, rojo del chasis, etc.) se ignora.

QUE HACE:
  - Abre la camara (o un video/imagen) y muestra la imagen en vista de
    pajaro (IPM), igual que lane_detector.py.
  - Crea 2 ventanas con "trackbars" (sliders) para ajustar en vivo los
    12 parametros HSV (6 para blanco, 6 para amarillo).
  - Muestra 3 ventanas: imagen IPM, mascara blanco, mascara amarillo, y
    una vista combinada con el centroide de cada color marcado.
  - Imprime en consola, cada ~1s, el bloque YAML listo para copiar a
    config/hsv_params.yaml con los valores actuales de los sliders.

COMO USARLO EN EL ROBOT (via SSH + VNC, para ver la ventana de OpenCV
necesitas un entorno grafico, p.ej. abrir esto desde la sesion VNC, no
solo SSH de texto):

    ssh pi@10.42.0.1
    sh ros2_humble.sh
    # con la camara ya encendida (ros2 launch usb_cam camera.launch.py)
    # o usando directamente /dev/video0 con --source 0:
    python3 hsv_calibrate.py --source 0
    python3 hsv_calibrate.py --source 0 --params ../capytown_esan_pkg/config/hsv_params.yaml

Controles:
  q / ESC : salir e imprimir el YAML final
  s       : guardar el YAML final en hsv_params_calibrado.yaml (mismo dir)

================================================================
GUIA RAPIDA: QUE PASA SI SUBO O BAJO CADA PARAMETRO
================================================================

Los 6 parametros por color son: H_min, H_max, S_min, S_max, V_min, V_max
El espacio HSV en OpenCV usa rangos H:[0,180], S:[0,255], V:[0,255].

--- BLANCO (borde del carril) ---
El blanco real tiene saturacion (S) MUY baja y brillo (V) MUY alto, el
tono (H) casi no importa (por eso h_min=0, h_max=180: deja pasar
cualquier tono).

  white_s_max (subir):
    - Deja pasar colores con MAS saturacion -> empiezas a detectar
      como "blanco" superficies grises claras o ligeramente
      amarillentas/verdosas con brillo. Riesgo: el verde del tile
      "lavado" por luz puede colarse en la mascara blanca.
  white_s_max (bajar):
    - Solo pixeles casi sin color (blanco/gris puro) pasan. Si lo bajas
      demasiado, el borde blanco real (que nunca es 100% puro por la
      camara) puede desaparecer de la mascara -> x_white = None.

  white_v_min (subir):
    - Exige mas brillo para considerar "blanco". Sube esto si el piso
      gris tambien se esta detectando como blanco (el piso gris suele
      tener V mas bajo que la pintura blanca).
  white_v_min (bajar):
    - Detecta blancos mas oscuros/sombreados (util si hay sombra sobre
      la linea), pero arriesga confundir el piso gris claro con blanco.

  white_h_min / white_h_max:
    - Normalmente se dejan en 0/180 (todo el espectro), porque el
      blanco no tiene un "tono" definido. Solo se acotan si hay un
      reflejo de color muy fuerte (p.ej. luces calidas que tinen el
      blanco de amarillo) y se quiere excluir ese tono.

--- AMARILLO (eje central) ---
El amarillo tiene H aprox 20-35 (en la escala 0-180 de OpenCV), y S, V
altos (color saturado y brillante).

  yellow_h_min / yellow_h_max (rango de tono):
    - Subir yellow_h_min o bajar yellow_h_max ESTRECHA el rango de
      tonos aceptados -> mas selectivo, menos falsos positivos (p.ej.
      el verde de los tiles tiene H ~ 60-90, normalmente no se cruza,
      pero luces calidas pueden "correr" el amarillo hacia naranja/H
      mas bajo).
    - Bajar yellow_h_min o subir yellow_h_max AMPLIA el rango -> mas
      tolerante a variaciones de iluminacion, pero riesgo de capturar
      tonos parecidos (naranja, verde-amarillento).

  yellow_s_min (subir):
    - Exige colores mas "puros" (muy saturados). Si la pista tiene
      iluminacion fuerte que "lava" el amarillo (se ve palido), subir
      esto demasiado hace que la linea amarilla deje de detectarse
      (x_yellow = None) en esas zonas.
  yellow_s_min (bajar):
    - Detecta amarillos mas palidos/desaturados, pero puede empezar a
      incluir el gris/beige del piso si tiene un tinte amarillento.

  yellow_v_min (subir/bajar):
    - Igual que en blanco: subirlo exige mas brillo (puede perder la
      linea en sombras); bajarlo detecta amarillos mas oscuros pero
      puede confundir con el piso oscuro si este tiene tinte amarillo.

--- min_area (en lane_detector / hsv_params.yaml, no es un slider aqui) ---
  Es el numero MINIMO de pixeles encendidos en la banda de muestreo
  para aceptar un centroide como deteccion real.
    - Subir min_area: ignora manchas pequenas / ruido, pero si la linea
      se ve muy delgada en la imagen puede empezar a descartarla
      tambien (x_* = None con mas frecuencia).
    - Bajar min_area: detecta lineas mas delgadas/lejanas, pero es mas
      sensible a ruido (reflejos, polvo) que puede generar "falsas
      detecciones" y saltos en /lane_error.

================================================================
"""
import argparse
import time

import cv2
import numpy as np


# --- Valores iniciales (= hsv_params.yaml de partida, sec. 5.1) ---
DEFAULTS = dict(
    white_h_min=0,   white_h_max=180,
    white_s_min=0,   white_s_max=30,
    white_v_min=180, white_v_max=255,
    yellow_h_min=20, yellow_h_max=35,
    yellow_s_min=100, yellow_s_max=255,
    yellow_v_min=100, yellow_v_max=255,
)


def load_yaml_seed(path):
    """Lee valores iniciales desde un hsv_params.yaml (parser simple, sin
    depender de PyYAML, igual que line_detect_standalone.py)."""
    vals = dict(DEFAULTS)
    if not path:
        return vals
    for line in open(path):
        line = line.split('#')[0].strip()
        if ':' in line:
            k, _, v = line.partition(':')
            k = k.strip()
            v = v.strip()
            if k in vals:
                try:
                    vals[k] = int(float(v))
                except ValueError:
                    pass
    return vals


def build_ipm(w, h):
    """Misma homografia que lane_detector.py / line_detect_standalone.py."""
    src = np.float32([[0.18 * w, 0.62 * h], [0.82 * w, 0.62 * h],
                       [1.00 * w, 0.98 * h], [0.00 * w, 0.98 * h]])
    dst = np.float32([[0.30 * w, 0], [0.70 * w, 0],
                       [0.70 * w, h], [0.30 * w, h]])
    return cv2.getPerspectiveTransform(src, dst)


def nothing(_):
    pass


def make_trackbars(window, prefix, seed):
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.createTrackbar('H min', window, seed[prefix + '_h_min'], 180, nothing)
    cv2.createTrackbar('H max', window, seed[prefix + '_h_max'], 180, nothing)
    cv2.createTrackbar('S min', window, seed[prefix + '_s_min'], 255, nothing)
    cv2.createTrackbar('S max', window, seed[prefix + '_s_max'], 255, nothing)
    cv2.createTrackbar('V min', window, seed[prefix + '_v_min'], 255, nothing)
    cv2.createTrackbar('V max', window, seed[prefix + '_v_max'], 255, nothing)


def read_trackbars(window):
    return (
        cv2.getTrackbarPos('H min', window),
        cv2.getTrackbarPos('H max', window),
        cv2.getTrackbarPos('S min', window),
        cv2.getTrackbarPos('S max', window),
        cv2.getTrackbarPos('V min', window),
        cv2.getTrackbarPos('V max', window),
    )


def yaml_block(white, yellow, extra):
    wh_min, wh_max, ws_min, ws_max, wv_min, wv_max = white
    yh_min, yh_max, ys_min, ys_max, yv_min, yv_max = yellow
    return f"""lane_detector:
  ros__parameters:
    # Blanco - borde derecho del carril
    white_h_min: {wh_min}
    white_h_max: {wh_max}
    white_s_min: {ws_min}
    white_s_max: {ws_max}
    white_v_min: {wv_min}
    white_v_max: {wv_max}
    # Amarillo - eje central discontinuo
    yellow_h_min: {yh_min}
    yellow_h_max: {yh_max}
    yellow_s_min: {ys_min}
    yellow_s_max: {ys_max}
    yellow_v_min: {yv_min}
    yellow_v_max: {yv_max}
    # Geometria / IPM (sin cambios por este script)
    min_area: {extra['min_area']}
    lane_width_m: {extra['lane_width_m']}
    px_per_meter: {extra['px_per_meter']}
    look_ahead_row: {extra['look_ahead_row']}
    publish_debug: true
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='0',
                     help='indice de camara (0,1,...) o ruta a video/imagen')
    ap.add_argument('--params', default=None,
                     help='hsv_params.yaml con valores iniciales (opcional)')
    ap.add_argument('--min-area', type=float, default=150)
    ap.add_argument('--lane-width-m', type=float, default=0.21)
    ap.add_argument('--px-per-meter', type=float, default=600.0)
    ap.add_argument('--look-ahead-row', type=float, default=0.6)
    args = ap.parse_args()

    seed = load_yaml_seed(args.params)
    extra = dict(min_area=int(args.min_area), lane_width_m=args.lane_width_m,
                  px_per_meter=args.px_per_meter,
                  look_ahead_row=args.look_ahead_row)

    static_img, cap = None, None
    if args.source.isdigit():
        cap = cv2.VideoCapture(int(args.source))
    else:
        img = cv2.imread(args.source)
        if img is not None:
            static_img = img
        else:
            cap = cv2.VideoCapture(args.source)
    if static_img is None and (cap is None or not cap.isOpened()):
        raise SystemExit(f'No pude abrir la fuente: {args.source}')

    make_trackbars('Blanco (borde)', 'white', seed)
    make_trackbars('Amarillo (eje)', 'yellow', seed)

    M = None
    kernel = np.ones((3, 3), np.uint8)
    last_print = 0.0

    print('[hsv_calibrate] q/ESC para salir e imprimir el YAML final.')
    print('[hsv_calibrate] s para guardar hsv_params_calibrado.yaml')

    while True:
        if static_img is not None:
            frame = static_img.copy()
        else:
            ok, frame = cap.read()
            if not ok:
                break
        frame = cv2.resize(frame, (640, 480))
        h, w = frame.shape[:2]
        if M is None:
            M = build_ipm(w, h)

        warp = cv2.warpPerspective(frame, M, (w, h))
        hsv = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)

        white = read_trackbars('Blanco (borde)')
        yellow = read_trackbars('Amarillo (eje)')

        mask_white = cv2.inRange(
            hsv,
            np.array([white[0], white[2], white[4]]),
            np.array([white[1], white[3], white[5]]))
        mask_yellow = cv2.inRange(
            hsv,
            np.array([yellow[0], yellow[2], yellow[4]]),
            np.array([yellow[1], yellow[3], yellow[5]]))
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)

        # Fila de muestreo (look-ahead), igual que lane_detector.py
        row = int(args.look_ahead_row * h)
        band = slice(max(0, row - 8), min(h, row + 8))

        def centroid_x(mask):
            m = cv2.moments(mask, binaryImage=True)
            if m['m00'] < args.min_area:
                return None
            return m['m10'] / m['m00']

        xw = centroid_x(mask_white[band, :])
        xy = centroid_x(mask_yellow[band, :])

        view = warp.copy()
        cv2.line(view, (0, row), (w, row), (0, 255, 0), 1)
        for x, color, label in ((xw, (255, 255, 255), 'W'),
                                 (xy, (0, 255, 255), 'Y')):
            if x is not None:
                cv2.circle(view, (int(x), row), 6, color, -1)
                cv2.putText(view, label, (int(x) - 5, row - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        combo_mask = cv2.cvtColor(cv2.bitwise_or(mask_white, mask_yellow),
                                   cv2.COLOR_GRAY2BGR)
        cv2.imshow('IPM + centroides', view)
        cv2.imshow('Mascara blanco', mask_white)
        cv2.imshow('Mascara amarillo', mask_yellow)

        now = time.time()
        if now - last_print > 1.0:
            print('\n--- hsv_params.yaml (valores actuales) ---')
            print(yaml_block(white, yellow, extra))
            last_print = now

        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('s'):
            with open('hsv_params_calibrado.yaml', 'w') as f:
                f.write(yaml_block(white, yellow, extra))
            print('[hsv_calibrate] guardado en hsv_params_calibrado.yaml')

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()

    print('\n========== YAML FINAL (copiar a config/hsv_params.yaml) ==========')
    print(yaml_block(white, yellow, extra))


if __name__ == '__main__':
    main()
