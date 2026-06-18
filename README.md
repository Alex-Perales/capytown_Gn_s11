# CapyTown G4 — Semana 11 (RC-2): "Las 3 Vueltas del Jiron"

Lane following con visión HSV + IPM y control PID + feedforward
(ROS2 Humble, Yahboom Pi5).

## Pista de este reto (RETO2 MODIFICADO)

Pista cuadrada de 2 m x 2 m (4 tiles verdes de 1 m x 1 m en el centro).

- Carril perimetral: borde **blanco** por fuera, eje **amarillo** por el
  centro del carril (`lane_width_m = 0.22 m`).
- Sentido de la vuelta: el robot recorre el borde superior hacia la
  izquierda, baja por el lado izquierdo, recorre el borde inferior hacia
  la derecha y sube por el lado derecho de regreso al punto de partida.
- Misión: **3 vueltas autónomas** (`NUM_VUELTAS = 3`,
  `METROS_POR_VUELTA = 6.4`) siguiendo el eje amarillo **sin pisar las
  líneas blancas**. Al completarlas el robot frena solo.

La detección es por visión (HSV + IPM sobre la imagen de la cámara), así
que **no depende del tamaño/forma del circuito**. Lo único que se ajusta
para una pista nueva es:

- **`hsv_params.yaml`**: recalibrar el blanco/amarillo con
  `scripts/hsv_calibrate.py` o `python3 lane_node.py --calibrar` bajo la
  iluminación real del día.
- **4 puntos `src` de la IPM** en `build_ipm()`: si en `/lane/debug_image`
  las líneas se ven curvas en la vista de pájaro, reajustarlos.
- **`pid_params.yaml`**: si las curvas son más cerradas, bajar un poco
  `linear_speed` y/o `kp` al sintonizar.

## Parámetros por defecto (estado actual)

| Parámetro | Valor | Archivo |
|---|---|---|
| Velocidad de crucero `linear_speed` | 0.34 m/s | `config/pid_params.yaml` |
| Ganancia proporcional `kp` | 2.2 | `config/pid_params.yaml` |
| Ganancia integral `ki` | 0.12 | `config/pid_params.yaml` |
| Ganancia derivativa `kd` | 0.25 | `config/pid_params.yaml` |
| Feedforward `kff` (anticipa curva) | 0.6 | `config/pid_params.yaml` |
| Velocidad angular máx. `max_angular` | 2.0 rad/s | `config/pid_params.yaml` |
| Frecuencia de control `control_rate` | 30 Hz | `config/pid_params.yaml` |
| Frenado seguro `error_timeout` | 0.5 s | `config/pid_params.yaml` |
| Ancho de carril `lane_width_m` | 0.22 m | `config/hsv_params.yaml` |
| Vueltas para terminar `NUM_VUELTAS` | 3 | `lane_node.py` |

## Estructura del repo

```
capytown_G4_s11/
├── capytown_esan_pkg/
│   ├── capytown_esan/
│   │   ├── lane_detector.py     # nodo de visión (HSV + IPM -> /lane_error, /lane_slope)
│   │   ├── lane_controller.py   # nodo de control (PID + feedforward + anti-windup)
│   │   └── lane_node.py         # TODO-EN-UNO: detector + controlador + calibrador (--calibrar)
│   ├── config/
│   │   ├── hsv_params.yaml       # rangos de color + geometría (RECALIBRAR el día del lab)
│   │   ├── pid_params.yaml       # ganancias PID + feedforward + velocidad
│   │   └── *.yaml.bak            # respaldos de los .yaml
│   ├── launch/lane_following.launch.py
│   ├── package.xml
│   ├── setup.py / setup.cfg
│   └── resource/capytown_esan_pkg
├── scripts/
│   ├── hsv_calibrate.py    # calibración en vivo de blanco/amarillo (sliders)
│   ├── echo_lane_error.py  # monitor de /lane_error en consola (barra ASCII)
│   └── plot_lane_error.py  # genera lane_error_s11.png desde el bag
├── docs/
│   └── activacion_cognitiva.md  # plantilla de la predicción de la sec. 2
├── sim_carrito.py          # simulador en PC (sin ROS): trayectoria + lane_error
└── README.md
```

## Dos formas de correr el código

1. **Todo-en-uno (lo que usamos en el robot):** `python3 lane_node.py`
   arranca el detector + el controlador en un solo proceso, posiciona la
   cámara automáticamente (publica `/servo_s2 = -45` al iniciar) y cuenta
   las vueltas.
2. **Por launch (paquete compilado):**
   `ros2 launch capytown_esan_pkg lane_following.launch.py`
   levanta los nodos `lane_detector` y `lane_controller` con los YAML.

Topics: suscribe `/image_raw`; publica `/lane_error`, `/lane_slope`,
`/lane/debug_image`, `/cmd_vel` y `/servo_s2`. Odometría: `/odom_raw`.

## Flujo de trabajo en el robot (SSH + VNC, contenedores Docker)

### Terminal 1 — chasis + IMU

```bash
ssh pi@10.42.0.1
sh ros2_humble.sh
source ~/yahboomcar_ws/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
```

### Terminal 2 — cámara

```bash
docker exec -it <id_contenedor_camara> /bin/bash
source ~/yahboomcar_ws/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 launch usb_cam camera.launch.py
```

### Terminal 3 — visor de imagen

```bash
docker exec -it <id_contenedor_camara> /bin/bash
source ~/yahboomcar_ws/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
# elegir /image_raw (cruda) o /lane/debug_image (máscaras + 3 bandas)
```

### Copiar el paquete al workspace (desde tu PC, PowerShell)

```powershell
scp -r "C:\Users\alexp\Documents\GitHub\capytown_G4_s11\capytown_esan_pkg" pi@10.42.0.1:~/yahboomcar_ws/src/
# (opcional, para los scripts de monitoreo/plot)
scp -r "C:\Users\alexp\Documents\GitHub\capytown_G4_s11\scripts" pi@10.42.0.1:~/yahboomcar_ws/src/
```

### Terminal 4 — correr el lane following

```bash
docker exec -it <id_contenedor_ros> /bin/bash
source ~/yahboomcar_ws/install/setup.bash
source /opt/ros/humble/setup.bash
cd ~/yahboomcar_ws/src/capytown_esan_pkg/capytown_esan
python3 lane_node.py
```

La cámara se posiciona sola al arrancar (`/servo_s2 = -45`). El robot
espera unos segundos quieto y luego avanza.

### Calibrar HSV el mismo día

Con la cámara encendida y conectado por VNC (para ver las ventanas de
OpenCV):

```bash
python3 lane_node.py --calibrar --source 0
# o:  python3 scripts/hsv_calibrate.py --source 0 --params ../capytown_esan_pkg/config/hsv_params.yaml
```

Ajustar los sliders hasta que la máscara blanca cubra solo el borde y la
amarilla solo el eje. Copiar el bloque YAML que imprime en consola a
`config/hsv_params.yaml`.

### Verificar la detección

En `rqt_image_view` elegir `/lane/debug_image`: las líneas deben verse
rectas en la vista de pájaro, con las **3 bandas verdes** y la línea de
recorrido en magenta. Si se ven curvas, ajustar los 4 puntos `src` de
`build_ipm()`. Para ver el error en texto:

```bash
ros2 topic echo /lane_error      # valor crudo
python3 scripts/echo_lane_error.py   # barra ASCII (izq/centro/der)
```

### Sintonizar PID

Un parámetro a la vez (primero `kp`, luego `kd`, al final `ki`), editando
`config/pid_params.yaml`. Grabar un bag por cada cambio:

```bash
ros2 bag record /lane_error /lane_slope /cmd_vel /odom_raw -o s11_tune
```

### Prueba final (3 vueltas, >= 0.2 m/s) y plot del entregable

```bash
ros2 bag record /lane_error /lane_slope /cmd_vel /odom_raw -o s11_final
# ... correr 3 vueltas ...
python3 scripts/plot_lane_error.py s11_final   # genera lane_error_s11.png
```

### Entrega

```bash
git add .
git commit -m "RC-2 Semana 11: lane following (PID + feedforward) + calibración"
git tag s11
git push origin <rama_del_grupo> --tags
```

## Simulador en PC (sin robot)

Para probar el controlador y ver el `lane_error` sin la pista física:

```bash
pip install numpy matplotlib
python3 sim_carrito.py
```

Usa las mismas ganancias del proyecto, recorre la pista cuadrada las 3
vueltas y genera `sim_carrito.png` con la trayectoria y el error lateral
(plano en rectas, picos en las esquinas). Cambiar las ganancias arriba
del archivo para experimentar (p. ej. subir `kp` muestra el zigzag).

## Entregables de la sec. 7 — checklist

- [x] `lane_detector.py` / `lane_controller.py` / `lane_node.py` con ganancias PID por YAML.
- [ ] `config/hsv_params.yaml` calibrado con los valores del DÍA DEL LAB.
- [ ] Video MP4 de 3 vueltas sin intervención humana.
- [ ] `lane_error_s11.png` (con `scripts/plot_lane_error.py`).
- [ ] Foto de la predicción de activación cognitiva (`docs/activacion_cognitiva.md`).
- [ ] (Bonus) comparativa con/sin IPM.
- [ ] `ros2 bag` de la corrida final (NO subir al repo).
- [ ] Tag de git `s11` sobre la rama del grupo.

## Convenio de signo de /lane_error

`error_m > 0` → el centro del carril está a la **derecha** del robot
(robot desviado a la izquierda) → el controlador gira a la derecha
(`omega < 0`).

## Diagnóstico rápido

| Síntoma | Causa probable | Acción |
|---|---|---|
| Zigzaguea | Kp muy alto | Bajar `kp` 20-30%; subir `kd` |
| Reacciona tarde / corta curvas | Kp bajo o feedforward bajo | Subir `kp`; subir `kff` |
| `/lane_error` = NaN | No detecta ninguna línea | Recalibrar HSV; revisar puntos `src` de la IPM |
| Líneas curvas en `debug_image` | IPM mal calibrada | Reajustar los 4 puntos `src` en `build_ipm()` |
| Frena solo en recta | `error_timeout` corto / cámara lenta | Subir `error_timeout`; bajar resolución de cámara |
| Deriva constante a un lado | Sesgo no corregido | Subir `ki` un poco (anti-windup ya implementado) |
| No detecta nada y no se mueve | Cámara mal apuntada | Revisar que `/servo_s2` quedó en -45; ver `/image_raw` |
