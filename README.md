# CapyTown G4 — Semana 11 (RC-2): "Las 3 Vueltas del Jiron"

Lane following con visión HSV + IPM y control PID (ROS2 Humble, Yahboom Pi5).

## Pista de este reto (RETO2 MODIFICADO)

Esta version usa la pista cuadrada de 2 m x 2 m (4 tiles verdes de 1 m x 1 m
en el centro)

- Carril perimetral: borde **blanco** por fuera, eje **amarillo** por el
  centro del carril (mismo ancho de carril, `lane_width_m = 0.21 m`).
- Sentido de la vuelta (segun las flechas del mapa): el robot recorre el
  borde superior hacia la izquierda, baja por el lado izquierdo, recorre
  el borde inferior hacia la derecha y sube por el lado derecho de regreso
  al punto de partida (arriba a la derecha).
- El robot debe seguir el eje amarillo por el centro del carril y **no
  pisar las lineas blancas** (fuera de carril = falla del reto).

El codigo (`lane_detector.py` + `lane_controller.py`) es el mismo que en
`RETO2`: la deteccion es por vision (HSV + IPM sobre la imagen de la
camara), no depende del tamano/forma del circuito, asi que **no requiere
cambios de codigo** para esta pista. Lo unico que cambia con la pista
nueva es:

- **`hsv_params.yaml`**: recalibrar igual con `scripts/hsv_calibrate.py`
  (el blanco/amarillo de esta pista puede tener una iluminacion distinta).
- **4 puntos `src` de la IPM** en `build_ipm()` (ambos nodos): si al ver
  `/lane/debug_image` las lineas se ven curvas en la vista de pajaro,
  reajustar esos 4 puntos sobre las esquinas de un tile de esta pista.
- **`pid_params.yaml`**: las curvas de este circuito son mas cerradas
  (radio menor que el escenario anterior) -> probablemente haya que bajar
  `linear_speed` y/o `kp` un poco al sintonizar (sec. 3.3 del enunciado).

## Estructura del repo

```
capytown_G4_s11/
├── capytown_esan_pkg/
│   ├── capytown_esan/
│   │   ├── lane_detector.py     # TODO 1 resuelto (HSV + IPM -> /lane_error)
│   │   └── lane_controller.py   # TODO 2 resuelto (PID con anti-windup)
│   ├── config/
│   │   ├── hsv_params.yaml       # RECALIBRAR el dia del lab (ver scripts/hsv_calibrate.py)
│   │   └── pid_params.yaml
│   ├── launch/lane_following.launch.py
│   ├── package.xml
│   ├── setup.py / setup.cfg
│   └── resource/capytown_esan_pkg
├── scripts/
│   ├── hsv_calibrate.py    # calibracion en vivo de blanco/amarillo (con comentarios)
│   ├── echo_lane_error.py  # monitor de /lane_error en consola
│   └── plot_lane_error.py  # genera lane_error_s11.png desde el bag
├── docs/
│   └── activacion_cognitiva.md  # plantilla para la prediccion de la sec. 2
└── README.md
```

## Entregables de la sec. 7 — checklist

- [x] `lane_detector.py` / `lane_controller.py` con ganancias PID por YAML.
- [ ] `config/hsv_params.yaml` calibrado con los valores del DIA DEL LAB
      (correr `scripts/hsv_calibrate.py` y pegar el YAML que imprime).
- [ ] Video MP4 de 3 vueltas sin intervencion humana (se genera al
      correr el robot — no incluido aqui).
- [ ] `lane_error_s11.png` (generado con `scripts/plot_lane_error.py`
      a partir del bag de la corrida final).
- [ ] Foto de la prediccion de activacion cognitiva (`docs/activacion_cognitiva.md`).
- [ ] (Bonus) comparativa con/sin IPM.
- [ ] `ros2 bag` de la corrida final (NO subir al repo, solo debe existir
      para generar el plot).
- [ ] Tag de git `s11` sobre la rama del grupo.

## Flujo de trabajo en el robot (SSH + VNC)

### 1. Conectarse y levantar el chasis + LIDAR

```powershell
ssh pi@10.42.0.1
sh ros2_humble.sh
source ~/yahboomcar_ws/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
```

### 2. Camara

```bash
ros2 launch usb_cam camera.launch.py
# para ver la imagen cruda:
ros2 run rqt_image_view rqt_image_view
```

### 3. Posicion de la camara (para alinear con la pista)

```bash
ros2 topic pub /servo_s1 std_msgs/msg/Int32 'data: 0' --once
ros2 topic pub /servo_s2 std_msgs/msg/Int32 'data: -80' --once
```

### 4. Copiar este paquete al workspace del robot

```bash
# desde tu PC (scp) o git clone directo en el robot:
scp -r capytown_G4_s11/capytown_esan_pkg pi@10.42.0.1:~/yahboomcar_ws/src/
ssh pi@10.42.0.1
cd ~/yahboomcar_ws
colcon build --packages-select capytown_esan_pkg
source install/setup.bash
```

### 5. Calibrar HSV el mismo dia (sec. 6, paso 2)

Con la camara encendida (paso 2) y, idealmente, conectado por VNC para
ver las ventanas de OpenCV:

```bash
cd ~/yahboomcar_ws/src/capytown_esan_pkg/../scripts   # o donde copies scripts/
python3 hsv_calibrate.py --source 0 --params ../capytown_esan_pkg/config/hsv_params.yaml
```

Ajustar los sliders hasta que la mascara blanca solo cubra el borde
blanco y la mascara amarilla solo el eje amarillo. Presionar `s` para
guardar `hsv_params_calibrado.yaml`, copiar ese bloque a
`capytown_esan_pkg/config/hsv_params.yaml` y recompilar (`colcon build`).

### 6. Lanzar el lane following

```bash
ros2 launch capytown_esan_pkg lane_following.launch.py
```

Verificar `/lane/debug_image` en RViz o `rqt_image_view`: las lineas
deben verse rectas y verticales en la vista de pajaro. Si se ven
curvas, ajustar los 4 puntos `src` de `build_ipm()` en
`lane_detector.py`.

Monitorear el error en consola (sin RViz):

```bash
python3 scripts/echo_lane_error.py
```

### 7. Sintonizar PID (sec. 3.3 / 6)

Un parametro a la vez (primero `kp`, luego `kd`, al final `ki`),
editando `config/pid_params.yaml` y recompilando. Grabar un bag por
cada cambio:

```bash
ros2 bag record /lane_error /cmd_vel /odom -o s11_kp25
```

### 8. Prueba final (3 vueltas, ≥ 0.2 m/s)

```bash
ros2 bag record /lane_error /cmd_vel /odom -o s11_final
# ... correr 3 vueltas ...
```

### 9. Generar el plot del entregable

```bash
python3 scripts/plot_lane_error.py s11_final
# genera lane_error_s11.png en el directorio actual -> moverlo al repo
```

### 10. Entrega

```bash
git add .
git commit -m "RC-2 Semana 11: lane_detector + lane_controller + calibracion"
git tag s11
git push origin <rama_del_grupo> --tags
```

## Convenio de signo de /lane_error

`error_m > 0` → el centro del carril esta a la **derecha** del robot
(robot desviado a la izquierda) → el controlador gira a la derecha
(`omega < 0`). Implementado en `lane_detector.py` y consumido por el
PID de `lane_controller.py` (signo de `kp` no se invierte).

## Diagnostico rapido (sec. 10)

| Sintoma | Causa probable | Accion |
|---|---|---|
| Zigzaguea | Kp muy alto | Bajar Kp 20-30%; subir Kd |
| Reacciona tarde / corta curvas | Kp bajo o look-ahead grande | Subir Kp; reducir `look_ahead_row` |
| `/lane_error` = NaN | No detecta ninguna linea | Recalibrar HSV (`hsv_calibrate.py`); revisar puntos `src` de la IPM |
| Lineas curvas en `debug_image` | IPM mal calibrada | Reajustar los 4 puntos `src` en `build_ipm()` |
| Frena solo en recta | `error_timeout` corto / camara lenta | Subir `error_timeout`; bajar resolucion de camara |
| Deriva constante a un lado | Sesgo no corregido | Subir `ki` un poco (anti-windup ya implementado) |
