#!/usr/bin/env python3
"""plot_lane_error.py - CapyTown G4 (Semana 11, RC-2)

Genera lane_error_s11.png a partir del ros2 bag de la corrida final
(3 vueltas). El bag NO se sube al repo (sec. 7), pero el plot SI.

Uso:
    python3 plot_lane_error.py <ruta_bag>

Ejemplo (bag grabado con: ros2 bag record /lane_error /cmd_vel /odom -o s11_final):
    python3 plot_lane_error.py ../s11_final
"""
import sys
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32

if len(sys.argv) < 2:
    raise SystemExit('Uso: python3 plot_lane_error.py <ruta_bag>')

reader = SequentialReader()
reader.open(StorageOptions(uri=sys.argv[1], storage_id='sqlite3'),
             ConverterOptions('', ''))

t0, ts, errs = None, [], []
while reader.has_next():
    topic, data, stamp = reader.read_next()
    if topic == '/lane_error':
        t0 = stamp if t0 is None else t0
        ts.append((stamp - t0) * 1e-9)
        errs.append(deserialize_message(data, Float32).data)

plt.plot(ts, errs)
plt.axhline(0, ls='--', c='gray')
plt.xlabel('tiempo (s)')
plt.ylabel('/lane_error (m)')
plt.title('Error lateral - 3 vueltas RC-2 (Grupo 4)')
plt.grid(True)
plt.savefig('lane_error_s11.png', dpi=150)
print('Guardado: lane_error_s11.png')
