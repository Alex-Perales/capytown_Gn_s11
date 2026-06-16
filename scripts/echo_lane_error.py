#!/usr/bin/env python3
"""echo_lane_error.py — Suscriptor de prueba para /lane_error (ROS2, CapyTown).

Nodo minimo que se suscribe a /lane_error y muestra el valor en consola con una
barra ASCII y el estado (centrado / izquierda / derecha / sin deteccion).
Sirve para verificar que `lane_detector` esta publicando bien ANTES de conectar
el controlador. Complemento del PPTX y de la guia de laboratorio.

Uso (en el robot o PC con ROS2):
    ros2 run ... no aplica (script suelto) -> ejecutar directo:
    python3 echo_lane_error.py
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class LaneErrorEcho(Node):
    def __init__(self):
        super().__init__("lane_error_echo")
        self.create_subscription(Float32, "/lane_error", self.cb, 10)
        self.get_logger().info("Escuchando /lane_error ... (Ctrl+C para salir)")

    def cb(self, msg):
        e = msg.data
        if math.isnan(e):
            print("  [sin deteccion]  error = NaN")
            return
        # Barra: centro en el medio, +/- 0.20 m a los extremos
        span, width = 0.20, 21
        pos = int((e / span) * (width // 2)) + width // 2
        pos = max(0, min(width - 1, pos))
        bar = ["-"] * width
        bar[width // 2] = "|"
        bar[pos] = "#"
        side = "centrado" if abs(e) < 0.01 else ("derecha" if e > 0 else "izquierda")
        print(f"  [{''.join(bar)}]  error = {e:+.3f} m  ({side})")


def main():
    rclpy.init()
    node = LaneErrorEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
