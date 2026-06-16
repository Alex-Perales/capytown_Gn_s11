#!/usr/bin/env python3
"""CapyTown lane_controller - Semana 11 (RC-2) - Grupo 4.

Controlador PID sobre /lane_error que publica /cmd_vel.

TODO 2 RESUELTO: terminos P, I (con anti-windup) y D, y saturacion de la
velocidad angular.
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')
        self.declare_parameters('', [
            ('kp', 2.5), ('ki', 0.0), ('kd', 0.3),
            ('linear_speed', 0.20),
            ('max_angular', 2.0),
            ('integral_limit', 0.5),
            ('error_timeout', 0.5),
            ('control_rate', 30.0),
        ])
        gp = self.get_parameter
        self.kp = gp('kp').value
        self.ki = gp('ki').value
        self.kd = gp('kd').value
        self.v = gp('linear_speed').value
        self.max_w = gp('max_angular').value
        self.i_limit = gp('integral_limit').value
        self.timeout = gp('error_timeout').value
        rate = gp('control_rate').value

        self.error = None
        self.last_error = 0.0
        self.integral = 0.0
        self.last_stamp = self.get_clock().now()
        self.last_rx = self.get_clock().now()

        self.sub = self.create_subscription(
            Float32, '/lane_error', self.on_error, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info('lane_controller listo.')

    def on_error(self, msg):
        if not math.isnan(msg.data):
            self.error = msg.data
            self.last_rx = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now
        if dt <= 0.0:
            return

        # Seguridad: sin error reciente (o NaN sostenido) => frenar
        age = (now - self.last_rx).nanoseconds * 1e-9
        if self.error is None or age > self.timeout:
            self.pub.publish(Twist())   # v=0, w=0
            self.integral = 0.0
            return

        error = self.error

        # --- PID ---
        # P: respuesta proporcional al error actual. Si Kp es muy alto el
        #    robot sobre-corrige y zigzaguea (oscila); si es muy bajo,
        #    reacciona tarde y "corta" las curvas.
        p_term = self.kp * error

        # I: acumula el error en el tiempo para eliminar un sesgo
        #    sostenido (p.ej. el robot siempre se va un poco a un lado en
        #    rectas largas). Se limita (anti-windup) a [-i_limit, i_limit]
        #    para que el termino integral no "explote" si el error se
        #    queda grande por mucho tiempo (p.ej. el robot atascado).
        #    Si Ki es muy alto -> sobreoscilacion / wind-up.
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        i_term = self.ki * self.integral

        # D: reacciona a la velocidad de cambio del error (anticipa).
        #    Amortigua la oscilacion que introduce un Kp alto, pero
        #    amplifica el ruido de la medicion si Kd es demasiado alto.
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative

        # omega = P + I + D, saturado a +/- max_angular (rad/s)
        w = p_term + i_term + d_term
        w = max(-self.max_w, min(self.max_w, w))

        self.last_error = error

        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = w
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())       # frena al salir
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
