#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# 1. Importamos la librería 'math' para usar pi.
import math

from std_msgs.msg import Int32, Float64

class SpeedConverterNode(Node):
  def __init__(self):
    # 2. Cambiamos el nombre del nodo.
    super().__init__("speed_converter_node")
    
    # 3. Nos suscribimos al tópico 'rpm_topic' para recibir los datos del publicador.
    self.subscriber_ = self.create_subscription(Int32, "rpm_topic", self.rpm_to_rads_callback, 10)
    
    # El publicador se mantiene, publica la velocidad convertida en el tópico 'rad_vel_topic'.
    self.rads_publisher_ = self.create_publisher(Float64, "rad_vel_topic", 10)
    
    self.get_logger().info("Nodo Conversor de Velocidad activo.")

  def rpm_to_rads_callback(self, msg):
    # 4. Aplicamos la fórmula de conversión.
    # Se elimina la lógica anterior de sumar los mensajes.
    rpm_received = msg.data
    rads_per_second = rpm_received * (math.pi / 30.0)
    
    # Preparamos el nuevo mensaje tipo Float64 para publicarlo.
    new_msg = Float64()
    new_msg.data = rads_per_second
    
    # Publicamos la velocidad convertida.
    self.rads_publisher_.publish(new_msg)
    
    # Opcional: Mostramos en la terminal la conversión que se está realizando.
    self.get_logger().info(f"Recibido: {rpm_received} RPM -> Publicando: {rads_per_second:.2f} rad/s")

def main(args=None):
  rclpy.init(args=args)
  node = SpeedConverterNode()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == "__main__":
  main()
