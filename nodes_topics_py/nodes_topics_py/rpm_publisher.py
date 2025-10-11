#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class RpmPublisherNode(Node):
  def __init__(self):
    # 1. Cambiamos el nombre del nodo para que sea más descriptivo.
    super().__init__("rpm_publisher_node")
    
    # 2. El valor ahora representa la velocidad en RPM que queremos simular.
    self.rpm_value_ = 120  
    
    # 3. El tópico ahora se llama 'rpm_topic' para indicar claramente qué dato se envía.
    self.publisher_ = self.create_publisher(Int32, "rpm_topic", 10)
    
    # El timer se mantiene para publicar la velocidad cada 0.5 segundos.
    self.timer_ = self.create_timer(0.5, self.publish_rpm)
    
    self.get_logger().info(f"Nodo Publicador de RPM activo. Enviando {self.rpm_value_} RPM...")

  def publish_rpm(self):
    msg = Int32()
    msg.data = self.rpm_value_
    self.publisher_.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  node = RpmPublisherNode()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
