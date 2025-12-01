import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class ScaraMover(Node):
    def __init__(self):
        super().__init__('scara_mover_node')
        
        # Conectamos con tus motores (Segun tu foto)
        self.pub_j1 = self.create_publisher(Float64, '/joint1/cmd_pos', 10)
        self.pub_j2 = self.create_publisher(Float64, '/joint2/cmd_pos', 10)
        self.pub_j3 = self.create_publisher(Float64, '/joint3/cmd_pos', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.t = 0.0
        print("¡Moviendo el Robot! Presiona Ctrl+C para detener.")

    def timer_callback(self):
        msg = Float64()
        
        # Movimiento suave
        msg.data = 1.0 * math.sin(self.t)       # Base
        self.pub_j1.publish(msg)
        
        msg.data = 1.0 * math.cos(self.t)       # Brazo
        self.pub_j2.publish(msg)
        
        msg.data = 0.05 * math.sin(self.t) + 0.05 # Sube y baja
        self.pub_j3.publish(msg)
        
        self.t += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = ScaraMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
