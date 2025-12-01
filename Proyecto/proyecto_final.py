import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class ScaraFinalProject(Node):
    def __init__(self):
        super().__init__('scara_final_node')
        
        # --- PARÁMETROS DEL ROBOT (CONFIRMADOS DEL URDF) ---
        self.L1 = 0.45
        self.L2 = 0.45
        self.L3 = 0.25
        
        # --- TRAYECTORIA "TRANSVERSAL" (Coincide con MATLAB) ---
        # Punto A: Lateral derecho
        self.p_ini = {'x': 0.5, 'y': -0.4, 'th': -0.5236} # -30 deg
        
        # Punto B: Frontal izquierdo
        self.p_fin = {'x': 0.6, 'y': 0.3, 'th': 0.5236}  # 30 deg
        
        self.tf = 6.0 # Tiempo de ejecución
        
        # Publicadores para Gazebo
        self.pub_j1 = self.create_publisher(Float64, '/joint1/cmd_pos', 10)
        self.pub_j2 = self.create_publisher(Float64, '/joint2/cmd_pos', 10)
        self.pub_j3 = self.create_publisher(Float64, '/joint3/cmd_pos', 10)
        
        self.start_time = None
        self.timer = self.create_timer(0.01, self.control_loop) # 100Hz
        
        print(f"Iniciando Trayectoria: {self.tf} segundos.")
        print(f"L1={self.L1}, L2={self.L2}, L3={self.L3}")

    def inverse_kinematics(self, x, y, th_global):
        # 1. Calcular posición de la muñeca (descontando L3)
        # x_w, y_w es donde termina el eslabón 2
        x_w = x - self.L3 * math.cos(th_global)
        y_w = y - self.L3 * math.sin(th_global)
        
        # 2. Ley de Cosenos para el Codo (q2)
        r_sq = x_w**2 + y_w**2
        # cos(q2) = (r^2 - L1^2 - L2^2) / (2*L1*L2)
        cos_q2 = (r_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        
        # Seguridad numérica
        if cos_q2 > 1.0: cos_q2 = 1.0
        if cos_q2 < -1.0: cos_q2 = -1.0
            
        q2 = math.acos(cos_q2) # Codo positivo (configuración típica)
        
        # 3. Cálculo del Hombro (q1)
        beta = math.atan2(y_w, x_w)
        
        # Ley de cosenos para ángulo interno psi
        # cos(psi) = (r^2 + L1^2 - L2^2) / (2*L1*r)
        num_psi = r_sq + self.L1**2 - self.L2**2
        den_psi = 2 * self.L1 * math.sqrt(r_sq)
        
        # Seguridad numérica para psi
        val_psi = num_psi / den_psi
        if val_psi > 1.0: val_psi = 1.0
        if val_psi < -1.0: val_psi = -1.0
            
        psi = math.acos(val_psi)
        
        q1 = beta - psi # Resta para configuración codo abajo/derecha
        
        # 4. Orientación (q3)
        # th_global = q1 + q2 + q3
        q3 = th_global - q1 - q2
        
        return q1, q2, q3

    def get_quintic_scalar(self, t_now):
        if t_now <= 0: return 0.0
        if t_now >= self.tf: return 1.0
        
        tau = t_now / self.tf
        # Polinomio 10t^3 - 15t^4 + 6t^5
        return 10*(tau**3) - 15*(tau**4) + 6*(tau**5)

    def control_loop(self):
        if self.start_time is None:
            self.start_time = time.time()
            return

        t = time.time() - self.start_time
        
        # Calculamos 's'. Tu funcion get_quintic_scalar YA devuelve 1.0
        # automaticamente si t > tf, asi que el robot se quedara quieto.
        s = self.get_quintic_scalar(t)
        
        # Interpolación Cartesiana
        x_cmd = self.p_ini['x'] + s * (self.p_fin['x'] - self.p_ini['x'])
        y_cmd = self.p_ini['y'] + s * (self.p_fin['y'] - self.p_ini['y'])
        th_cmd = self.p_ini['th'] + s * (self.p_fin['th'] - self.p_ini['th'])
        
        # Cinemática Inversa
        q1, q2, q3 = self.inverse_kinematics(x_cmd, y_cmd, th_cmd)
        
        # Publicar (Esto mantiene al robot en su lugar)
        self.pub_j1.publish(Float64(data=q1))
        self.pub_j2.publish(Float64(data=q2))
        self.pub_j3.publish(Float64(data=q3))

        # Solo avisamos cuando termine, pero NO cerramos el nodo (quitamos el exit)
        if t > self.tf and t < self.tf + 0.1:
            self.get_logger().info('Trayectoria completada. Manteniendo posición...')

def main(args=None):
    rclpy.init(args=args)
    node = ScaraFinalProject()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
