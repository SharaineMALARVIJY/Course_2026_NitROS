import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16
from sensor_msgs.msg import Imu
import numpy as np
import json
import time

class AutoCalibrator(Node):
    def __init__(self):
        super().__init__('auto_calibrator')

        # --- PARAMÈTRES ROS ---
        self.declare_parameter('wheelbase', 0.257)      # Tamiya TT02 Standard
        self.declare_parameter('v_test', 0.5)           # m/s vitesse stable
        self.declare_parameter('direction_mode', 'forward') 
        self.declare_parameter('servo_center', 512)     
        self.declare_parameter('servo_step', 4)         
        self.declare_parameter('stalling_threshold', 0.3) # Sensibilité butée en degrés
        self.declare_parameter('samples_per_step', 15)   # Retour du moyennage

        self.L = self.get_parameter('wheelbase').value
        self.v_test = self.get_parameter('v_test').value
        self.mode = self.get_parameter('direction_mode').value
        self.center = self.get_parameter('servo_center').value
        self.step = self.get_parameter('servo_step').value
        self.stall_limit = self.get_parameter('stalling_threshold').value
        self.num_samples = self.get_parameter('samples_per_step').value

        # --- ÉTAT ---
        self.current_yaw_rate = 0.0
        self.results = {} 
        
        # --- PUBS / SUBS ---
        self.pub_servo = self.create_publisher(Int16, '/raw_servo_pos', 10)
        self.pub_speed = self.create_publisher(Float32, '/cmd_speed_target', 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 10)

        # Lancement automatique après 2 secondes
        self.timer = self.create_timer(2.0, self.execute_calibration)

    def imu_cb(self, msg):
        self.current_yaw_rate = msg.angular_velocity.z

    def get_measured_angle_with_averaging(self):
        """Prend plusieurs mesures pour filtrer le bruit"""
        rates = []
        for _ in range(self.num_samples):
            rates.append(self.current_yaw_rate)
            time.sleep(0.03) # Fréquence d'échantillonnage ~33Hz
        
        avg_rate = np.mean(rates)
        speed = abs(self.v_test)
        
        if speed < 0.01:
            return 0.0
            
        # Formule cinématique : delta = atan(L * yaw_rate / v)
        angle_rad = np.arctan((self.L * avg_rate) / speed)
        return np.degrees(angle_rad)

    def sweep(self, direction_sign):
        """ direction_sign: 1 (droite), -1 (gauche) """
        last_angle = -999.0
        current_pos = self.center
        local_results = {}

        while rclpy.ok():
            # Commande Servo
            self.pub_servo.publish(Int16(data=current_pos))
            time.sleep(0.5) # Attente stabilisation physique du virage
            
            # Mesure filtrée
            angle = self.get_measured_angle_with_averaging()
            self.get_logger().info(f"Pos DXL: {current_pos} | Angle Réel: {angle:.2f}°")

            # Détection de butée mécanique
            if abs(angle - last_angle) < self.stall_limit and last_angle != -999.0:
                self.get_logger().warn(f"BUTÉE ATTEINTE à {current_pos}")
                break
            
            local_results[angle] = current_pos
            last_angle = angle
            
            # Incrément
            current_pos += (self.step * direction_sign)
            if current_pos <= 0 or current_pos >= 1023:
                break
        
        return local_results

    def execute_calibration(self):
        self.timer.cancel()
        
        self.get_logger().info(f"START CALIBRATION ({self.mode})...")
        speed_val = self.v_test if self.mode == 'forward' else -self.v_test
        self.pub_speed.publish(Float32(data=speed_val))
        time.sleep(3.0) # Prise d'élan

        # Balayage des deux côtés à partir du centre
        res_a = self.sweep(1)
        res_b = self.sweep(-1)

        self.results.update(res_a)
        self.results.update(res_b)
        
        self.stop_and_save()

    def stop_and_save(self):
        self.pub_speed.publish(Float32(data=0.0))
        self.pub_servo.publish(Int16(data=self.center))
        
        # Formatage propre
        sorted_lut = sorted([[float(a), int(p)] for a, p in self.results.items()])
        filename = f"lut_steering_{self.mode}.json"
        
        with open(filename, 'w') as f:
            json.dump({
                "mode": self.mode, 
                "wheelbase": self.L, 
                "samples_used": self.num_samples,
                "lut": sorted_lut
            }, f, indent=4)
        
        self.get_logger().info(f"TERMINÉ ! Fichier généré : {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()