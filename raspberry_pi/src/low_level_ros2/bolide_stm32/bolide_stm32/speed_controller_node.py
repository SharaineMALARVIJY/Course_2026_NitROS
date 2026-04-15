import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16
from bolide_interfaces.msg import ForkSpeed


class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller_node')

        # --- paramètres ---
        self.declare_parameter('max_speed_forward', 3.0)
        self.declare_parameter('max_speed_reverse', 1.5)
        self.declare_parameter('frequency', 50)
        self.declare_parameter('PID_enabled', True)

        # debug
        self.declare_parameter('debug', True)
        self.declare_parameter('info', True)
        self.declare_parameter('debug_freq', 2.0)  # Hz

        self.max_fwd = self.get_parameter('max_speed_forward').value
        self.max_rev = self.get_parameter('max_speed_reverse').value

        freq = self.get_parameter('frequency').value
        self.debug = self.get_parameter('debug').value
        self.info = self.get_parameter('info').value
        self.debug_freq = self.get_parameter('debug_freq').value
        self.PID_enabled = self.get_parameter('PID_enabled').value

        self.k_turn = 0.0
        self.vnom = 8.4
        self.min_speed_fwd = 0.3
        self.min_speed_rev = 0.15
        self.alpha_vbat = 0.01
        self.alpha_speed_meas = 0.05
        self.integral_max = 0.4 #0.3
        self.KP_rev = 0.1              # facteur de correction P
        self.KI_rev = 0.2
        self.KP_fw = 0.1    #0.1
        self.KI_fw = 0.2 #0.2
        self.gain_fwd = 0.05 #0.05
        self.gain_rev = 0.2

        # --- état ---
        self.DT = 1.0 / freq        # 50 Hz -> 0.02
        self.target = 0.0           # m/s
        self.abs_speed_meas = 0.0   # m/s > 0
        self.speed_meas = 0.0       # m/s
        self.vbat = self.vnom

        self.cmd = 0.0
        self.integral = 0.0
        self.gain = self.gain_fwd
        self.KI = 0.0
        self.motor_direction = 0.0

        # --- pub/sub ---
        self.pub = self.create_publisher(Float32, '/cmd_vel', 10)

        self.create_subscription(Float32, '/cmd_speed_target', self.cb_target, 10)
        self.create_subscription(Float32, '/battery_voltage', self.cb_battery, 10)
        self.create_subscription(ForkSpeed, '/raw_fork_data', self.cb_speed, 10)
        self.create_subscription(Int16, '/esc_state', self.cb_esc_state, 10)

        self.create_timer(self.DT, self.loop)  # 50 Hz

        # virage
        self.create_subscription(Float32, '/cmd_dir', self.cb_dir, 10)
        self.turn = 0.0

        if self.info:
            self.create_timer(1.0 / self.debug_freq, self.debug_loop)

        self.get_logger().info("Adaptive speed controller (m/s) started")

    # ---------------------- callbacks ----------------------

    def cb_target(self, msg):
        self.target = msg.data  # en m/s

    def cb_battery(self, msg):
        raw_vbat = min(8.5, max(5.0, msg.data))
        self.vbat = self.alpha_vbat * raw_vbat + (1.0 - self.alpha_vbat) * self.vbat

    def cb_speed(self, msg):
        speed_meas_raw = msg.speed  # toujours positive
        self.abs_speed_meas = self.alpha_speed_meas * speed_meas_raw + (1.0 - self.alpha_speed_meas) * self.abs_speed_meas

    def cb_dir(self, msg):
        self.turn = abs(msg.data)
    
    def cb_esc_state(self, msg):
        esc_state = msg.data
        self.motor_direction = 1.0 if esc_state >= 0 else -1.0
        self.PID_enabled = abs(esc_state) < 2
        if not self.PID_enabled:
            self.integral = 0.0

    # ---------------------- loop principale ----------------------

    def loop(self):
        v_target = self.target

        # --- voltage ---
        if self.vbat < 6.2:
            self.get_logger().info(f" Motor Battery under 6.2V. Please change your battery. ({self.vbat}V)\n\r")
            v_target = 0.0

        # --- PARAMÈTRES ---
        if v_target > 0 :
            sign = 1.0
            self.max_speed = self.max_fwd
            self.gain = self.gain_fwd
            self.min_speed = self.min_speed_fwd
            self.KI = self.KI_fw
            self.KP = self.KP_fw
            cmd_min = 1.1e-4
            cmd_max = 1.0
        else :
            sign = -1.0
            self.max_speed = self.max_rev
            self.gain = self.gain_rev
            self.min_speed = self.min_speed_rev
            self.KI = self.KI_rev
            self.KP = self.KP_rev
            cmd_max = -1.1e-4
            cmd_min = -1.0

        # --- DEAD ZONE ---
        if abs(v_target) < self.min_speed: 
            self.integral = 0.0
            self.cmd = 0.0
            self.publish(0.0)
            return

        # --- FEEDFORWARD AVEC OFFSET ---
        if abs(v_target) > 0.01:
            # (V / Vmax) * Gain_dynamique + Offset
            cmd_ff = (abs(v_target) / self.max_speed) * self.gain
            
            # Compensation batterie
            cmd_ff *= (self.vnom / self.vbat)

        else:
            cmd_ff = 0.0

        # --- PI (CORRECTEUR) ---
        correction = 0.0
        if self.PID_enabled :
            # L'erreur est : (Vitesse voulue) - (Vitesse mesurée)
            error = abs(v_target) - self.abs_speed_meas
            
            self.integral += error * self.DT
            self.integral = max(-self.integral_max, min(self.integral_max, self.integral))
            
            correction = (self.KP * error) + (self.KI * self.integral)

        # --- SORTIE ---
        # 1. Calculer la valeur brute dans une variable temporaire
        raw_output = (cmd_ff + correction) * sign
        
        # 2. Appliquer le clamp sur cette nouvelle valeur
        # Note la syntaxe propre : max(borne_basse, min(borne_haute, valeur))
        self.cmd = max(cmd_min, min(cmd_max, raw_output))
        
        self.publish(self.cmd)
    # ---------------------- debug ----------------------
    
    def debug_loop(self):
        display_speed = self.abs_speed_meas * self.motor_direction

        self.get_logger().info(
            f"\r\n[DEBUG cmd_speed_controler] Cible: {self.target:.2f} | Mesuré: {display_speed:.2f} | Batterie: {self.vbat:.2f}V"
        )
        
        if self.debug:
            # --- Calcul des recommandations ---
            rec_gain = self.gain
            
            # 1. Gain recommandé (seulement si on bouge assez pour que ce soit fiable)
            if self.abs_speed_meas > 0.1 and abs(self.target) > 0.1:
                # On calcule quel gain aurait été parfait pour atteindre la cible
                rec_gain = (abs(self.target) * self.gain) / self.abs_speed_meas

            self.get_logger().info(
                f"\r\n[ACTUEL] Gain: {self.gain:.4f} | K_turn: {self.k_turn:.2f}                                                     "
                f"\r\n[CONSEIL] Mets Gain = {rec_gain:.4f} | CMD: {self.cmd:.5f}                                         "
                f"\r\nIntégrale: {self.integral:.3f} PID {self.PID_enabled}"
                f"\r\n--------------------------------------------------------"
            )

    # ---------------------- publish ----------------------

    def publish(self, value):
        msg = Float32()
        msg.data = float(value)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()