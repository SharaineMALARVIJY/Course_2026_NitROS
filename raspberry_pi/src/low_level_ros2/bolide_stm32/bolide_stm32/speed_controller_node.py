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
        self.declare_parameter('debug_freq', 2.0)  # Hz

        self.max_fwd = self.get_parameter('max_speed_forward').value
        self.max_rev = self.get_parameter('max_speed_reverse').value

        freq = self.get_parameter('frequency').value
        self.debug = self.get_parameter('debug').value
        self.debug_freq = self.get_parameter('debug_freq').value
        self.PID_enabled = self.get_parameter('PID_enabled').value

        self.k_turn = 0.0
        self.vnom = 8.4
        self.min_speed_fwd = 0.3
        self.min_speed_rev = 0.15
        self.alpha_vbat = 0.01
        self.alpha_speed_meas = 0.05
        self.integral_max = 0.3 #0.1
        self.KP_rev = 0.1              # facteur de correction P
        self.KI_rev = 0.2
        self.KP_fw = 0.1 #0.01   # 0.05
        self.KI_fw = 0.2 #0.05   # 0.1
        self.gain_fwd = 0.05 # 0.15
        self.offset_fwd = 0.0
        self.gain_rev = 0.2  # 0.3
        self.offset_rev = 0.0

        # # --- Cas Marche Avant (FWD) ---
        # self.gain_fwd, self.offset_fwd = self.calculate_motor_model(
        #     v1=1.0, g1=0.2,   # Point 1 fwd
        #     v2=2.5, g2=0.18,  # Point 2 fwd
        #     v_max=self.max_fwd
        # )

        # # --- Cas Marche Arrière (REV) ---
        # self.gain_rev, self.offset_rev = self.calculate_motor_model(
        #     v1=0.4, g1=0.15,  # Point 1 rev
        #     v2=0.7, g2=0.12,  # Point 2 rev
        #     v_max=self.max_rev
        # )

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


        if self.debug:
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
            self.offset = self.offset_fwd
            self.min_speed = self.min_speed_fwd
            self.KI = self.KI_fw
            self.KP = self.KP_fw
            cmd_min = 1.1e-4
            cmd_max = 1.0
        else :
            sign = -1.0
            self.max_speed = self.max_rev
            self.gain = self.gain_rev
            self.offset = self.offset_rev
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
            cmd_ff = (abs(v_target) / self.max_speed) * self.gain + self.offset
            
            # Compensation batterie
            cmd_ff *= (self.vnom / self.vbat)
            
            # Compensation virage
            #cmd_ff *= (1.0 + self.k_turn * (self.turn ** 2))
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
        
        # --- Calcul des recommandations ---
        rec_gain = self.gain
        rec_k_turn = self.k_turn
        
        # 1. Gain recommandé (seulement si on bouge assez pour que ce soit fiable)
        if self.abs_speed_meas > 0.1 and abs(self.target) > 0.1:
            # On calcule quel gain aurait été parfait pour atteindre la cible
            rec_gain = (abs(self.target) * self.gain) / self.abs_speed_meas
            
        # 2. K_turn recommandé (seulement si on tourne significativement)
        if self.turn > 0.1 and self.abs_speed_meas > 0.1:
            # On regarde de combien on est en dessous de la cible à cause du virage
            ratio = abs(self.target) / self.abs_speed_meas
            if ratio > 1.0:
                # On manque de vitesse en virage -> on calcule le k_turn nécessaire
                rec_k_turn = (ratio - 1.0) / (self.turn ** 2)
            elif ratio < 1.0 - 0.01:
                # On va déjà trop vite -> on suggère de baisser k_turn vers 0
                rec_k_turn = 0.0

        self.get_logger().info(
            f"\r\n[DEBUG cmd_speed_controler] Cible: {self.target:.2f} | Mesuré: {display_speed:.2f} | CMD: {self.cmd:.5f}"
            f"\r\n[ACTUEL] Gain: {self.gain:.4f} | K_turn: {self.k_turn:.2f}                                                     "
            f"\r\n[CONSEIL] Mets Gain = {rec_gain:.4f} | Mets K_turn = {rec_k_turn:.2f}                                          "
            f"\r\nBatterie: {self.vbat:.2f}V | Intégrale: {self.integral:.3f} PID {self.PID_enabled}"
            f"\r\n--------------------------------------------------------"
        )

    # ---------------------- publish ----------------------

    def publish(self, value):
        msg = Float32()
        msg.data = float(value)
        self.pub.publish(msg)


    def calculate_motor_model(self, v1, g1, v2, g2, v_max):
        """
        Calcule l'Offset et le Gain pur à partir de deux points expérimentaux.
        v1, v2 : Vitesses cibles (m/s)
        g1, g2 : Gains relevés (ceux de ton ancien paramètre ROS)
        v_max  : La vitesse max définie dans ton paramètre (ex: 1.5 pour rev)
        """
        # 1. Conversion des Gains ROS en commandes PWM réelles (0.0 à 1.0)
        u1 = (v1 / v_max) * g1
        u2 = (v2 / v_max) * g2

        # 2. Calcul de la pente (Gain dynamique pur)
        # Formule : a = (y2 - y1) / (x2 - x1)
        gain_pure = (u2 - u1) / (v2 - v1)

        # 3. Calcul de l'ordonnée à l'origine (Offset de friction)
        # Formule : b = y1 - a * x1
        offset = u1 - (gain_pure * v1)

        # 4. Conversion du gain_pure en "Gain_norm" pour rester indépendant de max_speed
        # gain_ros = gain_pure * v_max
        gain_norm = gain_pure * v_max

        return gain_norm, offset


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