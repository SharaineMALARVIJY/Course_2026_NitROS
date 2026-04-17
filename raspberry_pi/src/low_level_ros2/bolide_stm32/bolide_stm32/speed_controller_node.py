import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16
from bolide_interfaces.msg import ForkSpeed

class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller_node')

        # --- Déclaration des paramètres ROS ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_speed_forward', 3.0),
                ('max_speed_reverse', 1.5),
                ('frequency', 50),
                ('PID_enabled', True),
                ('kp_fwd', 0.08),
                ('ki_fwd', 0.8),
                ('gain_fwd', 0.023),
                ('kp_rev', 0.1),
                ('ki_rev', 0.5),
                ('gain_rev', 0.02),
                ('integral_max', 0.3),
                ('info', True),       # Pour l'affichage minimal de statut
                ('calibrate', True),      # Pour l'affichage de calibration complet
                ('calibrate_freq', 2.0)
            ]
        )

        # --- Cache des paramètres ---
        self.pid_enabled_global = self.get_parameter('PID_enabled').value
        self.calibrate_enabled = self.get_parameter('calibrate').value
        self.info_enabled = self.get_parameter('info').value
        self.integral_max = self.get_parameter('integral_max').value
        
        freq = self.get_parameter('frequency').value
        self.DT = 1.0 / freq
        self.vnom = 8.4

        # Profils
        self.cfg_fwd = {
            'sign': 1.0, 'max': self.get_parameter('max_speed_forward').value,
            'gain': self.get_parameter('gain_fwd').value, 'min': 0.3,
            'kp': self.get_parameter('kp_fwd').value, 'ki': self.get_parameter('ki_fwd').value,
            'lim': (1.1e-4, 1.0)
        }
        self.cfg_rev = {
            'sign': -1.0, 'max': self.get_parameter('max_speed_reverse').value,
            'gain': self.get_parameter('gain_rev').value, 'min': 0.15,
            'kp': self.get_parameter('kp_rev').value, 'ki': self.get_parameter('ki_rev').value,
            'lim': (-1.0, -1.1e-4)
        }

        # --- État ---
        self.target = 0.0
        self.abs_speed_meas = 0.0
        self.vbat = self.vnom
        self.integral = 0.0
        self.motor_direction = 0.0
        self.pid_active_esc = True 
        self.cmd = 0.0
        self.current_gain = 0.0

        # --- Pub/Sub ---
        self.pub = self.create_publisher(Float32, '/cmd_vel', 10)
        self.create_subscription(Float32, '/cmd_speed_target', self.cb_target, 10)
        self.create_subscription(Float32, '/battery_voltage', self.cb_battery, 10)
        self.create_subscription(ForkSpeed, '/raw_fork_data', self.cb_speed, 10)
        self.create_subscription(Int16, '/esc_state', self.cb_esc_state, 10)

        # --- Timers ---
        self.create_timer(self.DT, self.loop)
        if self.info_enabled or self.calibrate_enabled:
            self.create_timer(1.0 / self.get_parameter('calibrate_freq').value, self.calibrate_loop)

    def cb_target(self, msg): self.target = msg.data
    def cb_battery(self, msg): self.vbat = msg.data
    def cb_speed(self, msg): self.abs_speed_meas = msg.speed
    def cb_esc_state(self, msg):
        state = msg.data
        self.motor_direction = 1.0 if state >= 0 else -1.0
        self.pid_active_esc = abs(state) < 2
        if not self.pid_active_esc: self.integral = 0.0

    def loop(self):
        v_target = self.target
        if self.vbat < 6.2:
            self.get_logger().warn(f"BAT LOW: {self.vbat:.1f}V", throttle_duration_sec=5)
            v_target = 0.0

        c = self.cfg_fwd if v_target >= 0 else self.cfg_rev
        self.current_gain = c['gain']
        abs_v_target = abs(v_target)

        if abs_v_target < c['min']:
            self.integral = 0.0
            self.publish(0.0)
            return

        cmd_ff = (abs_v_target / c['max']) * c['gain'] * (self.vnom / self.vbat)

        correction = 0.0
        if self.pid_enabled_global and self.pid_active_esc:
            error = abs_v_target - self.abs_speed_meas
            self.integral = max(-self.integral_max, min(self.integral_max, self.integral + error * self.DT))
            correction = (c['kp'] * error) + (c['ki'] * self.integral)

        self.cmd = max(c['lim'][0], min(c['lim'][1], (cmd_ff + correction) * c['sign']))
        self.publish(self.cmd)

    def publish(self, value):
        self.pub.publish(Float32(data=float(value)))

    def calibrate_loop(self):
        meas = self.abs_speed_meas * self.motor_direction
        
        # 1. AFFICHAGE STATUT (Compact)
        # Utile pour la surveillance générale
        if self.info_enabled:
            self.get_logger().info(f"STATUS | Tgt: {self.target:>5.2f} | Meas: {meas:>5.2f} | Bat: {self.vbat:.1f}V")

        # 2. AFFICHAGE CALIBRATION (Détaillé)
        # S'affiche seulement si calibrate est à True
        if self.calibrate_enabled:
            rec_gain = self.current_gain
            if self.abs_speed_meas > 0.1 and abs(self.target) > 0.1:
                rec_gain = (abs(self.target) * self.current_gain) / self.abs_speed_meas

            self.get_logger().info(
                f"calibrate CALIB | Cmd: {self.cmd:.4f} | Int: {self.integral:.3f}/{self.integral_max} | "
                f"Gain Actuel: {self.current_gain:.4f} -> CONSEIL: {rec_gain:.4f}"
            )

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