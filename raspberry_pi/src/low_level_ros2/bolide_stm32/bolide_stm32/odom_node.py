import math
import rclpy
from rclpy.node import Node
from bolide_interfaces.msg import ForkSpeed
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion

class ForkYawOdom(Node):
    MAX_OMEGA   = 3.0    # (rad/s) Clamp physique
    MAX_DT = 0.2        
    YAW_TIMEOUT = 0.5 
    ALPHA_OMEGA = 0.4

    def __init__(self):
        super().__init__("fork_yaw_odom")

        self.declare_parameter("publish_tf", True)
        self.publish_tf = self.get_parameter("publish_tf").value

        # --- État interne ---
        self.x     = 0.0
        self.y     = 0.0
        self.v     = 0.0      # Vitesse linéaire (lissée par stm32_node)
        self.omega = 0.0      # Vitesse angulaire
        self.yaw   = 0.0      # Orientation actuelle
        self.last_yaw_odom = 0.0 # Pour l'intégration trapézoïdale
        self.current_q = Quaternion()
        self.esc_state = 0
        self.first_yaw_received = False

        self.last_yaw = None
        self.last_imu_time = self.get_clock().now()

        # --- Timestamps ---
        self.last_update_time = self.get_clock().now()
        self.last_imu_time    = self.get_clock().now()

        # --- Messages pré-remplis ---
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"
        
        # Covariances fixes
        # --- Covariances Pose (Sécurisées pour Nav2) ---
        p_cov = [0.0] * 36
        p_cov[0]  = 0.3  # Certitude X
        p_cov[7]  = 0.3  # Certitude Y
        p_cov[14] = 1e6  # Z (Invalide)
        p_cov[21] = 1e6  # Roll (Invalide)
        p_cov[28] = 1e6  # Pitch (Invalide)
        p_cov[35] = 0.02 # Yaw (Très précis grâce au BNO055)
        self.odom_msg.pose.covariance = p_cov

        # --- Covariances Twist ---
        t_cov = [0.0] * 36
        t_cov[0]  = 0.1
        t_cov[7]  = 0.1
        t_cov[14] = 1e6
        t_cov[21] = 1e6
        t_cov[28] = 1e6
        t_cov[35] = 0.05
        self.odom_msg.twist.covariance = t_cov

        # --- Pub / Sub ---
        self.create_subscription(ForkSpeed, "/raw_fork_data", self.on_speed, 10)
        self.create_subscription(Imu, "/imu", self.on_imu, 10)
        self.create_subscription(Int16, '/esc_state', self.esc_state_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.publish_odom)
        self.get_logger().info("Odométrie lancée")

    def esc_state_callback(self, msg):
        self.esc_state = msg.data

    def on_speed(self, msg: ForkSpeed):
        self.v = float(msg.speed) if self.esc_state >= 0 else -float(msg.speed)

    def on_imu(self, msg: Imu):
            self.current_q = msg.orientation
            # Conversion quaternion vers yaw
            _, _, self.yaw = euler_from_quaternion([self.current_q.x, self.current_q.y, self.current_q.z, self.current_q.w])

            # initialisation
            if not self.first_yaw_received:
                self.last_yaw_odom = self.yaw
                self.last_yaw = self.yaw
                self.first_yaw_received = True

            now = self.get_clock().now()
            dt = (now - self.last_imu_time).nanoseconds * 1e-9
            
            # Calcul de l'omega par dérivation
            if self.last_yaw is not None and 0.001 < dt < 0.2:
                delta_yaw = self.yaw - self.last_yaw
                # Gestion du saut entre -pi et pi
                if delta_yaw > math.pi: delta_yaw -= 2.0 * math.pi
                elif delta_yaw < -math.pi: delta_yaw += 2.0 * math.pi
                
                raw_omega = delta_yaw / dt
                # Clamp physique
                if abs(raw_omega) > self.MAX_OMEGA:
                    raw_omega = 0.0
                    self.get_logger().warn(
                        f"Omega aberrant ignoré ({raw_omega:.2f} rad/s)",
                        throttle_duration_sec=2.0
                    )
                self.omega = (self.ALPHA_OMEGA * raw_omega) + (1.0 - self.ALPHA_OMEGA) * self.omega
            
            self.last_yaw = self.yaw
            self.last_imu_time = now

    def publish_odom(self):
        if not self.first_yaw_received: return

        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        # Sécurité timing
        imu_age = (now - self.last_imu_time).nanoseconds * 1e-9
        if imu_age > self.YAW_TIMEOUT or dt <= 0.0 or dt > self.MAX_DT: return

        # --- Intégration Trapézoïdale ---
        # On utilise la moyenne de l'angle sur le pas de temps pour plus de précision
        avg_yaw = self.last_yaw_odom + (self.yaw - self.last_yaw_odom) * 0.5
        self.x += self.v * math.cos(avg_yaw) * dt
        self.y += self.v * math.sin(avg_yaw) * dt
        self.last_yaw_odom = self.yaw

        # --- Remplissage du message ---
        self.odom_msg.header.stamp = now.to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation = self.current_q 
        self.odom_msg.twist.twist.linear.x = self.v
        self.odom_msg.twist.twist.angular.z = self.omega

        self.odom_pub.publish(self.odom_msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header = self.odom_msg.header
            t.child_frame_id = self.odom_msg.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = self.current_q
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ForkYawOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
