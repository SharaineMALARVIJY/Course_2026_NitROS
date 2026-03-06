import math
import rclpy
from rclpy.node import Node
from typing import Optional

from bolide_interfaces.msg import ForkSpeed
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


class ForkYawOdom(Node):
    """
    Odométrie ROS2 SLAM-ready pour robot à fourche.

    Corrections appliquées :
      - Filtre passe-bas sur omega
      - Covariances réalistes
      - Protection contre les dt aberrants
      - Intégration uniquement si yaw récent (< 500 ms)
      - Clamp sur omega
      - Correction repère X/Y : décalage de -π/2 sur le yaw brut
    """

    OMEGA_ALPHA = 0.15   # Lissage omega : 0=figé, 1=brut
    MAX_DT      = 0.5    # (s) Saut temporel max avant de rejeter l'intégration
    MAX_OMEGA   = 3.0    # (rad/s) Clamp physique
    YAW_TIMEOUT = 0.5    # (s) Si pas de yaw depuis X s → on n'intègre plus

    def __init__(self):
        super().__init__("fork_yaw_odom")

        # --- Paramètres ROS2 ---
        self.declare_parameter("yaw_index",   0)
        self.declare_parameter("publish_tf",  True)
        self.declare_parameter("invert_yaw",  False)

        self.yaw_index  = self.get_parameter("yaw_index").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.invert_yaw = self.get_parameter("invert_yaw").value

        # --- État interne ---
        self.x     = 0.0
        self.y     = 0.0
        self.v     = 0.0
        self.omega = 0.0
        self.yaw: Optional[float] = None

        # --- Timestamps ---
        self.last_update_time = self.get_clock().now()
        self.last_sensor_time = self.get_clock().now()

        # --- Pub / Sub ---
        self.create_subscription(ForkSpeed,         "/raw_fork_data", self.on_speed,   10)
        self.create_subscription(Float32MultiArray, "/stm32_sensors", self.on_sensors, 10)

        self.odom_pub       = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f"Odométrie SLAM-Ready lancée  "
            f"[yaw_index={self.yaw_index} | publish_tf={self.publish_tf} | "
            f"invert_yaw={self.invert_yaw}]"
        )

    # ------------------------------------------------------------------ #
    #  Callback capteur IMU / encodeur angulaire                          #
    # ------------------------------------------------------------------ #
    def on_sensors(self, msg: Float32MultiArray):
        if len(msg.data) <= self.yaw_index:
            self.get_logger().warn(
                f"on_sensors: index {self.yaw_index} hors limites "
                f"(taille={len(msg.data)})", throttle_duration_sec=5.0
            )
            return

        now       = self.get_clock().now()
        dt_sensor = (now - self.last_sensor_time).nanoseconds * 1e-9

        # --- 1. Conversion du yaw brut ---
        # Capteur envoie [-2π, 0]
        # On ajoute 2π pour ramener à [0, 2π]
        # On soustrait π/2 pour corriger le décalage de repère X/Y
        raw_val      = float(msg.data[self.yaw_index])
        adjusted_yaw = raw_val + (2.0 * math.pi) - (math.pi / 2.0)
        new_yaw      = math.atan2(math.sin(adjusted_yaw), math.cos(adjusted_yaw))

        if self.invert_yaw:
            new_yaw = -new_yaw

        # --- 2. Calcul de omega (vitesse angulaire) ---
        if self.yaw is not None and 0.0 < dt_sensor < self.MAX_DT:
            delta_yaw = new_yaw - self.yaw

            # Gérer le passage ±π
            if delta_yaw >  math.pi: delta_yaw -= 2.0 * math.pi
            if delta_yaw < -math.pi: delta_yaw += 2.0 * math.pi

            raw_omega = delta_yaw / dt_sensor

            # Clamp physique
            if abs(raw_omega) > self.MAX_OMEGA:
                raw_omega = 0.0
                self.get_logger().warn(
                    f"Omega aberrant ignoré ({raw_omega:.2f} rad/s)",
                    throttle_duration_sec=2.0
                )

            # Filtre passe-bas exponentiel
            self.omega = (self.OMEGA_ALPHA * raw_omega
                          + (1.0 - self.OMEGA_ALPHA) * self.omega)

        self.yaw              = new_yaw
        self.last_sensor_time = now

    # ------------------------------------------------------------------ #
    #  Callback vitesse linéaire                                          #
    # ------------------------------------------------------------------ #
    def on_speed(self, msg: ForkSpeed):
        now = self.get_clock().now()
        dt  = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        # --- Gardes de sécurité ---
        if self.yaw is None:
            return

        yaw_age = (now - self.last_sensor_time).nanoseconds * 1e-9
        if yaw_age > self.YAW_TIMEOUT:
            self.get_logger().warn(
                f"Yaw trop vieux ({yaw_age:.2f}s) – intégration suspendue",
                throttle_duration_sec=2.0
            )
            return

        if dt <= 0.0 or dt > self.MAX_DT:
            self.get_logger().warn(
                f"dt aberrant ignoré ({dt:.3f}s)", throttle_duration_sec=2.0
            )
            return

        # --- 1. Intégration de la position ---
        self.v  = float(msg.speed)
        self.x -= self.v * math.cos(self.yaw) * dt
        self.y -= self.v * math.sin(self.yaw) * dt

        # --- 2. Message Odometry ---
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = self._euler_to_quaternion(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation = q

        # Covariances Pose
        p_cov     = [0.0] * 36
        p_cov[0]  = 0.3    # Variance X   (m²)
        p_cov[7]  = 0.3    # Variance Y   (m²)
        p_cov[35] = 0.05   # Variance Yaw (rad²)
        p_cov[14] = 1e6    # Z  (robot planaire)
        p_cov[21] = 1e6    # Roll
        p_cov[28] = 1e6    # Pitch
        odom.pose.covariance = p_cov

        # Covariances Twist
        t_cov     = [0.0] * 36
        t_cov[0]  = 0.1    # Vitesse linéaire  (m/s)²
        t_cov[35] = 0.05   # Vitesse angulaire (rad/s)²
        odom.twist.covariance = t_cov

        odom.twist.twist.linear.x  = self.v
        odom.twist.twist.angular.z = self.omega

        self.odom_pub.publish(odom)

        # --- 3. TF odom → base_footprint ---
        if self.publish_tf:
            t                         = TransformStamped()
            t.header                  = odom.header
            t.child_frame_id          = odom.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation      = q
            self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------ #
    #  Utilitaires                                                        #
    # ------------------------------------------------------------------ #
    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        cy, sy = math.cos(yaw   * 0.5), math.sin(yaw   * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll  * 0.5), math.sin(roll  * 0.5)
        return Quaternion(
            x = sr * cp * cy - cr * sp * sy,
            y = cr * sp * cy + sr * cp * sy,
            z = cr * cp * sy - sr * sp * cy,
            w = cr * cp * cy + sr * sp * sy,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ForkYawOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
