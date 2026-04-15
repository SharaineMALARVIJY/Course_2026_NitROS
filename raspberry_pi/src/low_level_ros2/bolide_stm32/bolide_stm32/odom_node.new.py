import math
import rclpy
from rclpy.node import Node
from typing import Optional

from bolide_interfaces.msg import ForkSpeed
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import Imu

class ForkYawOdom(Node):
    def __init__(self):
        super().__init__("fork_yaw_odom")

        # --- Paramètres ROS2 ---
        self.declare_parameter("debug", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("invert_yaw", False)
        self.declare_parameter("yaw_offset", 0.0)

        self.debug = self.get_parameter("debug").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.invert_yaw = self.get_parameter("invert_yaw").value
        self.yaw_offset = self.get_parameter("yaw_offset").value

        # --- État interne ---
        self.x, self.y, self.v, self.omega = 0.0, 0.0, 0.0, 0.0
        self.yaw: Optional[float] = None
        self.esc_state = 0

        # --- Timestamps ---
        now = self.get_clock().now()
        self.last_update_time = now
        self.last_sensor_time = now

        # --- Pub / Sub ---
        self.create_subscription(ForkSpeed, "/raw_fork_data", self.on_speed, 10)
        self.create_subscription(Imu, "/raw_imu_data", self.on_imu, 10)
        self.create_subscription(Int16, '/esc_state', self.esc_state_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Odométrie IMU corrigée lancée")

        self.timer = self.create_timer(1.0 / 50.0, self.publish_odom)
        
    def on_imu(self, msg: Imu):
        now = self.get_clock().now()
        
        # 1. Récupération du Yaw Rate (Z) envoyé par le STM32
        yaw_rate_stm32 = msg.angular_velocity.z 

        # 2. Conversion Quaternion -> Yaw (Lacet)
        siny_cosp = 2 * (msg.orientation.w * msg.orientation.z)
        cosy_cosp = 1 - 2 * (msg.orientation.z * msg.orientation.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 3. Application des paramètres (Inversion / Offset)
        adjusted_yaw = current_yaw + self.yaw_offset
        if self.invert_yaw:
            adjusted_yaw = -adjusted_yaw
        
        # Normalisation [-pi, pi]
        new_yaw = math.atan2(math.sin(adjusted_yaw), math.cos(adjusted_yaw))

        # 4. Exécution du Debug (Comparaison avant mise à jour de self.yaw)
        if self.debug:
            self.debug_IMU(new_yaw, yaw_rate_stm32, now)

        # 5. Mise à jour de l'état interne
        self.omega = yaw_rate_stm32 # On utilise la valeur STM32 pour l'odom
        self.yaw = new_yaw
        self.last_sensor_time = now

    def debug_IMU(self, new_yaw, yaw_rate_stm, now):
        dt = (now - self.last_sensor_time).nanoseconds * 1e-9

        if self.yaw is not None and dt > 0:
            delta = new_yaw - self.yaw
            
            # Correction du passage de ligne de coupure pi / -pi
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi

            omega_calculated = delta / dt

            # Log de comparaison (toutes les 0.5 secondes pour lisibilité)
            self.get_logger().info(
                f"Comparaison Omega - STM32: {yaw_rate_stm:6.3f} | Calculé: {omega_calculated:6.3f} | Diff: {abs(yaw_rate_stm - omega_calculated):6.3f}",
                throttle_duration_sec=0.5
            )

    def esc_state_callback(self, msg):
        self.esc_state = msg.data

    def on_speed(self, msg: ForkSpeed):
        if self.esc_state < 0:
            self.v = -float(msg.speed)
        else:
            self.v = float(msg.speed)

    def publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if self.yaw is None:
            return

        yaw_age = (now - self.last_sensor_time).nanoseconds * 1e-9
        if yaw_age > self.YAW_TIMEOUT:
            return

        if dt <= 0.0 or dt > self.MAX_DT:
            return

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        q = quaternion_from_euler(0.0, 0.0, self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega

        # Covariances Pose
        p_cov = [0.0] * 36
        p_cov[0]  = 0.3
        p_cov[7]  = 0.3
        p_cov[14] = 1e6
        p_cov[21] = 1e6
        p_cov[28] = 1e6
        p_cov[35] = 0.05
        odom.pose.covariance = p_cov

        # Covariances Twist
        t_cov = [0.0] * 36
        t_cov[0]  = 0.1
        t_cov[7]  = 0.1
        t_cov[14] = 1e6
        t_cov[21] = 1e6
        t_cov[28] = 1e6
        t_cov[35] = 0.05
        odom.twist.covariance = t_cov

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = odom.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
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
