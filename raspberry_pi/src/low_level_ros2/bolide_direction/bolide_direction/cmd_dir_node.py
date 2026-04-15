import math
from dynamixel_sdk import PortHandler, PacketHandler

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


# =========================
# CALIBRATION
# =========================

DXL_RESOLUTION = 1023.0
DXL_FULL_RANGE_DEG = 300.0

SERVO_ARM_LENGTH_MM = 15.0
STEERING_LINK_LENGTH_MM = 25.0
STEERING_OFFSET_MM = 4.35

SERVO_ZERO_DEG = 60.0


class CommandDirection(Node):

    def degrees2pos(self, degrees):
        pos = int(
            (degrees + SERVO_ZERO_DEG)
            * (DXL_RESOLUTION / DXL_FULL_RANGE_DEG)
        )
        # 🔥 clamp sécurité Dynamixel
        return max(0, min(1023, pos))

    def compute_pos_from_angle(self, steering_angle_deg):
        psi_rad = math.radians(steering_angle_deg + self.STEERING_OFFSET_DEG)

        lever_projection = (
            STEERING_LINK_LENGTH_MM * math.sin(psi_rad)
            + STEERING_OFFSET_MM
        )

        ratio = lever_projection / SERVO_ARM_LENGTH_MM

        # 🔥 filtre physique STRICT
        if ratio < -1.0 or ratio > 1.0:
            return None

        theta_rad = math.acos(ratio)
        theta_deg = math.degrees(theta_rad)

        return self.degrees2pos(theta_deg)

    def build_lut(self):
        self.lut = {}

        for a in range(-30, 31):  # large plage volontaire
            pos = self.compute_pos_from_angle(a)

            if pos is not None:
                self.lut[a] = pos

        # 🔥 bornes valides
        self.valid_min = min(self.lut.keys())
        self.valid_max = max(self.lut.keys())

        self.get_logger().info(
            f"LUT built: valid range = [{self.valid_min}, {self.valid_max}]"
        )

    def get_safe_pos(self, angle):
        angle_int = int(round(angle))

        # clamp dans domaine valide LUT
        if angle_int < self.valid_min:
            angle_int = self.valid_min
        elif angle_int > self.valid_max:
            angle_int = self.valid_max

        return self.lut[angle_int]

    def __init__(self):
        super().__init__('cmd_dir_node')

        # PARAMS
        self.declare_parameter('steering_offset_deg', -1.5)
        self.declare_parameter('max_steering_angle', 20.0)
        self.declare_parameter('baudrate', 1000000)

        self.STEERING_OFFSET_DEG = self.get_parameter('steering_offset_deg').value
        self.MAX_STEERING_ANGLE_DEG = self.get_parameter('max_steering_angle').value
        self.BAUDRATE = self.get_parameter('baudrate').value

        # Dynamixel
        self.PROTOCOL_VERSION = 1.0
        self.DXL_ID = 1
        self.DEVICENAME = '/dev/ttyU2D2'

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open port")

        self.portHandler.setPacketTimeoutMillis(20)

        if not self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().error("Failed baudrate")

        # STATE
        self.target_angle = 0.0
        self.last_sent_angle = None

        # LUT SAFE
        self.build_lut()

        # ROS
        self.sub = self.create_subscription(
            Float32, "/cmd_dir", self.cmd_callback, 10
        )

        self.timer = self.create_timer(0.05, self.dxl_callback)  # 20 Hz

    def dxl_callback(self):

        # clamp user
        angle = max(
            min(self.target_angle, self.MAX_STEERING_ANGLE_DEG),
            -self.MAX_STEERING_ANGLE_DEG
        )

        # skip si stable
        if self.last_sent_angle is not None and abs(angle - self.last_sent_angle) < 0.5:
            return

        pos = self.get_safe_pos(angle)

        try:
            # ⚡ rapide
            self.packetHandler.write2ByteTxOnly(
                self.portHandler, self.DXL_ID, 30, pos
            )
            self.last_sent_angle = angle

        except Exception as e:
            self.get_logger().warn(f"Dynamixel error: {e}")

    def cmd_callback(self, data):
        self.target_angle = -data.data * self.MAX_STEERING_ANGLE_DEG


def main(args=None):
    rclpy.init(args=args)
    node = CommandDirection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.portHandler.closePort()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()