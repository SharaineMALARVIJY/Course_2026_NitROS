""" The node to command the direction """

# IMPORTS
import math
from dynamixel_sdk import PortHandler, PacketHandler

import rclpy
from rclpy.node import Node
from bolide_interfaces.msg import SpeedDirection
from std_msgs.msg import Float32


# =========================
# CALIBRATION PARAMETERS
# =========================

# --- Dynamixel ---
DXL_RESOLUTION = 1023.0 
DXL_FULL_RANGE_DEG = 300.0

# --- Direction limits ---
# MAX_STEERING_ANGLE_DEG = 22.0 #(en vrai c'est 15.5 mais pas le temps de trouver le probleme)

# --- Mechanical geometry ---
SERVO_ARM_LENGTH_MM = 15.0
STEERING_LINK_LENGTH_MM = 25.0
STEERING_OFFSET_MM = 4.35

# --- Servo zero calibration ---
SERVO_ZERO_DEG = 60.0 # (old 62.0) # car moteur code sur 300° => vertical à 150° = 60° + 90°
# L'angle moteur diminue côté axe de transmission

# --- Motor conversion ---
DXL_RAD_PER_UNIT = 5.24 / DXL_RESOLUTION
DXL_ZERO_RAD_OFFSET = 0.524


def pos2psi(pos):
    """
    Convert motor position (DXL units) to steering angle (rad)
    """
    theta_rad = pos * DXL_RAD_PER_UNIT + DXL_ZERO_RAD_OFFSET

    lever_projection = math.cos(theta_rad) * SERVO_ARM_LENGTH_MM - STEERING_OFFSET_MM
    psi_rad = math.asin(lever_projection / STEERING_LINK_LENGTH_MM)

    return psi_rad


class CommandDirection(Node):

    def degrees2pos(self,degrees):
        """
        Convert motor angle (deg) to DXL position
        """
        return int(
            (degrees + SERVO_ZERO_DEG)
            * (DXL_RESOLUTION / DXL_FULL_RANGE_DEG)
        )

    def set_dir_deg(self,steering_angle_deg):
        """
        Convert steering angle (deg) to motor position (DXL units)
        """
        psi_rad = math.radians(steering_angle_deg + self.STEERING_OFFSET_DEG)

        lever_projection = (
            STEERING_LINK_LENGTH_MM * math.sin(psi_rad)
            + STEERING_OFFSET_MM
        )

        theta_rad = math.acos(lever_projection / SERVO_ARM_LENGTH_MM)
        theta_deg = math.degrees(theta_rad)

        if self.debug:  # for debug
            self.get_logger().info(f"TARGET STEERING ANGLE : {steering_angle_deg:.3f}° -> TARGET MOTOR ANGLE (+offset): {theta_deg:.3f} ({theta_deg+SERVO_ZERO_DEG:.3f}) \r\n")

        return self.degrees2pos(theta_deg)

    def __init__(self):
        super().__init__('cmd_dir_node')

        # PARAMS
        self.declare_parameter('debug', False)

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        if self.debug:
            rclpy.logging.set_logger_level('cmd_dir', 10)  # 10 is for DEBUG level
        
        self.declare_parameter('steering_offset_deg', -1.5)
        self.STEERING_OFFSET_DEG = self.get_parameter('steering_offset_deg').value
        self.declare_parameter('max_steering_angle', 22.0)
        self.MAX_STEERING_ANGLE_DEG = self.get_parameter('max_steering_angle').value

        # Dynamixel stuff:
        # Protocol version
        self.PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID = 1
        self.declare_parameter('baudrate', 1000000)
        self.BAUDRATE = self.get_parameter('baudrate').value
        self.DEVICENAME = '/dev/ttyU2D2'
        # udev rule in /etc/udev/rules.d/99-usb-dynamixel.rules:
        # SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", SYMLINK+="ttyU2D2", MODE="0777"
        # see tutorials.md

        self.target_steering_angle_deg = 0.0
        self.curr_steering_angle_deg = 0.0

        self.MS = False

        # Setting the connection
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if self.portHandler.openPort():
            if self.debug:
                self.get_logger().info("[INFO] -- Succeeded to open the port\n\r")
        else:
            self.get_logger().error("[ERROR] -- Failed to open the port\n\r")
        self.portHandler.setPacketTimeoutMillis(10) # besoin pour éviter les erreurs de terminaison

        # Setting the baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            if self.debug:
                self.get_logger().info("[INFO] -- Succeeded to set the baudrate\n\r")
        else:
            self.get_logger().error("[ERROR] -- Failed to set the baudrate\n\r")

        # Subscription
        self.sub = self.create_subscription(Float32, "/cmd_dir", self.cmd_callback, 10)

        # Timer
        self.dynamixels_comms = self.create_timer(0.03, self.dxl_callback)  # Update the dynamixels every 30ms (33Hz)

    def dxl_callback(self):
        """Update the dynamixels.\
        We check if the target steering angle is within the limits of the steering angle
        """
        self.target_steering_angle_deg = max(min(self.target_steering_angle_deg, self.MAX_STEERING_ANGLE_DEG), -self.MAX_STEERING_ANGLE_DEG)
        try:
            pos, _, _ = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, 36)   # Read the current position of the steering servo
            self.curr_steering_angle_deg = -180/3.14159*pos2psi(pos)  # Convert the position to an angle in degrees
            pos = self.set_dir_deg(self.target_steering_angle_deg)  # Convert in DXL position

            if self.debug:  # for debug
                self.get_logger().debug(f"[DEBUG] -- DXL position : {pos}\r\n")

            self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, 30, pos)  # Sending position to servomotor
        except Exception as e:
            self.get_logger().warn("[WARNING] -- DYNAMIXEL PROBLEM")
            self.get_logger().warn(f"[WARNING] -- {e}")

    def cmd_callback(self, data):
        """The callback to update the command value

        Args:
            data (float): the direction in float between -1 and -1
        """

        if self.MS:
            self.target_steering_angle_deg = -data.data  # direction
        else:
            self.target_steering_angle_deg = -data.data * self.MAX_STEERING_ANGLE_DEG  # direction

        #self.get_logger().info(
        #    f"Target: {self.target_steering_angle_deg:.2f}°, "
        #    f"Current: {self.curr_steering_angle_deg:.2f}°"
        #)
        self.last_command_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    listener = CommandDirection()
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.get_logger().info("Shutting down")
        listener.portHandler.closePort()
        listener.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
