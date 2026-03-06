""" The node to command the speed of the car """

# IMPORTS
import math
from dynamixel_sdk import PortHandler, PacketHandler

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from bolide_interfaces.msg import SpeedDirection
from std_msgs.msg import Float32 


def get_sign(val):
    """get the sign of a value

    Args:
        val (any): any int or float value

    Returns:
        int: 1 or -1 if positive or negative
    """
    return (val > 0) - (val < 0)


class CommandSpeedNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_node')

        self.init = 0

        # PARAMS
        self.declare_parameter('debug', False)
        self.declare_parameter('minimal_speed', 8.3)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        if self.debug:
            rclpy.logging.set_logger_level('cmd_vel_node', 10)  # 10 is for DEBUG level

        self.MAXSPEED = 10
        self.MINSPEED = self.get_parameter('minimal_speed').get_parameter_value().double_value
        self.NEUTRAL = 8.0
        self.REVERSEMINSPEED = 7.7
        self.REVERSEMAXSPEED = 6.5
        self.DIR_VEL_THRESHOLD_M_S = 0.05
        self.curr_velocity_m_s = 0.0

        self.esc_period = 20000  # ns

        self.curr_dir = 1
        self.tx_data = Int16()

        self.safety_timeout = 0.5
        self.last_command_time = self.get_clock().now()

        self.sub = self.create_subscription(Float32, "/cmd_vel", self.cmd_callback, 10)
        self.stm32_publish = self.create_publisher(Int16, "/stm32_data", 10)

        # Unique Timer for the security
        self.timer_safety = self.create_timer(1.0, self.emergency_stop, autostart=False)

        self.init = 1

    def cmd_callback(self, data):
        self.timer_safety.cancel()  # cancel the brake timer
        self.cmd_velocity = data.data
        if not get_sign(self.cmd_velocity) == self.curr_dir:
            if (not self.curr_dir) or (abs(self.curr_velocity_m_s) < self.DIR_VEL_THRESHOLD_M_S):

                # We can command reverse while the car is going forwards
                # However, the car needs to stop and then start reversing before it actually goes in reverse.
                # So we only switch the sign if the car is going slow enough to be considered "stationnary".
                self.curr_dir = get_sign(self.cmd_velocity)

        # Update the last command time and reset the brake timer
        self.timer_safety.reset()
        self.command(self.cmd_velocity)

    def emergency_stop(self):
        self.get_logger().warn("Emergency stop triggered due to timeout.")
        self.command(0.0)

    def command(self, cmd_speed):
        if 1 > cmd_speed >= 0.01:
            self.forward()

        elif -0.01 < cmd_speed < 0.01:
            self.neutral()

        elif -1 < cmd_speed < -0.01:
            self.reverse()

    def forward(self):
        self.publish_stm32_data(self.MINSPEED + self.cmd_velocity * (self.MAXSPEED - self.MINSPEED))
        #self.get_logger().info(f"speed is :{self.MINSPEED}")

    def reverse(self):
        speed_ratio = abs(self.cmd_velocity)
        self.publish_stm32_data(self.REVERSEMINSPEED + speed_ratio * (self.REVERSEMAXSPEED - self.REVERSEMINSPEED))

    def neutral(self):
        self.publish_stm32_data(self.NEUTRAL)

    def publish_stm32_data(self, cycle_ratio):
        """Send to stm32 the cycle_ration of the motors

        Args:
            cycle_ratio (float): the cycle of the cars (netural, reverse, forward,...)
        """
        if self.init:
            self.tx_data = Int16()
            self.tx_data.data = int(cycle_ratio*0.00938 * self.esc_period)

        # The previous implementation used RPi PWM which was unreliable.
            # Experiments showed the RPi to overshoot duration by 1.066.

        # Cyclic ratio is the time of the period spent high. So 100 would be constantly high, 50 would be half high half low, etc.
        # The esc period is 20000 ns, and we send the actual pulse duration to the stm32. We also convert from RPi to "true".

        if rclpy.ok() and self.debug:
            self.get_logger().debug(f"[DEBUG] -- Tx_data = {self.tx_data}")
        self.stm32_publish.publish(self.tx_data)


def main(args=None):
    rclpy.init(args=args)
    listener = CommandSpeedNode()
    try:
        rclpy.spin(listener)
    except Exception as e:
        print(f"Error in Command Speed : {e}")


if __name__ == '__main__':
    main()
