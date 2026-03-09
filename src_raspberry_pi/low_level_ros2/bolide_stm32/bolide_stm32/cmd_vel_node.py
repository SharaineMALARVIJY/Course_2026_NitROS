import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32

# équivalent de l'ancien code
# pour Forward -> Reverse : faire Forward -> Brake ('s') -> Neutral ('n') -> Reverse 
# (attention : les transitions doivent avoir un délai : 150ms)
# machine à état cmd_vel_node en cours de construction (la séquence exacte a pris du temp à trouver)

class CommandSpeedNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_node')

        # PARAMETRES
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').value

        # # rétrocompatiblilité avec l'ancien calibrage ESC
        # ## NEW_FORWARD_MIN
        # NEUTRAL = 1500
        # OLD_FORWARD_MIN = 1532  #bolide1 : 1557 (conversion : value_centered_on_8 * 0.00938 * 20000)
        # OLD_FORWARD_MAX = 1876  #bolide1 : 1444
        # FORWARD_RATIO = (OLD_FORWARD_MIN - NEUTRAL)/(OLD_FORWARD_MAX - NEUTRAL)
        # NEW_FORWARD_MIN = NEUTRAL + FORWARD_RATIO * NEUTRAL
        # ## NEW_REVERSE_MIN
        # OLD_REVERSE_MIN = 1468
        # OLD_REVERSE_MAX = 1220
        # REVERSE_RATIO = (NEUTRAL - OLD_REVERSE_MIN)/(NEUTRAL - OLD_REVERSE_MAX)
        # NEW_REVERSE_MIN = NEUTRAL - REVERSE_RATIO * NEUTRAL

        # calibrage actuel = ancien
        # PWM en µs (deadzone ESC : 1500 ±40 µs)            # indépendant de la calibration ESC
        self.PWM_FORWARD_MAX = 1876 #new 2000
        self.PWM_FORWARD_MIN = 1532 #new 1628               # old => 1500+4%(Max-Min) -> 1546 (>1540)
        self.PWM_NEUTRAL = 1500
        self.PWM_REVERSE_MIN = 1468 #new 1329               # old => 1500-4%(Max-Min) -> 1453 (<1460)
        self.PWM_REVERSE_MAX = 1220 #new 1000

        self.CMD_VEL_DEADZONE = 0.01                        # deadzone de cmd_vel
        self.SAFETY_TIMEOUT = 0.5                           # temps max sans commande avant neutre

        # Pub / Sub
        self.publisher = self.create_publisher(Int16, "/stm32_data", 10)
        self.subscriber = self.create_subscription(Float32, "/cmd_vel", self.cmd_callback, 10)

        # Armement ESC au démarrage
        self.publish_pwm(self.PWM_NEUTRAL)

        # Timer sécurité
        self.timer_safety = self.create_timer(self.SAFETY_TIMEOUT,self.emergency_stop)

    # CALLBACK CMD_VEL
    def cmd_callback(self, msg):
        self.timer_safety.reset()
        cmd_vel = msg.data

        if cmd_vel == -2:
            self.publish_pwm(self.PWM_REVERSE_MAX)
            return

        cmd_vel = max(min(cmd_vel, 1.0), -1.0)

        if 1 >= cmd_vel >= self.CMD_VEL_DEADZONE:
            self.forward(cmd_vel)

        elif -self.CMD_VEL_DEADZONE < cmd_vel < self.CMD_VEL_DEADZONE:
            self.neutral()

        elif -1 <= cmd_vel <= -self.CMD_VEL_DEADZONE:
            self.reverse(cmd_vel)

    def forward(self, cmd_vel):
        # cmd_vel > 0
        pwm = int(self.PWM_FORWARD_MIN + cmd_vel * (self.PWM_FORWARD_MAX - self.PWM_FORWARD_MIN))
        self.publish_pwm(pwm)

    def reverse(self, cmd_vel):
        # cmd_vel < 0
        pwm = int(self.PWM_REVERSE_MIN + cmd_vel * (self.PWM_REVERSE_MIN - self.PWM_REVERSE_MAX))
        self.publish_pwm(pwm)

    def neutral(self):
        self.publish_pwm(self.PWM_NEUTRAL)

    # ------------------------------
    # SECURITE
    # ------------------------------
    def emergency_stop(self):
        self.get_logger().warn("Emergency brake triggered : '/cmd_vel' signal timeout")
        self.publish_pwm(self.PWM_NEUTRAL)
        self.timer_safety.cancel()

    # ------------------------------
    # PUBLICATION STM32
    # ------------------------------
    def publish_pwm(self, pwm_value):
        self.tx_data = Int16()
        self.tx_data.data = pwm_value
        if self.debug:
            self.get_logger().info(f"PWM sent: {pwm_value}")
        self.publisher.publish(self.tx_data)


def main(args=None):
    rclpy.init(args=args)
    node = CommandSpeedNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()