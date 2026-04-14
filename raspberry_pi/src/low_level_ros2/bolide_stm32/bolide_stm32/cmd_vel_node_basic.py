import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32

# équivalent de l'ancien code
# version simplifiée sans machine à états
# brake pas implémenté (toujours Neutre) : nécessite state_machine

class CommandSpeedNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_node')

        # PARAMETRES
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').value

        # PWM en µs (deadzone ESC : 1500 ±40 µs)            # indépendant de la calibration ESC
        self.declare_parameter('pwm_forward_max', 2000) # old: 1876
        self.declare_parameter('pwm_forward_min', 1540) # old: 1557
        self.declare_parameter('pwm_reverse_min', 1460) # old: 1444
        self.declare_parameter('pwm_reverse_max', 1000) # old: 1220
        
        self.PWM_FORWARD_MAX = self.get_parameter('pwm_forward_max').value
        self.PWM_FORWARD_MIN = self.get_parameter('pwm_forward_min').value
        self.PWM_NEUTRAL = 1500
        self.PWM_REVERSE_MIN = self.get_parameter('pwm_reverse_min').value
        self.PWM_REVERSE_MAX = self.get_parameter('pwm_reverse_max').value

        # deadzone de cmd_vel
        self.declare_parameter('cmd_vel_deadzone', 0.01)
        self.CMD_VEL_DEADZONE = self.get_parameter('cmd_vel_deadzone').value - 1e-6 # 1e-6 car de problème de get_parameter sur float
        # temps max sans commande avant neutre    
        self.SAFETY_TIMEOUT = 0.5

        # Pub
        self.publisher = self.create_publisher(Int16, "/stm32_data", 10)

        # Armement ESC au démarrage
        self.publish_pwm(self.PWM_NEUTRAL)

        # Timer sécurité
        self.timer_safety = self.create_timer(self.SAFETY_TIMEOUT,self.emergency_stop)

        # Sub
        self.subscriber = self.create_subscription(Float32, "/cmd_speed", self.cmd_callback, 10)

        

    # CALLBACK CMD_VEL
    def cmd_callback(self, msg):
        self.timer_safety.reset()
        cmd_vel = msg.data

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
        self.get_logger().warn("Emergency brake triggered : '/cmd_vel' signal timeout\n")
        self.publish_pwm(self.PWM_NEUTRAL)
        self.timer_safety.cancel()

    # ------------------------------
    # PUBLICATION STM32
    # ------------------------------
    def publish_pwm(self, pwm_value):
        self.tx_data = Int16()
        self.tx_data.data = pwm_value
        if self.debug:
            self.get_logger().info(f"PWM sent: {pwm_value}\n")
        self.publisher.publish(self.tx_data)


def main(args=None):
    rclpy.init(args=args)
    node = CommandSpeedNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()