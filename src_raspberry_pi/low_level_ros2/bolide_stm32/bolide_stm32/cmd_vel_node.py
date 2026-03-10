import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32
from enum import Enum


# Machine à états avec gestion du frein et de la marche arrière
# Elle implémente les transitions avec les délais de transitions de l'ESC et les délais de stabilisation du signal pour l'ESC
# Le frein s'active si on est en forward puis cmd_vel <= 0. Il a une durée limitée (car délai d'engagement du mode reverse)
# Faire Forward -> 0.0 impose un délai ! Utiliser : deadzone > cmd_vel > 0 si on veut arrêter depuis forward sans frein (ni délai)

class ESCState(Enum):
    ARMING = 0
    REVERSE = 1             # cmd_vel <= 0 
    FORWARD = 2             # cmd_vel > 0 (si cmd_vel == 0 -> brake)
    BRAKING = 3             # Brake transition : 1. braking (reverse PWM), 2. engage_reverse (stable neutral PWM)

# Reverse : brake does not exist - ESC limitation - so REVERSE->FORWARD is direct
# Forward : brake exists : going FORWARD->REVERSE will require the brake transition !


class CommandSpeedNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_node')

        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').value

        # PWM en µs (deadzone ESC : 1500 ±40 µs)        # indépendant de la calibration de l'ESC
        self.PWM_FORWARD_MAX = 2000 # old: 1876
        self.PWM_FORWARD_MIN = 1540 # old: 1532
        self.PWM_NEUTRAL = 1500
        self.PWM_REVERSE_MIN = 1460 # old: 1468
        self.PWM_REVERSE_MAX = 1000 # old: 1220


        # deadzone de cmd_vel (must be positif) -> DEADZONE in [PWM_NEUTRAL ± VEL_DEADZONE]
        self.declare_parameter('cmd_vel_deadzone', 0.01)
        self.CMD_VEL_DEADZONE = self.get_parameter('cmd_vel_deadzone').value
        # temps max sans commande avant neutre    
        self.SAFETY_TIMEOUT = 0.5

        # durées timers (en sec)
        # durée de freinage (court-circuit moteur): valeur libre
        # si vitesse moteur en fin reste trop grande => REVERSE pas enclenchable !!! (casse la machine à états) Besoin de la vitesse des roues !
        self.DURATION_BRAKING_1 = 0.3                   # 0.3 est considéré safe
        # durée du PWM neutre pour passer en mode Reverse : valeur limitée par l'ESP (signal stable + invertion courants)
        self.DURATION_BRAKING_2 = 0.2

        # timers (inactifs au départ : cancelled)
        self.braking_1_timer = self.create_timer(self.DURATION_BRAKING_1, self.finish_braking_1)
        self.braking_1_timer.cancel()
        self.braking_2_timer = self.create_timer(self.DURATION_BRAKING_2, self.finish_braking_2)
        self.braking_2_timer.cancel()
        self.braking_state = None

        # memorisation cmd_vel si ESC occupé
        self.pending_cmd_vel = None
        self.future_state = None

        # Pub
        self.publisher = self.create_publisher(Int16, "/stm32_data", 10)

        # Armement ESC
        # temps pour armer l'ESC au démarrage : bip de 2 secondes (déjà fait par le stm32 avant lancement du node)
        self.state = ESCState.ARMING
        self.publish_pwm(self.PWM_NEUTRAL)
        self.state = ESCState.REVERSE

        # securité déconnexion /cmd_vel
        self.timer_safety = self.create_timer(self.SAFETY_TIMEOUT, self.security_stop)

        # Sub
        self.subscriber = self.create_subscription(Float32, "/cmd_vel", self.cmd_callback, 10)


    def cmd_callback(self, msg = None):
        if msg :
            self.timer_safety.reset()
            cmd_vel = max(min(msg.data, 1.0), -1.0)     # msg.data in [-1;1]
        else :
            cmd_vel = self.pending_cmd_vel

        self.pending_cmd_vel = None                     # nouvelles valeurs écrasent celles pending

        if self.state == ESCState.REVERSE:
            if cmd_vel <= 0:
                self.reverse(cmd_vel)
                return
            else:
                # REVERSE -> FORWARD : direct
                self.state = ESCState.FORWARD
                self.forward(cmd_vel)
                return
    
        elif self.state == ESCState.FORWARD:
            if cmd_vel > 0:                             # si cmd_vel == 0 : passe en BRAKING
                self.forward(cmd_vel)
                return
            else:
                # FORWARD -> REVERSE : BRAKING
                self.pending_cmd_vel = cmd_vel          # remember cmd_vel for after BRAKING (if not overwritten)
                self.state = ESCState.BRAKING
                self.future_state = ESCState.REVERSE
                self.start_braking_1()
                return
            
        else: #self.state == ESCState.BRAKING
            if cmd_vel <= 0:
                # BRAKE -> REVERSE (cmd_vel <= 0)
                # wait for end of braking duration to process cmd_vel
                self.pending_cmd_vel = cmd_vel
                return
            else: # cmd_vel > 0
                # BRAKE -> FORWARD
                # il est dangereux de cancel braking_2 : changement de sens brutal du courant dans le circuit
                if self.braking_state == 1:
                    # cancel braking_1
                    self.braking_1_timer.cancel()
                    # activate braking_2 : besoin de donner future_state car cmd_vel peut-être overwritten
                    self.future_state = ESCState.FORWARD
                    self.start_braking_2()
                # wait for end of braking_2 to process cmd_vel
                self.pending_cmd_vel = cmd_vel
                return


    # FORWARD / REVERSE
    def forward(self, cmd_vel):
        if cmd_vel < self.CMD_VEL_DEADZONE:
            pwm = self.PWM_NEUTRAL
        else:
            # cmd_vel > 0
            pwm = int(self.PWM_FORWARD_MIN + cmd_vel * (self.PWM_FORWARD_MAX - self.PWM_FORWARD_MIN))
        self.publish_pwm(pwm)
        if self.debug:
            self.get_logger().info(f"FORWARD PWM: {pwm}\n\r")

    def reverse(self, cmd_vel):
        if cmd_vel > - self.CMD_VEL_DEADZONE:
            pwm = self.PWM_NEUTRAL
        else: 
            # cmd_vel < 0
            pwm = int(self.PWM_REVERSE_MIN + cmd_vel * (self.PWM_REVERSE_MIN - self.PWM_REVERSE_MAX))
        self.publish_pwm(pwm)
        if self.debug:
            self.get_logger().info(f"REVERSE PWM: {pwm}\n\r")

    # BRAKING
    ## braking 1 : reverse PWM
    def start_braking_1(self):
        self.braking_state = 1
        self.publish_pwm(self.PWM_REVERSE_MIN - 50)
        if self.debug:
            self.get_logger().info(f"BRAKING_1 started PWM: {self.PWM_REVERSE_MIN - 50}\n\r")
        self.braking_1_timer.reset()  # restart timer

    def finish_braking_1(self):
        self.braking_1_timer.cancel()
        if self.debug:
            self.get_logger().info("BRAKING_1 finished\n\r")
        # BRAKING_1 -> BRAKING_2
        self.start_braking_2()

    ## braking 2 : stable neutral PWM
    def start_braking_2(self):
        self.braking_state = 2
        self.publish_pwm(self.PWM_NEUTRAL)
        if self.debug:
            self.get_logger().info(f"BRAKING_2 started PWM: {self.PWM_NEUTRAL}\n\r")
        self.braking_2_timer.reset()

    def finish_braking_2(self):
        self.braking_2_timer.cancel()
        if self.debug:
            self.get_logger().info("BRAKING_2 finished\n\r")
        # BRAKING -> future_state = REVERSE/FORWARD
        self.state = self.future_state
        if self.pending_cmd_vel is not None:
            self.cmd_callback()

    # securité
    def security_stop(self):
        self.timer_safety.cancel()
        self.get_logger().warn(f"Emergency brake triggered : '/cmd_vel' signal timeout")
        self.pending_cmd_vel = 0.0
        self.cmd_callback()

    # PUBLICATION /stm32_data
    def publish_pwm(self, pwm_value: int):
        msg = Int16()
        msg.data = pwm_value
        self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = CommandSpeedNode()
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

