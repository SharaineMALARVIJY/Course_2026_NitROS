import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32
from bolide_interfaces.msg import ForkSpeed
from enum import Enum


# Machine à états avec gestion du frein et de la marche arrière
# Elle implémente les transitions avec les délais de transitions de l'ESC et les délais de stabilisation du signal pour l'ESC
# Le frein s'active si on est en forward puis cmd_vel <= 0.
# Faire Forward -> 0.0 impose un délai ! Utiliser : deadzone > cmd_vel > 0 si on veut arrêter depuis forward sans frein (ni délai)

class ESCState(Enum):
    BRAKING_REV = -2
    REVERSE = -1             # cmd_vel <= 0 
    ARMING = 0
    FORWARD = 1             # cmd_vel > 0 (si cmd_vel == 0 -> brake)
    BRAKING_FWD = 2             # Brake transition : 1. braking (reverse PWM), 2. engage_reverse (stable neutral PWM)

# Reverse : brake does not exist so REVERSE->FORWARD is direct
# Forward : brake exists : going FORWARD->REVERSE will require the brake transition !


class CommandSpeedNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_node')

        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').value
        self.declare_parameter('cmd_deadzone', 1e-4)

        # PWM en µs (deadzone ESC : 1500 ±40 µs)        # indépendant de la calibration de l'ESC
        self.PWM_FORWARD_MAX = 2000
        self.PWM_FORWARD_MIN = 1540
        self.PWM_NEUTRAL = 1500
        self.PWM_REVERSE_MIN = 1460
        self.PWM_REVERSE_MAX = 1000
        # deadzone de cmd_vel : remapping de la deadzone de l'ESC pour être plus linéaire en vitesse
        self.CMD_DEADZONE = self.get_parameter('cmd_deadzone').value
        # temps max sans commande avant neutre    
        self.SAFETY_TIMEOUT = 1.0 # en s : temp autorisé sans commandes cmd_vel avnt mise au neutre automatique

        # durées timers (en sec)
        # durée de freinage (court-circuit moteur): valeur libre
        # si vitesse moteur en fin reste trop grande => REVERSE pas enclenchable !!! (casse la machine à états) Besoin de la vitesse des roues !
        self.DURATION_BRAKING_1 = 0.05                   # 0.3 est safe à basse vitesse
        # durée du PWM neutre pour passer en mode Reverse : valeur limitée par l'ESP (signal stable + invertion courants)
        self.DURATION_BRAKING_2 = 0.2

        # braking timers (inactifs au départ : cancelled)
        self.braking_1_timer = self.create_timer(self.DURATION_BRAKING_1, self.finish_braking_1)
        self.braking_1_timer.cancel()
        self.braking_1b_timer = self.create_timer(0.2, self.finish_braking_1b)
        self.braking_1b_timer.cancel()
        self.braking_2_timer = self.create_timer(self.DURATION_BRAKING_2, self.finish_braking_2)
        self.braking_2_timer.cancel()
        self.braking_state = None
        self.braking_rev_timer = self.create_timer(0.01, self.finish_braking_rev)
        self.braking_rev_timer.cancel()

        # memorisation cmd_vel si ESC occupé
        self.pending_cmd_vel = None
        self.future_state = None

        # Pub
        self.publisher = self.create_publisher(Int16, "/stm32_data", 10)
        self.esc_state_pub = self.create_publisher(Int16, '/esc_state', 10) # pour PID et motor_direction

        # Armement ESC
        # temps pour armer l'ESC au démarrage : bip de 2 secondes (déjà fait par le stm32 avant lancement du node)
        # Mais besoin d'un temps de démarrage de autres noeuds avant de publier l'état initial
        self.state = ESCState.ARMING
        self.publish_pwm(self.PWM_NEUTRAL)
        self.publish_esc_state()
        self.arming_timer = self.create_timer(0.2, self.finish_arming)

        # securité déconnexion /cmd_vel
        self.timer_safety = self.create_timer(self.SAFETY_TIMEOUT, self.security_stop)

        # Sub
        self.subscriber = self.create_subscription(Float32, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(ForkSpeed, '/raw_fork_data', self.cb_speed, 10)
        self.speed_meas = 0.0       # m/s
    
    def finish_arming(self):
        self.arming_timer.cancel()
        self.state = ESCState.REVERSE
        self.publish_esc_state()

    def cb_speed(self, msg):
        self.speed_meas = msg.speed  # toujours positive en m/s

    def publish_esc_state(self):
        msg = Int16()
        msg.data = self.state.value  # ESCstate
        self.esc_state_pub.publish(msg)


    def cmd_callback(self, msg = None):
        if msg :
            self.timer_safety.reset()
            cmd_vel = max(min(msg.data, 1.0), -1.0)     # msg.data in [-1;1]
        else :
            cmd_vel = self.pending_cmd_vel

        # SÉCURITÉ : Si aucun message et rien en attente, on ne fait rien
        if cmd_vel is None:
            return

        self.pending_cmd_vel = None                     # nouvelles valeurs écrasent celles pending

        if self.state == ESCState.REVERSE:
            if cmd_vel <= 0:
                self.reverse(cmd_vel)
                return
            else:
                # REVERSE -> BRAKING_REV : permet de synchroniser motor_direction
                self.pending_cmd_vel = cmd_vel
                self.state = ESCState.BRAKING_REV
                self.publish_esc_state()
                self.future_state = ESCState.FORWARD
                # REVERSE -> FORWARD : en faite direct pour l'ESC
                self.start_braking_rev()
                return
    
        elif self.state == ESCState.FORWARD:
            if cmd_vel > 0:                             # si cmd_vel == 0 : passe en BRAKING_FWD
                self.forward(cmd_vel)
                return
            else:
                # FORWARD -> REVERSE : BRAKING_FWD
                self.pending_cmd_vel = cmd_vel          # remember cmd_vel for after BRAKING_FWD (if not overwritten)
                self.state = ESCState.BRAKING_FWD
                self.publish_esc_state()
                self.future_state = ESCState.REVERSE
                self.start_braking_1()
                return
            
        elif self.state == ESCState.BRAKING_FWD:
            if cmd_vel <= 0:
                # BRAKE -> REVERSE (cmd_vel <= 0)
                # wait for end of braking duration to process cmd_vel
                self.pending_cmd_vel = cmd_vel
                return
            else: # cmd_vel > 0
                # BRAKING_FWD -> FORWARD
                # il est dangereux de cancel braking_2 : changement de sens brutal du courant dans le circuit
                if self.braking_state == 1:
                    # cancel braking_1
                    self.braking_1_timer.cancel()
                    self.braking_1b_timer.cancel()
                    # activate braking_2 : besoin de donner future_state car cmd_vel peut-être overwritten
                    self.future_state = ESCState.FORWARD
                    self.start_braking_2()
                # wait for end of braking_2 to process cmd_vel
                self.pending_cmd_vel = cmd_vel
                return
        elif self.state == ESCState.BRAKING_REV:
            if cmd_vel > 0:
                self.pending_cmd_vel = cmd_vel
                return
            else: # cmd_vel <= 0
                # BRAKING_REV -> REVERSE
                self.braking_rev_timer.cancel()
                self.state = ESCState.REVERSE
                self.publish_esc_state()
                self.reverse(cmd_vel)
                return
        else :
            return


    # FORWARD / REVERSE
    def forward(self, cmd_vel):
        if cmd_vel < self.CMD_DEADZONE:
            pwm = self.PWM_NEUTRAL
        else:
            # cmd_vel > 0
            pwm = int(self.PWM_FORWARD_MIN + cmd_vel * (self.PWM_FORWARD_MAX - self.PWM_FORWARD_MIN))
        self.publish_pwm(pwm)
        if self.debug:
            self.get_logger().info(f"FORWARD PWM: {pwm}\n\r")

    def reverse(self, cmd_vel):
        if cmd_vel > - self.CMD_DEADZONE:
            pwm = self.PWM_NEUTRAL
        else: 
            # cmd_vel < 0
            pwm = int(self.PWM_REVERSE_MIN + cmd_vel * (self.PWM_REVERSE_MIN - self.PWM_REVERSE_MAX))
        self.publish_pwm(pwm)
        if self.debug:
            self.get_logger().info(f"REVERSE PWM: {pwm}\n\r")
    
    # BRAKING_REV
    def start_braking_rev(self):
        self.publish_pwm(self.PWM_NEUTRAL)
        if self.debug:
            self.get_logger().info(f"BRAKING_REV started PWM: {self.PWM_NEUTRAL}\n\r")
        self.braking_rev_timer.reset()  # restart timer

    def finish_braking_rev(self):
        if self.speed_meas > 0.05:
            return 
        self.braking_rev_timer.cancel()
        if self.debug:
            self.get_logger().info("BRAKING_REV finished\n\r")
        # BRAKING -> future_state = REVERSE/FORWARD
        self.state = self.future_state
        self.publish_esc_state()
        if self.pending_cmd_vel is not None:
            self.cmd_callback()

    # BRAKING_FWD
    ## braking 1 : reverse PWM
    def start_braking_1(self):
        self.braking_state = 1
        self.publish_pwm(self.PWM_REVERSE_MIN - 150)
        if self.debug:
            self.get_logger().info(f"BRAKING_1 started PWM: {self.PWM_REVERSE_MIN - 250}\n\r")
        self.braking_1_timer.reset()  # restart timer

    def finish_braking_1(self):
        if self.speed_meas > 0.05:
            return 
        self.braking_1_timer.cancel()
        if self.debug:
            self.get_logger().info("Lancement du délai de sécurité 1b\n\r")
        self.braking_1b_timer.reset()

    def finish_braking_1b(self):
        self.braking_1b_timer.cancel()
        if self.debug:
            self.get_logger().info(f"BRAKING_1b finished, speed : {self.speed_meas:.2f} \n\r")
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
        self.publish_esc_state()
        if self.pending_cmd_vel is not None:
            self.cmd_callback()

    # securité
    def security_stop(self):
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
        node.security_stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

