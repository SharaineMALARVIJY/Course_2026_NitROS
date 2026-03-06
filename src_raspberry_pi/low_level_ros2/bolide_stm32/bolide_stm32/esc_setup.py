import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import threading


class ESCCalibration(Node):
    def __init__(self, publish_hz: int = 10):
        super().__init__('esc_calibration')
        
        # Publication frequency to use while waiting for button presses
        self.publish_hz = int(publish_hz)

        # Publisher pour envoyer les commandes sur cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Float32, '/cmd_vel', 10)
        
        # Valeurs de vitesse pour la calibration
        self.VEL_MAX = 1.0       # Plein avant
        self.VEL_NEUTRAL = 0.0   # Point neutre
        self.VEL_MIN = -1.0      # Plein arrière
        
        self.get_logger().info(f'ESC Calibration Node initialisé (publish_hz={self.publish_hz})')
        
    def send_velocity(self, linear_x):
        """Envoie une commande de vitesse à l'ESC via cmd_vel"""
        msg = Float32()
        msg.data = linear_x
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Envoi cmd_vel: {linear_x}')

    def _wait_for_enter_async(self, prompt):
        """Start a background thread that waits for ENTER and return an Event set when done."""
        event = threading.Event()
        def _waiter():
            input(prompt)
            event.set()
        t = threading.Thread(target=_waiter, daemon=True)
        t.start()
        return event

    def send_velocity_while_waiting(self, linear_x, hz=None, prompt='Appuyez sur ENTRÉE pour continuer...'):
        """Publish the velocity at least `hz` times per second until user presses ENTER.
        If `hz` is None, the node's `publish_hz` is used."""
        hz = int(hz) if hz is not None else self.publish_hz
        hz = max(1, hz)
        event = self._wait_for_enter_async(prompt)
        period = 1.0 / hz
        self.get_logger().info(f'Publication continue de {linear_x} à {hz} Hz jusqu\'à appui sur ENTRÉE')
        try:
            while not event.is_set():
                self.send_velocity(linear_x)
                time.sleep(period)
        finally:
            # ensure we send a final value to record
            self.send_velocity(linear_x)
            self.get_logger().info('Appui détecté, arrêt de la publication continue')

    def calibrate(self):
        """Séquence de calibration de l'ESC Tamiya TBLE-04S"""
        self.get_logger().info('=== DÉBUT DE LA CALIBRATION ESC ===')
        self.get_logger().info('Assurez-vous que l\'ESC est allumé!')
        
        # Étape 1: Envoyer la valeur maximale
        self.get_logger().info(f'Étape 1: Vitesse avant -> Envoi de la valeur MAX ({self.VEL_MAX})')
        self.send_velocity_while_waiting(self.VEL_MAX, hz=10, prompt='Appuyez sur le bouton de l\'ESC pour enregistrer MAX, puis appuyez sur ENTRÉE...')
        
        # Étape 2: Envoyer la valeur minimale
        self.get_logger().info(f'Étape 2: Vitesse arrière -> Envoi de la valeur MIN ({self.VEL_MIN})')
        self.send_velocity_while_waiting(self.VEL_MIN, hz=10, prompt='Appuyez sur le bouton de l\'ESC pour enregistrer MIN, puis appuyez sur ENTRÉE...')
        
        self.get_logger().info('=== CALIBRATION TERMINÉE ===')
        
    def test_sequence(self):    # Le vrai test c'est avec teleop pas sure de l'utilite de cette fonction mais elle est la quand meme
        """Séquence de test après calibration"""
        self.get_logger().info('=== TEST DE L\'ESC ===')
        
        # Neutre
        self.get_logger().info('Position neutre...')
        self.send_velocity_while_waiting(self.VEL_NEUTRAL)
        time.sleep(2)
        
        # Léger avant
        self.get_logger().info('Léger avant (0.3)...')
        self.send_velocity_while_waiting(0.3)
        time.sleep(2)
        
        # Retour neutre     ATTENTION VA PROBABLEMENT PAS SUFFIRE A FAIRE RECULER LA VOITURE NE VOUS INQUIETEZ PAS
        self.get_logger().info('Retour neutre...')
        self.send_velocity_while_waiting(self.VEL_NEUTRAL)
        time.sleep(2)
        
        # Léger arrière     TOUT VA BIEN SI CA RECULE PAS TESTEZ AVEC TELEOP EN ENVOYANT PLEIN DE NEUTRE AVANT
        self.get_logger().info('Léger arrière (-0.3)...')
        self.send_velocity_while_waiting(-0.3)
        time.sleep(2)
        
        # Retour neutre
        self.get_logger().info('Retour neutre final...')
        self.send_velocity_while_waiting(self.VEL_NEUTRAL)
        
        self.get_logger().info('=== TEST TERMINÉ ===')


def main(args=None):
    rclpy.init(args=args)

    esc_node = ESCCalibration()
    
    print("\n" + "="*50)
    print("CALIBRATION ESC TAMIYA TBLE-04S")
    print("="*50)
    print(f"Publication frequency : {10} Hz")
    print("\nOptions:")
    print("1 - Calibration complète")
    print("2 - Test de l'ESC")
    print("3 - Envoi manuel de vitesse")
    print("="*50 + "\n")
    
    choice = input("Votre choix (1-3): ")
    
    if choice == '1':
        input("\nBranchez l'ESC, appuyez sur le bouton SET de l'ESC, relâchez quand il est rouge et appuyez sur ENTRÉE pour commencer la calibration. \n L'ESC devrait se mettre à clignoter en rouge.\n")
        esc_node.calibrate()
    elif choice == '2':
        esc_node.test_sequence()
    elif choice == '3':
        try:
            vel_value = float(input("Entrez la vitesse (-1.0 à 1.0): "))
            if -1.0 <= vel_value <= 1.0:
                esc_node.send_velocity_while_waiting(vel_value)
            else:
                print("Valeur hors limites!")
        except ValueError:
            print("Valeur invalide!")
    
    print("\nAppuyez sur Ctrl+C pour quitter...")
    try:
        rclpy.spin(esc_node)
    except KeyboardInterrupt:
        pass
    
    esc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
