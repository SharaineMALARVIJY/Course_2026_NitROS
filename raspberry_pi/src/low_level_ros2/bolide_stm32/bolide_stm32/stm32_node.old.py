import spidev
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray, Int16, Float32
from sensor_msgs.msg import Range, Imu
from bolide_interfaces.msg import ForkSpeed

class STM32_Parser(Node):
    def __init__(self):
        super().__init__('stm32_node')

        # --- PARAMÈTRES ---
        self.declare_parameter('debug', True)
        self.debug = self.get_parameter('debug').value
        self.declare_parameter('calibrate', False)
        self.calibrate = self.get_parameter('calibrate').value

        # --- VARIABLES DE CALIBRATION ---
        self.calib_gyro_data = []
        self.calib_acc_data = []
        self.nb_samples_calib = 500  # Réduit un peu pour que ce soit plus rapide (5 sec à 100Hz)
        self.calib_cooldown = 3.0    # Secondes d'attente entre deux calibrations
        self.last_calib_end_time = self.get_clock().now()
        self.is_calibrating = True   # État interne

        # --- OFFSETS & FACTEURS (BNO055) ---
        self.ACC_X_OFFSET_FIXE = 0.0      # À mettre à jour après calibration
        self.GYRO_OFFSET_FIXE = 0.0       # À mettre à jour après calibration
        self.YAW_FACTOR = -(2.0 * math.pi) / 5760.0      # 5760 LSB par tour (cf datasheet BNO055)
        self.YAW_RATE_FACTOR = 2.0 * -self.YAW_FACTOR    # shift 1 bit et signe
        self.VBAT_FACTOR = 8.0 * (3.3 / 4095.0) * (1560.0 / 560.0)
        ## Batterie (Conversion ADC -> Volts)
        # bat_raw : valeur ADC brute -> Le STM32 a un ADC 12 bits, il quantifie sur 4096 valeurs
        # bat_adc = bat_raw << 3  # les valeurs ADC semblent décalées de 3 bits => bat_adc : 0 <-> 4095
        # v_adc = bat_adc * (3.3 / 4095.0)  # conversion ADC -> Volts (V_ADC) : entre 0V et 3.3V
        # vbat = v_adc * ((1000 + 560) / 560)   # Pont diviseur de tension dans le schema du Hat : V_ADC = VS * (R3 / (R4 + R3)) avec R3=560, R4=1k

        # Hold anti-jitter (bug lecture SPI)
        self.last_valid_yaw_rate = 0.0

        # --- SPI ---
        self.spi = spidev.SpiDev()
        self.spi.open(0, 1)
        self.spi.mode = 0
        self.spi.max_speed_hz = 112500
        self.tx_buffer = [0] * 8
        self.crc_table = self._generate_crc_table()

        # --- PUBLISHERS ---
        self.speed_pub = self.create_publisher(ForkSpeed, '/raw_fork_data', 10)
        self.imu_pub = self.create_publisher(Imu, '/raw_imu_data', 10)
        self.vbat_pub = self.create_publisher(Float32, '/battery_voltage', 10)
        self.pub_ir_left = self.create_publisher(Range, '/range/ir_left', 10)
        self.pub_ir_right = self.create_publisher(Range, '/range/ir_right', 10)
        self.pub_sonar = self.create_publisher(Range, '/range/sonar_rear', 10)
        self.stm_pub = self.create_publisher(Float32MultiArray, '/stm32_sensors', 10)

        # --- SUBSCRIBERS ---
        self.create_subscription(Int16, '/stm32_data', self.get_command, 10)

        # --- CONFIG RANGES ---
        self.ir_min_range, self.ir_max_range = 0.06, 0.3
        self.ir_field_of_view = 0.09
        self.sonar_max_range = 0.7

        # --- MESSAGES PRÉ-REMPLIS ---
        self.sensor_data = Float32MultiArray()
        self.fork_data = ForkSpeed()
        self.vbat_msg = Float32()
        self.imu_data = Imu()
        self.imu_data.header.frame_id = "base_link"
        self.imu_data.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1e-2] # TODO : check value
        self.imu_data.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1e-2]
        self.imu_data.linear_acceleration_covariance = [1e-1, 0, 0, 0, 0, 0, 0, 0, 0]
        self.range_msg_left = self.create_range_msg("ir_left_frame", Range.INFRARED, self.ir_max_range, self.ir_min_range, self.ir_field_of_view)
        self.range_msg_right = self.create_range_msg("ir_right_frame", Range.INFRARED, self.ir_max_range, self.ir_min_range, self.ir_field_of_view)
        self.range_msg_sonar = self.create_range_msg("sonar_frame", Range.ULTRASOUND, self.sonar_max_range)

        # loop
        self.timer = self.create_timer(0.01, self.receiveSensorData)

    def create_range_msg(self, frame, r_type, max_range, min_range=0.0, field_of_view=0.26):
        msg = Range()
        msg.header.frame_id = frame
        msg.radiation_type = r_type
        msg.min_range, msg.max_range = min_range, max_range
        msg.field_of_view = field_of_view
        return msg

    def receiveSensorData(self):
        try:
            self.spi.writebytes(self.tx_buffer)
            data = self.spi.readbytes(20)
        except Exception as e:
            self.get_logger().error(f"SPI error: {e}", throttle_duration_sec=2.0)
            return

        timestamp = self.get_clock().now().to_msg()
        ir_max_range = self.ir_max_range

        if not self.crc32mpeg2(data):
            # Décodage
            vbat = ((data[0] << 8) | data[1]) * self.VBAT_FACTOR # tension batterie moteur en V
            raw_yaw = int.from_bytes([data[2], data[3]], byteorder='big', signed=True)
            raw_ir_left = (data[4] << 8) | data[5]  # mV - left infrared sensor -
            raw_ir_right = (data[6] << 8) | data[7]   # mV - right infrared sensor -
            raw_speed = 0.002 * ((data[8] << 8) | data[9])  # m/s speed from the fork - We have to add a factor 2, there is a 1 bit shift-
            raw_distance_US = 0.01 * ((data[10] << 8) | data[11])  # m  - distance from the ultrasound sensor -
            raw_acc_x = 0.01 * int.from_bytes([data[12], data[13]], byteorder='big', signed=True)  # ms^-2 - acceleration in the x axis -
            raw_yaw_rate = int.from_bytes([data[14], data[15]], byteorder='big', signed=True)  # rads^-1 - rotation speed around the z-axis of the car -

            ## Fork Speed (abs)
            self.fork_data.header.stamp = timestamp
            self.fork_data.speed = raw_speed 
            self.speed_pub.publish(self.fork_data)

            ##IMU
            # IMU Process
            acc_x = raw_acc_x - self.ACC_X_OFFSET_FIXE
            yaw = raw_yaw * self.YAW_FACTOR
            
            # Yaw Rate + Offset + Anti-jitter
            if raw_yaw_rate == 0 and abs(self.last_valid_yaw_rate) > 0.01:
                yaw_rate = self.last_valid_yaw_rate
            else:
                yaw_rate = (raw_yaw_rate * self.YAW_RATE_FACTOR) - self.GYRO_OFFSET_FIXE
                self.last_valid_yaw_rate = yaw_rate

            # Publish IMU
            self.imu_data.header.stamp = timestamp
            self.imu_data.orientation.z = math.sin(yaw * 0.5)
            self.imu_data.orientation.w = math.cos(yaw * 0.5)
            self.imu_data.angular_velocity.z = yaw_rate
            self.imu_data.linear_acceleration.x = acc_x
            self.imu_pub.publish(self.imu_data)

            ## Ranges
            # IR Left
            ir_left_v = raw_ir_left / 1000.0  # conversion mV --> V
            if ir_left_v > 0.005 : # Seuil de sécurité
                ir_left = (15.38/ir_left_v - 0.42)/100.0 # conversion V --> m (using component datasheet)
            else:
                ir_left = ir_max_range
            ir_left = min(max(ir_left, 0.0), ir_max_range)

            # IR Right
            ir_right_v = raw_ir_right / 1000.0  # conversion mV --> V
            if ir_right_v > 0.005: # Seuil de sécurité
                ir_right = (15.38/ir_right_v - 0.42)/100.0 # V -> m
            else:
                ir_right = ir_max_range
            ir_right = min(max(ir_right, 0.0), ir_max_range)

            # Sonar
            distance_US = raw_distance_US if raw_distance_US > 0.001 else self.sonar_max_range  # si renvoie 0.0 mets à max_range

            ## Publication individuelle pour Nav2
            self.publish_range(self.pub_ir_left, self.range_msg_left, ir_left, timestamp)
            self.publish_range(self.pub_ir_right, self.range_msg_right, ir_right, timestamp)
            self.publish_range(self.pub_sonar, self.range_msg_sonar, distance_US, timestamp)

            ## Batterie
            self.vbat_msg.data = float(vbat)
            self.vbat_pub.publish(self.vbat_msg)

            ## sensors data (debug)
            if self.debug or self.calibrate:
                self.sensor_data.data = [float(yaw), float(raw_speed), float(ir_left), float(ir_right), 
                                         float(raw_distance_US), float(acc_x), float(yaw_rate)]
                self.stm_pub.publish(self.sensor_data)
                # Calibration
                if self.calibrate:
                    self.run_calibration_logic(raw_yaw_rate, raw_acc_x)

    def _generate_crc_table(self):
        table = []
        for i in range(256):
            crc = i << 24
            for _ in range(8):
                crc = (crc << 1) ^ 0x104c11db7 if (crc & 0x80000000) else crc << 1
            table.append(crc & 0xffffffff)
        return table
    
    def crc32mpeg2(self, buf):
        crc = 0xffffffff
        for val in buf:
            crc = ((crc << 8) & 0xffffffff) ^ self.crc_table[(crc >> 24) ^ val]
        return crc

    def get_command(self, msg: Int16):
        val = msg.data
        cmd = [(val >> 8) & 0xFF, val & 0xFF]
        crc = self.crc32mpeg2(cmd)
        self.tx_buffer = cmd + [(crc >> 24) & 0xFF, (crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF] + [0]*2

    def publish_range(self, publisher, msg, distance, timestamp):
        msg.header.stamp = timestamp
        msg.range = float(distance)
        publisher.publish(msg)

    def run_calibration_logic(self, raw_gyro, raw_acc):
        now = self.get_clock().now()

        # Si on est en pause entre deux calibrations
        if not self.is_calibrating:
            if (now - self.last_calib_end_time).nanoseconds * 1e-9 > self.calib_cooldown:
                self.is_calibrating = True
                self.calib_gyro_data = []
                self.calib_acc_data = []
                self.get_logger().warn(">>> NOUVELLE SESSION DE CALIBRATION (Ne pas bouger...)")
            return

        # Phase d'acquisition
        self.calib_gyro_data.append(raw_gyro * self.YAW_RATE_FACTOR)
        self.calib_acc_data.append(raw_acc)

        # Progression (tous les 20%)
        if len(self.calib_gyro_data) % (self.nb_samples_calib // 5) == 0:
            progression = (len(self.calib_gyro_data) / self.nb_samples_calib) * 100
            self.get_logger().info(f"Calibration en cours... {progression:.0f}%")

        # Calcul final quand on a assez d'échantillons
        if len(self.calib_gyro_data) >= self.nb_samples_calib:
            # Moyennes (Offsets)
            avg_gyro = sum(self.calib_gyro_data) / len(self.calib_gyro_data)
            avg_acc = sum(self.calib_acc_data) / len(self.calib_acc_data)

            # Calcul de la stabilité (Écart-type / Standard Deviation)
            # Plus cette valeur est proche de 0, plus le capteur est stable (pas de vibrations)
            def get_std_dev(data, avg):
                variance = sum((x - avg) ** 2 for x in data) / len(data)
                return math.sqrt(variance)

            std_gyro = get_std_dev(self.calib_gyro_data, avg_gyro)
            std_acc = get_std_dev(self.calib_acc_data, avg_acc)

            # Affichage complet
            self.get_logger().info(
                f"\n--- RÉSULTATS CALIBRATION ---\n"
                f"OFFSETS (Moyennes) :\n"
                f"  > GYRO_OFFSET  : {avg_gyro:.6f}\n"
                f"  > ACC_X_OFFSET : {avg_acc:.6f}\n"
                f"STABILITÉ (Bruit/Vibration) :\n"
                f"  > Bruit Gyro   : {std_gyro:.6f} (idéal < 0.01)\n"
                f"  > Bruit Accel  : {std_acc:.6f} (idéal < 0.05)\n"
                f"-----------------------------\n"
                f"Redémarrage dans {self.calib_cooldown}s...\n"
            )

            # Reset pour le prochain cycle
            self.is_calibrating = False
            self.last_calib_end_time = now

def main(args=None):
    rclpy.init(args=args)
    node = STM32_Parser()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if hasattr(node, 'spi'): node.spi.close()
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass

if __name__ == '__main__':
    main()