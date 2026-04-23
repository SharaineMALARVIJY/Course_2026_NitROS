import spidev
import math
import struct
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
        self.declare_parameter('alpha_vbat', 0.05)
        self.declare_parameter('alpha_speed', 0.4)

        self.debug = self.get_parameter('debug').value

        # --- ÉTAT INTERNE & FILTRES ---
        self.speed_filtered = 0.0  
        self.old_vbat = 8.0  # tension nominale
        self.ALPHA_VBAT = self.get_parameter('alpha_vbat').value
        self.ALPHA_SPEED = self.get_parameter('alpha_speed').value

        # --- CONSTANTES & FACTEURS PRÉ-CALCULÉS ---
        self.YAW_FACTOR = -(2.0 * math.pi) / 5760.0                 # 5760 pour 2pi (cf datasheet BNO055), signe convention ROS2
        self.YAW_RATE_FACTOR = 2.0 * -self.YAW_FACTOR               # shift 1 bit et signe convention ROS2
        self.VBAT_FACTOR = 8.0 * (3.3 / 4095.0) * (1560.0 / 560.0)

        ## Batterie (Conversion ADC -> Volts)
        # bat_raw : valeur ADC brute -> Le STM32 a un ADC 12 bits, il quantifie sur 4096 valeurs
        # bat_adc = bat_raw << 3  # les valeurs ADC semblent décalées de 3 bits => bat_adc : 0 <-> 4095
        # v_adc = bat_adc * (3.3 / 4095.0)  # conversion ADC -> Volts (V_ADC) : entre 0V et 3.3V
        # vbat = v_adc * ((1000 + 560) / 560) # Pont diviseur de tension (schema du Hat) : V_ADC = VS * (R3 / (R4 + R3)) avec R3=560, R4=1k

        # --- SPI ---
        self.spi = spidev.SpiDev()
        self.spi.open(0, 1)
        self.spi.mode = 0
        self.spi.max_speed_hz = 112500
        # On initialise un buffer de 20 octets (8 cmd + 12 padding pour lecture)
        self.tx_full_buffer = [0] * 20 
        self.crc_table = self._generate_crc_table()

        # --- PUBLISHERS ---
        self.speed_pub = self.create_publisher(ForkSpeed, '/raw_fork_data', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.vbat_pub = self.create_publisher(Float32, '/battery_voltage', 10)
        self.pub_ir_left = self.create_publisher(Range, '/range/ir_left', 10)
        self.pub_ir_right = self.create_publisher(Range, '/range/ir_right', 10)
        self.pub_sonar = self.create_publisher(Range, '/range/sonar_rear', 10)
        self.stm_pub = self.create_publisher(Float32MultiArray, '/stm32_sensors', 10)

        self.create_subscription(Int16, '/stm32_data', self.get_command, 10)

        # --- CONFIG RANGES ---
        self.ir_min_range, self.ir_max_range = 0.06, 0.3
        self.ir_field_of_view = 0.09
        self.sonar_max_range = 0.7 # TODO check value

        # --- MESSAGES PRÉ-REMPLIS ---
        self.sensor_data = Float32MultiArray()
        self.fork_data = ForkSpeed()
        self.vbat_msg = Float32()

        self.imu_data = Imu()
        self.imu_data.header.frame_id = "base_link"
        self.imu_data.angular_velocity_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 0.01] # TODO check datasheet
        self.imu_data.orientation_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 0.01]
        self.imu_data.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        
        self.range_msg_left = self.create_range_msg("ir_left_frame", Range.INFRARED, self.ir_max_range, self.ir_min_range, self.ir_field_of_view)
        self.range_msg_right = self.create_range_msg("ir_right_frame", Range.INFRARED, self.ir_max_range, self.ir_min_range, self.ir_field_of_view)
        self.range_msg_sonar = self.create_range_msg("sonar_frame", Range.ULTRASOUND, self.sonar_max_range)

        # ESC state to solve fork issues
        self.esc_state = 0 # Par défaut neutre
        self.create_subscription(Int16, '/esc_state', self.esc_state_callback, 10)

        self.timer = self.create_timer(0.01, self.receiveSensorData)

    def create_range_msg(self, frame, r_type, max_range, min_range=0.0, field_of_view=0.26):
        msg = Range()
        msg.header.frame_id = frame
        msg.radiation_type = r_type
        msg.min_range, msg.max_range = min_range, max_range
        msg.field_of_view = field_of_view
        return msg
    
    def publish_range(self, publisher, msg, distance, timestamp):
        msg.header.stamp = timestamp
        msg.range = float(distance)
        publisher.publish(msg)

    def esc_state_callback(self, msg):
        self.esc_state = msg.data

    def receiveSensorData(self):
        now = self.get_clock().now()
        timestamp = now.to_msg()

        try:
            self.spi.writebytes(self.tx_full_buffer[:8])
            data = self.spi.readbytes(20)
        except Exception as e:
            self.get_logger().error(f"SPI error: {e}", throttle_duration_sec=2.0)
            return

        if self.crc32mpeg2(data): return    # frequent CRC errors -> reduce timer (STM32 is too slow) or cable length

        try: # struct.unpack -> !: Big-Endian, H: uint16, h: int16
            raw_vbat, raw_yaw, raw_ir_l, raw_ir_r, raw_speed, raw_us, raw_acc_x, raw_yaw_rate = struct.unpack('!HhHHHHhh', bytes(data[:16]))
        except struct.error: return

        vbat = raw_vbat * self.VBAT_FACTOR  # tension batterie moteur en V
        yaw = raw_yaw * self.YAW_FACTOR     # rad - twist z axis (vertical) -
        speed_val = raw_speed * 0.002       # m/s speed from the fork -1 bit shift- (There may be issues in the STM32 code, see below)
        dist_us = raw_us * 0.01             # m  - distance from the ultrasound sensor -
        acc_x = raw_acc_x * 0.01            # ms^-2 - acceleration in the x axis -
        yaw_rate = raw_yaw_rate * self.YAW_RATE_FACTOR # rad/s - rotation speed z-axis - (There may be issues in the STM32 code, see below)

        ## Fork Speed : There are sometimes weird values : nulls, or old values when it should be null
        if self.esc_state == 0: # Si l'ESC est au neutre, la vitesse est 0
            self.speed_filtered = 0.0
        else:
            # On protège seulement contre les glitchs SPI (zéros aberrants en plein mouvement)
            if abs(speed_val) < 0.001 and abs(self.speed_filtered) > 0.3:
                speed_val = self.speed_filtered
            # Lissage classique
            self.speed_filtered = (self.ALPHA_SPEED * speed_val) + (1.0 - self.ALPHA_SPEED) * self.speed_filtered
        speed_val = float(self.speed_filtered)
        # Publication
        self.fork_data.header.stamp = timestamp # The fork's publication freq is probably at 20Hz, we don't know (STM32 code)
        self.fork_data.speed = speed_val
        self.speed_pub.publish(self.fork_data)

        # IMU
        self.imu_data.header.stamp = timestamp
        self.imu_data.orientation.z = math.sin(yaw * 0.5)
        self.imu_data.orientation.w = math.cos(yaw * 0.5)
        self.imu_data.angular_velocity.z = float(yaw_rate)  # can be used if filtered but noisy (weird null values and shifting offset)
        self.imu_data.linear_acceleration.x = acc_x
        self.imu_pub.publish(self.imu_data)

        ## Ranges
        ir_l_v = raw_ir_l * 0.001
        ir_left = (15.38/ir_l_v - 0.42)/100.0 if ir_l_v > 0.005 else self.ir_max_range # experimental factor and noise threshold
        ir_left = min(max(ir_left, 0.0), self.ir_max_range)

        ir_r_v = raw_ir_r * 0.001
        ir_right = (15.38/ir_r_v - 0.42)/100.0 if ir_r_v > 0.005 else self.ir_max_range # experimental factor and noise threshold
        ir_right = min(max(ir_right, 0.0), self.ir_max_range)

        distance_US = dist_us if dist_us > 0.001 else self.sonar_max_range # experimental noise threshold (unchecked)

        self.publish_range(self.pub_ir_left, self.range_msg_left, ir_left, timestamp)
        self.publish_range(self.pub_ir_right, self.range_msg_right, ir_right, timestamp)
        self.publish_range(self.pub_sonar, self.range_msg_sonar, distance_US, timestamp)

        ## Batterie
        vbat_clamped = min(14.0, max(4.5, vbat))
        self.old_vbat = self.ALPHA_VBAT * vbat_clamped + (1.0 - self.ALPHA_VBAT) * self.old_vbat
        self.vbat_msg.data = float(self.old_vbat)
        self.vbat_pub.publish(self.vbat_msg)

        # Debug topic
        if self.debug:
            self.sensor_data.data = [float(yaw), float(speed_val), float(ir_left), 
                                     float(ir_right), float(dist_us), float(acc_x), float(yaw_rate)]
            self.stm_pub.publish(self.sensor_data)

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
        self.tx_full_buffer[0] = (val >> 8) & 0xFF
        self.tx_full_buffer[1] = val & 0xFF
        crc = self.crc32mpeg2(self.tx_full_buffer[:2])
        self.tx_full_buffer[2:6] = list(struct.pack('!I', crc))

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
