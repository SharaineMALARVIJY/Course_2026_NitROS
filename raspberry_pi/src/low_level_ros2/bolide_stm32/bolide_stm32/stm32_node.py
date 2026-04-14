"""stm32 node. It receives data of the sensors (IMU, infrared, fork) from the STM32 and send it the PWM for the propulsion motor."""

# IMPORTS
import threading
import spidev
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray, Int16, Float32
from sensor_msgs.msg import Range, Imu
from bolide_interfaces.msg import ForkSpeed


class STM32_Parser(Node):
    """ROS2 Class for the STM32
    """
    def __init__(self):
        super().__init__('stm32_node')

        # PARAMS
        self.declare_parameter('debug', False)
        self.declare_parameter('max_fork_speed', 4.1)   # valeur max attendue de la fouche en m/s (met à 0.0 sinon)

        self.debug = self.get_parameter('debug').value

        # Pré-calcul des facteurs
        self.VBAT_FACTOR = 8.0 * (3.3 / 4095.0) * (1560.0 / 560.0)
        self.YAW_FACTOR = -1.0 / 900.0      # le yaw doit augmenter quand on tourne à gauche (convention)
        self.YAW_RATE_FACTOR = -1.0 / 900.0  # TODO : check sign

        # STM32 set-up
        self.BAUDRATE = 112500
        bus = 0
        device = 1

        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.mode = 0
        self.spi.max_speed_hz = self.BAUDRATE

        # Publishers
        self.speed_pub = self.create_publisher(ForkSpeed, '/raw_fork_data', 10)
        self.imu_pub = self.create_publisher(Imu, '/raw_imu_data', 10)
        self.vbat_pub = self.create_publisher(Float32, '/battery_voltage', 10)

        # Publishers Nav2 RangeLayer
        self.pub_ir_left = self.create_publisher(Range, '/range/ir_left', 10)
        self.pub_ir_right = self.create_publisher(Range, '/range/ir_right', 10)
        self.pub_sonar = self.create_publisher(Range, '/range/sonar_rear', 10)

        # Publisher de debug à supprimer (utilisé par odom_node)
        self.stm_pub = self.create_publisher(Float32MultiArray, '/stm32_sensors', 10)
        # Sensors debug
        self.sensor_data = Float32MultiArray()  # table for the sensors data

        # Subscribers
        self.get_cmd = self.create_subscription(Int16, '/stm32_data', self.get_command, 10)

        # Fork
        self.fork_data = ForkSpeed()  # Fork message type (see bolide_interfaces)
        self.max_fork_speed = self.get_parameter('max_fork_speed').value

        # Battery
        self.vbat_msg = Float32()

        # Infrared sensors
        self.ir_min_range = 0.06
        self.ir_max_range = 0.3

        # Ultrasound sensor
        self.sonar_max_range = 0.7  # TODO : check value

        # IMU
        self.imu_data = Imu()  # IMU message type (see sheet online)
        self.imu_data.header.frame_id = "base_link"
        # Set the covariance of the IMU (surely from datasheet) TODO - check it
        self.imu_data.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1e-2]
        self.imu_data.linear_acceleration_covariance = [1e-1, 0, 0, 0, 0, 0, 0, 0, 0]
        self.imu_data.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1e-2]
    
        # Ranges
        self.range_msg_left = Range()
        self.range_msg_left.header.frame_id = "ir_left_frame"
        self.range_msg_left.radiation_type = Range.INFRARED
        self.range_msg_left.min_range = self.ir_min_range
        self.range_msg_left.max_range = self.ir_max_range

        self.range_msg_right = Range()
        self.range_msg_right.header.frame_id = "ir_right_frame"
        self.range_msg_right.radiation_type = Range.INFRARED
        self.range_msg_right.min_range = self.ir_min_range
        self.range_msg_right.max_range = self.ir_max_range

        self.range_msg_sonar = Range()
        self.range_msg_sonar.header.frame_id = "sonar_frame"
        self.range_msg_sonar.radiation_type = Range.ULTRASOUND
        self.range_msg_sonar.min_range = 0.0
        self.range_msg_sonar.max_range = self.sonar_max_range

        # Transmitter
        self.tx_buffer = [0] * 8  # data that we'll transmit to the STM32

        # crc table
        self.crc_table = self._generate_crc_table()

        # boucle principale à 200 Hz
        self.timer = self.create_timer(1.0 / 200.0, self.receiveSensorData)


    def get_command(self, msg: Int16):
        """Subscriber Function to transform the message received\
            for the speed to work with the controller

        Args:
            msg (Int16): the message
        """
        command = []
        cmded_bytes = msg.data
        command.append((cmded_bytes >> 8) & 0xFF)
        command.append(cmded_bytes & 0xFF)

        crc = self.crc32mpeg2(command)

        command.append((crc >> 24) & 0xFF)
        command.append((crc >> 16) & 0xFF)
        command.append((crc >> 8) & 0xFF)
        command.append((crc) & 0xFF)

        # This should not be too big.
        self.tx_buffer = command + [0] * 2

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
            # On combine l'octet actuel avec le CRC et on regarde le résultat dans la table
            crc = ((crc << 8) & 0xffffffff) ^ self.crc_table[(crc >> 24) ^ val]
        return crc

    def publish_range(self, pub, msg_obj, dist, stamp):
        """Met à jour et publie un message Range déjà alloué"""
        msg_obj.header.stamp = stamp
        msg_obj.range = float(dist)
        pub.publish(msg_obj)

    def receiveSensorData(self):
        """Function to receive sensors' data from the STM32 and convert it to publish them
        """
        try:
            self.spi.writebytes(self.tx_buffer)
            data = self.spi.readbytes(20)
        except Exception as e:
            # Affiche l'erreur maximum une fois toutes les 2 secondes
            self.get_logger().error(f"SPI error: {e}", throttle_duration_sec=2.0)
            return

        if self.debug:
            self.get_logger().debug(f"[DEBUG] raw SPI RX: {data}")
        # data = self.spi.xfer2([0x45]*20)  # SPI happens simultaneously, so we need to send to receive.

        # Constants
        timestamp = self.get_clock().now().to_msg()
        ir_max_range = self.ir_max_range

        if not self.crc32mpeg2(data):
            bat_adc_raw = (data[0] << 8) | data[1]   # valeur ADC brute : Le STM32 a un ADC 12 bits, il quantifie sur 4096 valeurs
            yaw = ((data[2] << 8) | data[3]) * self.YAW_FACTOR  # rad - twist around the z axis (vertical) -
            ir_left_raw = (data[4] << 8) | data[5]  # mV - left infrared sensor -
            ir_right_raw = (data[6] << 8) | data[7]   # mV - right infrared sensor -
            raw_speed = 0.002*((data[8] << 8) | data[9])  # m/s speed from the fork - We have to add a factor 2, there is a 1 bit shift for some reason-
            distance_US_raw = 0.01*((data[10] << 8) | data[11])  # m  - distance from the ultrasound sensor -
            acc_x_raw = 0.01*int.from_bytes([data[12], data[13]], byteorder='big', signed=True)  # ms^-2 - acceleration in the x axis -
            yaw_rate = int.from_bytes([data[14], data[15]], byteorder='big', signed=True) * self.YAW_RATE_FACTOR  # rads^-1 - rotation speed around the z-axis of the car -

            ## Fork Speed (abs)
            speed = raw_speed if raw_speed < self.max_fork_speed else 0.    # fork can send aberrant values if it stops partially on a hole.
            self.fork_data.header.stamp = timestamp
            self.fork_data.speed = speed 
            self.speed_pub.publish(self.fork_data)

            ## IMU
            acc_x = acc_x_raw + 0.043   # offset de calibration
            self.imu_data.header.stamp = timestamp
            self.imu_data.orientation.z = math.sin(yaw * 0.5)
            self.imu_data.orientation.w = math.cos(yaw * 0.5)
            self.imu_data.angular_velocity.z = yaw_rate
            self.imu_data.linear_acceleration.x = acc_x
            self.imu_pub.publish(self.imu_data)

            ## Ranges
            # IR Left
            ir_left_v = ir_left_raw / 1000.0  # conversion mV --> V
            if ir_left_v > 0.001 : # Seuil de sécurité
                ir_left = (15.38/ir_left_v - 0.42)/100.0 # conversion V --> m (using component datasheet)
            else:
                ir_left = ir_max_range
            ir_left = min(max(ir_left, 0.0), ir_max_range)

            # IR Right
            ir_right_v = ir_right_raw / 1000.0  # conversion mV --> V
            if ir_right_v > 0.001: # Seuil de sécurité
                ir_right = (15.38/ir_right_v - 0.42)/100.0 # V -> m
            else:
                ir_right = ir_max_range
            ir_right = min(max(ir_right, 0.0), ir_max_range)

            # Sonar
            distance_US = distance_US_raw if distance_US_raw > 0.001 else self.sonar_max_range  # si renvoie 0.0 mets à max_range

            ## Publication individuelle pour Nav2
            self.publish_range(self.pub_ir_left, self.range_msg_left, ir_left, timestamp)
            self.publish_range(self.pub_ir_right, self.range_msg_right, ir_right, timestamp)
            self.publish_range(self.pub_sonar, self.range_msg_sonar, distance_US, timestamp)

            ## Batterie (Conversion ADC -> Volts)
            # bat_adc = bat_adc_raw << 3  # les valeurs ADC semblent décalées de 3 bits. Si c'est vrai => bat_adc : 0 <-> 4095
            # v_adc = bat_adc * (3.3 / 4095.0)  # conversion ADC -> Volts (V_ADC) : entre 0V et 3.3V
            # vbat = v_adc * ((1000 + 560) / 560)   # Pont diviseur de tension dans le schema du Hat : V_ADC = VS * (R3 / (R4 + R3)) avec R3=560, R4=1k

            vbat = bat_adc_raw * self.VBAT_FACTOR   # tension de la batterie en V
            self.vbat_msg.data = float(vbat)
            self.vbat_pub.publish(self.vbat_msg)

            ## sensors data (debug)
            self.sensor_data.data = [yaw, speed, float(ir_left), float(ir_right), distance_US_raw, acc_x, yaw_rate]
            self.stm_pub.publish(self.sensor_data)
            if self.debug:
                self.get_logger().info(f"[DEBUG] -- stm32 sensors value :\n {self.sensor_data}")


def main(args=None):
    rclpy.init(args=args)
    node = STM32_Parser()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.spi.close() # fermer le bus hardware
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

