"""stm32 node. It receives data of the sensors (IMU, infrared, fork) from the STM32 and send it the PWM for the propulsion motor."""

# IMPORTS
import threading
import spidev
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray, Int16
from sensor_msgs.msg import Range, Imu
from bolide_interfaces.msg import ForkSpeed, MultipleRange


class STM32_Parser(Node):
    """ROS2 Class for the STM32
    """
    def __init__(self):
        super().__init__('stm32_node')

        # PARAMS
        self.declare_parameter('debug', False)

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        if self.debug:
            rclpy.logging.set_logger_level('stm32_node', 10)  # 10 is for DEBUG level

        # STM32 set-up
        self.BAUDRATE = 112500

        bus = 0
        device = 1

        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.mode = 0
        self.spi.max_speed_hz = self.BAUDRATE

        # Publishers
        self.stm_pub = self.create_publisher(Float32MultiArray, '/stm32_sensors', 10)
        self.speed_pub = self.create_publisher(ForkSpeed, '/raw_fork_data', 10)
        self.ranges_pub = self.create_publisher(MultipleRange, '/raw_rear_range_data', 10)
        self.imu_pub = self.create_publisher(Imu, '/raw_imu_data', 10)

        # Subscribers
        self.get_cmd = self.create_subscription(Int16, '/stm32_data', self.get_command, 10)

        # Values
        self.sensor_data = Float32MultiArray()  # table for the sensors data (see sheet online)

        # Battery TODO - check it
        self.vbat = 0.  # tension in the battery but it is not used because (maybe) it is broken

        # Infrared sensors
        self.ir_min_range = 0.06
        self.ir_max_range = 0.3
        self.ir_gauche = 0.  # distance from the left infrared sensor
        self.ir_droit = 0.  # distance from the right infrared sensor

        # Fork
        self.speed = 0.  # speed from the fork
        self.fork_data = ForkSpeed()  # Fork message type (see bolide_interfaces)

        # Ultrasound sensor
        self.distance_US = 0.  # distance from the ultrasound sensor

        # IMU
        self.yaw = 0.  # twist around the z axis (vertical)
        self.acc_x = 0.  # acceleration in the x axis
        self.yaw_rate = 0.  # rotation speed around the z-axis of the car

        self.imu_data = Imu()  # IMU message type (see sheet online)

        # Set the covariance of the IMU (surely from datasheet) TODO - check it
        self.imu_data.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1e-2]
        self.imu_data.linear_acceleration_covariance = [1e-1, 0, 0, 0, 0, 0, 0, 0, 0]
        self.imu_data.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1e-2]
        self.imu_data.header.frame_id = "base_link"

        # Transmitter
        self.tx_buffer = [0] * 8  # data that we'll transmit to the STM32

        self.sensors_init()

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

        # This should be not too big.
        self.tx_buffer = command + [0] * 2

    # Definitely not the most efficient way of doing this, but we're transmitting 16 bytes so it's OK.
    # Ideally we'd do all this in Cpp, but I don't think there is a spidev equivalent in Cpp, and it's
    # not like we're being limited by this script.
    def crc32mpeg2(self, buf, crc=0xffffffff):
        """Definitely not the most efficient way of doing this,\
            but we're transmitting 16 bytes so it's OK.
        Ideally we'd do all this in Cpp, but I don't think there\
            is a spidev equivalent in Cpp, and it's not like we're being limited by this script."""
        for val in buf:
            crc ^= val << 24
            for _ in range(8):
                crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
        return crc

    def sensors_init(self):
        """Initialize the sensors settings and the MultiRange message"""
        self.multi_range_frame = MultipleRange()

        # Rear IR sensors ===============================================================
        self.multi_range_frame.ir_rear_left = Range()
        self.multi_range_frame.ir_rear_left.header.frame_id = "rear_ir_range_frame"
        self.multi_range_frame.ir_rear_left.radiation_type = Range.INFRARED
        self.multi_range_frame.ir_rear_left.min_range = self.ir_min_range
        self.multi_range_frame.ir_rear_left.max_range = self.ir_max_range

        self.multi_range_frame.ir_rear_right = Range()
        self.multi_range_frame.ir_rear_right.header.frame_id = "rear_ir_range_frame"
        self.multi_range_frame.ir_rear_right.radiation_type = Range.INFRARED
        self.multi_range_frame.ir_rear_right.min_range = self.ir_min_range
        self.multi_range_frame.ir_rear_right.max_range = self.ir_max_range

        # Rear sonar sensor =============================================================
        # self.multi_range_frame.Sonar_rear = Range()
        # self.multi_range_frame.Sonar_rear.header.frame_id = "rear_sonar_range_frame"
        # self.multi_range_frame.Sonar_rear.radiation_type = Range.ULTRASOUND
        # self.multi_range_frame.Sonar_rear.min_range = self.sonar_min_range
        # self.multi_range_frame.Sonar_rear.max_range = self.sonar_max_range

    def receiveSensorData(self):
        """Function to receive sensors' data from the STM32 and convert it to publish them
        """
        self.spi.writebytes(self.tx_buffer)
        data = self.spi.readbytes(20)

        if self.debug:
            self.get_logger().debug(f"[DEBUG] raw SPI RX: {data}")
        # data = self.spi.xfer2([0x45]*20)  # SPI happens simultaneously, so we need to send to receive.

        if not self.crc32mpeg2(data):
            self.vbat = (data[0] << 8) | data[1]  # UNIT??? It's just an ADC, but I think it clips bc it's always 4095 lmao
            self.yaw = ((data[2] << 8) | data[3])/-900.  # rad
            self.ir_gauche = (data[4] << 8) | data[5]  # mV
            self.ir_droit = (data[6] << 8) | data[7]
            self.speed = 0.002*((data[8] << 8) | data[9])  # ms^-1 - We have to add a factor 2 for some reason, don't ask me why though.
            self.distance_US = 0.01*((data[10] << 8) | data[11])  # m
            self.acc_x = 0.01*int.from_bytes([data[12], data[13]], byteorder='big', signed=True)  # ms^-2
            self.yaw_rate = int.from_bytes([data[14], data[15]], byteorder='big', signed=True)/900.  # rads^-1

            self.sensor_data.data = [self.yaw, self.speed, self.ir_gauche, self.ir_droit, self.distance_US, self.acc_x, self.yaw_rate]

            self.fork_data.speed = self.speed
            self.fork_data.header.stamp = self.get_clock().now().to_msg()

            ##########################  RANGES  #############################

            # Left ====================================
            IR_rear_left = self.ir_gauche/1000.0  # conversion mV --> V
            if IR_rear_left:
                IR_rear_left = 15.38/IR_rear_left - 0.42  # conversion V --> cm
            else:
                IR_rear_left = self.ir_max_range*100.  # because we convert in m later
            # using component datasheet
            # ... and with a bit of experimentation and test to adapt the value
            IR_rear_left /= 100  # conversion cm --> m
            IR_rear_left = min(max(IR_rear_left, 0), self.ir_max_range)

            # Right ===================================
            IR_rear_right = self.ir_droit/1000.  # conversion mV --> V
            if IR_rear_right:
                IR_rear_right = 15.38/IR_rear_right - 0.42  # using component datasheet
            else:
                IR_rear_right = self.ir_max_range*100.  # because we convert in m later
            # using component datasheet
            # ... and with a bit of experimentation and test to adapt the value
            IR_rear_right = IR_rear_right/100.  # conversion cm --> m
            IR_rear_right = min(max(IR_rear_right, 0), self.ir_max_range)

            # self.multi_range_frame.Sonar_rear.range = Sonar_rear
            # self.multi_range_frame.Sonar_rear.header.stamp = rospy.Time.now()
            self.multi_range_frame.ir_rear_left.range = IR_rear_left
            self.multi_range_frame.ir_rear_left.header.stamp = self.get_clock().now().to_msg()
            self.multi_range_frame.ir_rear_right.range = IR_rear_right
            self.multi_range_frame.ir_rear_right.header.stamp = self.get_clock().now().to_msg()

            #########################################################

            self.imu_data.orientation.z = math.sin(self.yaw*0.5)
            self.imu_data.orientation.w = math.cos(self.yaw*0.5)

            self.imu_data.angular_velocity.z = self.yaw_rate
            # self.imu_data.linear_acceleration.x = self.acc_x + 0.043

        # stamp = rospy.Time.now()
            stamp = self.get_clock().now().to_msg()
            self.fork_data.header.stamp = stamp
            self.imu_data.header.stamp = stamp
            self.stm_pub.publish(self.sensor_data)
            if self.debug:
                self.get_logger().debug(f"[DEBUG] -- stm32 sensors value :\n {self.sensor_data}")

        # if (rclpy.ok()):
        self.speed_pub.publish(self.fork_data)
        self.ranges_pub.publish(self.multi_range_frame)
        self.imu_pub.publish(self.imu_data)


def main(args=None):
    rclpy.init(args=args)

    stm32_node = STM32_Parser()
    thread = threading.Thread(target=rclpy.spin, args=(stm32_node, ), daemon=True)
    thread.start()

    rate = stm32_node.create_rate(200)
    while rclpy.ok():
        stm32_node.receiveSensorData()
        rate.sleep()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
