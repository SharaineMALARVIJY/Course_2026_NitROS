#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


class PS4Controller(Node):
    def __init__(self):
        super().__init__('teleop_ps4_node')

        self.get_logger().info("[INFO] -- PS4 teleop node started")

        self.declare_parameter('debug', False)
        self.debug = bool(self.get_parameter('debug').value)

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('speed_topic', '/cmd_speed_target')
        self.declare_parameter('direction_topic', '/cmd_dir')

        joy_topic = self.get_parameter('joy_topic').value
        speed_topic = self.get_parameter('speed_topic').value
        direction_topic = self.get_parameter('direction_topic').value

        self.speed_pub = self.create_publisher(Float32, speed_topic, 10)
        self.direction_pub = self.create_publisher(Float32, direction_topic, 10)
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)

        self.declare_parameter('rate', 20.0)
        self.rate = float(self.get_parameter('rate').value)
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.declare_parameter('max_speed_forward', 3.0)
        self.declare_parameter('max_speed_reverse', 1.5)
        self.max_speed_forward = float(self.get_parameter('max_speed_forward').value)
        self.max_speed_reverse = float(self.get_parameter('max_speed_reverse').value)

        self.declare_parameter('deadzone', 0.08)
        self.deadzone = float(self.get_parameter('deadzone').value)

        self.declare_parameter('steering_scale', 1.0)
        self.steering_scale = float(self.get_parameter('steering_scale').value)

        self.declare_parameter('speed_scale', 1.0)
        self.speed_scale = float(self.get_parameter('speed_scale').value)

        self.declare_parameter('enable_button_required', True)
        self.enable_button_required = bool(self.get_parameter('enable_button_required').value)

        # Mapping PS4 / joy_node à ajuster si besoin
        self.declare_parameter('speed_axis', 1)          # stick gauche vertical
        self.declare_parameter('direction_axis', 3)      # stick droit horizontal
        self.declare_parameter('enable_button', 5)       # R1
        self.declare_parameter('emergency_button', 4)    # L1
        self.declare_parameter('invert_speed', True)
        self.declare_parameter('invert_direction', False)

        self.speed_axis = int(self.get_parameter('speed_axis').value)
        self.direction_axis = int(self.get_parameter('direction_axis').value)
        self.enable_button = int(self.get_parameter('enable_button').value)
        self.emergency_button = int(self.get_parameter('emergency_button').value)
        self.invert_speed = bool(self.get_parameter('invert_speed').value)
        self.invert_direction = bool(self.get_parameter('invert_direction').value)

        # Rampe
        self.declare_parameter('accel_rate', 1.0)   # augmentation de vitesse
        self.declare_parameter('decel_rate', 2.0)   # relâchement / freinage
        self.accel_rate = float(self.get_parameter('accel_rate').value)
        self.decel_rate = float(self.get_parameter('decel_rate').value)

        # Commandes internes normalisées dans [-1, 1]
        self.target_speed = 0.0
        self.current_speed = 0.0
        self.current_direction = 0.0

        self.get_logger().info(f"[INFO] -- Subscribed to {joy_topic}")
        self.get_logger().info(f"[INFO] -- Publishing speed on {speed_topic}")
        self.get_logger().info(f"[INFO] -- Publishing direction on {direction_topic}")

    def apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def clamp(self, value: float, vmin: float, vmax: float) -> float:
        return max(vmin, min(vmax, value))

    def axis_value(self, axes, index: int, default: float = 0.0) -> float:
        if 0 <= index < len(axes):
            return axes[index]
        return default

    def button_value(self, buttons, index: int, default: int = 0) -> int:
        if 0 <= index < len(buttons):
            return buttons[index]
        return default

    def ramp_towards(self, current: float, target: float, rate: float, dt: float) -> float:
        step = rate * dt
        if target > current:
            return min(current + step, target)
        elif target < current:
            return max(current - step, target)
        return current

    def joy_callback(self, msg: Joy):
        speed_input = -self.axis_value(msg.axes, self.speed_axis, 0.0)
        direction_input = -self.axis_value(msg.axes, self.direction_axis, 0.0)

        if self.invert_speed:
            speed_input = -speed_input
        if self.invert_direction:
            direction_input = -direction_input

        speed_input = self.apply_deadzone(speed_input)
        direction_input = self.apply_deadzone(direction_input)

        target_speed = self.clamp(speed_input * self.speed_scale, -1.0, 1.0)
        direction = self.clamp(direction_input * self.steering_scale, -1.0, 1.0)

        enabled = True
        if self.enable_button_required:
            enabled = bool(self.button_value(msg.buttons, self.enable_button, 0))

        if not enabled:
            target_speed = 0.0
            direction = 0.0

        emergency = bool(self.button_value(msg.buttons, self.emergency_button, 0))
        if emergency:
            target_speed = 0.0
            direction = 0.0
            self.current_speed = 0.0

        self.target_speed = target_speed
        self.current_direction = direction

        if self.debug:
            self.get_logger().info(
                f"[DEBUG] speed_in={speed_input:.2f}, target={self.target_speed:.2f}, "
                f"current={self.current_speed:.2f}, dir={self.current_direction:.2f}, "
                f"enabled={enabled}, emergency={emergency}"
            )

    def timer_callback(self):
        same_sign = (self.current_speed * self.target_speed) > 0.0
        increasing_magnitude = abs(self.target_speed) > abs(self.current_speed)

        if same_sign and increasing_magnitude:
            rate = self.accel_rate
        else:
            rate = self.decel_rate

        self.current_speed = self.ramp_towards(
            self.current_speed,
            self.target_speed,
            rate,
            self.dt
        )

        if self.current_speed > 0.0:
            speed_cmd = self.current_speed * self.max_speed_forward
        else:
            speed_cmd = self.current_speed * self.max_speed_reverse

        self.speed_pub.publish(Float32(data=float(speed_cmd)))
        self.direction_pub.publish(Float32(data=float(self.current_direction)))

    def stop_vehicle(self):
        self.target_speed = 0.0
        self.current_speed = 0.0
        self.current_direction = 0.0
        self.speed_pub.publish(Float32(data=0.0))
        self.direction_pub.publish(Float32(data=0.0))
        self.get_logger().info("[INFO] -- Vehicle set to neutral")


def main(args=None):
    rclpy.init(args=args)
    node = PS4Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("[INFO] -- Shutting down cleanly...")
        node.stop_vehicle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
