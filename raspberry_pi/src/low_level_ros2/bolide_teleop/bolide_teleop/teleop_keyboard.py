import threading
import click

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
from bolide_interfaces.msg import SpeedDirection, ForkSpeed


class KeyboardController(Node):
    """ROS2 Class of the Keyboard control
    """
    def __init__(self):
        super().__init__('teleop_node')

        self.get_logger().info("[INFO] -- Teleop node started, Keyboard interrupt (CTRL+C) will stop the node")

        # Debug
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').value

        # create publish of speed and direction
        self.speed_pub = self.create_publisher(Float32, '/cmd_speed_target', 10)
        self.direction_pub = self.create_publisher(Float32, '/cmd_dir', 10)

        # Timer for the data
        self.declare_parameter('rate', 10)                          # cmd_vel publication frequency (Hz)
        rate = self.get_parameter('rate').value
        self.timer = self.create_timer(1/rate, self.timer_callback)

        # init speed and direction
        self.current_speed = 0.0
        self.current_direction = 0.0
        
        # Speed and direction increments
        self.declare_parameter('speed_increment', 0.1)                         # 4% increment per key press
        self.speed_increment = self.get_parameter('speed_increment').value
        self.declare_parameter('direction_increment', 0.2)                      # 20% increment per key press
        self.direction_increment = self.get_parameter('direction_increment').value
        
        self.declare_parameter('max_speed_forward', 3.0)
        self.declare_parameter('max_speed_reverse', 1.5)
        self.max_speed_forward = self.get_parameter('max_speed_forward').value
        self.max_speed_reverse = self.get_parameter('max_speed_reverse').value
        
        self.key_mapping = {
            '\x1b[A': 'UP',     
            '\x1b[B': 'DOWN',
            '\x1b[C': 'RIGHT',
            '\x1b[D': 'LEFT', 
            's': 'STRAIGHT', 
            'q': 'QUIT', 
            'a': 'HALT',
            'r': 'DIRECTION REVERSAL',
            'v': 'VOLTAGE'
        }
        key_mapping_str = '\n'.join([f'\t{key} : {value}' for key, value in self.key_mapping.items()])
        self.get_logger().info(f"[INFO] -- Key control mapping:\n{key_mapping_str}")

        self.running = True

        # info subscription
        self.create_subscription(Float32, '/battery_voltage',  self.cb_vbat, 10)
        self.vbat = 0.0

    def cb_vbat(self, msg: Float32):
        self.vbat = msg.data

    def timer_callback(self):
        """Callback function of the timer to publish data to speed and direction topics
        """
        if self.debug:
            self.get_logger().debug(f"[DEBUG] -- current speed = {self.current_speed}\n\r")
            self.get_logger().debug(f"[DEBUG] -- current direction = {self.current_direction}\n\r")
        if self.current_speed > 0 :
            self.speed_pub.publish(Float32(data=self.current_speed * self.max_speed_forward))
        else :
            self.speed_pub.publish(Float32(data=self.current_speed * self.max_speed_reverse))
            
        self.direction_pub.publish(Float32(data=self.current_direction))
    
        
    def stop_vehicle(self):
        """Force the vehicle to neutral and publish once."""
        self.current_speed = 0.0
        self.current_direction = 0.0
        self.speed_pub.publish(Float32(data=0.0))
        self.direction_pub.publish(Float32(data=0.0))
        self.get_logger().info("[INFO] -- Vehicle set to neutral")

    def perform_action(self):
        """Perform action depending on the key pressed.
        """
        try:
            mykey = click.getchar()
        except KeyboardInterrupt:
            self.current_speed = 0.0
            self.current_direction = 0.0
            self.running = False
            return
        
        action = self.key_mapping.get(mykey, '')

        if action == '':
            # Unknown key, do nothing
            return
            
        if action == 'UP':
            # Increment speed (forward)
            self.current_speed += self.speed_increment
            # Clamp to max 1.0
            if self.current_speed > 1.0:
                self.current_speed = 1.0
                
        elif action == 'DOWN':
            # Decrement speed (backward)
            self.current_speed -= self.speed_increment
            # Clamp to min -1.0
            if self.current_speed < -1.0:
                self.current_speed = -1.0
                
        elif action == 'LEFT':
            # Turn left (negative direction)
            self.current_direction -= self.direction_increment
            # Clamp to min -1.0
            if self.current_direction < -1.0:
                self.current_direction = -1.0
                
        elif action == 'RIGHT':
            # Turn right (positive direction)
            self.current_direction += self.direction_increment
            # Clamp to max 1.0
            if self.current_direction > 1.0:
                self.current_direction = 1.0
                
        elif action == 'STRAIGHT':
            # sets the steering direction to neutral
            self.current_direction = 0.0
            
        elif action == 'QUIT':
            # Set to neutral before quitting
            self.current_speed = 0.0
            self.current_direction = 0.0
            self.running = False
            
        elif action == 'HALT':
            # Reset speed and direction to neutral
            self.current_speed = 0.0
            self.current_direction = 0.0
            # Note : brake si cmd_vel = 0.0

        elif action == 'DIRECTION REVERSAL':
            # Reverse drive direction with same speed (not steering direction)
            self.current_speed = -self.current_speed
        
        elif action == 'VOLTAGE':
            self.get_logger().info(f"\r[VBAT] tension = {self.vbat:.2f} V                                                      \r")

        else:
            self.get_logger().warn(f"Unknown action: {action}")

def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()

    try:
        while controller.running:
            controller.perform_action()

    except KeyboardInterrupt:
        pass  # capturé ici aussi par sécurité

    finally:
        controller.get_logger().info("[INFO] -- Shutting down cleanly...")
        controller.stop_vehicle()
        controller.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=1.0)

if __name__ == '__main__':
    main()


