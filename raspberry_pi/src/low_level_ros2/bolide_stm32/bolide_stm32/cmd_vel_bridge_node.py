import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import math


# Twist : linear.x, angular.z
# angle braquage = atan(L * angular.z / linear.x)   L : empattement roues av/ar
# si v ~= 0 -> angle = 0
# clamp angle max

def clamp(x, min_val, max_val):
    return max(min(x, max_val), min_val)

class CmdVelNodeBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge_node')

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )

        self.pub_speed = self.create_publisher(Float32, '/cmd_speed', 10)
        self.pub_steer = self.create_publisher(Float32, '/cmd_dir', 10)

        self.MAX_SPEED = 1.0
        self.MAX_DIR = math.radians(17.0)  # deg -> rad
        self.L = 0.258 # m

    def callback(self, msg):
        propulsion = clamp(msg.linear.x / self.MAX_SPEED, -0.01, 0.01)
        if abs(msg.linear.x) < 10e-9 : 
            angle = 0.0
        else:
            angle = -1*math.atan(self.L * msg.angular.z / msg.linear.x)
        direction = clamp(angle / self.MAX_DIR, -1.0, 1.0)

        self.pub_speed.publish(Float32(data=propulsion))
        self.pub_steer.publish(Float32(data=direction))

def main():
    rclpy.init()
    node = CmdVelNodeBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
