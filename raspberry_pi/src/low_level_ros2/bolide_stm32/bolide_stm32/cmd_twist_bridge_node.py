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

class CmdTwistNodeBridge(Node):
    def __init__(self):
        super().__init__('cmd_twist_bridge_node')

        self.sub = self.create_subscription(
            Twist,
            '/cmd_twist',
            self.callback,
            10
        )

        self.pub_speed = self.create_publisher(Float32, '/cmd_speed_target', 10)
        self.pub_steer = self.create_publisher(Float32, '/cmd_dir', 10)


        self.MAX_DIR = math.radians(22.8)  # deg -> rad
        self.L = 0.257 # m

    def callback(self, msg):
        propulsion = clamp(msg.linear.x, -1.5, 3.0)
        if abs(msg.linear.x) < 10e-9 : 
            angle = 0.0
        else:
            angle = math.atan(self.L * msg.angular.z / msg.linear.x)
        direction = clamp(-angle / self.MAX_DIR, -1.0, 1.0)

        self.pub_speed.publish(Float32(data=propulsion))
        self.pub_steer.publish(Float32(data=direction))

def main():
    rclpy.init()
    node = CmdTwistNodeBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
