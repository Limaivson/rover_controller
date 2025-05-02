import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from motor_controller.motor_controller import MotorController

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')
        self.motor_driver = MotorController(
            pwm_left=12, in1=5, in2=6,
            pwm_right=13, in3=20, in4=21
        )
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        speed_left = linear - angular
        speed_right = linear + angular
        self.motor_driver.control_left_side(speed_left)
        self.motor_driver.control_right_side(speed_right)