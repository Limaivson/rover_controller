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

        if linear != 0.0 and angular == 0.0:
            speed_left = linear
            speed_right = linear

        elif linear == 0.0 and angular != 0.0:
            speed_left = -angular
            speed_right = angular

        elif linear != 0.0 and angular != 0.0:
            speed_left = linear - angular
            speed_right = linear + angular

        else:
            speed_left = 0.0
            speed_right = 0.0

        self.motor_driver.control_left_side(speed_left)
        self.motor_driver.control_right_side(speed_right)


def main(args=None):
    rclpy.init(args=args)
    node = RoverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
