import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pigpio

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.pi = pigpio.pi()
        self.pwm_left = 18
        self.in1_left = 23
        self.in2_left = 24

        self.pwm_right = 17
        self.in1_right = 22
        self.in2_right = 27

        for pin in [self.pwm_left, self.in1_left, self.in2_left,
                     self.pwm_right, self.in1_right, self.in2_right]:
            self.pi.set_mode(pin, pigpio.OUTPUT)

        self.subcription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.vel_callback,
            10
        )

        self.get_logger().info("Motor Controller Node has been started.")

    def vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        left_speed = v - w
        right_speed = v + w

        self.execute_commando_motor(left_speed, self.in1_left, self.in2_left, self.pwm_left)
        self.execute_commando_motor(right_speed, self.in1_right, self.in2_right, self.pwm_right)

    def execute_commando_motor(self, speed, in1, in2, pwm):
        if speed > 0:
            self.pi.write(in1, 1)
            self.pi.write(in2, 0)
        elif speed < 0:
            self.pi.write(in1, 0)
            self.pi.write(in2, 1)
        else:
            self.pi.write(in1, 0)
            self.pi.write(in2, 0)
        
        duty = min(abs(speed), 1.0) * 255
        self.pi.set_PWM_dutycycle(pwm, duty)

    def destroy_node(self):
        self.get_logger().info("Stopping motors...")
        self.pi.set_PWM_dutycycle(self.pwm_left, 0)
        self.pi.set_PWM_dutycycle(self.pwm_right, 0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
