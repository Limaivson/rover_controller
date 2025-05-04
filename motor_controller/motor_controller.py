from gpio.gpio_controller import GPIOController

class MotorController:
    def __init__(self, pwm_left, in1, in2, pwm_right, in3, in4):
        self.gpio = GPIOController()
        self.pins = {
            "pwm_left": pwm_left,
            "in1": in1,
            "in2": in2,
            "pwm_right": pwm_right,
            "in3": in3,
            "in4": in4,
        }
        self.gpio.setup_output(self.pins.values())

    def _control_motor(self, speed, in_a, in_b, pwm_pin):
        if speed > 0:
            self.gpio.write(in_a, True)
            self.gpio.write(in_b, False)
        elif speed < 0:
            self.gpio.write(in_a, False)
            self.gpio.write(in_b, True)
        else:
            self.gpio.write(in_a, False)
            self.gpio.write(in_b, False)

        duty = min(abs(speed), 1.0) * 255
        self.gpio.set_pwm(pwm_pin, duty)

    def control_left_side(self, speed):
        self._control_motor(speed, self.pins["in1"], self.pins["in2"], self.pins["pwm_left"])

    def control_right_side(self, speed):
        self._control_motor(speed, self.pins["in3"], self.pins["in4"], self.pins["pwm_right"])

    def stop(self):
        self.gpio.stop_pwm(self.pins["pwm_left"])
        self.gpio.stop_pwm(self.pins["pwm_right"])
