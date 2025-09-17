import pigpio

class GPIOController:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Error: could not connect to pigpio daemon.")

    def setup_output(self, pins):
        for pin in pins:
            self.pi.set_mode(pin, pigpio.OUTPUT)

    def write(self, pin: int, state: bool):
        self.pi.write(pin, 1 if state else 0)

    def read(self, pin: int) -> bool:
        return bool(self.pi.read(pin))

    def set_pwm(self, pin: int, duty: float):
        duty = max(0, min(duty, 255))
        self.pi.set_PWM_dutycycle(pin, duty)

    def stop_pwm(self, pin: int):
        self.pi.set_PWM_dutycycle(pin, 0)

    def cleanup(self):
        self.pi.stop()