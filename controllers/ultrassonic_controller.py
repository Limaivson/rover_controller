from gpio_controller import GPIOController
import time

class UltrassonicController:
    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.distance = 0
        self.gpio = GPIOController()

    def setup(self):
        self.gpio.setup_output([self.trigger_pin, self.echo_pin])
        self.gpio.write(self.trigger_pin, False)
        self.gpio.write(self.echo_pin, False)
        

    def measure_distance(self):
        self.gpio.write(self.trigger_pin, True)
        time.sleep(0.00001)
        self.gpio.write(self.trigger_pin, False)

        start_time = time.time()
        while not self.gpio.read(self.echo_pin):
            start_time = time.time()

        stop_time = time.time()
        while self.gpio.read(self.echo_pin):
            stop_time = time.time()

        elapsed_time = stop_time - start_time
        self.distance = (elapsed_time * 34300) / 2

    def get_distance(self):
        self.measure_distance()
        return self.distance