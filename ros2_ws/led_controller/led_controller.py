from gpio.gpio_controller import GPIOController

class LEDController:
    def __init__(self, led_pins):
        self.gpio = GPIOController()
        self.led_pins = led_pins
        self.gpio.setup_output(self.led_pins)

    def turn_on(self, index):
        self.gpio.write(self.led_pins[index], True)

    def turn_off(self, index):
        self.gpio.write(self.led_pins[index], False)

    def toggle(self, index):
        current = self.gpio.read(self.led_pins[index])
        self.gpio.write(self.led_pins[index], not current)

    def turn_off_all(self):
        for pin in self.led_pins:
            self.gpio.write(pin, False)