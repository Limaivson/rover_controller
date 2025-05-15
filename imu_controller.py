import smbus

class IMUController:
    def __init__(self, address=0x68, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Wake MPU6050

    def read_data(self):
        def read_word(reg):
            high = self.bus.read_byte_data(self.address, reg)
            low = self.bus.read_byte_data(self.address, reg+1)
            val = (high << 8) + low
            return val - 65536 if val >= 0x8000 else val

        ax = read_word(0x3B) / 16384.0
        ay = read_word(0x3D) / 16384.0
        az = read_word(0x3F) / 16384.0
        gx = read_word(0x43) / 131.0
        gy = read_word(0x45) / 131.0
        gz = read_word(0x47) / 131.0

        return {"accel": {"x": ax, "y": ay, "z": az}, "gyro": {"x": gx, "y": gy, "z": gz}}
    