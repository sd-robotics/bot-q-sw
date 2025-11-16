from smbus2 import SMBus

bus = SMBus(1)
addr = 0x40

data = bus.read_byte_data(addr, 0x00)

print(f"Data: {data}")
