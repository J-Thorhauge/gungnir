value = 10000
byte = list(value.to_bytes(4, byteorder='little', signed=True))
print(byte)
print(int.from_bytes(byte, byteorder='little', signed=True))
print(int.from_bytes([0xD4, 0x56, 0x1B, 0x03], byteorder='little', signed=True))