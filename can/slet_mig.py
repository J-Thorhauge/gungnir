value = 10000
byte = list(value.to_bytes(4, byteorder='little', signed=True))
print(byte)
print(int.from_bytes(byte, byteorder='little', signed=True))
print(int.from_bytes([0xFF, 0xFF, 0xD8, 0xF0], byteorder='big', signed=True))