value = 10000
byte = list(value.to_bytes(4, byteorder='little', signed=True))
print(int.from_bytes(byte, byteorder='little', signed=True))