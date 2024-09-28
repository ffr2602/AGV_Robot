import struct
from bridge_plc_modbus import bridge_hub

app = bridge_hub()

def float_to_registers(float_value):
    # Convert float to 4 bytes (big-endian)
    float_bytes = struct.pack('>f', float_value)
    # Unpack the bytes into two 16-bit Modbus registers
    reg1, reg2 = struct.unpack('>HH', float_bytes)
    return reg1, reg2

# Example float value
float_value = 11.50

# Convert float to Modbus registers
register1, register2 = float_to_registers(float_value)

print(f"Float: {float_value}")
print(f"Register 1: {register1}")
print(f"Register 2: {register2}")

data = app.registers_to_float(float_to_registers(float_value))
print(data)
