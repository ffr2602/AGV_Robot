import struct

def int8_to_registers(value:int):
    return value

def uint8_to_registers(value:int):
    return value

def int16_to_registers(value:int):
    return struct.unpack('H', struct.pack('h', value))[0]

def uint16_to_registers(value:int):
    return value

def int32_to_registers(value:int) -> list[int]:
    return list(struct.unpack('<'+'H'*2, struct.pack('<i', value)))

def uint32_to_registers(value:int) -> list[int]:
    return list(struct.unpack('<'+'H'*2, struct.pack('<I', value)))

def int64_to_registers(value:int) -> list[int]:
    return list(struct.unpack('<'+'H'*4, struct.pack('<q', value)))

def uint64_to_registers(value:int) -> list[int]:
    return list(struct.unpack('<'+'H'*4, struct.pack('<Q', value)))

def float32_to_registers(value:float) -> list[int]:
    return list(struct.unpack('<'+'H'*2, struct.pack('<f', value)))

def float64_to_registers(value:float) -> list[int]:
    return list(struct.unpack('<'+'H'*4, struct.pack('<d', value)))

def string_to_registers(string:str, max_length:int) -> list[int]:
    if len(string) < max_length:
        while True:
            if len(string) != max_length:
                string += '\x00'
            else:
                break 
    if len(string) % 2 != 0:
        string += '\x00'
    return list(struct.unpack('H'*int(len(string)*0.5), struct.pack(f'{len(string)}s', string.encode('ascii'))))

def registers_to_int8(registers:int) -> int:
    return struct.unpack('b', struct.pack('B', registers))[0]

def registers_to_uint8(registers:int) -> int:
    return registers

def registers_to_int16(registers:int) -> int:
    return struct.unpack('h', struct.pack('H', registers))[0]
    
def registers_to_uint16(registers:int) -> int:
    return registers

def registers_to_int32(registers:list[int]):
    return struct.unpack('<i', struct.pack('<'+'H'*len(registers), *registers))[0]

def registers_to_uint32(registers:list[int]):
    return struct.unpack('<I', struct.pack('<'+'H'*len(registers), *registers))[0]

def registers_to_int64(registers:list[int]):
    return struct.unpack('<q', struct.pack('<'+'H'*len(registers), *registers))[0]

def registers_to_uint64(registers:list[int]):
    return struct.unpack('<Q', struct.pack('<'+'H'*len(registers), *registers))[0]

def registers_to_float32(registers:list[int]): 
    return round(struct.unpack('<f', struct.pack('<'+'H'*len(registers), *registers))[0], 2)

def registers_to_float64(registers:list[int]): 
    return round(struct.unpack('<d', struct.pack('<'+'H'*len(registers), *registers))[0], 4)

def registers_to_string(registers:list[int]) -> str:
    return struct.unpack(f'{len(registers)*2}s', struct.pack('H'*len(registers), *registers))[0].decode('ascii').strip('\x00')
