import os
import json
import serial
from pymodbus.client import ModbusTcpClient, ModbusSerialClient
from pymodbus.server import StartSerialServer, StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusServerContext, ModbusSlaveContext, ModbusSequentialDataBlock
from module.konversi import *
from module.can_bus import *

slave = 1
input = 0x0000
output = 0x0600
auxiliary = 0x0C00

store = ModbusSlaveContext(
    di=ModbusSequentialDataBlock(0, [0] * 10000),
    co=ModbusSequentialDataBlock(0, [0] * 10000),
    hr=ModbusSequentialDataBlock(0, [0] * 10000),
    ir=ModbusSequentialDataBlock(0, [0] * 10000)
)
context = ModbusServerContext(slaves={0x01: store}, single=False)

identity = ModbusDeviceIdentification()
identity.VendorName = 'Ruang Industri Indonesia'
identity.ProductCode = 'RII-AGV'
identity.VendorUrl = 'https://www.ruangindustri.com/'
identity.ProductName = 'Modbus Server'
identity.ModelName = 'Modbus Server'
identity.MajorMinorRevision = '1.0.0-alpha'

file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../rule/node.json'))
with open(os.path.abspath(os.path.join(os.path.dirname(__file__), '../config/setting.json')), "r") as aa:
    config = json.load(aa)
with open(os.path.abspath(os.path.join(os.path.dirname(__file__), "../config/io_map.json")), "r") as bb:
    io_map = json.load(bb)
with open(os.path.abspath(os.path.join(os.path.dirname(__file__), "../config/slave_map.json")), "r") as cc:
    slave_map_io = json.load(cc)


def delete_data(route_id, node_id):
    with open(file_path, "r") as json_file:
        data = json.load(json_file)
    if str(route_id) in data and str(node_id) in data[str(route_id)]:
        del data[str(route_id)][str(node_id)]
        if not data[str(route_id)]:
            del data[str(route_id)]
    with open(file_path, "w") as json_file:
        json.dump(data, json_file, indent=4)

def add_or_update_data(route_id, node_id):
    if route_id != 0 and node_id != 0:
        new_data = {}
        for item in range(len(slave_map_io['map_io']['node']['motion'])):
            new_data[f"motion_{f'{item+1}'}"] = [
                registers_to_string(get_multiple_holding_registers(slave_map_io['map_io']['node']['motion'][f'{item+1}']['name'], 6)), 
                get_single_holding_register(slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter'])
            ]
        with open(file_path, "r") as json_file:
            data = json.load(json_file)
        if str(route_id) not in data:
            data[str(route_id)] = {}
        data[str(route_id)][str(node_id)] = new_data
        with open(file_path, "w") as json_file:
            json.dump(data, json_file, indent=4)

def view_data(route_id, node_id):
    with open(file_path, "r") as json_file:
        data = json.load(json_file)
    for item in range(len(slave_map_io['map_io']['node']['motion'])):
        set_multiple_holding_registers(slave_map_io['map_io']['node']['motion'][f'{item+1}']['name'], list(string_to_registers("", 12)))
        set_multiple_holding_registers(slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter'], [0])
    if str(route_id) in data and str(node_id) in data[str(route_id)]:
        for item in range(len(slave_map_io['map_io']['node']['motion'])):
            set_multiple_holding_registers(slave_map_io['map_io']['node']['motion'][f'{item+1}']['name'], list(string_to_registers(data[str(route_id)][str(node_id)]["motion_{:}".format(item + 1)][0], 12)))
            set_multiple_holding_registers(slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter'], [data[str(route_id)][str(node_id)]["motion_{:}".format(item + 1)][1]])

def insert_data(route_id, pre_id):
    with open(file_path, "r") as json_file:
        data = json.load(json_file)
    for item in range(len(slave_map_io['map_io']['node']['motion'])):
        set_multiple_holding_registers(slave_map_io['map_io']['node']['motion'][f'{item+1}']['name'], list(string_to_registers("", 12)))
        set_multiple_holding_registers(slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter'], [0])
    if str(route_id) in data and str(pre_id) in data[str(route_id)]:
        for item in range(len(slave_map_io['map_io']['node']['motion'])):
            set_multiple_holding_registers(slave_map_io['map_io']['node']['motion'][f'{item+1}']['name'], list(string_to_registers(data[str(route_id)][str(pre_id)]["motion_{:}".format(item + 1)][0], 12)))
            set_multiple_holding_registers(slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter'], [data[str(route_id)][str(pre_id)]["motion_{:}".format(item + 1)][1]])

def query_route(route_id):
    for i in range(slave_map_io['map_io']['node']['view_route']['end']):
        set_multiple_holding_registers(slave_map_io['map_io']['node']['view_route']['start'] + i, [0])
    with open(file_path, "r") as file_json:
        data = json.load(file_json)
    if str(route_id) in data:
        data_list = list(data[str(route_id)].keys())
        for item in range(len(data_list)):
            set_multiple_holding_registers(slave_map_io['map_io']['node']['view_route']['start'] + item, [int(data_list[item])])


def start_modbus_server_serial(context, identity, serial_kwargs):
    StartSerialServer(context=context, identity=identity, **serial_kwargs)

def start_modbus_server_tcp(context, identity, tcp_kwargs):
    StartTcpServer(context=context, identity=identity, **tcp_kwargs)


def get_cpu_temperature():
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as file:
        temp_str = file.read().strip()
        temperature = int(temp_str) / 1000.0
    return temperature

def get_uptime():
    with open("/proc/uptime", "r") as f:
        uptime_seconds = int(float(f.readline().split()[0]))
    hours = uptime_seconds // 3600
    minutes = (uptime_seconds % 3600) // 60
    seconds = uptime_seconds % 60
    return [hours, minutes, seconds]

def speed(value: int):
    speed = value * config['Max_Speed'] * 0.01
    return speed

def save_config():
    config['ID_CAR'] = get_single_holding_register(slave_map_io['map_io']['setting']['idcar'])
    config['Acceleration'] = get_single_holding_register(slave_map_io['map_io']['setting']['accel'])
    config['Default_Music'] = get_single_holding_register(slave_map_io['map_io']['setting']['music'])
    config['Select_Track'] = get_single_holding_register(slave_map_io['map_io']['setting']['track'])
    config['Obstacle'] = get_single_holding_register(slave_map_io['map_io']['setting']['obstacle'])
    with open(os.path.abspath(os.path.join(os.path.dirname(__file__), '../config/setting.json')), "w") as json_file:
        json.dump(config, json_file, indent=4)

# ============================================================
# ============================== Modbus Function ==============================
# ============================== Coils ==============================

def get_single_coil(addr):
    return context[slave].getValues(1, addr, 1)[0] 

def get_multiple_coils(addr, count):
    return context[slave].getValues(1, addr, count)

def set_single_coil(addr, value: bool):
    context[slave].setValues(1, addr, [value])

def set_multiple_coils(addr, value: list[bool]):
    context[slave].setValues(1, addr, value)

# ============================== Discrete Inputs ==============================

def get_single_discrete_input(addr):
    return context[slave].getValues(2, addr, 1)[0] 

def get_multiple_discrete_inputs(addr, count):
    return context[slave].getValues(2, addr, count)

def set_single_discrete_input(addr, value: bool):
    context[slave].setValues(2, addr, [value])

def set_multiple_discrete_inputs(addr, value: list[bool]):
    context[slave].setValues(2, addr, value)

# ============================== Input Registers ==============================

def get_single_input_register(addr):
    return context[slave].getValues(4, addr, 1)[0] 

def get_multiple_input_registers(addr, count):
    return context[slave].getValues(4, addr, count)

def set_single_input_register(addr, value: int):
    context[slave].setValues(4, addr, [value])

def set_multiple_input_registers(addr, value: list[int]):
    context[slave].setValues(4, addr, value)

# ============================== Holding Registers ==============================

def get_single_holding_register(addr):
    return context[slave].getValues(3, addr, 1)[0] 

def get_multiple_holding_registers(addr, count):
    return context[slave].getValues(3, addr, count)

def set_single_holding_register(addr, value: int):
    context[slave].setValues(3, addr, [value])

def set_multiple_holding_registers(addr, value: list[int]):
    context[slave].setValues(3, addr, value)

# ============================================================ 
# ============================ I/O Mapping ================================
# ============================ inputs ============================

def emergency_stop(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['emergency_stop'])
    return data.bits[0]

def start(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['start'])
    return data.bits[0]

def stop(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['stop'])
    return data.bits[0]

def reset(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['reset'])
    return data.bits[0]

def Obstacle_warning(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['obstacle_sensor']['warning'])
    return data.bits[0]

def obstacle_stop(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['obstacle_sensor']['stop'])
    return data.bits[0]

def button_wheel(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['wheel']['button'])
    return data.bits[0]

def wheel_up(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['wheel']['wheel_up'])
    return data.bits[0]

def wheel_down(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['wheel']['wheel_down'])
    return data.bits[0]

def hook_up(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['hook']['hook_up'])
    return data.bits[0]

def hook_down(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['hook']['hook_down'])
    return data.bits[0]

def bumper(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['bumper'])
    return data.bits[0]

# ============================ outputs ============================

def wheel_c(conn: ModbusTcpClient | ModbusSerialClient, data: bool):
    conn.write_coil(auxiliary + io_map['outputs']['wheel'], data)

def hook_c(conn: ModbusTcpClient | ModbusSerialClient, data: bool):
    conn.write_coil(auxiliary + io_map['outputs']['hook'], data)

def indicator(conn: ModbusTcpClient | ModbusSerialClient, data: list[bool]):
    conn.write_coil(output + io_map['outputs']['lamp']['red'], data[0])
    conn.write_coil(output + io_map['outputs']['lamp']['green'], data[1])
    conn.write_coil(output + io_map['outputs']['lamp']['yellow'], data[2])

def sound(conn: ModbusTcpClient | ModbusSerialClient, data: list[bool]):
    conn.write_coil(output + io_map['outputs']['sound']['sound1'], data[0])
    conn.write_coil(output + io_map['outputs']['sound']['sound2'], data[1])
    conn.write_coil(output + io_map['outputs']['sound']['sound3'], data[2])
    conn.write_coil(output + io_map['outputs']['sound']['sound4'], data[3])

# ============================ node ============================

def route():
    if get_single_coil(slave_map_io['map_io']['node']['button']['add&change']):
        data_1 = get_single_holding_register(slave_map_io['map_io']['node']['route'])
        data_2 = get_single_holding_register(slave_map_io['map_io']['node']['rfid'])
        if data_1 != 0 and data_2 != 0:
            add_or_update_data(data_1, data_2)
        set_single_coil(slave_map_io['map_io']['node']['button']['add&change'], 0)
    elif get_single_coil(slave_map_io['map_io']['node']['button']['insert']):
        data_1 = get_single_holding_register(slave_map_io['map_io']['node']['route'])
        data_2 = get_single_holding_register(slave_map_io['map_io']['node']['pre_rfid'])
        if data_1 != 0 and data_2 != 0:
            insert_data(data_1, data_2)
        set_single_coil(slave_map_io['map_io']['node']['button']['insert'], 0)
    elif get_single_coil(slave_map_io['map_io']['node']['button']['delete']):
        data_1 = get_single_holding_register(slave_map_io['map_io']['node']['route'])
        data_2 = get_single_holding_register(slave_map_io['map_io']['node']['rfid'])
        if data_1 != 0 and data_2 != 0:
            delete_data(data_1, data_2)
        set_single_coil(slave_map_io['map_io']['node']['button']['delete'], 0)
    elif get_single_coil(slave_map_io['map_io']['node']['button']['view']):
        data_1 = get_single_holding_register(slave_map_io['map_io']['node']['route'])
        data_2 = get_single_holding_register(slave_map_io['map_io']['node']['rfid'])
        if data_1 != 0 and data_2 != 0:
            view_data(data_1, data_2)
    elif get_single_coil(slave_map_io['map_io']['node']['button']['query']):
        data_1 = get_single_holding_register(slave_map_io['map_io']['node']['route'])
        if data_1 != 0:
            query_route(data_1)

def status(data:list, com: bool = True):
    set_multiple_input_registers(slave_map_io['map_io']['status']['temp']['CPU'], float32_to_registers(data[0]))
    if com == True:
        set_multiple_input_registers(slave_map_io['map_io']['status']['temp']['Driver'], float32_to_registers(data[1]))
        set_single_input_register(slave_map_io['map_io']['status']['motor']['fault']['m1'], data[2])
        set_single_input_register(slave_map_io['map_io']['status']['motor']['fault']['m2'], data[3])
        set_multiple_input_registers(slave_map_io['map_io']['status']['motor']['current']['m1'], float32_to_registers(struct.unpack("h", struct.pack("H", data[4]))[0]))
        set_multiple_input_registers(slave_map_io['map_io']['status']['motor']['current']['m2'], float32_to_registers(struct.unpack("h", struct.pack("H", data[5]))[0]))
        set_multiple_input_registers(slave_map_io['map_io']['status']['voltage'], float32_to_registers(data[9]))
    else:
        set_multiple_input_registers(slave_map_io['map_io']['status']['temp']['Driver'], float32_to_registers(0.0))
        set_single_input_register(slave_map_io['map_io']['status']['motor']['fault']['m1'], 0)
        set_single_input_register(slave_map_io['map_io']['status']['motor']['fault']['m2'], 0)
        set_multiple_input_registers(slave_map_io['map_io']['status']['motor']['current']['m1'], float32_to_registers(0.0))
        set_multiple_input_registers(slave_map_io['map_io']['status']['motor']['current']['m2'], float32_to_registers(0.0))
        set_multiple_input_registers(slave_map_io['map_io']['status']['voltage'], float32_to_registers(0.0))
    set_single_discrete_input(slave_map_io['map_io']['status']['comm']['TCP'], data[6])
    set_single_discrete_input(slave_map_io['map_io']['status']['comm']['CAN'], data[7])
    set_single_discrete_input(slave_map_io['map_io']['status']['comm']['bumper'], data[8])

def set_RFID(data):
    ser = None
    ser = serial.Serial()
    ser.port = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0'
    ser.baudrate = 19200
    ser.timeout = 1000
    ser.open()

    if ser.is_open:
        bufSend = [170, 85, 4, 0, 0, 1, 171]
        ser.write(bufSend)
        SetID = int(data)
        parset = [1, 2, 3, 4, 5, 6, 7, 8, 172]
        parset[4] = SetID >> 56 & 255
        parset[5] = SetID >> 48 & 255
        parset[6] = SetID >> 40 & 255
        parset[7] = SetID >> 32 & 255
        parset[0] = SetID >> 24 & 255
        parset[1] = SetID >> 16 & 255
        parset[2] = SetID >> 8 & 255
        parset[3] = SetID >> 0 & 255
        bufSend = [170, 81, 11, 0, 0]
        bufSend.extend(parset)
        ser.write(bufSend)
    ser.close()
