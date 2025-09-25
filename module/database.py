import os
import json
import serial
from pymodbus.client import ModbusTcpClient, ModbusSerialClient
from pymodbus.server import StartSerialServer, StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusServerContext, ModbusSlaveContext, ModbusSequentialDataBlock
from module.konversi import *
from module.can_bus import *
from threading import Thread

slave = 1
input = 0x0C00
output = 0x0C00
conn = ModbusTcpClient(host="192.168.1.111")
serial_kwargs = {
    "port": "/dev/ttyS0",
    "baudrate": 57600,
    "parity": "N",
    "stopbits": 1,
    "bytesize": 8,
    "framer": None,
}

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
                registers_to_string(get_value(slave_map_io['map_io']['node']['motion'][f'{item+1}']['name']['fc'], slave_map_io['map_io']['node']['motion'][f'{item+1}']['name']['addr'], 6)), 
                get_single_value(slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter']['fc'], slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter']['addr'])
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
    if str(route_id) in data and str(node_id) in data[str(route_id)]:
        for item in range(len(slave_map_io['map_io']['node']['motion'])):
            set_value(slave_map_io['map_io']['node']['motion'][f'{item+1}']['name']['fc'], slave_map_io['map_io']['node']['motion'][f'{item+1}']['name']['addr'], list(string_to_registers("", 12)))
            set_value(slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter']['fc'], slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter']['addr'], [0])
            set_value(slave_map_io['map_io']['node']['motion'][f'{item+1}']['name']['fc'], slave_map_io['map_io']['node']['motion'][f'{item+1}']['name']['addr'], list(string_to_registers(data[str(route_id)][str(node_id)]["motion_{:}".format(item + 1)][0], 12)))
            set_value(slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter']['fc'], slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter']['addr'], [data[str(route_id)][str(node_id)]["motion_{:}".format(item + 1)][1]])


def insert_data(route_id, pre_id):
    with open(file_path, "r") as json_file:
        data = json.load(json_file)
    if str(route_id) in data and str(pre_id) in data[str(route_id)]:
        for item in range(5):
            set_value(slave_map_io['map_io']['node']['motion'][f'{item+1}']['name']['fc'], slave_map_io['map_io']['node']['motion'][f'{item+1}']['name']['addr'], list(string_to_registers("", 12)))
            set_value(slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter']['fc'], slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter']['addr'], [0])
            set_value(slave_map_io['map_io']['node']['motion'][f'{item+1}']['name']['fc'], slave_map_io['map_io']['node']['motion'][f'{item+1}']['name']['addr'], list(string_to_registers(data[str(route_id)][str(pre_id)]["motion_{:}".format(item + 1)][0], 12)))
            set_value(slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter']['fc'], slave_map_io['map_io']['node']['motion'][f'{item+1}']['parameter']['addr'], [data[str(route_id)][str(pre_id)]["motion_{:}".format(item + 1)][1]])


def query_route(route_id):
    for i in range(slave_map_io['map_io']['node']['view_route']['end']['addr']):
        set_value(slave_map_io['map_io']['node']['view_route']['start']['fc'], slave_map_io['map_io']['node']['view_route']['start']['addr'] + i, [0])
    with open(file_path, "r") as file_json:
        data = json.load(file_json)
    if str(route_id) in data:
        data_list = list(data[str(route_id)].keys())
        for item in range(len(data_list)):
            set_value(slave_map_io['map_io']['node']['view_route']['start']['fc'], slave_map_io['map_io']['node']['view_route']['start']['addr'] + item, [int(data_list[item])])


def start_modbus_server_serial(context, identity, serial_kwargs):
    StartSerialServer(context=context, identity=identity, **serial_kwargs)


def start_modbus_server_tcp(context, identity, tcp_kwargs):
    StartTcpServer(context=context, identity=identity, **tcp_kwargs)


def set_value(fc, address, value: list[int]):
    context[slave].setValues(fc, address, value)


def get_value(fc, address, count, slave=1):
    return context[slave].getValues(fc, address, count)


def get_single_value(fc, address, slave=1):
    return context[slave].getValues(fc, address, 1)[0]


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
    speed = value * get_value(3, 1010, 1)[0] * 0.01
    return -speed


def save_config():
    config['ID_CAR'] = get_single_value(slave_map_io['map_io']['setting']['idcar']['fc'], slave_map_io['map_io']['setting']['idcar']['addr'])
    config['Acceleration'] = get_single_value(slave_map_io['map_io']['setting']['accel']['fc'], slave_map_io['map_io']['setting']['accel']['addr'])
    config['Default_Music'] = get_single_value(slave_map_io['map_io']['setting']['music']['fc'], slave_map_io['map_io']['setting']['music']['addr'])
    config['Select_Track'] = get_single_value(slave_map_io['map_io']['setting']['track']['fc'], slave_map_io['map_io']['setting']['track']['addr'])
    config['Obstacle'] = get_single_value(slave_map_io['map_io']['setting']['obstacle']['fc'], slave_map_io['map_io']['setting']['obstacle']['addr'])
    with open(os.path.abspath(os.path.join(os.path.dirname(__file__), '../config/setting.json')), "w") as json_file:
        json.dump(config, json_file, indent=4)


# ============================ inputs ============================


def emergency_stop(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['emergency_stop']['address'])
    return data.bits[0]


def start(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['start']['address'])
    return data.bits[0]


def stop(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['stop']['address'])
    return data.bits[0]


def reset(conn: ModbusTcpClient | ModbusSerialClient):
    data = conn.read_coils(input + io_map['inputs']['reset']['address'])
    return data.bits[0]

# ============================ outputs ============================


def indicator(conn: ModbusTcpClient | ModbusSerialClient, data: str):
    if data == "R":
        conn.write_coil(output + io_map['outputs']['lamp']['red']['address'], 1)
        conn.write_coil(output + io_map['outputs']['lamp']['green']['address'], 0)
        conn.write_coil(output + io_map['outputs']['lamp']['yellow']['address'], 0)
    elif data == "G":
        conn.write_coil(output + io_map['outputs']['lamp']['red']['address'], 0)
        conn.write_coil(output + io_map['outputs']['lamp']['green']['address'], 1)
        conn.write_coil(output + io_map['outputs']['lamp']['yellow']['address'], 0)
    elif data == "Y":
        conn.write_coil(output + io_map['outputs']['lamp']['red']['address'], 0)
        conn.write_coil(output + io_map['outputs']['lamp']['green']['address'], 0)
        conn.write_coil(output + io_map['outputs']['lamp']['yellow']['address'], 1)
    else:
        conn.write_coil(output + io_map['outputs']['lamp']['red']['address'], 0)
        conn.write_coil(output + io_map['outputs']['lamp']['green']['address'], 0)
        conn.write_coil(output + io_map['outputs']['lamp']['yellow']['address'], 0)


def sound(conn: ModbusTcpClient | ModbusSerialClient, data: int):
    if data == 1:
        conn.write_coil(io_map['outputs']['sound']['sound1']['address'], 1)
        conn.write_coil(io_map['outputs']['sound']['sound2']['address'], 0)
        conn.write_coil(io_map['outputs']['sound']['sound3']['address'], 0)
        conn.write_coil(io_map['outputs']['sound']['sound4']['address'], 0)
    elif data == 2:
        conn.write_coil(io_map['outputs']['sound']['sound1']['address'], 0)
        conn.write_coil(io_map['outputs']['sound']['sound2']['address'], 1)
        conn.write_coil(io_map['outputs']['sound']['sound3']['address'], 0)
        conn.write_coil(io_map['outputs']['sound']['sound4']['address'], 0)
    elif data == 3:
        conn.write_coil(io_map['outputs']['sound']['sound1']['address'], 0)
        conn.write_coil(io_map['outputs']['sound']['sound2']['address'], 0)
        conn.write_coil(io_map['outputs']['sound']['sound3']['address'], 1)
        conn.write_coil(io_map['outputs']['sound']['sound4']['address'], 0)
    elif data == 4:
        conn.write_coil(io_map['outputs']['sound']['sound1']['address'], 0)
        conn.write_coil(io_map['outputs']['sound']['sound2']['address'], 0)
        conn.write_coil(io_map['outputs']['sound']['sound3']['address'], 0)
        conn.write_coil(io_map['outputs']['sound']['sound4']['address'], 1)
    else:
        conn.write_coil(io_map['outputs']['sound']['sound1']['address'], 0)
        conn.write_coil(io_map['outputs']['sound']['sound2']['address'], 0)
        conn.write_coil(io_map['outputs']['sound']['sound3']['address'], 0)
        conn.write_coil(io_map['outputs']['sound']['sound4']['address'], 0)


def route():
    if get_single_value(slave_map_io['map_io']['node']['button']['add&change']['fc'], slave_map_io['map_io']['node']['button']['add&change']['addr']):
        data_1 = get_single_value(slave_map_io['map_io']['node']['route']['fc'], slave_map_io['map_io']['node']['route']['addr'])
        data_2 = get_single_value(slave_map_io['map_io']['node']['rfid']['fc'], slave_map_io['map_io']['node']['rfid']['addr'])
        if data_1 != 0 and data_2 != 0:
            add_or_update_data(data_1, data_2)
        set_value(slave_map_io['map_io']['node']['button']['add&change']['fc'], slave_map_io['map_io']['node']['button']['add&change']['addr'], [0])
    elif get_single_value(slave_map_io['map_io']['node']['button']['insert']['fc'], slave_map_io['map_io']['node']['button']['insert']['addr']):
        data_1 = get_single_value(slave_map_io['map_io']['node']['route']['fc'], slave_map_io['map_io']['node']['route']['addr'])
        data_2 = get_single_value(slave_map_io['map_io']['node']['pre_rfid']['fc'], slave_map_io['map_io']['node']['pre_rfid']['addr'])
        if data_1 != 0 and data_2 != 0:
            insert_data(data_1, data_2)
        set_value(slave_map_io['map_io']['node']['button']['insert']['fc'], slave_map_io['map_io']['node']['button']['insert']['addr'], [0])
    elif get_single_value(slave_map_io['map_io']['node']['button']['delete']['fc'], slave_map_io['map_io']['node']['button']['delete']['addr']):
        data_1 = get_single_value(slave_map_io['map_io']['node']['route']['fc'], slave_map_io['map_io']['node']['route']['addr'])
        data_2 = get_single_value(slave_map_io['map_io']['node']['rfid']['fc'], slave_map_io['map_io']['node']['rfid']['addr'])
        if data_1 != 0 and data_2 != 0:
            delete_data(data_1, data_2)
        set_value(slave_map_io['map_io']['node']['button']['delete']['fc'], slave_map_io['map_io']['node']['button']['delete']['addr'], [0])
    elif get_single_value(slave_map_io['map_io']['node']['button']['view']['fc'], slave_map_io['map_io']['node']['button']['view']['addr']):
        data_1 = get_single_value(slave_map_io['map_io']['node']['route']['fc'], slave_map_io['map_io']['node']['route']['addr'])
        data_2 = get_single_value(slave_map_io['map_io']['node']['rfid']['fc'], slave_map_io['map_io']['node']['rfid']['addr'])
        if data_1 != 0 and data_2 != 0:
            view_data(data_1, data_2)
    elif get_single_value(slave_map_io['map_io']['node']['button']['query']['fc'], slave_map_io['map_io']['node']['button']['query']['addr']):
        data_1 = get_single_value(slave_map_io['map_io']['node']['route']['fc'], slave_map_io['map_io']['node']['route']['addr'])
        if data_1 != 0:
            query_route(data_1)


def status(data:list):
    set_value(slave_map_io['map_io']['status']['temp']['CPU']['fc'], slave_map_io['map_io']['status']['temp']['CPU']['addr'], float32_to_registers(data[0]))
    set_value(slave_map_io['map_io']['status']['temp']['Driver']['fc'], slave_map_io['map_io']['status']['temp']['Driver']['addr'], float32_to_registers(data[1]))
    set_value(slave_map_io['map_io']['status']['motor']['fault']['m1']['fc'], slave_map_io['map_io']['status']['motor']['fault']['m1']['addr'], [data[2]])
    set_value(slave_map_io['map_io']['status']['motor']['fault']['m2']['fc'], slave_map_io['map_io']['status']['motor']['fault']['m2']['addr'], [data[3]])
    set_value(slave_map_io['map_io']['status']['motor']['current']['m1']['fc'], slave_map_io['map_io']['status']['motor']['current']['m1']['addr'], float32_to_registers(struct.unpack("h", struct.pack("H", data[4]))[0]))
    set_value(slave_map_io['map_io']['status']['motor']['current']['m2']['fc'], slave_map_io['map_io']['status']['motor']['current']['m2']['addr'], float32_to_registers(struct.unpack("h", struct.pack("H", data[5]))[0]))
    set_value(slave_map_io['map_io']['status']['comm']['TCP']['fc'], slave_map_io['map_io']['status']['comm']['TCP']['addr'], [data[6]])
    set_value(slave_map_io['map_io']['status']['comm']['CAN']['fc'], slave_map_io['map_io']['status']['comm']['CAN']['addr'], [data[7]])
    set_value(slave_map_io['map_io']['status']['comm']['bumper']['fc'], slave_map_io['map_io']['status']['comm']['bumper']['addr'], [data[8]])


def setting():
    pass


def set_RFID():
    ser = None
    ser = serial.Serial()
    ser.port = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0'
    ser.baudrate = 9600
    ser.timeout = 1000
    ser.open()

    if ser.is_open:
        bufSend = [170, 85, 4, 0, 0, 1, 171]
        ser.write(bufSend)
        SetID = int(get_value(3, 221, 1)[0])
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
