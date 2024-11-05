from pymodbus.client.serial import ModbusSerialClient
import struct
import serial
import json
import time

m_reg = 3072
v_reg = 512

class bridge_hub():

    file = "rule/node.json"

    def __init__(self):
        self.client = ModbusSerialClient(port='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0', baudrate=115200, parity='N', stopbits=2, timeout=1000)
        self.client.connect()
        self.connection = self.client.connected 

    def parameter_data(self):

        route_id = self.client.read_holding_registers(v_reg + 35, 3, 1).registers
        param = self.client.read_holding_registers(v_reg + 30 , 5, 1).registers
        data_route_id = self.client.read_holding_registers(v_reg + 158, 1, 1).registers[0]

        motion_1 = self.read_string_from_modbus(v_reg, 6)
        motion_2 = self.read_string_from_modbus(v_reg + 6, 6)
        motion_3 = self.read_string_from_modbus(v_reg + 12, 6)
        motion_4 = self.read_string_from_modbus(v_reg + 18, 6)
        motion_5 = self.read_string_from_modbus(v_reg + 24, 6)

        if self.client.read_coils(m_reg + 1, 1, 1).bits[0] == 1:
            self.insert_data(route_id[0], route_id[1])
            self.client.write_coil(m_reg + 1, 0, 1)
        
        if self.client.read_coils(m_reg + 2, 1, 1).bits[0] == 1:
            self.delete_data(route_id[0], route_id[2])
            self.client.write_coil(m_reg + 2, 0, 1)

        if self.client.read_coils(m_reg + 3, 1, 1).bits[0] == 1:
            self.query_data(route_id[0], route_id[2])

        if self.client.read_coils(m_reg + 4, 1, 1).bits[0] == 1:
            self.query_route(data_route_id)
            
        if self.client.read_coils(m_reg, 1, 1).bits[0] == 1:
            data = {
                "motion_1" : [motion_1, param[0]],
                "motion_2" : [motion_2, param[1]],
                "motion_3" : [motion_3, param[2]],
                "motion_4" : [motion_4, param[3]],
                "motion_5" : [motion_5, param[4]]
            }
            if route_id[0] != 0 and route_id[2] != 0:
                self.add_or_update_data(route_id[0], route_id[2], data)
            self.client.write_coil(m_reg, 0, 1)
    
    def read_string_from_modbus(self, start_address, length):
        result = self.client.read_holding_registers(start_address, length, 1)
        byte_array = b''.join([reg.to_bytes(2, byteorder='little') for reg in result.registers]).decode('ascii').strip('\x00')
        return byte_array
    
    def write_string_to_modbus(self, start_address, string_data, length):
        if  string_data == "":
            zero_data = [0] * length
            self.client.write_registers(start_address, zero_data)
        else:
            if len(string_data) % 2 != 0:
                string_data += '\x00' 
            byte_data = string_data.encode('ascii')
            registers = [int.from_bytes(byte_data[i:i+2], byteorder='little') for i in range(0, len(byte_data), 2)]
            self.client.write_registers(start_address, registers)
    
    def add_or_update_data(self, route_id, node_id, new_data, file_path=file):
        with open(file_path, "r") as json_file:
            data = json.load(json_file)
        if str(route_id) not in data:
            data[str(route_id)] = {}
        data[str(route_id)][str(node_id)] = new_data 
        with open(file_path, "w") as json_file:
            json.dump(data, json_file, indent=4)
    
    def delete_data(self, route_id, node_id, file_path=file):
        with open(file_path, "r") as json_file:
            data = json.load(json_file)
        if str(route_id) in data and str(node_id) in data[str(route_id)]:
            del data[str(route_id)][str(node_id)]
            if not data[str(route_id)]:
                del data[str(route_id)]
        with open(file_path, "w") as json_file:
            json.dump(data, json_file, indent=4)

    def query_data(self, route_id, node_id, file_path=file):
        with open(file_path, "r") as json_file:
            data = json.load(json_file)
        if str(route_id) in data and str(node_id) in data[str(route_id)]:
            data_list = list(data[str(route_id)][str(node_id)].keys())
            data_motion = []
            for item in range(len(data_list)):
                data_motion.append(data[str(route_id)][str(node_id)][data_list[item]])
                self.write_string_to_modbus(v_reg + item * 6, "", 6)
                self.client.write_register(v_reg + item + 30, 0, 1)
            for item in range(len(data_motion)):
                self.write_string_to_modbus(v_reg + item * 6, data_motion[item][0], 6)
                self.client.write_register(v_reg + item + 30, data_motion[item][1], 1)
    
    def insert_data(self, route_id, pre_id, file_path=file):
        with open(file_path, "r") as json_file:
            data = json.load(json_file)
        if str(route_id) in data and str(pre_id) in data[str(route_id)]:
            data_list = list(data[str(route_id)][str(pre_id)].keys())
            data_motion = []
            for item in range(len(data_list)):
                data_motion.append(data[str(route_id)][str(pre_id)][data_list[item]])
                self.write_string_to_modbus(v_reg + item * 6, "", 6)
                self.client.write_register(v_reg + item + 30, 0, 1)
            for item in range(len(data_motion)):
                self.write_string_to_modbus(v_reg + item * 6, data_motion[item][0], 6)
                self.client.write_register(v_reg + item + 30, data_motion[item][1], 1)
    
    def query_route(self, route_id, file_path=file):
        for i in range(120):
            self.client.write_register(v_reg + 38 + i, 0, 1)
        with open(file_path, "r") as file_json:
            data = json.load(file_json)
        if str(route_id) in data:
            data_list = list(data[str(route_id)].keys())
            for item in range(len(data_list)):
                self.client.write_register(v_reg + 38 + item, int(data_list[item]), 1)
        
if __name__ == '__main__':
    app = bridge_hub()
    while True:
        if app.connection:
            app.parameter_data()
        else:
            print('false')