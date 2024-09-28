from pymodbus.client.serial import ModbusSerialClient
from RFID import RFID_setting
import struct

m_reg = 3072
v_reg = 512

PID_reg = [1000, 1002, 1004, 1006]
VI_reg = [1014, 1016]
c_kecepatan_reg = 1018

class bridge_hub():

    PID = [0.0, 0.0, 0.0]
    Kecepatan = 0.0
    c_kecepatan = 0.0
    dir_data = [False, False, False]
    Kontrol_data = [False, False, False, False]
    Obstacel_data = [False, False]

    start_b = False
    manual_b = False
    set_ID_b = False

    def __init__(self):
        self.client = ModbusSerialClient(port='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0', baudrate=115200, parity='N', stopbits=1)
        self.client.connect()
        self.RFID_ss = RFID_setting()
        self.connection = self.client.connected 

    def parameter_data(self):
        self.PID[0] = self.registers_to_float(self.client.read_holding_registers((v_reg + PID_reg[0]), 2, 1).registers)
        self.PID[1] = self.registers_to_float(self.client.read_holding_registers((v_reg + PID_reg[1]), 2, 1).registers)
        self.PID[2] = self.registers_to_float(self.client.read_holding_registers((v_reg + PID_reg[2]), 2, 1).registers)
        self.Kecepatan = self.registers_to_float(self.client.read_holding_registers((v_reg + PID_reg[3]), 2, 1).registers)
        self.c_kecepatan = self.registers_to_float(self.client.read_holding_registers((v_reg + c_kecepatan_reg), 2, 1).registers)

        self.Obstacel_data = self.client.read_coils((m_reg + 11), 2, 1).bits
        self.Kontrol_data = self.client.read_coils((m_reg + 22), 4, 1).bits
        self.dir_data = self.client.read_coils((m_reg + 63), 3, 1).bits
        self.manual_b = self.client.read_coils((m_reg + 76), 1, 1).bits[0]
        self.set_ID_b = self.client.read_coils((m_reg + 78), 1, 1).bits[0]
        self.start_b = self.client.read_coils(m_reg, 1, 1).bits[0]

    def tegangan_robot(self, tegangan, arus, ac_ID):
        if self.connection:
            self.client.write_registers((v_reg + VI_reg[1]), self.float_to_registers(tegangan), 1)
            self.client.write_registers((v_reg + VI_reg[0]), self.float_to_registers(arus), 1) 
            self.client.write_register((v_reg + 1020), ac_ID, 1)       
    
    def buzzer_select(self, data):
        if self.connection:
            if data == 'S1':
                voice = [1, 0, 0, 0]
            elif data == 'S2':
                voice = [0, 1, 0, 0]
            elif data == 'S3':
                voice = [0, 0, 1, 0]
            elif data == 'S4':
                voice = [0, 0, 0, 1]
            else:
                voice = [0, 0, 0, 0]
            self.client.write_coils((m_reg + 3), voice, 1)
    
    def set_ID_card(self):
        self.RFID_ss.set_ID(int(self.client.read_holding_registers((v_reg + 1021), 1, 1).registers[0]))

    def float_to_registers(self, float_value):
        float_bytes = struct.pack('>f', float_value)  
        reg1, reg2 = struct.unpack('>HH', float_bytes)  
        return [reg2, reg1]
    
    def registers_to_float(self, registers):
        reg1, reg2 = registers
        float_bytes = struct.pack('>HH', reg2, reg1) 
        float_value = struct.unpack('>f', float_bytes)[0]
        return float_value
   
